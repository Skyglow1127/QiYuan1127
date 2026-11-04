#!/usr/bin/env python3
"""Build a design-only PL bank/config image; never opens a device.

Example:
  /usr/bin/python3 sw/pl_bank.py --design-example --f-num 275 --f-den 2 \
      --origin-cycle 0 --first-index 8 --output-dir /tmp/pl_bank_example

The example is stationary invented data. A normal input JSON must explicitly
provide all limits, clock/source metadata, gripper policy, and jerk vectors.
No clock mapping, model inference, robot transport, /dev/mem or ARM is performed.
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import struct
import sys
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.tick_reference import DDSConfig
from sim.player_reference import Bounds, State, advance, encode_can, encode_q32, to_q16


def uint(value, bits=64):
    if type(value) is not int or not 0 <= value < 1 << bits:
        raise ValueError(f'expected uint{bits}')
    return value


@dataclass(frozen=True)
class BankArtifact:
    image: bytes
    config: bytes
    register_writes: tuple[tuple[int, int], ...]
    report: dict


def build_bank(data: dict) -> BankArtifact:
    if data.get('design_only') is not True:
        raise ValueError('only explicitly design_only input is supported; no live clock/transport backend')
    spec = json.loads((ROOT/'spec/regmap.yaml').read_text())
    clock_hz = data.get('clock_hz', 100_000_000)
    dds = DDSConfig(data['f_num'], data['f_den'], clock_hz=clock_hz)
    if dds.interval_bounds[1] > (1 << 32)-1:
        raise ValueError('DDS interval exceeds player uint32 duration')
    origin, first = uint(data['origin_cycle']), uint(data['first_tick_index'])
    if not first:
        raise ValueError('first_tick_index is one-based')
    fields = {name: uint(data[name]) for name in (
        'session_nonce','bank_sequence','previous_sequence','observation_cycle',
        'produced_cycle','normal_deadline_cycle','validation_at_cycle',
        'lease_cycles','source_max_age_cycles','stop_timeout_cycles','seed_cycle','seed_sequence')}
    for name in ('session_nonce','bank_sequence','lease_cycles','source_max_age_cycles','stop_timeout_cycles','seed_sequence'):
        if not fields[name]: raise ValueError(f'{name} must be nonzero')
    for name in ('lease_cycles','source_max_age_cycles','stop_timeout_cycles'):
        if fields[name] > (1 << 63)-1: raise ValueError(f'{name} exceeds signed comparison horizon')
    epoch, error, max_error = (uint(data[n],32) for n in ('config_epoch','map_error_cycles','max_map_error_cycles'))
    if not epoch or error > max_error:
        raise ValueError('epoch/error bound')
    obs, produced, now, deadline = (fields[n] for n in ('observation_cycle','produced_cycle','validation_at_cycle','normal_deadline_cycle'))
    if obs < error or produced < error:
        raise ValueError('mapped uncertainty would represent a negative source time')
    uint(obs-error+fields['source_max_age_cycles'])
    if not (obs+error <= produced-error and produced+error <= now and
            now+error < deadline <= obs-error+fields['source_max_age_cycles']):
        raise ValueError('source/production/deadline mapping inequalities')
    bounds = Bounds.from_physical(**data['limits'])
    if len(bounds.lower) != 7: raise ValueError('seven physical axes required')
    state = State.from_physical(**data['initial_state'])
    if len(state.q) != 7: raise ValueError('seven initial axes required')
    if fields['previous_sequence'] == 0 and any((*state.v,*state.a)):
        raise ValueError('initial bank needs stationary seed')
    if fields['previous_sequence'] >= fields['bank_sequence']:
        raise ValueError('non-increasing bank sequence')
    seed_deadline=uint(fields['seed_cycle']+fields['source_max_age_cycles'])
    if not fields['seed_cycle'] <= now < seed_deadline:
        raise ValueError('seed future/stale')
    if data['gripper_explicit_reviewed'] is not True or uint(data['gripper_status_code'],8) != 1 or uint(data['gripper_set_zero'],8) != 0:
        raise ValueError('explicit reviewed width-mode gripper policy required, set_zero must be zero')
    force = uint(data['gripper_force_mn'],16)
    if not force: raise ValueError('explicit positive gripper force required')
    normals, brakes = data['normal_jerks'], data['brake_jerks']
    if not normals or not brakes or len(normals)+len(brakes) > spec['max_records_per_bank']:
        raise ValueError('normal and stop tail must both fit the bank')
    cfg = bytearray(spec['register_window_bytes'])
    regs = {r['name']:r for r in spec['registers']}
    writes=[]
    def put(name,value):
        reg=regs[name];mask=(1 << reg['width'])-1
        struct.pack_into('<I' if reg['width']==32 else '<Q',cfg,reg['offset'],value & mask)
    for name,value in dict(SESSION_NONCE=fields['session_nonce'],CONFIG_EPOCH=epoch,BACKEND_MODE=0,
        RATE_NUM=dds.f_num,RATE_DEN=dds.f_den,LEASE_CYCLES=fields['lease_cycles'],
        SOURCE_MAX_AGE=fields['source_max_age_cycles'],MAX_MAP_ERROR=max_error,STOP_TIMEOUT=fields['stop_timeout_cycles'],
        GRIPPER_POLICY=0x10001,GRIPPER_FORCE_MN=force).items(): put(name,value)
    for axis in range(7):
        for attr,name in (('lower','LOWER'),('upper','UPPER'),('velocity','VELOCITY'),('acceleration','ACCELERATION'),('jerk','JERK'),('dq_max','DQ_MAX')):
            put(f'AXIS{axis}_{name}',getattr(bounds,attr)[axis])
        put(f'SEED_Q{axis}',state.q[axis])
    put('SEED_CYCLE',fields['seed_cycle']);put('SEED_SEQUENCE',fields['seed_sequence'])
    put('SEED_CRC32',zlib.crc32(cfg[0x400:0x448]))
    canonical=b''.join(cfg[a:b] for a,b in spec['config_crc_ranges'])
    config_crc=zlib.crc32(canonical);put('CONFIG_CRC32',config_crc)
    for reg in spec['registers']:
        if reg['access']=='RW_SHADOW' and reg['name'] not in ('BANK0_SEQUENCE','BANK1_SEQUENCE','ARM_FIRST_TICK','LEASE_KICK_SEQUENCE'):
            for offset in range(reg['offset'],reg['offset']+reg['width']//8,4):
                writes.append((offset,struct.unpack_from('<I',cfg,offset)[0]))
    start0=uint(origin+(dds.tick_cycle(first-1) if first>1 else 0))
    previous_wire=encode_can(tuple(to_q16(q,l,u) for q,l,u in zip(state.q,bounds.lower,bounds.upper)),bounds)
    records=[];q16_samples=[];wire_samples=[]
    for i,jerk_physical in enumerate(normals+brakes):
        tick=first+i
        start=uint(origin+(dds.tick_cycle(tick-1) if tick>1 else 0))
        end=uint(origin+dds.tick_cycle(tick))
        if uint(end+spec['latency_cycles']+error) >= deadline and i<len(normals):
            raise ValueError('normal output would be expired')
        if len(jerk_physical)!=7: raise ValueError('seven jerk axes required')
        jerk=tuple(encode_q32(v) for v in jerk_physical)
        result=advance(state,jerk,end-start,bounds,expected_start=state,clock_hz=clock_hz)
        wire=encode_can(result.q16,bounds,previous_wire)
        axis_words=[word for axis in range(7) for word in (state.q[axis],state.v[axis],state.a[axis],jerk[axis])]
        records.append(struct.pack('<QQQII'+'q'*28,start,end,tick,1 if i<len(normals) else 2,0,*axis_words))
        q16_samples.append(result.q16);wire_samples.append(wire)
        state=result.next_state;previous_wire=wire
    if any((*state.v,*state.a)):
        raise ValueError('stop tail must end at EXACT integer v=a=0; no threshold snap')
    if end-start0 > fields['stop_timeout_cycles']:
        raise ValueError('whole bank exceeds stop timeout')
    payload=b''.join(records);header=bytearray(256)
    header[:4]=b'RTBK'
    values=dict(version=1,header_bytes=256,record_bytes=256,bank_sequence=fields['bank_sequence'],
        session_nonce=fields['session_nonce'],config_epoch=epoch,normal_count=len(normals),brake_count=len(brakes),
        observation_cycle=obs,produced_cycle=produced,normal_deadline_cycle=deadline,first_tick_index=first,
        first_cycle=start0,previous_sequence=fields['previous_sequence'],payload_crc32=zlib.crc32(payload),
        config_crc32=config_crc,map_error_cycles=error)
    for field in spec['bank_header']:
        if field['name'] in values:
            struct.pack_into('<I' if field['bytes']==4 else '<Q',header,field['offset'],values[field['name']])
    struct.pack_into('<I',header,100,zlib.crc32(header))
    image=bytes(header)+payload+bytes(65536-256-len(payload))
    return BankArtifact(image,bytes(cfg),tuple(writes),{
        'design_only':True,'hardware_access':False,'f_exec_measured_hz':None,'backend':'DESC_ONLY',
        'clock_hz':clock_hz,'clock_note':'Nominal compiled profile; not measured oscillator frequency',
        'bank_sequence':fields['bank_sequence'],'normal_records':len(normals),'brake_records':len(brakes),
        'config_crc32':f'{config_crc:08x}','bank_sha256':hashlib.sha256(image).hexdigest(),
        'integer_end_state':state.as_dict(),'q16_samples':q16_samples,'CAN_integer_samples':wire_samples,
        'limits_are_robot_verified':False,'origin_cycle':origin,'first_tick_index':first})


def example(num,den,origin,first,clock_hz=100_000_000):
    return dict(design_only=True,f_num=num,f_den=den,clock_hz=clock_hz,origin_cycle=origin,first_tick_index=first,
        session_nonce=1,config_epoch=1,bank_sequence=1,previous_sequence=0,observation_cycle=origin,
        produced_cycle=origin,validation_at_cycle=origin,normal_deadline_cycle=origin+200000000,
        map_error_cycles=0,max_map_error_cycles=0,lease_cycles=200000000,source_max_age_cycles=200000000,
        stop_timeout_cycles=200000000,seed_cycle=origin,seed_sequence=1,
        gripper_explicit_reviewed=True,gripper_status_code=1,gripper_set_zero=0,gripper_force_mn=1000,
        limits=dict(lower=[-10]*6+[0],upper=[10]*6+[0.1],velocity=[10]*7,acceleration=[10]*7,jerk=[10]*7,dq_max=[1]*7),
        initial_state=dict(q=[0]*6+[0.05],v=[0]*7,a=[0]*7),normal_jerks=[[0]*7]*2,brake_jerks=[[0]*7]*2)


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--input',type=Path);mode.add_argument('--design-example',action='store_true')
    parser.add_argument('--f-num',type=int);parser.add_argument('--f-den',type=int)
    parser.add_argument('--clock-hz',type=int,help='Nominal compiled CLOCK_HZ; example default 100000000; match build addresses.json')
    parser.add_argument('--origin-cycle',type=int);parser.add_argument('--first-index',type=int)
    parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args()
    try:
        if args.input:
            if any(value is not None for value in (args.f_num,args.f_den,args.origin_cycle,args.first_index,args.clock_hz)):
                raise ValueError('input JSON owns its rate/time fields; example options cannot silently override it')
            data=json.loads(args.input.read_text())
        else:
            if None in (args.f_num,args.f_den,args.origin_cycle,args.first_index):
                raise ValueError('example requires explicit rate, origin and first index')
            data=example(args.f_num,args.f_den,args.origin_cycle,args.first_index,
                         clock_hz=100_000_000 if args.clock_hz is None else args.clock_hz)
        result=build_bank(data)
        args.output_dir.mkdir(parents=True,exist_ok=False)
        (args.output_dir/'input.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
        (args.output_dir/'bank0.bin').write_bytes(result.image)
        (args.output_dir/'bank0.mem').write_text(''.join(f'{word[0]:08x}\n' for word in struct.iter_unpack('<I',result.image)))
        (args.output_dir/'config_writes.mem').write_text(''.join(f'{address:08x}{value:08x}\n' for address,value in result.register_writes))
        (args.output_dir/'report.json').write_text(json.dumps(result.report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result.report,ensure_ascii=False,indent=2))
    except (ValueError,TypeError,KeyError,OSError) as exc:
        parser.exit(2,f'PL bank: {exc}\n')


if __name__=='__main__': main()
