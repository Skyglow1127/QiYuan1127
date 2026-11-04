#!/usr/bin/env python3
"""Offline pi05 service response -> source-time scheduler -> integer PL bank.

python3 sw/pi05_to_pl.py --input examples/pi05/simulated.json \
    --config examples/pi05/offline_config.json --output-dir /tmp/pi05-bank
No inference call, device, CAN, MMIO, build, or ARM operation is performed.
"""
from __future__ import annotations
import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from sw.rt_adapter.pi05 import Pi05Contract
from sw.rt_adapter.scheduler import ChunkScheduler
from sw.rt_adapter.trajectory import MotionBounds
from sw.rt_adapter.integer_planner import IntegerPlanner, physical
from sim.player_reference import State, floor, ceil
from sim.tick_reference import DDSConfig
from sw.pl_bank import build_bank
from sw.verify_pl_bank import verify_bank


def number(value):
    if isinstance(value,bool):
        raise ValueError('boolean is not a time')
    return Fraction(str(value))


def compile_response(envelope, config):
    if config['design_only'] is not True or envelope['clock_domain'] != 'caller_monotonic':
        raise ValueError('offline design_only and caller_monotonic timestamps required')
    if config['bank_identity']['previous_sequence'] != 0:
        raise ValueError('this offline compiler supports an initial bank only; successor seam needs live state')
    steps = config['consume_steps']
    if type(steps) is not int or not 1 <= steps <= 50:
        raise ValueError('explicit consume_steps must be 1..50, independent of checkpoint default')
    tail_budget = config['max_brake_records']
    if type(tail_budget) is not int or not 2 <= tail_budget <= 254:
        raise ValueError('explicit max_brake_records must be 2..254')
    contract = Pi05Contract(**config['contract'])
    chunk = contract.adapt(envelope['response'], sequence=envelope['sequence'],
        observation_at=envelope['observation_at'], available_at=envelope['available_at'],
        request_id=envelope['request_id'], input_sha256=envelope['input_sha256'])
    bounds = MotionBounds.from_dict(config['limits'])
    state = State.from_physical(**envelope['initial_state'])
    if any((*state.v,*state.a)):
        raise ValueError('single initial bank needs an explicitly stationary seed; moving state is not reset')
    scheduler = ChunkScheduler(consume_steps=steps, source_max_age_s=config['source_max_age_s'],
                              slot_lateness_s=config['slot_lateness_s'],bounds=bounds)
    now = envelope['planning_at']
    accepted = scheduler.accept(chunk,now=now,current_target=physical(state.q))
    C = config['clock_hz']
    dds = DDSConfig(config['f_num'],config['f_den'],clock_hz=C)
    mapping = envelope['design_clock_mapping']
    if mapping['kind'] != 'offline_affine_not_hardware_synchronized':
        raise ValueError('only explicitly offline affine clock mapping is implemented')
    anchor_time, anchor_cycle = number(mapping['monotonic_at_anchor']), mapping['anchor_cycle']
    if type(anchor_cycle) is not int or anchor_cycle < 0:
        raise ValueError('invalid mapping anchor cycle')
    def cycle(t): return anchor_cycle+(number(t)-anchor_time)*C
    def time(c): return float(anchor_time+Fraction(c-anchor_cycle,C))
    origin = envelope['dds_origin_cycle']
    if type(origin) is not int or origin < 0 or origin > floor(cycle(now)):
        raise ValueError('DDS origin must be explicit, nonnegative and no later than planning')
    lead = number(config['planning_lead_s'])
    if lead < 0:
        raise ValueError('planning_lead_s must be nonnegative')
    earliest = max(cycle(number(now)+lead),cycle(chunk.slot_at(accepted['first_valid_slot'])))
    # Segment START must be after availability/validation. No backdated segment.
    preceding = max(0,ceil((earliest-origin)*dds.f_num/(C*dds.f_den)))
    first = preceding+1
    def end(tick): return origin+dds.tick_cycle(tick)
    def start(tick): return origin+(dds.tick_cycle(tick-1) if tick>1 else 0)
    obs = floor(cycle(chunk.observation_at))
    source_age = floor(number(config['source_max_age_s'])*C)
    deadline = min(obs+source_age,floor(cycle(number(chunk.observation_at)+Fraction(steps,20))))
    spec = json.loads((ROOT/'spec/regmap.yaml').read_text())
    planner = IntegerPlanner(state,bounds,clock_hz=C,response_time_s=config['response_time_s'])
    normal, trace = [], []
    tick = first
    while end(tick)+spec['latency_cycles'] < deadline:
        sample = scheduler.reference_at(now=time(end(tick)))
        if sample is None:
            break
        if len(normal)+2 >= spec['max_records_per_bank']:
            raise ValueError('normal window exceeds single-bank capacity; choose smaller explicit consumption window')
        normal.append(planner.propose(sample.target,end(tick)-start(tick)))
        trace.append(dict(tick_index=tick,start_cycle=start(tick),end_cycle=end(tick),
            slot_index=sample.slot_index,scheduled_at=sample.scheduled_at,
            observation_at=sample.observation_at,target=sample.target.tolist(),
            planned_q=physical(planner.state.q).tolist(),
            tracking_error=(physical(planner.state.q)-sample.target).tolist()))
        tick += 1
    if not normal:
        raise ValueError('no unexpired normal output fits availability, lead and original deadline')
    normal_end_state = planner.state.as_dict()
    count = min(tail_budget,spec['max_records_per_bank']-len(normal))
    brake = planner.stop([end(tick+i)-start(tick+i) for i in range(count)])
    data = dict(config['bank_identity'],design_only=True,clock_hz=C,
        f_num=dds.f_num,f_den=dds.f_den,origin_cycle=origin,first_tick_index=first,
        observation_cycle=obs,produced_cycle=ceil(cycle(chunk.produced_at)),
        validation_at_cycle=ceil(cycle(now)),normal_deadline_cycle=deadline,
        map_error_cycles=0,max_map_error_cycles=0,source_max_age_cycles=source_age,
        lease_cycles=floor(number(config['lease_s'])*C),
        stop_timeout_cycles=floor(number(config['stop_timeout_s'])*C),
        seed_cycle=floor(cycle(envelope['seed_at'])),
        limits=config['limits'],initial_state=envelope['initial_state'],
        normal_jerks=normal,brake_jerks=brake,**config['gripper'])
    if end(tick+len(brake)-1)+spec['latency_cycles'] >= data['validation_at_cycle']+data['lease_cycles']:
        raise ValueError('whole offline bank exceeds explicit lease; no future kick is assumed')
    artifact = build_bank(data)
    verification = verify_bank(data,artifact.image,artifact.config)
    if artifact.report['integer_end_state'] != planner.state.as_dict():
        raise AssertionError('bank packing diverged from planner integer endpoint')
    report = dict(artifact.report,adapter='pi05 postprocessed absolute physical units',
        source_provenance=envelope['provenance'],clock_mapping=mapping,
        observation_at=chunk.observation_at,available_at=chunk.produced_at,
        planning_at=now,chunk_sequence=chunk.sequence,sample_hz=chunk.sample_hz,
        consume_steps=steps,execution_rate=dict(num=dds.f_num,den=dds.f_den),
        accepted=accepted,normal_end_state=normal_end_state,
        binary_verification={k:v for k,v in verification.items() if k!='wire_outputs'},
        normal_deadline_cycle=deadline,normal_trace=trace,
        source_retimestamped=False,normalization_applied=False,
        planning_runtime_validated=False,
        limitations=['Offline single initial bank, no live PS/PL clock synchronization',
                     'Stationary seed is caller-supplied, not independently measured',
                     'No physical feedback, transport, hardware enable or collision validation'])
    return data,artifact,report


def write_artifacts(folder,envelope,config,data,artifact,report):
    folder.mkdir(parents=True,exist_ok=False)
    for name,value in (('response_input.json',envelope),('run_config.json',config),
                       ('bank_input.json',data),('report.json',report)):
        (folder/name).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (folder/'bank0.bin').write_bytes(artifact.image)
    (folder/'bank0.mem').write_text(''.join(f'{v[0]:08x}\n' for v in struct.iter_unpack('<I',artifact.image)))
    (folder/'config.bin').write_bytes(artifact.config)
    (folder/'config_writes.mem').write_text(''.join(f'{a:08x}{v:08x}\n' for a,v in artifact.register_writes))
    sources = ['sw/pi05_to_pl.py','sw/rt_adapter/pi05.py','sw/rt_adapter/integer_planner.py',
               'sw/rt_adapter/scheduler.py','sw/rt_adapter/trajectory.py','sw/pl_bank.py',
               'sim/player_reference.py','sim/tick_reference.py','spec/regmap.yaml','sw/verify_pl_bank.py']
    manifest = {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}
    (folder/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True,type=Path)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args()
    try:
        envelope,config = (json.loads(p.read_text()) for p in (args.input,args.config))
        data,artifact,report=compile_response(envelope,config)
        write_artifacts(args.output_dir,envelope,config,data,artifact,report)
        print(json.dumps({k:report[k] for k in ('bank_sha256','normal_records','brake_records',
            'clock_hz','consume_steps','accepted','integer_end_state')},indent=2))
    except (ValueError,TypeError,KeyError,OSError) as exc:
        parser.exit(2,f'pi05 offline bank: {exc}\n')


if __name__=='__main__': main()
