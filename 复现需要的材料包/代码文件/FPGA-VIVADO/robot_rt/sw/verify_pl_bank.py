"""Read packed PL bytes back through the existing integer reference, offline."""
import json
from pathlib import Path
import struct
import zlib
from sim.player_reference import State, Bounds, advance, encode_can, to_q16
from sim.tick_reference import DDSConfig

SPEC = json.loads((Path(__file__).resolve().parents[1]/'spec/regmap.yaml').read_text())


def verify_bank(data, image, config):
    if len(image) != 65536 or len(config) != SPEC['register_window_bytes']:
        raise ValueError('bank/config image length')
    if image[:16] != b'RTBK'+struct.pack('<III',1,256,256):
        raise ValueError('bank layout/version')
    header = bytearray(image[:256])
    h = {f['name']:struct.unpack_from('<I' if f['bytes']==4 else '<Q',header,f['offset'])[0]
         for f in SPEC['bank_header'] if f['bytes'] in (4,8) and f['name'] != 'magic'}
    stored = struct.unpack_from('<I',header,100)[0]
    header[100:104] = bytes(4)
    if zlib.crc32(header) != stored:
        raise ValueError('header CRC')
    count = h['normal_count']+h['brake_count']
    if not 0 < h['normal_count'] < count <= SPEC['max_records_per_bank']:
        raise ValueError('record capacity/count')
    if zlib.crc32(image[256:256*(count+1)]) != h['payload_crc32']:
        raise ValueError('payload CRC')
    if any(image[256*(count+1):]) or any(header[112:]) or h['reserved0']:
        raise ValueError('nonzero reserved/padding bytes')
    canonical = bytearray(config)
    canonical[0x70:0x74] = bytes(4)
    cfg_crc = zlib.crc32(b''.join(canonical[a:b] for a,b in SPEC['config_crc_ranges']))
    if cfg_crc != h['config_crc32'] or cfg_crc != struct.unpack_from('<I',config,0x70)[0]:
        raise ValueError('config CRC')
    if zlib.crc32(config[0x400:0x448]) != struct.unpack_from('<I',config,0x448)[0]:
        raise ValueError('seed CRC')
    for key in ('bank_sequence','session_nonce','config_epoch','previous_sequence',
                'observation_cycle','produced_cycle','normal_deadline_cycle',
                'first_tick_index','map_error_cycles'):
        if h[key] != data[key]:
            raise ValueError(f'header {key} mismatch')
    bounds = Bounds.from_physical(**data['limits'])
    dds = DDSConfig(data['f_num'],data['f_den'],data['clock_hz'])
    state = State.from_physical(**data['initial_state'])
    regs = {r['name']:r for r in SPEC['registers']}
    expected = dict(SESSION_NONCE=data['session_nonce'],CONFIG_EPOCH=data['config_epoch'],
        RATE_NUM=data['f_num'],RATE_DEN=data['f_den'],BACKEND_MODE=0,
        SOURCE_MAX_AGE=data['source_max_age_cycles'],LEASE_CYCLES=data['lease_cycles'],
        MAX_MAP_ERROR=data['max_map_error_cycles'],STOP_TIMEOUT=data['stop_timeout_cycles'],
        GRIPPER_POLICY=0x10001,GRIPPER_FORCE_MN=data['gripper_force_mn'],
        SEED_CYCLE=data['seed_cycle'],SEED_SEQUENCE=data['seed_sequence'])
    for axis in range(7):
        expected[f'SEED_Q{axis}'] = state.q[axis]
        for attr in bounds.__dataclass_fields__:
            expected[f'AXIS{axis}_{attr.upper()}'] = getattr(bounds,attr)[axis]
    for name,value in expected.items():
        reg = regs[name]
        raw = struct.unpack_from('<I' if reg['width']==32 else '<Q',config,reg['offset'])[0]
        if raw != value & ((1 << reg['width'])-1):
            raise ValueError(f'config {name} differs from replay inputs')
    wire = encode_can(tuple(to_q16(q,l,u) for q,l,u in zip(state.q,bounds.lower,bounds.upper)),bounds)
    intervals, outputs = set(), []
    for i in range(count):
        start,end,index,flags,reserved,*words = struct.unpack_from('<QQQII28q',image,256*(i+1))
        tick = data['first_tick_index']+i
        expected_start = data['origin_cycle']+(dds.tick_cycle(tick-1) if tick>1 else 0)
        expected_end = data['origin_cycle']+dds.tick_cycle(tick)
        if (start,end,index,flags,reserved) != (expected_start,expected_end,tick,1 if i<h['normal_count'] else 2,0):
            raise ValueError('record DDS timing/flags/reserved mismatch')
        if i==0 and h['first_cycle'] != start:
            raise ValueError('header first cycle')
        if i<h['normal_count'] and end+SPEC['latency_cycles']+data['map_error_cycles'] >= h['normal_deadline_cycle']:
            raise ValueError('expired normal output')
        decoded = State(tuple(words[0::4]),tuple(words[1::4]),tuple(words[2::4]))
        if decoded != state:
            raise ValueError('record integer q/v/a continuity mismatch')
        result = advance(state,tuple(words[3::4]),end-start,bounds,
                         expected_start=decoded,clock_hz=data['clock_hz'])
        wire = encode_can(result.q16,bounds,wire)
        state = result.next_state
        intervals.add(end-start);outputs.append(wire)
    if any((*state.v,*state.a)):
        raise ValueError('nonzero terminal velocity/acceleration')
    return dict(records=count,interval_cycles=sorted(intervals),
                integer_end_state=state.as_dict(),wire_outputs=outputs,
                header_payload_config_seed_crc_pass=True,exact_seams_pass=True)
