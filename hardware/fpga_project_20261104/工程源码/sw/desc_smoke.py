#!/usr/bin/env python3
"""Plan/parse a stationary DESC_ONLY smoke test, entirely offline.

Examples (input.json is the explicit, design-only input of sw/pl_bank.py):
  python sw/desc_smoke.py plan --input input.json --arm-by-cycle 1000000 \
      --begin-cycle 1100000 --end-cycle 50000000 --output-dir /tmp/desc_plan
  python sw/desc_smoke.py verify --plan /tmp/desc_plan/plan.json \
      --capture exported_capture.json --output /tmp/desc_result.json

No frequency default, device/transport imports, MMIO, board access, or real CMD
writes. The printed ARM/KICK/STOP operations are data for review only. Capture
validation checks supplied receipts, not their physical authenticity. A plan's
CONFIG_COMMIT changes the DDS origin: read it back and abort on mismatch; never
blindly replay the remainder with a guessed origin.
"""
from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import stat
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sw.pl_bank import build_bank, uint
from sim.tick_reference import DDSConfig
from sim.player_reference import round_even

PLAN_SCHEMA = 'robot_rt.desc_smoke_plan.v1'
CAPTURE_SCHEMA = 'robot_rt.desc_smoke_capture.v1'
RESULT_SCHEMA = 'robot_rt.desc_smoke_result.v1'
SPEC = json.loads((ROOT/'spec/regmap.yaml').read_text())
REGS = {r['name']: r for r in SPEC['registers']}
CMDS = SPEC['commands']
LATENCY = SPEC['latency_cycles']
SNAP_NAMES = tuple(name for name in REGS if name.startswith('SNAP_'))
IDENTITY_NAMES = ('ID', 'VERSION', 'CAPS', 'STATE', 'STATUS', 'FAULT_CODE', 'REJECT_CODE',
                  'SESSION_NONCE', 'CONFIG_EPOCH', 'BACKEND_MODE', 'RATE_NUM', 'RATE_DEN',
                  'CYCLE_ORIGIN', 'NOMINAL_CLOCK_HZ')
CAPTURE_NAMES = IDENTITY_NAMES + SNAP_NAMES + tuple(f'DESC_SNAPSHOT_WORD{i}' for i in range(24))
CONTROL_NAMES = ('CONFIG_COMMIT', 'COMMIT_BANK0', 'KICK', 'ARM', 'STOP')


def canonical(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)+'\n').encode()


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def read_json(path):
    # Refuse devices/FIFOs, including symlinks resolving to a non-regular file.
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError(f'JSON input must be a regular file: {path}')
    return json.loads(path.read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(f'non-finite JSON {x}')))


def register_words(names=CAPTURE_NAMES):
    return {f'0x{offset:03x}': None for name in names for offset in
            range(REGS[name]['offset'], REGS[name]['offset']+REGS[name]['width']//8, 4)}


def _word_writes(address, value):
    return [{'op': 'write32', 'address': address+i*4, 'value': (value >> (i*32)) & 0xffffffff,
             'wstrb': 15} for i in range(2)]


def _command(name):
    return {'op': 'CMD_review_only', 'name': name, 'address': REGS['CMD']['offset'],
            'value': CMDS[name], 'wstrb': 15, 'require_bresp': 0}


def _ticks_at(plan, cycle):
    data = plan['bank_input']
    if cycle < data['origin_cycle']:
        return 0
    return (cycle-data['origin_cycle'])*data['f_num']//(data['clock_hz']*data['f_den'])


def _counts_at(plan, cycle):
    first = plan['bank_input']['first_tick_index']
    return {'tick_count': _ticks_at(plan, cycle),
            'emit_count': max(0, _ticks_at(plan, cycle-LATENCY)-first+1)}


def make_plan(data, *, arm_by_cycle, begin_cycle, end_cycle, seed_after_config=False):
    """Pure stationary smoke recipe. All supplied times are explicit PL cycles."""
    if not isinstance(data, dict):
        raise ValueError('bank input must be a JSON object')
    artifact = build_bank(data)
    # This small smoke entry is not a general trajectory executor.
    if data['previous_sequence'] != 0 or any(v != 0 for key in ('v', 'a') for v in data['initial_state'][key]):
        raise ValueError('smoke requires an initial stationary seed and previous_sequence=0')
    if any(v != 0 for row in data['normal_jerks']+data['brake_jerks'] for v in row):
        raise ValueError('smoke accepts stationary zero-jerk records only')
    d = dict(data, clock_hz=artifact.report['clock_hz'])
    dds = DDSConfig(d['f_num'], d['f_den'], d['clock_hz'])
    arm_by_cycle, begin_cycle, end_cycle = map(uint, (arm_by_cycle, begin_cycle, end_cycle))
    first = d['first_tick_index']
    count = len(d['normal_jerks'])+len(d['brake_jerks'])
    first_cycle = uint(d['origin_cycle']+dds.tick_cycle(first))
    last_brake_output = uint(d['origin_cycle']+dds.tick_cycle(first+count-1)+LATENCY)
    # These are explicit review targets. Verify uses ACTUAL captured cycles,
    # not a fiction that a host can hit one requested PL cycle exactly.
    if not d['validation_at_cycle'] <= arm_by_cycle < begin_cycle < first_cycle:
        raise ValueError('require validation_at <= arm_by < begin < first tick')
    if not last_brake_output <= end_cycle:
        raise ValueError('end target must include the complete brake tail plus 112 cycles')
    safe_until = min(uint(d['validation_at_cycle']+d['lease_cycles']),
                     uint(d['validation_at_cycle']+d['stop_timeout_cycles']))
    if end_cycle >= safe_until or arm_by_cycle >= uint(d['seed_cycle']+d['source_max_age_cycles']):
        raise ValueError('planned targets exceed the conservative lease/stop/seed horizon')
    q_raw = struct.unpack_from('<7q', artifact.config, 0x410)
    q16 = artifact.report['q16_samples'][0]
    wire = artifact.report['CAN_integer_samples'][0]
    clamped = any(round_even(Fraction(q, 65536)) != encoded for q, encoded in zip(q_raw, q16))
    clamped |= any(round_even(Fraction(q*s, 65536)) != encoded for q, s, encoded in
                   zip(q16, (1000,)*6+(1000000,), wire))
    used = count*SPEC['record_bytes']
    provenance = {name: sha((ROOT/name).read_bytes()) for name in
                  ('sw/desc_smoke.py', 'sw/pl_bank.py', 'sim/tick_reference.py',
                   'sim/player_reference.py', 'spec/regmap.yaml')}
    writes = [{'op': 'write32', 'address': address, 'value': value, 'wstrb': 15}
              for address, value in artifact.register_writes]
    config_writes = writes if not seed_after_config else [w for w in writes if not 0x400 <= w['address'] < 0x44c]
    seed_writes = [w for w in writes if 0x400 <= w['address'] < 0x44c]
    transactions = [
        {'op': 'require_readback', 'values': {'ID': 0x5254504c, 'CAPS': 1,
         'STATE': 0, 'FAULT_CODE': 0, 'STATUS': 0, 'NOMINAL_CLOCK_HZ': d['clock_hz']},
         'note': 'Matching DESC_ONLY platform and free banks; no hardware access by this tool'},
        {'op': 'config_shadow_writes' if seed_after_config else 'config_and_seed_shadow_writes', 'writes': config_writes},
        _command('CONFIG_COMMIT'),
        {'op': 'require_readback', 'values': {'CYCLE_ORIGIN': d['origin_cycle'], 'STATUS': 256},
         'note': 'Origin is the actual CONFIG_COMMIT origin. Abort/rebase on mismatch; do not send ARM'},
        {'op': 'bank_write', 'bank': 0, 'file': 'bank0.bin', 'file_offset': 256,
         'bank_byte_offset': 256, 'bytes': used, 'sha256': sha(artifact.image[256:256+used])},
        {'op': 'bank_write', 'bank': 0, 'file': 'bank0.bin', 'file_offset': 0,
         'bank_byte_offset': 0, 'bytes': 256, 'sha256': sha(artifact.image[:256])},
        {'op': 'device_memory_barrier_and_readback',
         'note': 'External host must drain writes and compare used payload/header bytes before COMMIT'},
        {'op': 'mailbox_writes', 'writes': _word_writes(REGS['BANK0_SEQUENCE']['offset'], d['bank_sequence'])},
        _command('COMMIT_BANK0'),
        {'op': 'wait_readback', 'values': {'STATUS': 258, 'REJECT_CODE': 0},
         'deadline_cycle': arm_by_cycle, 'note': 'CMD response is not READY; asynchronous validation must finish'},
        {'op': 'mailbox_writes', 'writes': _word_writes(REGS['LEASE_KICK_SEQUENCE']['offset'], 1)},
        _command('KICK'),
        {'op': 'mailbox_writes', 'writes': _word_writes(REGS['ARM_FIRST_TICK']['offset'], first)},
        _command('ARM'), _command('STOP'),
        {'op': 'snapshot_capture', 'name': 'begin', 'target_cycle': begin_cycle,
         'sequence': 'read live CYCLE; CMD SNAPSHOT/OKAY; read all capture words; read live CYCLE'},
        {'op': 'snapshot_capture', 'name': 'end', 'target_cycle': end_cycle,
         'sequence': 'read live CYCLE; CMD SNAPSHOT/OKAY; read all capture words; read live CYCLE'},
        _command('DISARM'), {'op': 'require_readback', 'values': {'STATE': 0, 'STATUS': 0}},
    ]
    if seed_after_config:
        transactions.insert(4, {'op': 'seed_shadow_writes_after_origin', 'writes': seed_writes})
    plan = {'schema': PLAN_SCHEMA, 'design_only': True, 'hardware_access': False,
            'backend': 'DESC_ONLY', 'physical_f_exec_hz': None, 'bank_input': d,
            'seed_after_config': seed_after_config,
            'schedule': {'arm_by_cycle': arm_by_cycle, 'begin_target_cycle': begin_cycle,
                         'end_target_cycle': end_cycle, 'first_tick_cycle': first_cycle,
                         'last_brake_output_cycle': last_brake_output,
                         'conservative_end_before_cycle': safe_until},
            'bank': {'file': 'bank0.bin', 'bytes': len(artifact.image),
                     'sha256': sha(artifact.image), 'config_crc32': artifact.report['config_crc32']},
            'stationary_output': {'wire_values': list(wire), 'clamp_per_emit': int(clamped)},
            'transactions': transactions, 'source_sha256': provenance,
            'limitations': ['Review-only recipe; no command has been sent',
              'Stationary invented/explicit seed data is not measured robot feedback',
              'Origin must be read after configuration; mismatch invalidates this bank',
              'Reported 112-cycle statistics are not independent per-event edge measurements',
              'No board, CAN, physical execution frequency or final STA acceptance is supplied']}
    before, after = _counts_at(plan, begin_cycle), _counts_at(plan, end_cycle)
    plan['expected_at_targets'] = {'begin': before, 'end': after,
        'tick_delta': after['tick_count']-before['tick_count'],
        'emit_delta': after['emit_count']-before['emit_count'], 'latency_cycles': LATENCY}
    plan['plan_id'] = sha(canonical(plan))
    return plan, artifact


def capture_template(plan):
    def ack(name):
        return dict(command=name, address=16, value=CMDS[name], wstrb=15, bresp=None,
                    cycle_before=None, cycle_after=None)
    snapshots = [dict(name=name, live_cycle_before=None, command_ack=ack('SNAPSHOT'),
                      words=register_words(), live_cycle_after=None) for name in ('begin', 'end')]
    return {'schema': CAPTURE_SCHEMA, 'plan_id': plan['plan_id'], 'source_kind': None,
            'capture_source': None, 'run_id': None,
            'control_receipts': [ack(name) for name in CONTROL_NAMES],
            'snapshots': snapshots,
            'cleanup': {'command_ack': ack('DISARM'), 'state_after': None, 'status_after': None},
            'note': 'Fill from ordered raw external transactions; null/template values never pass. '
                    'No other CMD/reset/reconfiguration may occur between the two snapshots.'}


def _assert(condition, message):
    if not condition:
        raise ValueError(message)


def _ack(receipt, name):
    _assert(isinstance(receipt, dict), f'{name}: missing command receipt')
    for key, expected in (('command', name), ('address', 16), ('value', CMDS[name]), ('wstrb', 15)):
        actual = receipt.get(key)
        _assert(type(actual) is type(expected) and actual == expected, f'{name}: invalid {key}')
    kind = receipt.get('kind', 'raw_axi')
    if kind == 'raw_axi':
        _assert(type(receipt.get('bresp')) is int and receipt['bresp'] == 0, f'{name}: invalid bresp')
    elif kind == 'mmio_readback':
        _assert('bresp' not in receipt, f'{name}: MMIO receipt must not fabricate AXI BRESP')
        _assert(receipt.get('write_completed') is True and isinstance(receipt.get('readback'), dict),
                f'{name}: MMIO completion/readback missing')
        rb = receipt['readback']
        _assert(type(rb.get('FAULT_CODE')) is int and rb['FAULT_CODE'] == 0 and
                type(rb.get('REJECT_CODE')) is int and rb['REJECT_CODE'] == 0, f'{name}: MMIO fault/reject readback')
        states = {'CONFIG_COMMIT': (0,256), 'COMMIT_BANK0': (0,258), 'KICK': (0,258),
                  'ARM': (1,259), 'STOP': (1,259), 'DISARM': (0,0)}
        allowed = [states[name]] if name in states else [(1,259), (4,259)]
        _assert(type(rb.get('STATE')) is int and type(rb.get('STATUS')) is int and
                (rb['STATE'], rb['STATUS']) in allowed, f'{name}: MMIO state/status readback')
    else:
        raise ValueError(f'{name}: unknown acknowledgement kind')
    before, after = uint(receipt.get('cycle_before')), uint(receipt.get('cycle_after'))
    _assert(before < after, f'{name}: invalid live cycle bracket')
    return before, after


def _decode_words(words):
    _assert(isinstance(words, dict), 'snapshot requires raw register words')
    required = register_words()
    _assert(set(words) == set(required), 'snapshot register word set differs from capture contract')
    checked = {key: uint(value, 32) for key, value in words.items()}
    decoded = {}
    for name in CAPTURE_NAMES:
        reg = REGS[name]
        decoded[name] = sum(checked[f'0x{offset:03x}'] << (8*(offset-reg['offset']))
            for offset in range(reg['offset'], reg['offset']+reg['width']//8, 4))
    return decoded


def _arm_source_window(data, before, after):
    """Conservative proof over the supplied ARM acknowledgement interval.

    A receipt is not the actual validator completion event. Never infer that
    event from COMMIT, whose response can precede asynchronous validation.
    """
    before, after = uint(before), uint(after)
    _assert(before < after, 'ARM: invalid cycle interval')
    error = uint(data['map_error_cycles'], 32)
    seed = uint(data['seed_cycle'])
    seed_deadline = uint(seed+data['source_max_age_cycles'])
    produced_latest = uint(data['produced_cycle']+error)
    source_deadline = uint(data['observation_cycle']-error+data['source_max_age_cycles'])
    deadline = uint(data['normal_deadline_cycle'])
    _assert(seed <= before, 'ARM: seed timestamp is future within actual receipt interval')
    _assert(after < seed_deadline, 'ARM: seed is stale within actual receipt interval')
    _assert(produced_latest <= before, 'ARM: mapped source production is future within actual receipt interval')
    _assert(uint(after+error) < deadline <= source_deadline,
            'ARM: mapped source/deadline is expired within actual receipt interval')
    return {'arm_cycle_bracket': [before, after], 'seed_cycle': seed,
            'seed_expiry_cycle': seed_deadline, 'produced_latest_cycle': produced_latest,
            'normal_deadline_cycle': deadline, 'source_age_expiry_cycle': source_deadline,
            'validation_completion_timestamp': None,
            'scope': 'Necessary conservative ARM/source ordering only; COMMIT ack is not READY timing'}


def _descriptor(plan, emitted):
    if emitted == 0:
        return bytes(96)
    d = plan['bank_input'];tick = d['first_tick_index']+emitted-1
    cycle = d['origin_cycle']+DDSConfig(d['f_num'], d['f_den'], d['clock_hz']).tick_cycle(tick)
    normal, total = len(d['normal_jerks']), len(d['normal_jerks'])+len(d['brake_jerks'])
    flags = (0 if emitted <= normal else 1 if emitted <= total else 2) | (plan['stationary_output']['clamp_per_emit'] << 2)
    result = bytearray(struct.pack('<QQQII', emitted, cycle, tick, d['config_epoch'], flags))
    wire = plan['stationary_output']['wire_values']
    for i, ident in enumerate((0x155, 0x156, 0x157, 0x159)):
        payload = (struct.pack('>ii', *wire[i*2:i*2+2]) if i < 3 else
                   struct.pack('>iHBB', wire[6], d['gripper_force_mn'], 1, 0))
        result.extend(struct.pack('<HBB8sI', ident, 8, 0, payload, 0))
    return bytes(result)


def verify_capture(plan, capture):
    """Validate an ordered capture contract; never authenticates external data."""
    _assert(isinstance(plan, dict) and plan.get('schema') == PLAN_SCHEMA, 'plan schema mismatch')
    schedule = plan['schedule']
    rebuilt, _ = make_plan(plan['bank_input'], arm_by_cycle=schedule['arm_by_cycle'],
                          begin_cycle=schedule['begin_target_cycle'], end_cycle=schedule['end_target_cycle'],
                          seed_after_config=plan.get('seed_after_config', False))
    _assert(plan == rebuilt, 'plan/source hash or generated contract changed; regenerate a new plan')
    _assert(isinstance(capture, dict) and capture.get('schema') == CAPTURE_SCHEMA, 'capture schema mismatch')
    _assert(capture.get('plan_id') == plan['plan_id'], 'capture belongs to a different plan')
    _assert(capture.get('source_kind') in ('synthetic', 'external_capture'), 'source_kind must be explicit')
    for key in ('capture_source', 'run_id'):
        _assert(isinstance(capture.get(key), str) and bool(capture[key].strip()), f'{key} required')
    controls = capture.get('control_receipts')
    _assert(isinstance(controls, list) and len(controls) == len(CONTROL_NAMES), 'exact ordered control receipts required')
    last = -1;bounds = {}
    for receipt, name in zip(controls, CONTROL_NAMES):
        before, after = _ack(receipt, name)
        _assert(before > last, 'control receipt order/overlap invalid')
        bounds[name] = (before, after);last = after
    d = plan['bank_input']
    _assert(bounds['CONFIG_COMMIT'][0] < d['origin_cycle'] < bounds['CONFIG_COMMIT'][1],
            'configuration origin is not inside its receipt bracket')
    _assert(bounds['ARM'][1] <= schedule['arm_by_cycle'], 'ARM receipt misses explicit arm-by bound')
    _assert(bounds['STOP'][1] < schedule['first_tick_cycle'], 'STOP must precede first tick')
    source_window = _arm_source_window(d, *bounds['ARM'])
    snapshots = capture.get('snapshots')
    _assert(isinstance(snapshots, list) and len(snapshots) == 2, 'two fresh snapshots required')
    results = []
    for item, name in zip(snapshots, ('begin', 'end')):
        _assert(isinstance(item, dict) and item.get('name') == name, 'snapshot order/name mismatch')
        before, after = uint(item.get('live_cycle_before')), uint(item.get('live_cycle_after'))
        cmd_before, cmd_after = _ack(item.get('command_ack'), 'SNAPSHOT')
        _assert(last < before <= cmd_before < cmd_after <= after, 'snapshot command/read order lacks independent live bracket')
        values = _decode_words(item.get('words'))
        c = values['SNAP_CYCLE']
        _assert(cmd_before < c < cmd_after, 'SNAP_CYCLE is stale or outside acknowledged SNAPSHOT bracket')
        expected_identity = dict(ID=0x5254504c, VERSION=0x10000, CAPS=1, STATE=1 if name == 'begin' else 4,
            STATUS=259, FAULT_CODE=0, REJECT_CODE=0, SESSION_NONCE=d['session_nonce'],
            CONFIG_EPOCH=d['config_epoch'], BACKEND_MODE=0, RATE_NUM=d['f_num'], RATE_DEN=d['f_den'],
            CYCLE_ORIGIN=d['origin_cycle'], NOMINAL_CLOCK_HZ=d['clock_hz'])
        for key, value in expected_identity.items():
            _assert(values[key] == value, f'{name}: {key} differs from stationary DESC_ONLY contract')
        if name == 'begin':
            _assert(c < schedule['first_tick_cycle'], 'begin snapshot must precede first output tick')
        else:
            _assert(c >= schedule['last_brake_output_cycle'], 'end snapshot is before complete brake output')
        _assert(c < bounds['KICK'][0]+d['lease_cycles'], 'snapshot exceeds conservative actual KICK lease')
        _assert(c < bounds['STOP'][0]+d['stop_timeout_cycles'], 'snapshot exceeds conservative actual STOP timeout')
        counts = _counts_at(plan, c);emitted = counts['emit_count']
        expected = dict(SNAP_TICK_COUNT=counts['tick_count'], SNAP_EMIT_COUNT=emitted,
            SNAP_REJECT_COUNT=0, SNAP_CLAMP_COUNT=emitted*plan['stationary_output']['clamp_per_emit'],
            SNAP_STARVE_COUNT=0, SNAP_BACKPRESSURE_COUNT=0, SNAP_LATENCY_MIN=LATENCY if emitted else 0,
            SNAP_LATENCY_MAX=LATENCY if emitted else 0, SNAP_LAST_SEQUENCE=emitted, SNAP_FAULT_CYCLE=0)
        for key, value in expected.items():
            _assert(values[key] == value, f'{name}: {key} mismatch (expected {value}, got {values[key]})')
        raw_desc = struct.pack('<24I', *(values[f'DESC_SNAPSHOT_WORD{i}'] for i in range(24)))
        _assert(raw_desc == _descriptor(plan, emitted), f'{name}: descriptor bytes do not match the bank/tick/session')
        results.append(dict(name=name, snap_cycle=c, live_cycle_bracket=[before, after],
                            command_cycle_bracket=[cmd_before, cmd_after], **counts))
        last = after
    cleanup = capture.get('cleanup')
    _assert(isinstance(cleanup, dict), 'DISARM cleanup receipt required')
    cb, ca = _ack(cleanup.get('command_ack'), 'DISARM')
    _assert(cb > last and type(cleanup.get('state_after')) is int and cleanup['state_after'] == 0 and
            type(cleanup.get('status_after')) is int and cleanup['status_after'] == 0,
            'cleanup must follow snapshots and read back DISARMED/free/unconfigured')
    delta_cycles = results[1]['snap_cycle']-results[0]['snap_cycle']
    tick_delta = results[1]['tick_count']-results[0]['tick_count']
    emit_delta = results[1]['emit_count']-results[0]['emit_count']
    rate = Fraction(tick_delta*d['clock_hz'], delta_cycles)
    receipts = controls+[item['command_ack'] for item in snapshots]+[cleanup['command_ack']]
    axi_observed = all(r.get('kind', 'raw_axi') == 'raw_axi' for r in receipts)
    return {'schema': RESULT_SCHEMA, 'status': 'offline_capture_consistent',
            'source_kind': capture['source_kind'], 'capture_source': capture['capture_source'],
            'run_id': capture['run_id'], 'plan_id': plan['plan_id'], 'hardware_access': False,
            'hardware_independently_authenticated': False, 'board_acceptance': False,
            'axi_bresp_observed': axi_observed,
            'physical_f_exec_hz': None, 'fresh_latch_evidence_consistent': True,
            'source_window': source_window, 'snapshots': results,
            'interval': {'basis': 'actual SNAP_CYCLE/COUNT from two acknowledged fresh latches',
                'cycles': delta_cycles, 'tick_delta': tick_delta, 'emit_delta': emit_delta,
                'nominal_counter_tick_rate_hz': {'numerator': rate.numerator, 'denominator': rate.denominator}},
            'latency_check': {'reported_min_cycles': LATENCY, 'reported_max_cycles': LATENCY,
                'independent_measurement': False,
                'scope': 'snapshot-reported extrema; no independent per-event edge timing'},
            'cleanup_ack_cycle_bracket': [cb, ca],
            'limitations': ['External receipts/dumps are not independently authenticated',
                'No oscillator frequency, physical CAN rate, robot motion, final STA or board acceptance is established']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    subs = parser.add_subparsers(dest='command', required=True)
    plan = subs.add_parser('plan', help='write a fresh offline recipe; never executes it')
    plan.add_argument('--input', type=Path, required=True, help='explicit sw/pl_bank.py input JSON including rational rate/time fields')
    plan.add_argument('--arm-by-cycle', type=int, required=True)
    plan.add_argument('--begin-cycle', type=int, required=True, help='review target, not a promise of host-cycle accuracy')
    plan.add_argument('--end-cycle', type=int, required=True)
    plan.add_argument('--output-dir', type=Path, required=True)
    verify = subs.add_parser('verify', help='validate a raw external/synthetic capture, without hardware access')
    verify.add_argument('--plan', type=Path, required=True)
    verify.add_argument('--capture', type=Path, required=True)
    verify.add_argument('--output', type=Path, required=True, help='new result JSON; existing files are not overwritten')
    args = parser.parse_args(argv)
    try:
        if args.command == 'plan':
            data = read_json(args.input)
            result, artifact = make_plan(data, arm_by_cycle=args.arm_by_cycle,
                                        begin_cycle=args.begin_cycle, end_cycle=args.end_cycle)
            args.output_dir.mkdir(parents=True, exist_ok=False)
            (args.output_dir/'plan.json').write_bytes(canonical(result))
            (args.output_dir/'bank0.bin').write_bytes(artifact.image)
            (args.output_dir/'capture_template.json').write_bytes(canonical(capture_template(result)))
            print(json.dumps({'design_only': True, 'hardware_access': False, 'plan_id': result['plan_id'],
                              'output_dir': str(args.output_dir), 'expected_at_targets': result['expected_at_targets']}, indent=2))
        else:
            result = verify_capture(read_json(args.plan), read_json(args.capture))
            result['input_sha256'] = {'plan': sha(args.plan.read_bytes()), 'capture': sha(args.capture.read_bytes())}
            with args.output.open('xb') as handle:
                handle.write(canonical(result))
            print(json.dumps(result, indent=2))
    except (ValueError, KeyError, TypeError, OSError, struct.error) as exc:
        parser.exit(2, f'DESC_ONLY offline smoke: {exc}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
