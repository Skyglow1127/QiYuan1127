#!/usr/bin/env python3
"""Check the stage-C address/packet contract, without RTL or hardware.

Example: /usr/bin/python3 measure/check_pl_spec.py --output docs/stage_c_verification/layout.json
This validates layouts and arithmetic design constants, not synthesis/timing.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import zlib

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def disjoint(fields, total, size_key):
    seen = set()
    for field in fields:
        size = field[size_key] if size_key == 'bytes' else field[size_key] // 8
        require(size > 0, f"empty field: {field['name']}")
        addresses = set(range(field['offset'], field['offset']+size))
        require(min(addresses) >= 0 and max(addresses) < total, f"outside window: {field['name']}")
        require(not (seen & addresses), f"overlapping field: {field['name']}")
        seen |= addresses
    return seen


def check(path):
    spec = json.loads(path.read_text(encoding='utf-8'))  # JSON is YAML 1.2 subset.
    registers = spec['registers']
    require(len({r['name'] for r in registers}) == len(registers), 'duplicate register')
    disjoint(registers, spec['register_window_bytes'], 'width')
    for reg in registers:
        require(reg['offset'] % 4 == 0 and reg['width'] in (32, 64), f"invalid AXI word: {reg['name']}")
        require(reg['access'] in ('RO', 'WO', 'RW_SHADOW'), 'unknown access')
    require(len(disjoint(spec['bank_header'], 256, 'bytes')) == 256, 'header must describe every byte')
    require(len(disjoint(spec['record_layout'], 256, 'bytes')) == 256, 'record must describe every byte')
    require(spec['header_bytes']+spec['max_records_per_bank']*spec['record_bytes'] == spec['bank_bytes'], 'bank capacity mismatch')
    require(spec['bank_count']*spec['bank_bytes'] == spec['bank_window_bytes'] == 131072, 'BRAM window mismatch')
    record = struct.Struct('<QQQII'+'q'*28)
    require(record.size == spec['record_bytes'], 'packet format mismatch')
    sample = record.pack(0, 5000000, 1, 1, 0, *([0]*28))
    require(record.unpack(sample)[1] == 5000000, 'packet roundtrip failed')
    require(zlib.crc32(b'123456789') == 0xcbf43926, 'CRC32 oracle differs')
    require(32+4*struct.calcsize('<HBB8sI') == spec['descriptor_bytes'] == 96, 'descriptor width mismatch')
    by_name = {r['name']: r for r in registers}
    for name in ('RATE_NUM', 'RATE_DEN', 'CYCLE_ORIGIN'):
        require(by_name[name]['width'] == 64, f'{name} must be 64 bit')
    require(spec['min_tick_interval_cycles'] >= spec['latency_cycles']+64, 'no next-record prefetch budget')
    require(spec['latency_cycles'] <= 120, 'latency budget exceeds requirement')
    require(spec['f_exec_measured_hz'] is None and spec['physical_base'] is None, 'invented physical setting')
    canonical_config = bytes(0x74-0x30 + 0x2c0-0x100 + 8)
    require(len(canonical_config) == 524, 'canonical config byte count')
    return {
        'design_only': True, 'rtl_simulated': False, 'vivado_run': False, 'board_tested': False,
        'physical_f_exec_hz': None, 'layout_pass': True,
        'register_count': len(registers), 'record_bytes': len(sample),
        'bank_bytes': spec['bank_bytes'], 'max_records_per_bank': spec['max_records_per_bank'],
        'descriptor_bytes': spec['descriptor_bytes'],
        'latency_cycles_design_budget': spec['latency_cycles'],
        'zero_static_config_crc32_test_vector': f'{zlib.crc32(canonical_config):08x}',
        'zero_static_config_is_valid': False,
        'crc32_standard_test_vector': 'cbf43926',
        'sources': {str(path): hashlib.sha256(path.read_bytes()).hexdigest()},
        'scope': 'Address nonoverlap, complete packet layouts, capacity and design budget consistency; not bank/FSM RTL behavior or measured timing.'
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--regmap', type=Path, default=ROOT/'spec/regmap.yaml')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        result = check(args.regmap.resolve())
        output = json.dumps(result, ensure_ascii=False, indent=2)+'\n'
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(output)
        print(output, end='')
    except (ValueError, KeyError, OSError) as exc:
        parser.exit(2, f'PL contract: {exc}\n')


if __name__ == '__main__':
    main()
