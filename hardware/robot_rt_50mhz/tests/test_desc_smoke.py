"""Synthetic receipts only. No board, ACT, /dev/mem, CAN or Vivado access."""
import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sw.desc_smoke import _arm_source_window, capture_template, make_plan, verify_capture
from sw.pl_bank import build_bank, example


def synthetic_capture(plan):
    """Independent fixed-layout oracle for the 275/2 Hz stationary fixture.

    Direct 32bit register addresses and CAN bytes deliberately do not use the
    production decoder/descriptor helper. Every time/receipt here is invented.
    """
    def ack(name, value, before, after):
        return dict(command=name, address=16, value=value, wstrb=15, bresp=0,
                    cycle_before=before, cycle_after=after)
    receipts = [ack(name, value, before, before+100) for name, value, before in
                (('CONFIG_COMMIT', 1, 900), ('COMMIT_BANK0', 32, 1200),
                 ('KICK', 256, 1400), ('ARM', 2, 1600), ('STOP', 4, 1800))]
    # The first configuration response must follow origin 1000 strictly.
    receipts[0]['cycle_after'] = 1100
    snapshots = []
    for name, cycle in (('begin', 2200), ('end', 9_000_000)):
        words = {}
        def put(address, value, width=32):
            for offset in range(0, width//8, 4):
                words[f'0x{address+offset:03x}'] = (value >> (8*offset)) & 0xffffffff
        for address, value in ((0, 0x5254504c), (4, 0x10000), (8, 1), (12, 1 if name == 'begin' else 4),
                               (0x14, 0x103), (0x18, 0), (0x1c, 0), (0x38, 1), (0x3c, 0)):
            put(address, value)
        for address, value in ((0x30, 1), (0x40, 275), (0x48, 2), (0xa0, 1000), (0xa8, 100_000_000)):
            put(address, value, 64)
        ticks = (cycle-1000)*275//200_000_000
        emit = max(0, (cycle-112-1000)*275//200_000_000-7)
        values = [cycle, ticks, emit, 0, 0, 0, 0, 112 if emit else 0, 112 if emit else 0, emit, 0]
        for index, value in enumerate(values):
            put(0x500+index*8, value, 64)
        raw = bytes(96)
        if emit:
            tick = 7+emit
            tick_cycle = 1000+(tick*200_000_000+274)//275
            raw = struct.pack('<QQQII', emit, tick_cycle, tick, 1, 2)
            for ident, payload in ((0x155, bytes(8)), (0x156, bytes(8)), (0x157, bytes(8)),
                                   (0x159, struct.pack('>iHBB', 50003, 1000, 1, 0))):
                raw += struct.pack('<HBB8sI', ident, 8, 0, payload, 0)
        for index, (value,) in enumerate(struct.iter_unpack('<I', raw)):
            put(0x600+index*4, value)
        snapshots.append(dict(name=name, live_cycle_before=cycle-6,
            command_ack=ack('SNAPSHOT', 16, cycle-4, cycle+4), words=words, live_cycle_after=cycle+6))
    return dict(schema='robot_rt.desc_smoke_capture.v1', plan_id=plan['plan_id'], source_kind='synthetic',
                capture_source='Invented independent register fixture; no hardware', run_id='synthetic-unit-only',
                control_receipts=receipts, snapshots=snapshots,
                cleanup=dict(command_ack=ack('DISARM', 128, 9_000_100, 9_000_120), state_after=0, status_after=0))


class DescSmokeTests(unittest.TestCase):
    def setUp(self):
        self.data = example(275, 2, 1000, 8)
        self.plan, self.artifact = make_plan(self.data, arm_by_cycle=2000, begin_cycle=2200, end_cycle=9_000_000)

    def test_plan_reuses_actual_packer_crc_and_order_but_contains_no_executor(self):
        packed = build_bank(self.data)
        self.assertEqual(self.artifact.image, packed.image)
        self.assertEqual(self.plan['bank']['sha256'], hashlib.sha256(packed.image).hexdigest())
        self.assertEqual(self.plan['bank']['config_crc32'], packed.report['config_crc32'])
        commands = [t['name'] for t in self.plan['transactions'] if t['op'] == 'CMD_review_only']
        self.assertEqual(commands, ['CONFIG_COMMIT', 'COMMIT_BANK0', 'KICK', 'ARM', 'STOP', 'DISARM'])
        writes = [t for t in self.plan['transactions'] if t['op'] == 'bank_write']
        self.assertEqual([t['bank_byte_offset'] for t in writes], [256, 0])
        cfg = self.plan['transactions'][1]['writes']
        self.assertTrue(all(t['address'] != 16 for t in cfg))
        self.assertEqual([(t['address'], t['value']) for t in cfg], list(packed.register_writes))
        self.assertFalse(self.plan['hardware_access'])
        self.assertIsNone(self.plan['physical_f_exec_hz'])

    def test_fresh_actual_counter_interval_and_fixed_reported_latency(self):
        capture = synthetic_capture(self.plan)
        before = copy.deepcopy(capture)
        result = verify_capture(self.plan, capture)
        self.assertEqual(capture, before)
        self.assertEqual(result['interval']['cycles'], 8_997_800)
        self.assertEqual(result['interval']['tick_delta'], 12)
        self.assertEqual(result['interval']['emit_delta'], 5)
        self.assertEqual(result['snapshots'][1]['snap_cycle'], 9_000_000)
        self.assertEqual(result['latency_check']['reported_min_cycles'], 112)
        self.assertFalse(result['board_acceptance'])
        self.assertFalse(result['hardware_independently_authenticated'])
        self.assertIsNone(result['physical_f_exec_hz'])
        for item in capture['snapshots']:
            item['live_cycle_before'] = item['command_ack']['cycle_before']
            item['live_cycle_after'] = item['command_ack']['cycle_after']
        self.assertEqual(verify_capture(self.plan, capture)['interval']['tick_delta'], 12)
        external = copy.deepcopy(capture);external['source_kind'] = 'external_capture'
        self.assertFalse(verify_capture(self.plan, external)['board_acceptance'])

    def test_old_snapshot_missing_receipt_or_unordered_bracket_cannot_pass(self):
        for change in ('stale', 'repeat', 'missing_ack', 'bad_bresp', 'overlap'):
            capture = synthetic_capture(self.plan)
            if change == 'stale':capture['snapshots'][1]['words']['0x500'] = 2200
            if change == 'repeat':capture['snapshots'][1] = copy.deepcopy(capture['snapshots'][0])
            if change == 'missing_ack':capture['snapshots'][0].pop('command_ack')
            if change == 'bad_bresp':capture['snapshots'][1]['command_ack']['bresp'] = 2
            if change == 'overlap':capture['snapshots'][1]['live_cycle_before'] = 2205
            with self.subTest(change=change), self.assertRaises(ValueError):verify_capture(self.plan, capture)

    def test_counts_session_descriptor_latency_fault_and_cleanup_reject(self):
        for address in ('0x508', '0x510', '0x518', '0x528', '0x530', '0x538', '0x540',
                        '0x548', '0x550', '0x030', '0x038', '0x040', '0x0a0', '0x0a8',
                        '0x014', '0x018', '0x01c', '0x600', '0x610', '0x65c'):
            capture = synthetic_capture(self.plan);capture['snapshots'][1]['words'][address] ^= 1
            with self.subTest(address=address), self.assertRaises(ValueError):verify_capture(self.plan, capture)
        capture = synthetic_capture(self.plan);capture['cleanup']['status_after'] = 256
        with self.assertRaises(ValueError):verify_capture(self.plan, capture)

    def test_raw_words_missing_bool_noninteger_overflow_and_template_reject(self):
        for value in (None, True, 0.0, float('nan'), -1, 1 << 32):
            capture = synthetic_capture(self.plan);capture['snapshots'][0]['words']['0x000'] = value
            with self.subTest(value=value), self.assertRaises(ValueError):verify_capture(self.plan, capture)
        capture = synthetic_capture(self.plan);capture['snapshots'][0]['words'].pop('0x500')
        with self.assertRaises(ValueError):verify_capture(self.plan, capture)
        with self.assertRaises(ValueError):verify_capture(self.plan, capture_template(self.plan))

    def test_explicit_schedule_stationary_scope_and_rate_have_no_fallback(self):
        for kwargs in (dict(arm_by_cycle=True, begin_cycle=2200, end_cycle=9_000_000),
                       dict(arm_by_cycle=2000, begin_cycle=1, end_cycle=9_000_000),
                       dict(arm_by_cycle=2000, begin_cycle=2200, end_cycle=5000),
                       dict(arm_by_cycle=2000, begin_cycle=2200, end_cycle=201_001_000)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):make_plan(self.data, **kwargs)
        for key in ('normal_jerks', 'brake_jerks'):
            data = copy.deepcopy(self.data);data[key][0][0] = 1
            with self.subTest(key=key), self.assertRaises(ValueError):make_plan(data, arm_by_cycle=2000, begin_cycle=2200, end_cycle=9_000_000)
        with self.assertRaises(ValueError):make_plan([], arm_by_cycle=2000, begin_cycle=2200, end_cycle=9_000_000)
        data = copy.deepcopy(self.data);data.pop('f_num')
        with self.assertRaises(KeyError):make_plan(data, arm_by_cycle=2000, begin_cycle=2200, end_cycle=9_000_000)
        altered = copy.deepcopy(self.plan);altered['source_sha256']['sw/pl_bank.py'] = '0'*64
        with self.assertRaisesRegex(ValueError, 'plan/source'):verify_capture(altered, synthetic_capture(altered))

    def test_actual_arm_cannot_precede_seed_or_mapped_production(self):
        # Both cases were accepted before the source-order fix even though a
        # real RTL ARM could not have succeeded. The old fixture is preserved.
        for change in ('future_seed', 'future_source', 'future_source_with_error'):
            data = copy.deepcopy(self.data)
            if change == 'future_seed':
                data.update(seed_cycle=3000, validation_at_cycle=3000)
            elif change == 'future_source':
                data.update(observation_cycle=3000, produced_cycle=3000, validation_at_cycle=3000)
            else:
                data.update(observation_cycle=3000, produced_cycle=3002, validation_at_cycle=3003,
                            map_error_cycles=1, max_map_error_cycles=1)
            plan, _ = make_plan(data, arm_by_cycle=4000, begin_cycle=5000, end_cycle=9_000_000)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'ARM: .*future'):
                verify_capture(plan, synthetic_capture(plan))
        capture = synthetic_capture(self.plan)
        # Explicitly retain the post-commit raw origin mismatch negative case.
        capture['snapshots'][0]['words']['0x0a0'] += 1
        with self.assertRaisesRegex(ValueError, 'CYCLE_ORIGIN'):
            verify_capture(self.plan, capture)

    def test_arm_source_window_uses_both_receipt_edges_and_exact_expiry(self):
        data = copy.deepcopy(self.data)
        # Equality at the conservative earlier edge is allowed for a seed or
        # already-produced source; equality at an expiry edge is not.
        data.update(seed_cycle=1600, observation_cycle=1597, produced_cycle=1599,
                    map_error_cycles=1, source_max_age_cycles=200, normal_deadline_cycle=1750)
        proof = _arm_source_window(data, 1600, 1700)
        self.assertEqual(proof['produced_latest_cycle'], 1600)
        self.assertEqual(proof['seed_expiry_cycle'], 1800)
        self.assertIsNone(proof['validation_completion_timestamp'])
        for change in ('future_seed', 'future_source', 'stale_seed', 'expired_source', 'u64_wrap'):
            bad = copy.deepcopy(data)
            if change == 'future_seed':bad['seed_cycle'] = 1601
            if change == 'future_source':bad['produced_cycle'] = 1600
            if change == 'stale_seed':bad['source_max_age_cycles'] = 100
            if change == 'expired_source':bad['normal_deadline_cycle'] = 1701
            if change == 'u64_wrap':bad.update(seed_cycle=(1 << 64)-100, source_max_age_cycles=200)
            with self.subTest(change=change), self.assertRaises(ValueError):
                _arm_source_window(bad, 1600, 1700)

    def test_integer_and_ps_nominal_profiles_use_explicit_dds_not_wall_time(self):
        for clock in (99_999_001, 100_000_000):
            data = example(clock, 256, 1000, 64, clock_hz=clock)
            plan, _ = make_plan(data, arm_by_cycle=2000, begin_cycle=2200, end_cycle=19000)
            self.assertEqual(plan['schedule']['first_tick_cycle'], 1000+64*256)
            self.assertEqual(plan['expected_at_targets']['end'], {'tick_count': 70, 'emit_count': 6})

    def test_cli_help_roundtrip_fresh_outputs_and_no_hardware_imports(self):
        with tempfile.TemporaryDirectory(prefix='desc_smoke_unit_') as tmp:
            path = Path(tmp);marker = path/'hardware_imported'
            for name in ('piper_sdk', 'can', 'lerobot'):
                (path/f'{name}.py').write_text(f"from pathlib import Path\nPath({str(marker)!r}).write_text('BAD')\nraise RuntimeError('hardware import')\n")
            import os
            env = dict(os.environ, PYTHONPATH=str(path), PYTHONDONTWRITEBYTECODE='1')
            def run(*args):
                return subprocess.run([sys.executable, str(ROOT/'sw/desc_smoke.py'), *map(str, args)],
                                      cwd=path, env=env, capture_output=True, text=True, timeout=10)
            self.assertEqual(run('--help').returncode, 0)
            source = path/'input.json';source.write_text(json.dumps(self.data))
            command = ('plan', '--input', source, '--arm-by-cycle', 2000, '--begin-cycle', 2200,
                       '--end-cycle', 9_000_000, '--output-dir', path/'plan')
            generated = run(*command);self.assertEqual(generated.returncode, 0, generated.stderr)
            self.assertNotEqual(run(*command).returncode, 0)
            plan = json.loads((path/'plan/plan.json').read_text())
            capture = path/'capture.json';capture.write_text(json.dumps(synthetic_capture(plan)))
            result = path/'result.json'
            args = ('verify', '--plan', path/'plan/plan.json', '--capture', capture, '--output', result)
            verified = run(*args);self.assertEqual(verified.returncode, 0, verified.stderr)
            old = result.read_bytes()
            self.assertNotEqual(run(*args).returncode, 0)
            self.assertEqual(result.read_bytes(), old)
            bad = path/'nonfinite.json';bad.write_text('{"bad": NaN}')
            self.assertNotEqual(run('plan', '--input', bad, '--arm-by-cycle', 1, '--begin-cycle', 2,
                                    '--end-cycle', 3, '--output-dir', path/'bad').returncode, 0)
            self.assertFalse((path/'bad').exists())
            self.assertFalse(marker.exists())


if __name__ == '__main__':
    unittest.main()
