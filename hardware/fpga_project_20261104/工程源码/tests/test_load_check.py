"""Synthetic files only: check mode must reject unsafe/mismatched artifacts."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


class LoadCheckTests(unittest.TestCase):
    def run_check(self, *, clock=99_999_001, corrupt=False):
        with tempfile.TemporaryDirectory(prefix='robot_rt_loader_unit_') as tmp:
            path=Path(tmp);manager=path/'manager';manager.mkdir()
            (manager/'state').write_text('synthetic-test-only\n')
            addresses=dict(schema='robot_rt.addresses.v1',board_part='xilinx.com:kr260_som:part0:1.1',
                part='xck26-sfvc784-2LV-c',clock_hz=clock,backend='DESC_ONLY',
                cfg_base=0xA0000000,cfg_bytes=4096,chunk_base=0xA0020000,
                chunk_bytes=131072,expected_id=0x5254504c)
            (path/'addresses.json').write_text(json.dumps(addresses))
            for name in ('robot_rt.bit','robot_rt.bit.bin','robot_rt.xsa'):
                (path/name).write_bytes(b'SYNTHETIC UNIT TEST; NOT A BITSTREAM')
            files={name:hashlib.sha256((path/name).read_bytes()).hexdigest()
                   for name in ('robot_rt.bit','robot_rt.bit.bin','robot_rt.xsa','addresses.json')}
            (path/'artifacts.sha256.json').write_text(json.dumps(dict(
                schema='robot_rt.build.v1',timing_passed=True,files=files)))
            if corrupt:(path/'robot_rt.bit.bin').write_bytes(b'corrupted')
            before={p.name:p.read_bytes() for p in path.rglob('*') if p.is_file()}
            run=subprocess.run(['bash',str(ROOT/'sw/load.sh'),'--check','--build',str(path),
                '--manager',str(manager)],capture_output=True,text=True,timeout=5)
            after={p.name:p.read_bytes() for p in path.rglob('*') if p.is_file()}
            self.assertEqual(before,after,'check mode must not write files or manager')
            return run

    def test_actual_nominal_and_reference_profiles_pass_read_only(self):
        for clock in (99_999_001,100_000_000):
            with self.subTest(clock=clock):
                run=self.run_check(clock=clock)
                self.assertEqual(run.returncode,0,run.stderr)
                self.assertIn('synthetic-test-only',run.stdout)

    def test_invalid_clock_rejected_after_hash_verification(self):
        for clock in (0,True,2**31):
            with self.subTest(clock=clock):
                run=self.run_check(clock=clock)
                self.assertNotEqual(run.returncode,0)
                self.assertIn('Invalid compiled nominal',run.stderr)

    def test_corrupted_artifact_rejected(self):
        run=self.run_check(corrupt=True)
        self.assertNotEqual(run.returncode,0)
        self.assertIn('Artifact hash mismatch',run.stderr)


if __name__=='__main__':unittest.main()
