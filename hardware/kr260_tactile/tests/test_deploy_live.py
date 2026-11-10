import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deploy_live import classify_board, parse_listapps, require_application, summarize_samples
import deploy_live

HEADER = 'Accelerator Accel_type Base Pid Base_type #slots(RPU+PL+AIE) slot->handle\n'
STOCK = 'k26-starter-kits XRT_FLAT k26-starter-kits id_ok XRT_FLAT (0+0+0) 0->0,\n'
TARGET = 'kr260-fsr XRT_FLAT kr260-fsr id_ok XRT_FLAT (0+0+0) -1\n'


class DeployParsingTests(unittest.TestCase):
    def test_installed_2024_table_stock_slot0(self):
        rows = parse_listapps(HEADER + STOCK + TARGET)
        self.assertEqual(require_application(rows, 'k26-starter-kits')['active_slots'], [0])
        rows = parse_listapps(HEADER + STOCK.replace('0->0,', '0->5,') + TARGET)
        self.assertEqual(require_application(rows, 'k26-starter-kits')['active_handles'], [5])

    def test_legacy_documented_layout(self):
        output = ('Accelerator Accel_type Base Base_type #slots(PL+AIE) Active_slot\n'
                  'k26-starter-kits XRT_FLAT k26-starter-kits XRT_FLAT (0+0) 0,\n'
                  'kr260-fsr XRT_FLAT kr260-fsr XRT_FLAT (0+0) -1\n')
        require_application(parse_listapps(output), 'k26-starter-kits')

    def test_unknown_or_ambiguous_output_rejected(self):
        for output in (HEADER.replace('Pid', 'Unknown') + STOCK + TARGET,
                       HEADER + STOCK + TARGET + TARGET,
                       HEADER + STOCK + TARGET + 'ERROR: bad\n',
                       HEADER + STOCK.replace('0->0,', '0,') + TARGET):
            with self.subTest(output=output), self.assertRaises(ValueError):
                parse_listapps(output)

    def test_no_target_multiple_active_or_wrong_slot_rejected(self):
        for output in (HEADER + STOCK,
                       HEADER + STOCK + TARGET.replace('-1', '1->1,'),
                       HEADER + STOCK.replace('0->0,', '1->1,') + TARGET):
            with self.subTest(output=output), self.assertRaises(ValueError):
                require_application(parse_listapps(output), 'k26-starter-kits')

    def test_absent_pid_metadata_allowed_but_mismatch_rejected(self):
        require_application(parse_listapps(HEADER + STOCK + TARGET.replace('id_ok', 'no_id')),
                            'k26-starter-kits')
        for value in ('id_err', 'unknown'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                require_application(parse_listapps(HEADER + STOCK + TARGET.replace('id_ok', value)),
                                    'k26-starter-kits')

    def test_only_matching_unique_stock_or_target_allowed(self):
        state = {'model':'ZynqMP KR260 revB', 'manager_state':'operating',
                 'firmware':'k26-starter-kits.bit.bin',
                 'overlays':[{'name':'k26-starter-kits_image_1', 'status':'applied'}]}
        self.assertEqual(classify_board(state), 'k26-starter-kits')
        for bad in ({**state, 'firmware':'other.bit.bin'},
                    {**state, 'overlays':state['overlays'] * 2},
                    {**state, 'manager_state':'unknown'}):
            with self.assertRaises(ValueError):
                classify_board(bad)

    def test_real_samples_constant_and_change_reported_separately(self):
        rows = [{'seq':i+1, 't_s':i / 100, 'do1':0, 'do2':1, 'source':'real',
                 'chip':'/dev/gpiochip7', 'offsets':[0,1]} for i in range(100)]
        pack = lambda: '\n'.join(json.dumps(row) for row in rows)
        summary = summarize_samples(pack(), '/dev/gpiochip7')
        self.assertFalse(summary['dynamic_levels_observed'])
        rows[-1]['do1'] = 1
        self.assertEqual(summarize_samples(pack(), '/dev/gpiochip7')['observed_level_changes']['do1'], 1)
        rows[-1]['source'] = 'simulated'
        with self.assertRaises(ValueError):
            summarize_samples(pack(), '/dev/gpiochip7')


class DeploymentPersistenceTests(unittest.TestCase):
    def test_write_and_atomic_status_sync_files_and_directories(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(deploy_live, 'ROOT', Path(directory)):
            synced = []
            real_fsync = os.fsync

            def observe(fd):
                info = os.fstat(fd)
                synced.append((info.st_ino, stat.S_ISDIR(info.st_mode)))
                real_fsync(fd)

            with patch.object(deploy_live.os, 'fsync', side_effect=observe):
                job = deploy_live.Deployment()
                artifact = job.write('snapshot.txt', 'retained\n')
                job.state['checkpoint'] = 'after_write'
                job.save()
            latest = job.evidence / 'deployment_latest.json'
            self.assertEqual(json.loads(latest.read_text())['checkpoint'], 'after_write')
            for path in (artifact, latest, job.folder / 'result.json'):
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o644)
                self.assertIn((path.stat().st_ino, False), synced)
            for path in (job.evidence, job.folder):
                self.assertIn((path.stat().st_ino, True), synced)
            self.assertFalse(list(job.folder.glob('*.tmp')))

    def test_log_and_running_checkpoint_are_fsynced_before_child_exit(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(deploy_live, 'ROOT', Path(directory)):
            job = deploy_live.Deployment()
            errors, outputs, synced = [], [], []
            real_fsync = os.fsync

            def observe(fd):
                synced.append(os.fstat(fd).st_ino)
                real_fsync(fd)

            def run():
                try:
                    outputs.append(job.run('ongoing', [sys.executable, '-u', '-c',
                                   "import time; print('first', flush=True); time.sleep(1.5); print('last')"], timeout=5))
                except BaseException as exc:
                    errors.append(exc)

            with patch.object(deploy_live.os, 'fsync', side_effect=observe):
                worker = threading.Thread(target=run)
                worker.start()
                deadline = time.monotonic() + 4
                snapshot = None
                while time.monotonic() < deadline and worker.is_alive():
                    snapshot = json.loads((job.evidence / 'deployment_latest.json').read_text())
                    if snapshot['steps'] and snapshot['steps'][-1].get('stdout_bytes', 0) > 0:
                        break
                    time.sleep(0.05)
                self.assertTrue(worker.is_alive(), 'must observe checkpoint before command exits')
                self.assertEqual(snapshot['steps'][-1]['status'], 'RUNNING')
                self.assertIn('first', (job.folder / 'ongoing.log').read_text())
                self.assertIn((job.folder / 'ongoing.log').stat().st_ino, synced)
                worker.join(timeout=6)
            self.assertFalse(worker.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(outputs, ['first\nlast\n'])
            self.assertEqual(json.loads((job.evidence / 'deployment_latest.json').read_text())['steps'][-1]['status'], 'COMPLETED')

    def test_timeout_keeps_flushed_partial_logs_and_failure_state(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(deploy_live, 'ROOT', Path(directory)):
            job = deploy_live.Deployment()
            with self.assertRaises(RuntimeError):
                job.run('timeout', [sys.executable, '-u', '-c',
                        "import time; print('before timeout', flush=True); time.sleep(10)"], timeout=0.7)
            self.assertEqual((job.folder / 'timeout.log').read_text(), 'before timeout\n')
            step = json.loads((job.evidence / 'deployment_latest.json').read_text())['steps'][-1]
            self.assertEqual(step['status'], 'FAILED')
            self.assertIn('timed out', step['error'])
            self.assertNotEqual(step['returncode'], 0)
            self.assertGreater(step['stdout_bytes'], 0)


if __name__ == '__main__':
    unittest.main()
