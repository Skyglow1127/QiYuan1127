"""Exercise unmodified official recording with synthetic hardware, no CAN TX."""
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import draccus
import numpy as np

from lerobot.utils.import_utils import register_third_party_plugins
register_third_party_plugins()
from lerobot.scripts.lerobot_record import RecordConfig, record
from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.configs import RGBEncoderConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot_robot_piper_shared import devices as d


class SyntheticCAN:
    tx_calls = 0
    def __init__(self, config): self.config, self.raw, self.number = config, None, 0
    def snapshot(self):
        now = time.monotonic()
        self.number += 1
        if self.raw: self.raw.write(json.dumps({'synthetic': True, 'receive_monotonic': now})+'\n')
        return {'ready': True, 'issues': [], 'observation_issues': [],
            'state': [32.+self.number*.001]*6+[.03], 'action': [32.1]*6+[0.],
            'ages_ms': {f'action_{i}': 1. for i in range(4)},
            'action_sampling': {'policy':'fresh_targets'}, 'parts': {}}
    def attach_raw(self, output): self.raw = output
    def close(self): pass
    def send(self, *args, **kwargs):
        type(self).tx_calls += 1
        raise AssertionError('CAN TX is forbidden')


def synthetic_fetch(self):
    self.test_index = getattr(self, 'test_index', 0)+1
    now = time.monotonic()
    self.last_metadata = {'index': self.test_index, 'serial': self.config.serial,
        'receive_monotonic': now, 'estimated_capture_monotonic': now-.001, 'age_ms': 1.}
    image = np.zeros((self.height, self.width, 3), np.uint8)
    image[:, :20, 0 if self.config.role=='global' else 2] = 220
    return self.test_index, image


class OfficialRecordTests(unittest.TestCase):
    def test_native_argument_registration_and_factories(self):
        cfg = draccus.parse(RecordConfig, args=[
            '--robot.type=piper_shared_observer', '--robot.id=test',
            '--teleop.type=piper_shared_leader', '--teleop.id=test',
            '--dataset.repo_id=local/registration_test', '--dataset.no_stamp=true',
            '--dataset.num_episodes=1', '--dataset.push_to_hub=false', '--play_sounds=false'])
        robot, teleop = make_robot_from_config(cfg.robot), make_teleoperator_from_config(cfg.teleop)
        self.assertIsInstance(robot, d.PiperSharedObserver)
        self.assertIsInstance(teleop, d.PiperSharedLeader)
        self.assertEqual(list(robot.action_features), d.KEYS)
        self.assertEqual(robot.observation_features['global'], (480,640,3))
        self.assertFalse(robot.is_connected)

    def test_unmodified_official_record_saves_decodable_dataset(self):
        plan = d.WORKSPACE/'projects/piper/collection_plan.json'
        proof = json.loads(d.PROOF.read_text())
        with tempfile.TemporaryDirectory(prefix='piper_official_synthetic_') as directory:
            root = Path(directory)
            (root/'projects/piper').mkdir(parents=True)
            (root/'projects/piper/collection_plan.json').write_text(plan.read_text())
            cameras = {k: d.PiperHttpCameraConfig(role=k, width=64, height=48,
                serial=v.serial, url=v.url) for k,v in d.default_cameras().items()}
            data_root = root/'dataset'
            cfg = RecordConfig(
                robot=d.PiperSharedObserverConfig(id='synthetic', cameras=cameras, data_root=data_root,
                    calibration_dir=root/'calibration/follower'),
                teleop=d.PiperSharedLeaderConfig(id='synthetic', calibration_dir=root/'calibration/leader'),
                dataset=DatasetRecordConfig(repo_id='local/native_official_test',root=data_root,
                    fps=10,num_episodes=2,episode_time_s=.4,reset_time_s=.1,push_to_hub=False,
                    no_stamp=True,streaming_encoding=True,encoder_threads=1,
                    rgb_encoder=RGBEncoderConfig(vcodec='h264',crf=18,preset='fast')),
                display_data=False,play_sounds=False)
            def fake_connect(camera, warmup=True): camera.connected = True
            events = {'exit_early':False,'rerecord_episode':False,'stop_recording':False}
            with patch.object(d, 'WORKSPACE', root), patch.object(d, 'PassiveCAN', SyntheticCAN), \
                 patch.object(d, 'check_endpoint', return_value={'link': {'ready':True}}), \
                 patch.object(d.PiperHttpCamera, 'connect', fake_connect), \
                 patch.object(d.PiperHttpCamera, '_fetch', synthetic_fetch), \
                 patch('lerobot.scripts.lerobot_record.init_keyboard_listener', return_value=(None, events)):
                d._contexts.clear()
                output = record(cfg)
                self.assertEqual(output.num_episodes, 2)
                context = next(iter(d._contexts.values()))
                self.assertEqual(context.refs, 0)
                self.assertIsNone(context.bus)
                self.assertEqual(SyntheticCAN.tx_calls, 0)
                context.expected_action = dict.fromkeys(d.KEYS, 0.)
                with self.assertRaises(RuntimeError): context.acknowledge(dict.fromkeys(d.KEYS, 90.))
                d._contexts.clear()
            dataset = LeRobotDataset('local/native_official_test', root=data_root, video_backend='torchcodec')
            self.assertEqual(dataset.num_episodes, 2)
            self.assertEqual(dataset.features['action']['names'], d.KEYS)
            for i in [0,len(dataset)-1]:
                row = dataset[i]
                self.assertEqual(tuple(row['observation.images.global'].shape), (3,48,64))
                self.assertEqual(tuple(row['observation.images.wrist'].shape), (3,48,64))
                self.assertTrue(np.allclose(row['action'].numpy()[:6], 32.1))
            manifest = json.loads((data_root/'collection_manifest.json').read_text())
            self.assertEqual(manifest['units']['joints'],'deg')


if __name__ == '__main__': unittest.main()
