"""Synthetic offline sequence-data tests; no real logs, model training or hardware."""
import copy
from collections import Counter, OrderedDict
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.preaction import prepare_sequence_data as data


def write_json(path, value):
    Path(path).write_text(json.dumps(value)+'\n')


def write_lines(path, rows):
    Path(path).write_text(''.join(json.dumps(row)+'\n' for row in rows))


class SyntheticSource:
    def __init__(self, root, name, split, start, partial=False):
        self.folder = root/name
        self.folder.mkdir()
        (self.folder/'frames').mkdir()
        self.start = start
        self.split = split
        self.name = name
        self.q = [0.]*6+[35.9]
        self.candidate = self.folder/'candidate.json'
        write_json(self.candidate, {'synthetic': True})
        self.prior = self.folder/'source_plan.json'
        write_json(self.prior, {'stages': [dict(name='lift', sha256='a'*64)]})
        self.plan = dict(stage='lift', source_plan=str(self.prior), source_plan_sha256=data.sha(self.prior),
                         source_trajectory_sha256='a'*64, candidate=str(self.candidate),
                         candidate_sha256=data.sha(self.candidate), requested_gripper_target_sdk=36.,
                         sdk_plan=dict(sdk=[[0.]*6, [3., 0., 0., 0., 0., 0.]], ticks=[30], rate=5., duration_s=6.))
        if partial:
            self.plan.update(commanded_joint_mask=[True]+[False]*6,
                             action_semantics='single_joint_command_no_updates_to_other_axes',
                             requested_gripper_target_sdk=None)
        write_json(self.folder/'plan.json', self.plan)
        refs = data.planned_references(self.plan['sdk_plan'])
        self.events = []
        for i, (_, ref) in enumerate(refs):
            t = start+i*.2
            self.events += [dict(t=t-.02, kind='output', line='[rx] '+json.dumps(
                dict(address=2, funcode=1, data={f'angle{j}': q for j, q in enumerate(self.q)}))),
                            dict(t=t-.01, kind='output', line='[rx] '+json.dumps(
                dict(address=2, funcode=3, data=dict(power_status=1, enable_status=1, error_status=0)))),
                            dict(t=t, kind='command', sdk=[ref[0]]+[None]*6 if partial else ref+[36.])]
        write_lines(self.folder/'transport.jsonl', self.events)
        self.frames = []
        base = 250 if split == 'train' else 270
        for i in range(-1, 31):
            stamp = start-.05 if i == -1 else start+i*.2+.03
            filename = f'wrist_{i+1:03d}.png'
            image = np.zeros((480, 640, 3), np.uint8)
            image[210:240, base+i:base+i+30] = [0, 0, 255]
            image[0, 0] = [i+2, 0, 0]
            cv2.imwrite(str(self.folder/'frames'/filename), image)
            self.frames.append(dict(role='wrist', serial=data.SERIAL, file=filename,
                                    index=i+2, age_ms=10., host_monotonic=stamp))
        write_lines(self.folder/'frames/frames.jsonl', self.frames)
        write_json(self.folder/'result.json', dict(completed=True, commands=len(refs), hardware_commands_sent=True))

    def entry(self):
        paths = [self.folder/x for x in ('plan.json', 'transport.jsonl', 'frames/frames.jsonl', 'result.json')]
        paths += [self.prior, self.candidate]
        return dict(path=str(self.folder), segment_id=self.name, group=self.name+'_session',
                    suggested_split=self.split, original_splits=['train'] if self.split=='train' else [],
                    source_paths_sha256={str(p): data.sha(p) for p in paths},
                    verified_executed_plan_prefix_commands=31, command_count=31)


class SequenceDataTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.train = SyntheticSource(self.root, 'train_source', 'train', 850481.926877216, partial=True)
        self.val = SyntheticSource(self.root, 'val_source', 'development_validation', 950481.926877216)
        self.inventory = self.root/'inventory.json'
        self.protocol = self.root/'protocol.json'
        self.refresh()

    def refresh(self, entries=None, role='wrist'):
        write_json(self.inventory, dict(schema='preaction-sequence-source-inventory-v1', manifests=[],
                                       sources=entries or [self.train.entry(), self.val.entry()]))
        write_json(self.protocol, dict(schema='preaction-sequence-pilot-protocol-v1',
            inventory_path=str(self.inventory), inventory_sha256=data.sha(self.inventory),
            data=dict(camera_role=role, chunk_size=10, rate_hz=5., anchor_stride=10,
                      min_valid_labels=5, feedback_max_age_s=.6, image_max_age_s=.25,
                      command_schedule_tolerance_s=.1, label_time_tolerance_s=.12, label_step_zero=False)))

    def build(self, name='out'):
        return data.build_dataset(self.inventory, self.root/name, self.protocol)

    def read_output(self, name='out'):
        arrays = dict(np.load(self.root/name/'dataset.npz', allow_pickle=False))
        samples = data.jsonlines(self.root/name/'samples.jsonl')
        return arrays, samples

    def test_exact_schema_types_masks_hashes_and_plan_features(self):
        manifest = self.build()
        arrays, samples = self.read_output()
        self.assertEqual(manifest['counts']['split'], {'train': 3, 'val': 3})
        expected = {'rgb_mask': ((6,4,96,96), np.float32), 'q_features': ((6,7), np.float32),
                    'sequence_features': ((6,10,17), np.float32), 'motion_target': ((6,10,4), np.float32),
                    'label_mask': ((6,10), np.bool_), 'future_rgb': ((6,10,3,96,96), np.uint8),
                    'future_mask': ((6,10,96,96), np.uint8), 'current_box': ((6,4), np.float32),
                    'future_box': ((6,10,4), np.float32), 'crop_specs': ((6,3), np.int32),
                    'centers': ((6,2), np.float32), 'target_offsets': ((6,10), np.float64),
                    'label_offsets': ((6,10), np.float64)}
        self.assertEqual(set(arrays), set(expected)|{'split','ids','group','stage'})
        for key, (shape, dtype) in expected.items():
            self.assertEqual(arrays[key].shape, shape, key)
            self.assertEqual(arrays[key].dtype, dtype, key)
        for key in ('split','ids','group','stage'):
            self.assertEqual(arrays[key].dtype.kind, 'U')
            self.assertEqual(arrays[key].shape, (6,))
        self.assertTrue(np.all(arrays['sequence_features'][:,:,15] == 1))
        self.assertTrue(np.all(arrays['sequence_features'][:,0,16] == 0))
        self.assertTrue(np.all(arrays['sequence_features'][:,1:,16] == 1))
        self.assertFalse(arrays['label_mask'][:,0].any())
        self.assertTrue(np.all(arrays['label_mask'].sum(1) >= 5))
        self.assertTrue(np.all(arrays['sequence_features'][:3,:,1:7] == 0))
        self.assertEqual(samples[0]['raw_planned_targets'][0], [0.]+[None]*6)
        self.assertEqual(samples[0]['prediction_started_at'], self.train.start)
        self.assertEqual(samples[0]['target_times'][9], self.train.start+1.8)
        self.assertEqual([x['command_index'] for x in samples[:3]], [0,10,20])
        self.assertEqual(arrays['ids'].tolist(), [x['id'] for x in samples])
        self.assertEqual(manifest['dataset_sha256'], data.sha(self.root/'out/dataset.npz'))
        self.assertEqual(manifest['samples_sha256'], data.sha(self.root/'out/samples.jsonl'))
        np.testing.assert_allclose(arrays['q_features'][0,6], .359)
        # Full-arm J6 is raw SDK position: (36 - 35.9) / 20.
        np.testing.assert_allclose(arrays['sequence_features'][3,:,6], .005)
        for key in ('future_rgb', 'future_mask', 'future_box', 'motion_target', 'label_offsets'):
            self.assertTrue(np.all(arrays[key][~arrays['label_mask']] == 0), key)
        for offsets, mask in zip(arrays['label_offsets'], arrays['label_mask']):
            self.assertTrue(np.all(np.diff(offsets[mask]) > 0))
        self.assertEqual(manifest['robot_commands'], 0)

    def test_hash_tamper_and_protocol_binding_fail_before_output(self):
        with (self.train.folder/'transport.jsonl').open('a') as stream:
            stream.write('\n')
        with self.assertRaisesRegex(ValueError, 'source_hash_mismatch'):
            self.build()
        self.assertFalse((self.root/'out').exists())
        self.refresh()
        protocol = data.read_json(self.protocol)
        protocol['inventory_sha256'] = 'b'*64
        write_json(self.protocol, protocol)
        with self.assertRaisesRegex(ValueError, 'protocol_inventory_mismatch'):
            self.build()

    def test_plan_chain_rejected_even_after_inventory_refresh(self):
        write_json(self.train.prior, {'stages':[dict(name='lift', sha256='b'*64)]})
        self.refresh()
        with self.assertRaisesRegex(ValueError, 'plan_source_chain_mismatch'):
            self.build()

    def test_group_split_and_prior_holdout_rejected(self):
        a,b = self.train.entry(), self.val.entry()
        b['group'] = a['group']
        self.refresh([a,b])
        with self.assertRaisesRegex(ValueError, 'group_crosses_split'):
            self.build()
        b['group'] = 'validation_session'
        a['original_splits'] = ['val']
        self.refresh([a,b])
        with self.assertRaisesRegex(ValueError, 'prior_holdout'):
            self.build()

    def test_existing_output_never_overwritten(self):
        self.build()
        before = data.sha(self.root/'out/dataset.npz')
        with self.assertRaises(FileExistsError):
            self.build()
        self.assertEqual(before, data.sha(self.root/'out/dataset.npz'))

    def test_future_feedback_invalidates_entire_window_without_crossing_gap(self):
        # A bad status during step 5 invalidates anchor 0, not merely its label.
        message = json.loads(self.train.events[16]['line'].split('[rx] ')[1])
        message['data']['error_status'] = 1
        self.train.events[16]['line'] = '[rx] '+json.dumps(message)
        write_lines(self.train.folder/'transport.jsonl', self.train.events)
        self.refresh()
        manifest = self.build()
        _, samples = self.read_output()
        self.assertEqual([x['command_index'] for x in samples if x['split']=='train'], [10,20])
        self.assertEqual(manifest['rejected_windows']['window_error_status_valid'], 1)

    def test_command_schedule_gap_rejects_window(self):
        # Keep event order while the fifth command is 110 ms late.
        self.train.events[17]['t'] += .11
        write_lines(self.train.folder/'transport.jsonl', self.train.events)
        self.refresh()
        manifest = self.build()
        self.assertEqual(manifest['rejected_windows']['window_command_schedule'], 1)

    def test_wrong_none_mask_or_future_command_cannot_become_feature(self):
        self.train.events[17]['sdk'][1] = 0.
        write_lines(self.train.folder/'transport.jsonl', self.train.events)
        self.refresh()
        with self.assertRaisesRegex(ValueError, 'inventory_prefix'):
            self.build()

    def test_missing_red_future_only_masks_label(self):
        image = self.train.folder/'frames'/self.train.frames[4]['file']
        cv2.imwrite(str(image), np.zeros((480,640,3),np.uint8))
        self.build()
        arrays, samples = self.read_output()
        self.assertFalse(arrays['label_mask'][0,3])
        self.assertTrue(np.all(arrays['sequence_features'][0,:,15] == 1))
        self.assertEqual(samples[0]['labels'][3]['reason'], 'label_no_usable_red_target_or_rgb')
        self.assertTrue(np.all(arrays['future_rgb'][0,3] == 0))

    def test_truncated_future_target_is_masked_in_fixed_current_roi(self):
        image = np.zeros((480,640,3),np.uint8)
        image[210:240, 400:430] = [0,0,255]
        cv2.imwrite(str(self.train.folder/'frames'/self.train.frames[4]['file']), image)
        self.build()
        arrays, samples = self.read_output()
        self.assertFalse(arrays['label_mask'][0,3])
        self.assertEqual(samples[0]['labels'][3]['reason'], 'label_target_boundary_cut')
        self.assertTrue(np.all(arrays['sequence_features'][0,:,15] == 1))

    def test_current_wrong_identity_and_stale_capture_rejected(self):
        self.train.frames[0]['serial'] = 'wrong'
        write_lines(self.train.folder/'frames/frames.jsonl', self.train.frames)
        self.refresh()
        manifest = self.build()
        self.assertEqual(manifest['rejected_windows']['current_image_invalid'], 1)
        self.train.frames[0]['serial'] = data.SERIAL
        self.train.frames[0]['age_ms'] = 240.
        write_lines(self.train.folder/'frames/frames.jsonl', self.train.frames)
        self.refresh()
        manifest = self.build('out2')
        self.assertEqual(manifest['rejected_windows']['current_image_invalid'], 1)

    def test_image_hash_verified_before_decoding(self):
        frames, _ = data.load_frames(self.train.folder)
        image = Path(frames[0]['path'])
        image.write_bytes(image.read_bytes()+b'tamper')
        with self.assertRaisesRegex(ValueError, 'image_hash_mismatch'):
            data.image_data(frames[0], OrderedDict())

    def test_partial_features_reject_bool_nonfinite_mask_mismatch(self):
        targets = [[1.]+[None]*6 for _ in range(10)]
        mask = [True]+[False]*6
        offsets = [i*.2 for i in range(10)]
        q, features = data.encode_plan(self.train.q, targets, mask, offsets)
        self.assertEqual(features.shape, (10,17))
        self.assertTrue(np.all(features[:,1:7] == 0))
        for value in (True, float('nan'), float('inf')):
            malformed = copy.deepcopy(targets)
            malformed[2][0] = value
            with self.assertRaises(ValueError):
                data.encode_plan(self.train.q, malformed, mask, offsets)
        with self.assertRaisesRegex(ValueError, 'invalid_joint_mask'):
            data.encode_plan(self.train.q, targets, [1]+[False]*6, offsets)
        targets[0][1] = 0.
        with self.assertRaisesRegex(ValueError, 'raw_target_mask_mismatch'):
            data.encode_plan(self.train.q, targets, mask, offsets)

    def test_interrupted_tail_is_not_used(self):
        # Retain a verified 21-command prefix. Anchor 20 cannot form a complete input chunk.
        self.train.events = self.train.events[:63]
        write_lines(self.train.folder/'transport.jsonl', self.train.events)
        write_json(self.train.folder/'result.json', dict(completed=False, commands=21,
                   hardware_commands_sent=True, error='synthetic interrupted tail'))
        a = self.train.entry()
        a.update(command_count=21, verified_executed_plan_prefix_commands=21)
        self.refresh([a,self.val.entry()])
        self.build()
        _, samples = self.read_output()
        self.assertEqual([x['command_index'] for x in samples if x['split']=='train'], [0,10])

    def test_missing_next_command_masks_last_label(self):
        self.train.events = self.train.events[:60]
        write_lines(self.train.folder/'transport.jsonl', self.train.events)
        write_json(self.train.folder/'result.json', dict(completed=False, commands=20, hardware_commands_sent=True))
        a = self.train.entry()
        a.update(command_count=20, verified_executed_plan_prefix_commands=20)
        self.refresh([a,self.val.entry()])
        self.build()
        arrays, samples = self.read_output()
        self.assertFalse(arrays['label_mask'][1,9])
        self.assertEqual(samples[1]['labels'][9]['reason'], 'label_reaches_next_command_or_boundary')

    def test_fewer_than_five_labels_rejects_only_affected_anchor(self):
        for i in range(1,6):
            cv2.imwrite(str(self.train.folder/'frames'/self.train.frames[i+1]['file']),
                        np.zeros((480,640,3),np.uint8))
        manifest = self.build()
        _, samples = self.read_output()
        self.assertEqual(manifest['rejected_windows']['fewer_than_five_labels'], 1)
        self.assertEqual([x['command_index'] for x in samples if x['split']=='train'], [10,20])

    def test_frame_reused_from_current_image_masks_label(self):
        first = self.train.folder/'frames'/self.train.frames[0]['file']
        future = self.train.folder/'frames'/self.train.frames[4]['file']
        future.write_bytes(first.read_bytes())
        self.build()
        arrays, samples = self.read_output()
        self.assertFalse(arrays['label_mask'][0,3])
        self.assertEqual(samples[0]['labels'][3]['reason'], 'label_repeated_or_current_image')

    def test_nonmonotonic_label_rejected_even_with_distinct_hash_in_tolerance(self):
        # Adjacent +/-120 ms ranges overlap; distinct hashes do not prove temporal order.
        frame = dict(capture_time_estimate=1000.29, sha256='new_hash')
        self.assertEqual(data.label_time_rejection(frame, 1000.4, 1000., 1000.6,
                                                   {'prior_hash'}, 1000.31), 'label_nonmonotonic')
        frame['capture_time_estimate'] = 1000.31
        self.assertEqual(data.label_time_rejection(frame, 1000.4, 1000., 1000.6,
                                                   {'prior_hash'}, 1000.31), 'label_nonmonotonic')
        frame['capture_time_estimate'] = 1000.42
        self.assertIsNone(data.label_time_rejection(frame, 1000.4, 1000., 1000.6,
                                                    {'prior_hash'}, 1000.31))

    def test_protocol_parameters_and_no_causal_future_q_rejected(self):
        protocol = data.read_json(self.protocol)
        protocol['data']['anchor_stride'] = 1
        write_json(self.protocol, protocol)
        with self.assertRaisesRegex(ValueError, 'unsupported_frozen_data_protocol'):
            self.build()
        self.refresh()
        # The first command has no preceding observation after removing its RX.
        del self.train.events[0]
        write_lines(self.train.folder/'transport.jsonl', self.train.events)
        self.refresh()
        manifest = self.build()
        self.assertEqual(manifest['rejected_windows']['window_feedback_present'], 1)

    def test_excluded_source_stays_excluded_and_val_alias_is_supported(self):
        excluded = self.train.entry()
        excluded.update(segment_id='old_val', group='old_val_group', suggested_split='excluded', original_splits=['val'])
        val = self.val.entry()
        val['suggested_split'] = 'val'
        self.refresh([self.train.entry(),val,excluded])
        manifest = self.build()
        arrays, _ = self.read_output()
        self.assertEqual(set(arrays['split']), {'train','val'})
        self.assertEqual(manifest['excluded_sources'], [str(self.train.folder)])

    def test_failure_preserves_per_source_and_split_diagnostics(self):
        for frame in self.val.frames:
            cv2.imwrite(str(self.val.folder/'frames'/frame['file']), np.zeros((480,640,3),np.uint8))
        with self.assertRaisesRegex(ValueError, 'no_eligible_windows_in_both'):
            self.build()
        diagnostics = data.read_json(self.root/'out/build_diagnostics.json')
        self.assertEqual(diagnostics['status'], 'FAILED')
        self.assertEqual(diagnostics['candidate_windows_by_split'], {'train':3,'val':3})
        self.assertEqual(diagnostics['retained_windows_by_split'], {'train':3})
        self.assertEqual(diagnostics['sources'][1]['windows'], 0)
        self.assertEqual(sum(diagnostics['sources'][1]['rejected_windows'].values()), 3)
        self.assertFalse((self.root/'out/dataset.npz').exists())
        self.assertTrue((self.root/'out/causal/01_source.jsonl').exists())

    def use_global_fixtures(self):
        for source in (self.train, self.val):
            for frame in source.frames:
                frame.update(role='global', serial=data.SERIALS['global'])
            write_lines(source.folder/'frames/frames.jsonl', source.frames)
        self.refresh(role='global')

    def test_global_protocol_uses_same_role_for_current_and_all_labels(self):
        self.use_global_fixtures()
        manifest = self.build()
        arrays, samples = self.read_output()
        self.assertEqual(manifest['camera_role'], 'global')
        self.assertEqual(manifest['camera_serial'], data.SERIALS['global'])
        self.assertEqual(manifest['counts']['split'], {'train':3,'val':3})
        for sample in samples:
            self.assertEqual(sample['current_image']['role'], 'global')
            self.assertEqual(sample['current_image']['serial'], data.SERIALS['global'])
            for label in sample['labels']:
                if label['valid']:
                    self.assertEqual(label['image']['role'], 'global')
                    self.assertEqual(label['image']['serial'], data.SERIALS['global'])
        # Passing a frame of the other camera directly cannot silently become a label.
        frames, _ = data.load_frames(self.train.folder, role='global')
        frames.append({**frames[0], 'role':'wrist', 'serial':data.SERIALS['wrist']})
        rows = data.jsonlines(self.root/'out/causal/00_source.jsonl')
        with self.assertRaisesRegex(ValueError, 'mixed_or_wrong_camera_identity'):
            data.make_sample(self.train.entry(), self.train.plan, rows,
                             data.planned_references(self.train.plan['sdk_plan']),
                             [True]+[False]*6, 0, frames, OrderedDict(), Counter(), role='global')

    def test_global_serial_mismatch_is_not_rescued_by_wrist_identity(self):
        self.use_global_fixtures()
        self.train.frames[0]['serial'] = data.SERIALS['wrist']
        write_lines(self.train.folder/'frames/frames.jsonl', self.train.frames)
        self.refresh(role='global')
        manifest = self.build()
        self.assertEqual(manifest['rejected_windows']['current_image_invalid'], 1)
        self.assertEqual(manifest['sources'][0]['rejected_frame_records']['frame_identity_age_or_order'], 1)

    def test_unknown_protocol_camera_role_is_rejected(self):
        self.refresh(role='both')
        with self.assertRaisesRegex(ValueError, 'unsupported_camera_role'):
            self.build()


if __name__ == '__main__':
    unittest.main()
