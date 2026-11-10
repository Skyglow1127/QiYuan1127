"""Permanent offline sequence-contract tests using synthetic local evidence.

These tests prepare inputs and audit provenance; they do not train a predictor
or treat a historical teacher image as the outcome of a proposed ACT action.
"""
import copy
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.preaction.sequence_contract import build_act_sequence, validate_sequence_packet, UNITS
from scripts.preaction.act_sequence import REQUEST_SCHEMA, prepare_request, read_json
from scripts.preaction.audit_act_sequence import request_from_record, audit


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def contract():
    started = 850481.926877216
    q = [1., 2., 3., 4., 5., 6., 30.]
    chunk = dict(commands=[[v + (i + 1) * .03 for v in q] for i in range(10)],
                 target_times=[started + i * .2 for i in range(10)],
                 observation_id='synthetic/fragment:0', model_sha256='a' * 64,
                 action_type='sdk_joint_position', coordinate_frame='sdk_native', units=list(UNITS))
    return dict(chunk=chunk, q_current=q, observation_id=chunk['observation_id'],
                q_observed_at=started - .03, prediction_started_at=started,
                prediction_completed_at=started + .04, valid_steps=[True] * 10,
                commanded_joint_mask=[[True] * 7 for _ in range(10)])


def record_and_example(image=None):
    c = contract()
    example = dict(id=c['observation_id'], q=c['q_current'],
                   command_host_monotonic=c['prediction_started_at'],
                   cameras={'global': image or {}}, actions=[['teacher_only']],
                   is_pad=[True] * 10, split='train')
    record = dict(observation_id=example['id'], sent_sdk_targets=[], actual_policy_robot_feedback=None,
                  historical_observation=dict(q=c['q_current'], time=c['q_observed_at']),
                  review=dict(timing=dict(historical_trigger_time=c['prediction_started_at'],
                                          virtual_completion_time=c['prediction_completed_at'])),
                  raw_policy_action=c['chunk'], status='REJECTED', rejection_reasons=['synthetic_review'])
    return record, example


class SequenceContractTests(unittest.TestCase):
    def test_full_chunk_float64_clock_and_j6_units_preserved(self):
        c = contract()
        packet = build_act_sequence(**c)
        self.assertEqual(packet['chunk']['commands'], c['chunk']['commands'])
        self.assertEqual(packet['chunk']['units'], ['degree'] * 6 + ['uncalibrated_sdk_raw'])
        self.assertEqual(packet['feature_shape'], [10, 17])
        self.assertEqual(len(packet['sequence_features']), 10)
        self.assertTrue(all(len(row) == 17 for row in packet['sequence_features']))
        self.assertEqual(packet['absolute_time_dtype'], 'float64')
        for actual, expected in zip(packet['chunk']['target_times'], c['chunk']['target_times']):
            self.assertEqual(struct.pack('!d', actual), struct.pack('!d', expected))
        self.assertNotEqual(float(np.float32(c['prediction_started_at'])), packet['prediction_started_at'])
        self.assertAlmostEqual(packet['sequence_features'][0][0], .03 / 5, places=8)
        self.assertAlmostEqual(packet['sequence_features'][0][6], .03 / 20, places=8)
        self.assertAlmostEqual(packet['q_features'][6], 30 / 100, places=7)
        self.assertEqual(validate_sequence_packet(packet), packet)

    def test_invalid_steps_and_uncommanded_axes_preserve_raw_but_mask_delta(self):
        c = contract()
        c['valid_steps'][2] = False
        c['chunk']['commands'][2] = [900.] * 7
        c['commanded_joint_mask'][3][6] = False
        c['chunk']['commands'][3][6] = 1e250
        packet = build_act_sequence(**c)
        self.assertEqual(packet['chunk']['commands'][2], [900.] * 7)
        self.assertEqual(packet['chunk']['commands'][3][6], 1e250)
        self.assertEqual(packet['sequence_features'][2][:7], [0.] * 7)
        self.assertEqual(packet['sequence_features'][3][6], 0.)
        self.assertFalse(packet['valid_steps'][2])
        self.assertFalse(packet['commanded_joint_mask'][3][6])
        self.assertEqual(packet['sequence_features'][3][13], 0.)

    def test_expired_steps_are_retained_without_retiming(self):
        c = contract()
        c['prediction_completed_at'] = c['chunk']['target_times'][2]
        packet = build_act_sequence(**c)
        self.assertEqual(packet['chunk']['target_times'], c['chunk']['target_times'])
        self.assertEqual(packet['chunk']['commands'], c['chunk']['commands'])
        self.assertEqual(packet['future_at_completion_mask'], [False] * 3 + [True] * 7)
        self.assertEqual(packet['valid_steps'], [True] * 10)
        self.assertNotEqual(packet['sequence_features'][0][0], 0.)
        self.assertEqual(packet['sequence_features'][0][14], 0.)
        self.assertEqual(packet['valid_future_count'], 7)

    def test_all_expired_and_all_invalid_have_explicit_zero_future_count(self):
        for all_invalid in (False, True):
            with self.subTest(all_invalid=all_invalid):
                c = contract()
                if all_invalid:
                    c['valid_steps'] = [False] * 10
                else:
                    c['prediction_completed_at'] = c['chunk']['target_times'][-1] + 1
                packet = build_act_sequence(**c)
                self.assertEqual(packet['valid_future_count'], 0)
                self.assertEqual(packet['valid_future_steps'], [False] * 10)
                self.assertEqual(packet['chunk']['commands'], c['chunk']['commands'])
                if all_invalid:
                    self.assertTrue(all(row[:7] == [0.] * 7 for row in packet['sequence_features']))

    def test_shape_rejections(self):
        mutations = [lambda c: c['chunk']['commands'].pop(),
                     lambda c: c['chunk']['commands'][0].pop(),
                     lambda c: c['chunk']['target_times'].pop(),
                     lambda c: c['q_current'].pop(),
                     lambda c: c['valid_steps'].pop(),
                     lambda c: c['commanded_joint_mask'].pop(),
                     lambda c: c['commanded_joint_mask'][0].pop()]
        for index, mutate in enumerate(mutations):
            with self.subTest(case=index):
                c = contract(); mutate(c)
                with self.assertRaises(ValueError):
                    build_act_sequence(**c)

    def test_nonfinite_values_rejected_even_in_masked_steps(self):
        for key, bad in [('command', float('nan')), ('command', float('inf')),
                         ('q', float('-inf')), ('time', float('nan'))]:
            with self.subTest(key=key, bad=bad):
                c = contract(); c['valid_steps'][0] = False
                if key == 'command': c['chunk']['commands'][0][0] = bad
                elif key == 'q': c['q_current'][0] = bad
                else: c['chunk']['target_times'][0] = bad
                with self.assertRaises(ValueError):
                    build_act_sequence(**c)
        c = contract(); c['q_current'][0] = 1e300
        with self.assertRaisesRegex(ValueError, 'float32'):
            build_act_sequence(**c)

    def test_bool_numbers_and_nonboolean_masks_rejected(self):
        mutations = [lambda c: c['chunk']['commands'][0].__setitem__(0, True),
                     lambda c: c['q_current'].__setitem__(0, False),
                     lambda c: c.__setitem__('prediction_started_at', True),
                     lambda c: c['valid_steps'].__setitem__(0, 1),
                     lambda c: c['commanded_joint_mask'][0].__setitem__(0, 0)]
        for index, mutate in enumerate(mutations):
            with self.subTest(case=index):
                c = contract(); mutate(c)
                with self.assertRaises(ValueError):
                    build_act_sequence(**c)

    def test_time_order_and_causal_observation_rejected(self):
        for mode in ('duplicate', 'reverse', 'future_q', 'equal_q', 'stale_q', 'completion_before_start'):
            with self.subTest(mode=mode):
                c = contract(); start = c['prediction_started_at']
                if mode == 'duplicate': c['chunk']['target_times'][1] = c['chunk']['target_times'][0]
                elif mode == 'reverse': c['chunk']['target_times'].reverse()
                elif mode == 'future_q': c['q_observed_at'] = start + .01
                elif mode == 'equal_q': c['q_observed_at'] = start
                elif mode == 'stale_q': c['q_observed_at'] = start - .601
                else: c['prediction_completed_at'] = start - .01
                with self.assertRaises(ValueError):
                    build_act_sequence(**c)

    def test_identity_sha_units_and_coordinate_semantics_rejected(self):
        for key, value in [('observation_id', ''), ('model_sha256', 'z' * 64),
                           ('model_sha256', 'a' * 63), ('units', ['degree'] * 7),
                           ('action_type', 'joint_delta'), ('coordinate_frame', 'world')]:
            with self.subTest(key=key, value=value):
                c = contract(); c['chunk'][key] = value
                with self.assertRaises(ValueError):
                    build_act_sequence(**c)
        c = contract(); c['observation_id'] = 'different'
        with self.assertRaisesRegex(ValueError, 'provenance'):
            build_act_sequence(**c)

    def test_derived_packet_tampering_rejected(self):
        packet = build_act_sequence(**contract())
        mutations = [lambda p: p['sequence_features'][0].__setitem__(0, 2.),
                     lambda p: p['valid_future_steps'].__setitem__(0, True),
                     lambda p: p['valid_future_steps'].__setitem__(0, 0),
                     lambda p: p.__setitem__('valid_future_count', 10),
                     lambda p: p.__setitem__('absolute_time_dtype', 'float32'),
                     lambda p: p.__setitem__('unexpected', 'not allowed')]
        for index, mutate in enumerate(mutations):
            with self.subTest(case=index):
                changed = copy.deepcopy(packet); mutate(changed)
                with self.assertRaises(ValueError):
                    validate_sequence_packet(changed)


class RequestPreparationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        path = self.root / 'current.png'
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[200:245, 290:335] = [0, 0, 255]
        self.assertTrue(cv2.imwrite(str(path), image))
        started = contract()['prediction_started_at']
        available = started - .1
        self.request = dict(schema=REQUEST_SCHEMA, **contract(), current_image=dict(
            path=str(path), sha256=digest(path), role='global', serial='CC1365302K0',
            available_t=available, board_age_at_fetch_ms=12.5,
            capture_age_estimate_s=started - available + .0125))

    def test_prepare_request_exports_tensors_raw_times_and_current_image_only(self):
        out = self.root / 'prepared'
        summary = prepare_request(self.request, out)
        self.assertEqual(summary['status'], 'INPUTS_PREPARED_ONLY')
        self.assertFalse(summary['forecast_generated'])
        self.assertFalse(summary['image']['future_image_used'])
        self.assertFalse(summary['image']['selection_is_ground_truth'])
        self.assertEqual(summary['image']['sha256'], self.request['current_image']['sha256'])
        with np.load(out / 'inputs.npz', allow_pickle=False) as data:
            self.assertEqual(data['rgb_mask'].shape, (1, 4, 96, 96))
            self.assertEqual(data['sequence_features'].shape, (1, 10, 17))
            self.assertEqual(data['q_features'].shape, (1, 7))
            self.assertEqual(data['rgb_mask'].dtype, np.float32)
            self.assertEqual(data['target_times'].dtype, np.float64)
            self.assertEqual(data['raw_commands'].dtype, np.float64)
            np.testing.assert_array_equal(data['target_times'][0], self.request['chunk']['target_times'])
            self.assertGreater(float(data['rgb_mask'][0, 0].max()), .9)
            self.assertEqual(float(data['rgb_mask'][0, 2].max()), 0.)
            self.assertTrue(set(np.unique(data['rgb_mask'][0, 3])).issubset({0., 1.}))
        for name, expected in summary['files_sha256'].items():
            self.assertEqual(digest(out / name), expected)

    def test_prepare_request_refuses_overwrite(self):
        out = self.root / 'prepared'
        prepare_request(self.request, out)
        before = {p.name: digest(p) for p in out.iterdir()}
        with self.assertRaises(FileExistsError):
            prepare_request(self.request, out)
        self.assertEqual(before, {p.name: digest(p) for p in out.iterdir()})

    def test_image_hash_identity_and_availability_rejections(self):
        for key, value in [('sha256', '0' * 64), ('serial', 'wrong-camera'), ('role', 'unknown'),
                           ('available_t', True), ('available_t', float('inf')),
                           ('available_t', contract()['prediction_started_at']),
                           ('available_t', contract()['prediction_started_at'] + .1),
                           ('available_t', contract()['prediction_started_at'] - .251)]:
            with self.subTest(key=key, value=value):
                request = copy.deepcopy(self.request); request['current_image'][key] = value
                with self.assertRaises(ValueError):
                    prepare_request(request, self.root / 'rejected')
                self.assertFalse((self.root / 'rejected').exists())

    def test_recently_received_old_cached_image_is_rejected(self):
        request = copy.deepcopy(self.request)
        started = request['prediction_started_at']
        # A response received one millisecond ago still contains a one-second
        # old frame; fresh receipt must not turn that cached frame into fresh RGB.
        available = started - .001
        request['current_image'].update(available_t=available, board_age_at_fetch_ms=1000.,
                                        capture_age_estimate_s=started - available + 1.)
        with self.assertRaisesRegex(ValueError, 'stale_image_capture'):
            prepare_request(request, self.root / 'rejected')
        self.assertFalse((self.root / 'rejected').exists())

    def test_missing_or_invalid_capture_age_is_rejected(self):
        for field in ('board_age_at_fetch_ms', 'capture_age_estimate_s'):
            for bad in ('missing', None, True, -1., float('nan'), float('inf')):
                with self.subTest(field=field, bad=bad):
                    request = copy.deepcopy(self.request)
                    if bad == 'missing':
                        request['current_image'].pop(field)
                    else:
                        request['current_image'][field] = bad
                    with self.assertRaisesRegex(ValueError, 'missing_or_invalid_image_capture_age'):
                        prepare_request(request, self.root / 'rejected')
                    self.assertFalse((self.root / 'rejected').exists())

    def test_inconsistent_capture_age_is_rejected(self):
        for field, change in [('capture_age_estimate_s', .001), ('board_age_at_fetch_ms', 1.)]:
            with self.subTest(field=field):
                request = copy.deepcopy(self.request)
                request['current_image'][field] += change
                with self.assertRaisesRegex(ValueError, 'inconsistent_image_capture_age'):
                    prepare_request(request, self.root / 'rejected')
                self.assertFalse((self.root / 'rejected').exists())

    def test_wrong_image_dimensions_rejected(self):
        path = self.root / 'small.png'
        cv2.imwrite(str(path), np.zeros((240, 320, 3), dtype=np.uint8))
        self.request['current_image'].update(path=str(path), sha256=digest(path))
        with self.assertRaisesRegex(ValueError, '640x480'):
            prepare_request(self.request, self.root / 'rejected')

    def test_nonfinite_json_constant_rejected(self):
        path = self.root / 'bad.json'
        path.write_text('{"time": NaN}')
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            read_json(path)

    def test_record_adapter_ignores_teacher_future_actions_and_padding(self):
        record, example = record_and_example(self.request['current_image'])
        actual = request_from_record(record, example)
        other = copy.deepcopy(example)
        other['actions'] = {'deliberately': 'not a usable action tensor'}
        other['is_pad'] = 'must not be inspected as an ACT mask'
        other['future_image'] = '/not/read/teacher_future.png'
        self.assertEqual(request_from_record(record, other), actual)
        self.assertEqual(actual['chunk'], record['raw_policy_action'])
        self.assertEqual(actual['valid_steps'], [True] * 10)
        self.assertEqual(actual['commanded_joint_mask'], [[True] * 7 for _ in range(10)])
        self.assertFalse(actual['provenance']['future_image_ground_truth_available'])
        self.assertFalse(actual['provenance']['teacher_future_used_as_input'])

    def test_record_adapter_rejects_join_or_execution_provenance_mismatch(self):
        for mode in ('id', 'q', 'trigger', 'sent', 'feedback', 'role'):
            with self.subTest(mode=mode):
                record, example = record_and_example(self.request['current_image'])
                record, example = copy.deepcopy(record), copy.deepcopy(example)
                if mode == 'id': record['observation_id'] = 'other'
                elif mode == 'q': record['historical_observation']['q'][0] += 1
                elif mode == 'trigger': example['command_host_monotonic'] += 1
                elif mode == 'sent': record['sent_sdk_targets'] = [[0.] * 7]
                elif mode == 'feedback': record['actual_policy_robot_feedback'] = {'q': [0.] * 7}
                with self.assertRaises(ValueError):
                    request_from_record(record, example, role='bad' if mode == 'role' else 'global')

    def make_audit_sources(self, records=None):
        dataset, review = self.root / 'dataset', self.root / 'review'
        dataset.mkdir(); review.mkdir()
        record, example = record_and_example(self.request['current_image'])
        examples = dataset / 'examples.jsonl'
        examples.write_text(json.dumps(example) + '\n')
        manifest = dataset / 'manifest.json'
        manifest.write_text(json.dumps(dict(examples_file=str(examples), examples_sha256=digest(examples))))
        protocol = review / 'protocol.json'
        protocol.write_text(json.dumps(dict(dataset_manifest_sha256=digest(manifest), checkpoint_sha256='a' * 64)))
        predictions = review / 'predictions.jsonl'
        predictions.write_text(''.join(json.dumps(r) + '\n' for r in (records or [record])))
        (review / 'summary.json').write_text(json.dumps(dict(predictions_sha256=digest(predictions), protocol_sha256=digest(protocol))))
        return dataset, review

    def test_audit_preserves_act_chunk_and_labels_no_physical_truth(self):
        dataset, review = self.make_audit_sources()
        out = self.root / 'audit'
        summary = audit(dataset, review, out)
        self.assertEqual((summary['accepted'], summary['rejected']), (1, 0))
        self.assertEqual(summary['teacher_padded_rows_not_used_as_ACT_masks'], 1)
        self.assertFalse(summary['hypothetical_actions_have_physical_labels'])
        packet = json.loads((out / 'packets.jsonl').read_text())['sequence']
        self.assertEqual(packet['chunk']['commands'], contract()['chunk']['commands'])
        self.assertEqual(packet['valid_steps'], [True] * 10)
        with self.assertRaises(FileExistsError):
            audit(dataset, review, out)

    def test_audit_rejects_model_identity_and_duplicate_records(self):
        record, _ = record_and_example(self.request['current_image'])
        wrong = copy.deepcopy(record); wrong['raw_policy_action']['model_sha256'] = 'b' * 64
        dataset, review = self.make_audit_sources([wrong, record])
        summary = audit(dataset, review, self.root / 'audit')
        self.assertEqual((summary['accepted'], summary['rejected']), (0, 2))
        errors = (self.root / 'audit/rejections.jsonl').read_text()
        self.assertIn('model_provenance_mismatch', errors)
        self.assertIn('duplicate_prediction_id', errors)

    def test_audit_rejects_modified_source_hash(self):
        dataset, review = self.make_audit_sources()
        with (dataset / 'examples.jsonl').open('a') as stream:
            stream.write('\n')
        with self.assertRaisesRegex(ValueError, 'examples_hash'):
            audit(dataset, review, self.root / 'audit')
        self.assertFalse((self.root / 'audit').exists())


if __name__ == '__main__':
    unittest.main()
