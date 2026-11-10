"""Sequence-training contract tests using only tiny synthetic observations."""

from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch

from scripts.preaction.sequence_model import PreActionSequence
from scripts.preaction import train_sequence as training


def synthetic_arrays(count=4):
    seq = np.zeros((count, 10, 17), np.float32)
    seq[:, :, 0] = np.linspace(0, .4, 10, dtype=np.float32)
    seq[:, :, 7] = 1
    offsets = np.tile(np.arange(10, dtype=np.float64) * .2, (count, 1))
    seq[:, :, 14] = offsets.astype(np.float32)
    seq[:, :, 15:] = 1
    seq[:, 0, 16] = 0
    rgb = np.zeros((count, 4, 96, 96), np.float32)
    rgb[:, 3] = 1
    label_mask = np.ones((count, 10), bool)
    label_mask[:, 0] = False
    boxes = np.tile(np.array([0, 0, 96, 96], np.float32), (count, 1))
    target = np.zeros((count, 10, 4), np.float32)
    target[:, :, :2] = .4
    target[:, 0] = 0
    return dict(rgb_mask=rgb, q_features=np.zeros((count, 7), np.float32),
                sequence_features=seq, motion_target=target, label_mask=label_mask,
                future_rgb=np.zeros((count, 10, 3, 96, 96), np.uint8),
                future_mask=np.ones((count, 10, 96, 96), np.uint8),
                centers=np.full((count, 2), .5, np.float32), current_box=boxes,
                future_box=np.repeat(boxes[:, None], 10, axis=1),
                crop_specs=np.tile(np.array([0, 0, 96], np.float32), (count, 1)),
                split=np.array(['train'] * (count - 1) + ['val']),
                ids=np.array([f'window-{i}' for i in range(count)]),
                group=np.array(['train-session'] * (count - 1) + ['val-session']),
                stage=np.array(['lift'] * count), target_offsets=offsets,
                label_offsets=offsets.copy())


def write_dataset(root, arrays=None, protocol_hash='a' * 64):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    arrays = synthetic_arrays() if arrays is None else arrays
    np.savez_compressed(root / 'dataset.npz', **arrays)
    records = [dict(id=str(arrays['ids'][i]), split=str(arrays['split'][i]),
                    group=str(arrays['group'][i]), stage=str(arrays['stage'][i]))
               for i in range(len(arrays['ids']))]
    (root / 'samples.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in records))
    manifest = dict(schema=training.DATA_SCHEMA,
                    dataset_path=str((root / 'dataset.npz').resolve()),
                    samples_path=str((root / 'samples.jsonl').resolve()),
                    dataset_sha256=training.sha(root / 'dataset.npz'),
                    samples_sha256=training.sha(root / 'samples.jsonl'),
                    protocol_sha256=protocol_hash,
                    counts=dict(windows=len(records), labels=int(arrays['label_mask'].sum()),
                                split={s: int(np.sum(arrays['split'] == s)) for s in ('train', 'val')},
                                independent_test=0))
    (root / 'manifest.json').write_text(json.dumps(manifest))
    return manifest


class FixedModel(torch.nn.Module):
    def forward(self, rgb_mask, q, seq):
        valid = seq[:, :, 15].bool()
        return dict(motion=torch.full((len(q), 10, 4), .2, dtype=rgb_mask.dtype),
                    valid_steps=valid, future_mask=valid & seq[:, :, 16].bool())


class SequenceTrainingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        cls.old_deterministic = torch.are_deterministic_algorithms_enabled()
        torch.set_num_threads(1)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)
        torch.use_deterministic_algorithms(cls.old_deterministic)

    def test_step_normalization_not_number_of_available_labels(self):
        arrays = synthetic_arrays(2)
        args = list(training.batch_tensors(arrays, np.array([0])))
        args[4][:] = False
        args[4][:, 1] = True
        one, _, one_parts = training.sequence_objective(FixedModel(), *args)
        args[4][:, 1:] = True
        nine, _, nine_parts = training.sequence_objective(FixedModel(), *args)
        torch.testing.assert_close(one, nine)
        self.assertEqual(one_parts['valid_steps'], 1)
        self.assertEqual(nine_parts['valid_steps'], 9)
        for key in ('parameter', 'rgb', 'silhouette'):
            torch.testing.assert_close(one_parts[key], nine_parts[key])

    def test_masked_label_placeholders_do_not_enter_any_loss(self):
        arrays = synthetic_arrays(2)
        args = list(training.batch_tensors(arrays, np.array([0])))
        args[4][:, 3] = False  # Missing outcome label, input step remains valid.
        before, prediction, parts = training.sequence_objective(FixedModel(), *args)
        args[3][:, [0, 3]] = float('nan')
        args[5][:, [0, 3]] = float('nan')
        args[6][:, [0, 3]] = float('nan')
        after, unchanged, after_parts = training.sequence_objective(FixedModel(), *args)
        torch.testing.assert_close(before, after, rtol=0, atol=0)
        torch.testing.assert_close(prediction, unchanged, rtol=0, atol=0)
        self.assertEqual(parts['valid_steps'], 8)
        self.assertEqual(after_parts['valid_steps'], 8)
        self.assertEqual(float(args[2][0, 3, 15]), 1.)

    def test_input_valid_and_future_flags_independently_exclude_loss(self):
        args = list(training.batch_tensors(synthetic_arrays(2), np.array([0])))
        args[4][:] = True
        args[2][:, 3, 15] = 0
        args[2][:, 4, 16] = 0
        _, _, parts = training.sequence_objective(FixedModel(), *args)
        self.assertEqual(parts['valid_steps'], 7)  # 0 expired, 3 invalid, 4 expired.
        args[4][:] = False
        with self.assertRaisesRegex(ValueError, 'no_supervised'):
            training.sequence_objective(FixedModel(), *args)

    def test_rgb_union_is_normalized_per_step(self):
        args = list(training.batch_tensors(synthetic_arrays(2), np.array([0, 1])))
        args[4][:] = False
        args[4][:, 1] = True
        args[0][:, 3] = 0
        args[0][0, 3, 40:44, 40:44] = 1
        args[0][1, 3, 16:80, 16:80] = 1
        args[6][:] = 0
        args[6][:, 1] = args[0][:, 3]
        args[5][0, 1] = .25
        args[5][1, 1] = .75

        class ZeroModel(FixedModel):
            def forward(self, rgb_mask, q, seq):
                result = super().forward(rgb_mask, q, seq)
                result['motion'].zero_()
                return result

        _, _, parts = training.sequence_objective(ZeroModel(), *args)
        # The 16-pixel and 4096-pixel examples have equal weight, rather than
        # allowing the larger foreground to dominate the RGB component.
        self.assertAlmostEqual(float(parts['rgb']), .5, places=6)

    def test_endpoint_hides_intermediate_delta_and_mask_without_mutation(self):
        seq = torch.from_numpy(synthetic_arrays(2)['sequence_features'])
        original = seq.clone()
        changed = seq.clone()
        changed[:, 2:8, :7] = 123
        changed[:, 2:8, 7:14] = 1 - changed[:, 2:8, 7:14]
        a = training.endpoint_features(seq)
        b = training.conditioning_features(changed, 'endpoint')
        torch.testing.assert_close(a, b, rtol=0, atol=0)
        torch.testing.assert_close(seq, original, rtol=0, atol=0)
        torch.testing.assert_close(a[:, :, 14:], seq[:, :, 14:], rtol=0, atol=0)
        torch.testing.assert_close(a[:, :, :14], seq[:, -1:, :14].expand(-1, 10, -1))
        self.assertIs(training.conditioning_features(seq, 'sequence'), seq)
        with self.assertRaises(ValueError):
            training.conditioning_features(seq, 'wrong')

    def test_endpoint_uses_last_valid_step_and_handles_all_padding(self):
        seq = torch.from_numpy(synthetic_arrays(2)['sequence_features'])
        seq[0, 8:, 15] = 0
        seq[1, :, 15] = 0
        result = training.endpoint_features(seq)
        torch.testing.assert_close(result[0, :, :14], seq[0, 7, :14].expand(10, -1))
        self.assertTrue(torch.equal(result[1, :, :14], torch.zeros(10, 14)))

    def test_loader_verifies_hashes_and_leaves_future_images_uint8(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            write_dataset(root)
            arrays, _ = training.load_dataset(root)
            self.assertEqual(arrays['future_rgb'].dtype, np.uint8)
            self.assertEqual(arrays['future_mask'].dtype, np.uint8)
            with (root / 'dataset.npz').open('ab') as stream:
                stream.write(b'tamper')
            with self.assertRaisesRegex(ValueError, 'dataset_hash_mismatch'):
                training.load_dataset(root)
            write_dataset(root)
            with (root / 'samples.jsonl').open('a') as stream:
                stream.write('{}\n')
            with self.assertRaisesRegex(ValueError, 'samples_hash_mismatch'):
                training.load_dataset(root)

    def test_loader_rejects_cross_split_groups_and_sample_join_changes(self):
        with tempfile.TemporaryDirectory() as folder:
            arrays = synthetic_arrays()
            arrays['group'][:] = 'same'
            write_dataset(folder, arrays)
            with self.assertRaisesRegex(ValueError, 'group_crosses_split'):
                training.load_dataset(folder)
            write_dataset(folder)
            root = Path(folder)
            records = [json.loads(x) for x in (root / 'samples.jsonl').read_text().splitlines()]
            records[0]['id'] = 'different'
            (root / 'samples.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in records))
            manifest = json.loads((root / 'manifest.json').read_text())
            manifest['samples_sha256'] = training.sha(root / 'samples.jsonl')
            (root / 'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'sample_array_join_mismatch'):
                training.load_dataset(root)

    def test_loader_rejects_mask_dtype_inactive_deltas_and_time_mismatch(self):
        with tempfile.TemporaryDirectory() as folder:
            arrays = synthetic_arrays()
            arrays['label_mask'] = arrays['label_mask'].astype(np.float32)
            write_dataset(folder, arrays)
            with self.assertRaisesRegex(ValueError, 'label_mask_dtype'):
                training.load_dataset(folder)
            arrays = synthetic_arrays()
            arrays['sequence_features'][:, :, 1] = 10
            write_dataset(folder, arrays)
            with self.assertRaisesRegex(ValueError, 'uncommanded_delta'):
                training.load_dataset(folder)
            arrays = synthetic_arrays()
            arrays['target_offsets'][:, 3] += .05
            write_dataset(folder, arrays)
            with self.assertRaisesRegex(ValueError, 'target_offset_feature_mismatch'):
                training.load_dataset(folder)

    def test_frozen_protocol_rejects_budget_changes(self):
        with tempfile.TemporaryDirectory() as folder:
            legacy = Path(folder) / 'legacy.pt'
            torch.save(PreActionSequence().state_dict(), legacy)
            protocol = dict(schema=training.PROTOCOL_SCHEMA,
                            training=deepcopy(training.FIXED_TRAINING), modes=list(training.MODES),
                            legacy_checkpoint_sha256=training.sha(legacy))
            training._validate_protocol(protocol, legacy)
            protocol['training']['steps'] += 1
            with self.assertRaisesRegex(ValueError, 'steps'):
                training._validate_protocol(protocol, legacy)

    def test_real_optimizer_tiny_runs_and_checkpoint_mode_binding(self):
        # Only this test replaces the frozen budget with three synthetic steps.
        # The production CLI has no such override; the normal protocol rejection
        # is tested above. Both complete training paths and reloads are exercised.
        tiny = {**deepcopy(training.FIXED_TRAINING), 'steps': 3, 'eval_every': 1,
                'batch_size': 2, 'threads': 1}
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            torch.manual_seed(9)
            legacy = root / 'legacy.pt'
            source_model = PreActionSequence()
            torch.save({'vision.' + k: v for k, v in source_model.vision.state_dict().items()}, legacy)
            protocol_path = root / 'protocol.json'
            protocol_path.write_text(json.dumps(dict(schema=training.PROTOCOL_SCHEMA,
                training=tiny, modes=list(training.MODES), legacy_checkpoint_sha256=training.sha(legacy))))
            data = root / 'data'
            write_dataset(data, protocol_hash=training.sha(protocol_path))
            outputs = {}
            with patch.object(training, 'FIXED_TRAINING', tiny), redirect_stdout(io.StringIO()):
                for mode in training.MODES:
                    outputs[mode] = training.train(data, protocol_path, legacy, root / mode, mode=mode)
            self.assertEqual(outputs['sequence']['initial_state_sha256'], outputs['endpoint']['initial_state_sha256'])
            self.assertEqual(outputs['sequence']['initial_validation'], outputs['endpoint']['initial_validation'])
            for mode, result in outputs.items():
                self.assertGreater(result['best_step'], 0)
                self.assertEqual(result['steps_run'], 3)
                checkpoint = root / mode / 'best.pt'
                model = training.load_trained_sequence(checkpoint, conditioning_mode=mode,
                    data_hash=training.sha(data / 'manifest.json'), protocol_hash=training.sha(protocol_path),
                    legacy_hash=training.sha(legacy))
                self.assertTrue(torch.any(model.motion_head.weight != 0))
                self.assertTrue(all(not p.requires_grad for p in model.vision.parameters()))
                for key, tensor in model.vision.state_dict().items():
                    torch.testing.assert_close(tensor, source_model.vision.state_dict()[key], rtol=0, atol=0)
                self.assertEqual(result['best_validation'], result['validation_after_reload'])
                other_mode = 'endpoint' if mode == 'sequence' else 'sequence'
                with self.assertRaisesRegex(ValueError, 'conditioning_mode'):
                    training.load_trained_sequence(checkpoint, conditioning_mode=other_mode)
                with self.assertRaisesRegex(ValueError, 'binding_mismatch'):
                    training.load_trained_sequence(checkpoint, conditioning_mode=mode, data_hash='f' * 64)
                curve = json.loads((root / mode / 'learning_curve.json').read_text())
                self.assertFalse(curve[0]['trained'])
                self.assertTrue(all(x['trained'] for x in curve[1:]))
                expected_best = min(curve[1:], key=lambda x: x['validation']['objective'])
                self.assertEqual(expected_best['step'], result['best_step'])
            with self.assertRaises(FileExistsError):
                training.train(data, protocol_path, legacy, root / 'sequence', mode='sequence')


if __name__ == '__main__':
    unittest.main()
