"""Untrained sequence-model contract tests; no optimization or device access.

Random-weight sensitivity demonstrates interface/order preservation only, not
useful forecasting. A backward pass checks gradients without taking any step.
"""
from pathlib import Path
import sys
import tempfile
import unittest

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/preaction'))
from sequence_model import PreActionSequence, load_sequence_checkpoint, save_sequence_checkpoint


def inputs(batch=1):
    rgb_mask = torch.rand(batch, 4, 96, 96)
    rgb_mask[:, 3] = 0
    rgb_mask[:, 3, 30:65, 35:70] = 1
    q = torch.linspace(-.4, .4, 7).repeat(batch, 1)
    seq = torch.zeros(batch, 10, 17)
    seq[:, :, 0] = torch.linspace(.1, .9, 10)
    seq[:, :, 7] = 1
    seq[:, :, 14] = torch.arange(10) * .2
    seq[:, :, 15:17] = 1
    seq[:, 0, 16] = 0
    return rgb_mask, q, seq


class SequenceModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        torch.set_num_threads(1)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)

    def setUp(self):
        torch.manual_seed(42)
        self.model = PreActionSequence()
        self.x, self.q, self.seq = inputs()

    def motion(self, seq):
        return self.model(self.x, self.q, seq)['motion']

    def test_same_endpoint_different_middle_and_order_change_forecast(self):
        middle = self.seq.clone()
        middle[:, 4, 0] = -1.2
        ordered = self.seq.clone()
        ordered[:, 2, 0], ordered[:, 6, 0] = self.seq[:, 6, 0], self.seq[:, 2, 0]
        base = self.motion(self.seq)
        for other in (middle, ordered):
            self.assertTrue(torch.equal(other[:, -1], self.seq[:, -1]))
            self.assertGreater(float((base[:, -1] - self.motion(other)[:, -1]).abs().max().detach()), 1e-6)

    def test_padding_values_and_inserted_padding_do_not_advance_state(self):
        seq = self.seq.clone()
        seq[:, 3, 15] = 0
        changed = seq.clone()
        changed[:, 3, :7] = 100
        changed[:, 3, 7:14] = 1
        changed[:, 3, 14] = 999
        changed[:, 3, 16] = 1
        out = self.model(self.x, self.q, seq)
        torch.testing.assert_close(out['motion'], self.motion(changed), rtol=0, atol=0)
        self.assertTrue(torch.equal(out['motion'][:, 3], torch.zeros(1, 4)))
        self.assertFalse(out['future_mask'][0, 3])
        # Removing the padded step while preserving each valid step's timestamp
        # must leave the recurrence at every valid step unchanged.
        compact = torch.cat((seq[:, :3], seq[:, 4:], torch.zeros_like(seq[:, :1])), dim=1)
        torch.testing.assert_close(out['motion'][:, [0, 1, 2, 4, 5, 6, 7, 8, 9]],
                                   self.motion(compact)[:, :9], rtol=0, atol=0)

    def test_uncommanded_axis_values_and_gradients_have_no_effect(self):
        changed = self.seq.clone()
        changed[:, :, 1:7] = 1234
        torch.testing.assert_close(self.motion(self.seq), self.motion(changed), rtol=0, atol=0)
        changed.requires_grad_()
        self.motion(changed).sum().backward()
        self.assertTrue(torch.equal(changed.grad[:, :, 1:7], torch.zeros_like(changed.grad[:, :, 1:7])))
        self.assertGreater(float(changed.grad[:, :, 0].abs().sum()), 0)

    def test_time_and_expired_history_remain_features(self):
        later = self.seq.clone()
        later[:, :, 14] += .37
        self.assertGreater(float((self.motion(later) - self.motion(self.seq)).abs().max().detach()), 1e-5)
        expired_change = self.seq.clone()
        expired_change[:, 0, 0] += 2
        result = self.model(self.x, self.q, self.seq)
        self.assertTrue(result['valid_steps'][0, 0])
        self.assertFalse(result['future_mask'][0, 0])
        self.assertGreater(float((self.motion(expired_change)[:, 1] - result['motion'][:, 1]).abs().max().detach()), 1e-5)
        # The flag is also explicit input, rather than inferred from the index.
        future = self.seq.clone()
        future[:, 0, 16] = 1
        self.assertGreater(float((self.motion(future) - result['motion']).abs().max().detach()), 1e-5)

    def test_all_padding_is_explicit_zero_even_with_future_flags(self):
        seq = self.seq.clone()
        seq[:, :, 15] = 0
        result = self.model(self.x, self.q, seq)
        self.assertTrue(torch.equal(result['motion'], torch.zeros(1, 10, 4)))
        self.assertFalse(result['valid_steps'].any())
        self.assertFalse(result['future_mask'].any())

    def test_shapes_backward_and_frozen_legacy_vision(self):
        # A legacy endpoint head is intentionally unrelated to the new head.
        source = {'vision.' + k: torch.full_like(v, .02) for k, v in self.model.vision.state_dict().items()}
        source['head.10.weight'] = torch.full((4, 16, 1, 1), 999.)
        old_head = {k: v.clone() for k, v in self.model.motion_head.state_dict().items()}
        self.model.load_legacy_vision(source)
        self.assertTrue(self.model.legacy_vision_loaded)
        for k, v in self.model.vision.state_dict().items():
            torch.testing.assert_close(v, source['vision.' + k], rtol=0, atol=0)
        for k, v in self.model.motion_head.state_dict().items():
            torch.testing.assert_close(v, old_head[k], rtol=0, atol=0)
        before = {k: v.clone() for k, v in self.model.vision.state_dict().items()}
        self.model.train()
        x, q, seq = inputs(2)
        seq[1, :, 15] = 0
        result = self.model(x, q, seq)
        self.assertEqual(result['motion'].shape, (2, 10, 4))
        self.assertEqual(result['valid_steps'].dtype, torch.bool)
        self.assertEqual(result['future_mask'].dtype, torch.bool)
        result['motion'].square().mean().backward()
        self.assertTrue(all(p.grad is None and not p.requires_grad for p in self.model.vision.parameters()))
        self.assertGreater(float(self.model.recurrent.weight_ih.grad.abs().sum()), 0)
        self.assertGreater(float(self.model.initial_state.weight.grad.abs().sum()), 0)
        for k, v in self.model.vision.state_dict().items():
            torch.testing.assert_close(v, before[k], rtol=0, atol=0)
        self.assertTrue(torch.equal(result['motion'][1], torch.zeros(10, 4)))
        self.assertEqual(self.model.training_status, 'untrained_contract_test')

    def test_input_validation(self):
        bad_cases = [
            (self.x[:, :, :95], self.q, self.seq, ValueError),
            (self.x, self.q[:, :6], self.seq, ValueError),
            (self.x, self.q, self.seq[:, :9], ValueError),
            (self.x, self.q.repeat(2, 1), self.seq, ValueError),
            (self.x, self.q.long(), self.seq, TypeError),
            (self.x.double(), self.q, self.seq, TypeError),
        ]
        for x, q, seq, error in bad_cases:
            with self.subTest(shape=tuple(seq.shape), error=error), self.assertRaises(error):
                self.model(x, q, seq)
        for column in (7, 15, 16):
            bad = self.seq.clone()
            bad[:, 1, column] = .25
            with self.assertRaisesRegex(ValueError, 'binary'):
                self.motion(bad)
        bad = self.seq.clone()
        bad[:, -1, 15] = 0
        bad[:, -1, 0] = float('nan')
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            self.motion(bad)
        bad_x = self.x.clone()
        bad_x[:, 3, 0, 0] = .5
        with self.assertRaisesRegex(ValueError, 'binary'):
            self.model(bad_x, self.q, self.seq)

    def test_checkpoint_schema_roundtrip_and_old_checkpoint_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'sequence.pt'
            save_sequence_checkpoint(self.model, path)
            restored = load_sequence_checkpoint(path)
            self.assertEqual(restored.training_status, 'untrained_contract_test')
            self.assertFalse(restored.training_metadata['training_performed'])
            torch.testing.assert_close(restored(self.x, self.q, self.seq)['motion'], self.motion(self.seq))
            self.assertTrue(all(not p.requires_grad for p in restored.vision.parameters()))
            with self.assertRaises(FileExistsError):
                save_sequence_checkpoint(self.model, path)
            old_path = Path(folder) / 'legacy.pt'
            torch.save({'vision.0.weight': torch.zeros(8, 3, 3, 3), 'head.0.weight': torch.ones(16, 24, 3, 3)}, old_path)
            with self.assertRaisesRegex(ValueError, 'legacy'):
                load_sequence_checkpoint(old_path)
            with self.assertRaisesRegex(ValueError, 'two vision'):
                self.model.load_legacy_vision({'head.0.weight': torch.ones(1)})

    def test_checkpoint_metadata_and_forged_old_head_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'sequence.pt'
            save_sequence_checkpoint(self.model, path)
            payload = torch.load(path, weights_only=True)
            payload['training_status'] = 'trained_candidate_unvalidated'
            bad_path = Path(folder) / 'status.pt'
            torch.save(payload, bad_path)
            with self.assertRaisesRegex(ValueError, 'status mismatch'):
                load_sequence_checkpoint(bad_path)
            payload['training_status'] = 'untrained_contract_test'
            payload['state_dict'] = {'head.0.weight': torch.ones(16, 24, 3, 3)}
            torch.save(payload, bad_path)
            with self.assertRaisesRegex(ValueError, 'layout mismatch'):
                load_sequence_checkpoint(bad_path)


if __name__ == '__main__':
    unittest.main()
