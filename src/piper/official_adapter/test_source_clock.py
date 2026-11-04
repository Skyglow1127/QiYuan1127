"""Clock-read scheduling regression; no sockets, SSH, or hardware."""
import unittest
from unittest.mock import patch

from lerobot_robot_piper_shared import remote_can_source as source


class SourceClockSamplingTests(unittest.TestCase):
    def test_one_preempted_read_is_discarded_then_precise_read_used(self):
        clock = source.ClockMapper()
        with patch.object(source.time, 'monotonic_ns', side_effect=[
                1_000_000_000, 1_006_000_000, 1_007_000_000, 1_007_000_200]), \
             patch.object(source.time, 'time_ns', side_effect=[
                101_003_000_000, 101_007_000_100]):
            self.assertEqual(clock.sample(), (1_007_000_000, 101_007_000_100, 200))
        self.assertEqual(clock.previous_offset_ns, 100_000_000_000)

    def test_persistent_uncertainty_still_fails_with_same_five_ms_bound(self):
        stamps = [stamp for i in range(5) for stamp in (i*10_000_000, i*10_000_000+6_000_000)]
        clock = source.ClockMapper()
        with patch.object(source.time, 'monotonic_ns', side_effect=stamps) as reads, \
             patch.object(source.time, 'time_ns', return_value=100_000_000_000):
            with self.assertRaisesRegex(RuntimeError, 'exceeds 5 ms after 5 samples'):
                clock.sample()
        self.assertEqual(reads.call_count, 10)
        self.assertIsNone(clock.previous_offset_ns)

    def test_real_clock_jump_is_not_hidden_by_retry(self):
        clock = source.ClockMapper()
        clock.previous_offset_ns = 100_000_000_000
        with patch.object(source.time, 'monotonic_ns', side_effect=[1_000_000_000, 1_000_000_200]), \
             patch.object(source.time, 'time_ns', return_value=101_010_000_100):
            with self.assertRaisesRegex(RuntimeError, 'offset jumped'):
                clock.sample()

    def test_backwards_monotonic_clock_is_rejected_immediately(self):
        with patch.object(source.time, 'monotonic_ns', side_effect=[10, 9]), \
             patch.object(source.time, 'time_ns', return_value=100):
            with self.assertRaisesRegex(RuntimeError, 'moved backwards'):
                source.ClockMapper().sample()

    def test_resampling_preserves_kernel_queue_age(self):
        clock = source.ClockMapper()
        with patch.object(source.time, 'monotonic_ns', side_effect=[
                1_090_000_000, 1_096_000_000, 1_100_000_000, 1_100_000_200]), \
             patch.object(source.time, 'time_ns', side_effect=[
                101_093_000_000, 101_100_000_100]):
            mapped, uncertainty = clock.convert(101_000_000_000)
        self.assertEqual(mapped, 999_999_900)
        self.assertEqual(uncertainty, 200)
        self.assertGreaterEqual(1_100_000_200-mapped, 100_000_000)


if __name__ == '__main__':
    unittest.main()
