"""Exact arithmetic checks of the design-only rational DDS reference."""
import contextlib
from dataclasses import FrozenInstanceError
from fractions import Fraction
import importlib.util
import io
import json
from pathlib import Path
import random
import sys
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "sim" / "tick_reference.py"
SPEC = importlib.util.spec_from_file_location("robot_rt_tick_reference", SOURCE)
tick = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = tick
SPEC.loader.exec_module(tick)


class TickReferenceTests(unittest.TestCase):
    def test_integer_twenty_hz_exact_period_and_first_edge(self):
        reference = tick.TickReference(20, 1)
        self.assertEqual(reference.config.tick_cycle(1), 5_000_000)
        events = list(reference.events(20))
        self.assertEqual([event.cycle for event in events], [5_000_000 * n for n in range(1, 21)])
        self.assertTrue(all(event.phase_after == 0 and event.timing_error_cycles == 0 for event in events))

    def test_fractional_137p5_hz_has_no_period_drift(self):
        config = tick.DDSConfig(275, 2)
        self.assertEqual(config.frequency_hz, Fraction(275, 2))
        self.assertEqual(config.tick_cycle(1), 727_273)
        events = [config.event(n) for n in range(1, 5001)]
        intervals = [events[0].cycle, *[b.cycle-a.cycle for a, b in zip(events, events[1:])]]
        self.assertEqual(set(intervals), {727_272, 727_273})
        self.assertTrue(all(Fraction(0) <= event.timing_error_cycles < 1 for event in events))
        self.assertEqual(config.event(11).timing_error_cycles, 0)

    def test_long_horizon_exact_ceil_bound_without_clock_iteration(self):
        generator = random.Random(20261102)
        for numerator, denominator in ((20, 1), (275, 2), (17, 3), (1375, 10)):
            config = tick.DDSConfig(numerator, denominator)
            indices = [1, 10**9, 10**12, 10**18, *[generator.randrange(1, 10**20) for _ in range(100)]]
            for n in indices:
                event = config.event(n)
                ideal = Fraction(n * config.threshold, numerator)
                self.assertEqual(event.timing_error_cycles, event.cycle - ideal)
                self.assertGreaterEqual(event.cycle - ideal, 0)
                self.assertLess(event.cycle - ideal, 1)
                self.assertLess(event.phase_after, numerator)
        # Equivalent rational representations have exactly the same event edges.
        self.assertEqual(tick.DDSConfig(275, 2).tick_cycle(10**18), tick.DDSConfig(1375, 10).tick_cycle(10**18))

    def test_analytic_advance_matches_independent_small_clock_loop(self):
        # A high logical rate keeps the *independent test oracle* short; this
        # configuration is not a physical actuator capability claim.
        reference = tick.TickReference(390_000, 1)
        phase, emitted, edges = 0, 0, []
        for cycle in range(1, 50_001):
            phase += 390_000
            if phase >= 100_000_000:
                phase -= 100_000_000
                emitted += 1
                edges.append(cycle)
        self.assertEqual(reference.advance_cycles(50_000), emitted)
        self.assertEqual((reference.cycle, reference.tick_count, reference.phase), (50_000, emitted, phase))
        self.assertEqual([reference.config.tick_cycle(n) for n in range(1, emitted+1)], edges)
        next_event = reference.next_event()
        self.assertGreater(next_event.cycle, 50_000)
        self.assertEqual(next_event, reference.config.event(emitted+1))

    def test_advance_at_tick_boundaries_and_reset_repeats_origin(self):
        reference = tick.TickReference(275, 2)
        first = reference.config.tick_cycle(1)
        self.assertEqual(reference.advance_cycles(first-1), 0)
        self.assertEqual(reference.advance_cycles(1), 1)
        self.assertEqual(reference.phase, reference.config.event(1).phase_after)
        self.assertEqual(reference.advance_cycles(0), 0)
        reference.advance_cycles(10**18)
        self.assertEqual(reference.tick_count, reference.cycle * 275 // 200_000_000)
        self.assertEqual(reference.phase, reference.cycle * 275 % 200_000_000)
        reference.reset()
        self.assertEqual((reference.cycle, reference.tick_count, reference.phase), (0, 0, 0))
        self.assertEqual(reference.next_event(), reference.config.event(1))
        with self.assertRaises(FrozenInstanceError):
            reference.config.f_num = 20
        with self.assertRaises(AttributeError):
            reference.config = tick.DDSConfig(20, 1)

    def test_positive_integer_and_uint64_threshold_validation(self):
        for numerator, denominator in ((0, 1), (-1, 1), (1, 0), (1, -1), (True, 1), (1, False),
                                       (1.0, 1), (1, "2"), (float("nan"), 1), (float("inf"), 1)):
            with self.subTest(numerator=numerator, denominator=denominator), self.assertRaises(ValueError):
                tick.DDSConfig(numerator, denominator)
        maximum_denominator = tick.UINT64_MAX // tick.CLOCK_HZ
        config = tick.DDSConfig(1, maximum_denominator)
        self.assertLessEqual(config.threshold, tick.UINT64_MAX)
        with self.assertRaisesRegex(ValueError, "64-bit"):
            tick.DDSConfig(1, maximum_denominator+1)
        # Do not automatically reduce an overflowing register representation.
        with self.assertRaisesRegex(ValueError, "64-bit"):
            tick.DDSConfig(20 * (maximum_denominator+1), maximum_denominator+1)

    def test_clock_and_shortest_interval_budget_are_separate_checks(self):
        with self.assertRaisesRegex(ValueError, "exceeds"):
            tick.DDSConfig(tick.CLOCK_HZ+1, 1)
        self.assertEqual(tick.DDSConfig(390_625, 1).interval_bounds, (256, 256))
        # First edge would still be cycle256, but subsequent gaps can be255.
        with self.assertRaisesRegex(ValueError, "256-cycle"):
            tick.DDSConfig(390_626, 1)
        near_maximum = tick.UINT64_MAX // tick.CLOCK_HZ
        config = tick.DDSConfig((tick.CLOCK_HZ * near_maximum) // 256, near_maximum)
        event = config.event(10**12)
        self.assertTrue(0 <= event.phase_after < config.f_num)
        self.assertLess(event.timing_error_cycles, 1)

    def test_invalid_event_advance_and_cli_remain_offline(self):
        reference = tick.TickReference(20, 1)
        for value in (-1, True, 1.0):
            with self.assertRaises(ValueError):
                reference.advance_cycles(value)
        with self.assertRaises(ValueError):
            reference.config.event(0)
        with self.assertRaises(ValueError):
            list(reference.events(-1))
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(tick.main(["--f-num", "275", "--f-den", "2", "--first-tick", "1000000000000", "--count", "2"]), 0)
        report = json.loads(output.getvalue())
        self.assertTrue(report["design_only"])
        self.assertFalse(report["physical_frequency_validated"])
        self.assertIsNone(report["f_exec_measured_hz"])
        self.assertEqual(report["events"][0]["tick_index"], 10**12)
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(SystemExit) as caught:
            tick.main(["--help"])
        self.assertEqual(caught.exception.code, 0)

    def test_ps_nominal_profile_and_rational_intervals(self):
        clock_hz = 99_999_001
        reference = tick.TickReference(20, 1, clock_hz)
        self.assertEqual(reference.next_event().cycle, 4_999_951)
        self.assertEqual(reference.config.clock_hz, clock_hz)
        for numerator, denominator, bounds in ((clock_hz, 256, (256, 256)),
                                               (2*clock_hz, 513, (256, 257)),
                                               (275, 2, (727_265, 727_266))):
            config = tick.DDSConfig(numerator, denominator, clock_hz)
            self.assertEqual(config.interval_bounds, bounds)
            for index in (1, 2, 3, 10**12, 10**18):
                event = config.event(index)
                self.assertEqual(event.cycle, -(-index*clock_hz*denominator//numerator))
                self.assertTrue(0 <= event.timing_error_cycles < 1)
        with self.assertRaisesRegex(ValueError, "256-cycle"):
            tick.DDSConfig(390_625, 1, clock_hz)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            tick.main(["--clock-hz", str(clock_hz), "--f-num", "20", "--f-den", "1", "--count", "1"])
        self.assertEqual(json.loads(output.getvalue())["clock_hz"], clock_hz)

    def test_clock_parameter_matches_positive_rtl_integer_domain(self):
        for clock_hz in (0, -1, True, 1.5, float("nan"), 1 << 31):
            with self.subTest(clock_hz=clock_hz), self.assertRaises(ValueError):
                tick.DDSConfig(1, 256, clock_hz)


if __name__ == "__main__":
    unittest.main()
