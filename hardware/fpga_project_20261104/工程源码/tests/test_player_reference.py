"""Exact-integer player tests; no device, simulator, model, or new dependencies."""
from fractions import Fraction
import importlib.util
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"sim"))
import player_reference as player
import make_gate_vectors as gate_vectors
import make_player_vectors as math_vectors


def broad_bounds(dim=1):
    return player.Bounds.from_physical(lower=[-100]*dim, upper=[100]*dim, velocity=[100]*dim,
        acceleration=[100]*dim, jerk=[100]*dim, dq_max=[100]*dim)


class PlayerReferenceTests(unittest.TestCase):
    def test_signed_ties_even_no_float_shortcuts(self):
        for numerator, expected in ((1, 0), (3, 2), (5, 2), (-1, 0), (-3, -2), (-5, -2)):
            self.assertEqual(player.round_even(Fraction(numerator, 2)), expected)

    def test_constant_jerk_exact_integral_one_reduction(self):
        state = player.State.from_physical([1], [2], [-3])
        jerk, cycles = player.encode_q32("4"), 7_272_727
        step = player.advance(state, [jerk], cycles, broad_bounds())
        h = Fraction(cycles, player.CLOCK_HZ)
        expected = {"q": Fraction(1)+2*h-3*h*h/2+4*h**3/6,
                    "v": Fraction(2)-3*h+2*h*h, "a": Fraction(-3)+4*h}
        for name, exact in expected.items():
            self.assertEqual(step.exact_endpoint_q32[name][0], exact*player.SCALE)
            self.assertLessEqual(abs(Fraction(getattr(step.next_state, name)[0])-exact*player.SCALE), Fraction(1, 2))

    def test_ps_metadata_clock_uses_exact_selected_denominator(self):
        clock_hz, cycles = 99_999_001, 7_272_727
        state = player.State.from_physical([-1], [2], [-3])
        jerk = player.encode_q32(-4)
        step = player.advance(state, [jerk], cycles, broad_bounds(), clock_hz=clock_hz)
        h = Fraction(cycles, clock_hz)
        expected = (-player.SCALE+2*player.SCALE*h-3*player.SCALE*h*h/2-4*player.SCALE*h**3/6,
                    2*player.SCALE-3*player.SCALE*h-2*player.SCALE*h*h,
                    -3*player.SCALE-4*player.SCALE*h)
        for name, exact in zip(("q", "v", "a"), expected):
            self.assertEqual(step.exact_endpoint_q32[name][0], exact)
            self.assertEqual(getattr(step.next_state, name)[0], player.round_even(exact))
        self.assertEqual(step.certificate["clock_hz"], clock_hz)
        default = player.advance(state, [jerk], cycles, broad_bounds())
        self.assertNotEqual(default.next_state, step.next_state)

    def test_selected_clock_changes_continuous_gate_not_only_metadata(self):
        bounds = player.Bounds.from_physical(lower=[-1], upper=[1], velocity=[2], acceleration=[2], jerk=[2], dq_max=[2])
        state = player.State.from_physical([0], [1], [0])
        self.assertEqual(player.advance(state, [0], 100_000_000, bounds).next_state.q, (player.SCALE,))
        with self.assertRaisesRegex(player.PlayerFault, "continuous q"):
            player.advance(state, [0], 100_000_000, bounds, clock_hz=99_999_001)

    def test_sign_symmetric_integration_and_gate(self):
        positive = player.State.from_physical([1], [2], [3])
        negative = player.State.from_physical([-1], [-2], [-3])
        first = player.advance(positive, [player.encode_q32(4)], 727_273, broad_bounds())
        second = player.advance(negative, [player.encode_q32(-4)], 727_273, broad_bounds())
        for name in ("q", "v", "a"):
            self.assertEqual(getattr(second.next_state, name), tuple(-value for value in getattr(first.next_state, name)))

    def test_q16_endpoints_are_inward_not_rounded_outside(self):
        bounds = player.Bounds.from_physical(lower=[0], upper=["0.1"], velocity=[1], acceleration=[1], jerk=[1], dq_max=[1])
        upper = bounds.upper[0]
        self.assertEqual(player.to_q16(upper, bounds.lower[0], upper), 6553)
        self.assertLessEqual(Fraction(6553, 65536), Fraction("0.1"))
        self.assertEqual(player.to_q16(0, 0, upper), 0)
        self.assertEqual(player.to_q16(1 << 15, -player.SCALE, player.SCALE), 0)
        self.assertEqual(player.to_q16(3 << 15, -player.SCALE, player.SCALE), 2)

    def test_q16_empty_representable_domain_rejected(self):
        with self.assertRaises(player.PlayerFault): player.to_q16(101, 100, 102)

    def test_q16_rounding_cannot_bypass_actual_output_dq_limit(self):
        bounds = player.Bounds.from_physical(lower=[-1], upper=[1], velocity=[1], acceleration=[1], jerk=[1], dq_max=["0.000001"])
        # Internal movement is only 2 Q32 LSB; crossing a Q16 half-step would
        # nevertheless create a full Q16 LSB on the transmitted position.
        state = player.State((32767,), (2,), (0,))
        with self.assertRaisesRegex(player.PlayerFault, "quantized Q16 output"):
            player.advance(state, [0], 100_000_000, bounds)

    def test_can_quantization_millidegree_jump_rejected_after_q16_passes(self):
        bounds = player.Bounds.from_physical(lower=[-1]*6+[0], upper=[1]*6+["0.1"],
            velocity=[1]*7, acceleration=[1]*7, jerk=[1]*7, dq_max=["0.0006"]*6+["0.01"])
        before, after = (32, 0, 0, 0, 0, 0, 0), (33, 0, 0, 0, 0, 0, 0)
        self.assertLessEqual(abs(after[0]-before[0])*65536, bounds.dq_max[0])
        seed_wire = player.encode_can(before, bounds)
        self.assertEqual(seed_wire[0], 0)
        self.assertEqual(player.encode_can(after, bounds)[0], 1)  # seed-only encoding has no adjacency claim.
        with self.assertRaisesRegex(player.PlayerFault, "decoded CAN adjacent DQ_MAX"):
            player.encode_can(after, bounds, previous_wire=seed_wire)

    def test_can_gripper_endpoint_remains_within_width_domain(self):
        bounds = player.Bounds.from_physical(lower=[-1]*6+[0], upper=[1]*6+["0.1"],
            velocity=[1]*7, acceleration=[1]*7, jerk=[1]*7, dq_max=[1]*7)
        gripper_q16 = player.to_q16(bounds.upper[-1], bounds.lower[-1], bounds.upper[-1])
        q16 = (0, 0, 0, 0, 0, 0, gripper_q16)
        seed = player.encode_can(q16, bounds)
        self.assertEqual(gripper_q16, 6553)
        self.assertEqual(seed[-1], 99991)
        self.assertLessEqual(Fraction(seed[-1], 1_000_000), player.decode_q32(bounds.upper[-1]))
        self.assertEqual(player.encode_can(q16, bounds, previous_wire=seed), seed)
        with self.assertRaisesRegex(player.PlayerFault, "CAN source Q16 outside"):
            player.encode_can((0, 0, 0, 0, 0, 0, 6554), bounds, previous_wire=seed)

    def test_can_endpoint_recovery_is_at_most_one_integer_step(self):
        bounds = player.Bounds.from_physical(lower=["-0.04"]+[-1]*5+[0], upper=["0.04"]+[1]*5+["0.1"],
            velocity=[1]*7, acceleration=[1]*7, jerk=[1]*7, dq_max=[1]*7)
        for sign in (1, -1):
            value = bounds.upper[0] if sign > 0 else bounds.lower[0]
            q16_axis = player.to_q16(value, bounds.lower[0], bounds.upper[0])
            nearest = player.round_even(Fraction(q16_axis*1000, 65536))
            wire = player.encode_can((q16_axis, 0, 0, 0, 0, 0, 0), bounds)
            self.assertEqual(nearest, sign*40)
            self.assertEqual(wire[0], sign*39)
            self.assertEqual(abs(wire[0]-nearest), 1)
        with self.assertRaisesRegex(player.PlayerFault, "requires six degree axes"):
            player.encode_can((0,), broad_bounds())

    def test_continuous_integer_seam_uses_committed_end_state(self):
        state = player.State.from_physical([0], [0], [0])
        first = player.advance(state, [player.encode_q32("0.1")], 727_272, broad_bounds(), expected_start=state)
        second = player.advance(first.next_state, [player.encode_q32("0.1")], 727_273, broad_bounds(), expected_start=first.next_state)
        self.assertEqual(second.certificate["dt_cycles"], 727_273)
        with self.assertRaisesRegex(player.PlayerFault, "does not match"):
            player.advance(first.next_state, [player.encode_q32("0.1")], 727_273, broad_bounds(), expected_start=state)

    def test_gate_catches_intermediate_velocity_even_if_end_returns(self):
        bounds = player.Bounds.from_physical(lower=[-100], upper=[100], velocity=[2], acceleration=[10], jerk=[100], dq_max=[100])
        state = player.State.from_physical([0], [2], [1])
        # v(t)=2+t-5t² violates 2 at t=.1 but v(1)=-2 is within bounds.
        with self.assertRaisesRegex(player.PlayerFault, "continuous v"):
            player.advance(state, [player.encode_q32(-10)], 100_000_000, bounds)

    def test_gate_catches_outward_acceleration_at_position_boundary(self):
        bounds = player.Bounds.from_physical(lower=[-1], upper=[1], velocity=[10], acceleration=[10], jerk=[100], dq_max=[2])
        for sign in (1, -1):
            state = player.State.from_physical([sign], [0], [sign])
            with self.assertRaisesRegex(player.PlayerFault, "continuous q"):
                player.advance(state, [player.encode_q32(-30*sign)], 10_000_000, bounds)

    def test_jerk_and_dq_fault_leave_original_state_unchanged(self):
        state = player.State.from_physical([0], [0], [0])
        with self.assertRaisesRegex(player.PlayerFault, "jerk bound"):
            player.advance(state, [player.encode_q32(101)], 1, broad_bounds())
        bounds = player.Bounds.from_physical(lower=[-100], upper=[100], velocity=[100], acceleration=[100], jerk=[100], dq_max=["0.01"])
        with self.assertRaisesRegex(player.PlayerFault, "DQ_MAX"):
            player.advance(player.State.from_physical([0], [1], [0]), [0], 10_000_000, bounds)
        self.assertEqual(state, player.State.from_physical([0], [0], [0]))

    def test_outward_envelopes_include_exact_polynomial_samples(self):
        rng = random.Random(72)
        for _ in range(50):
            state = player.State.from_physical([rng.uniform(-1, 1)], [rng.uniform(-1, 1)], [rng.uniform(-1, 1)])
            jerk = player.encode_q32(rng.uniform(-1, 1))
            cycles = rng.randrange(1, 1_000_001)
            step = player.advance(state, [jerk], cycles, broad_bounds())
            h = Fraction(cycles, player.CLOCK_HZ)
            certificate = step.certificate["axes"][0]
            for t_index in range(31):
                t = h*Fraction(t_index, 30)
                q = state.q[0]+state.v[0]*t+state.a[0]*t*t/2+jerk*t**3/6
                v = state.v[0]+state.a[0]*t+jerk*t*t/2
                a = state.a[0]+jerk*t
                for name, value in (("q", q), ("v", v), ("a", a)):
                    enclosures = certificate[name+"_enclosure"]
                    self.assertGreaterEqual(value, min(pair[0] for pair in enclosures))
                    self.assertLessEqual(value, max(pair[1] for pair in enclosures))

    def test_fixed_point_overflow_invalid_dt_and_dimensions_reject(self):
        with self.assertRaises(player.PlayerFault): player.State([1 << 63], [0], [0])
        state = player.State.from_physical([0], [0], [0])
        for cycles in (0, -1, 1 << 32, True):
            with self.assertRaises(player.PlayerFault): player.advance(state, [0], cycles, broad_bounds())
        with self.assertRaises(player.PlayerFault): player.advance(state, [0, 0], 1, broad_bounds())

    def test_width_bound_covers_all_int64_inputs_uint32_dt(self):
        widths = player.width_requirements()["signed_bits"]
        self.assertEqual(widths, {"q_numerator": 161, "v_numerator": 129, "a_numerator": 97})
        self.assertLess(max(widths.values()), 192)
        for clock_hz in (99_999_001, player.MAX_CLOCK_HZ):
            profile = player.width_requirements(clock_hz=clock_hz)
            self.assertEqual(profile["clock_hz"], clock_hz)
            self.assertLess(max(profile["signed_bits"].values()), 192)
        self.assertEqual(player.width_requirements(clock_hz=player.MAX_CLOCK_HZ)["signed_bits"],
                         {"q_numerator": 163, "v_numerator": 130, "a_numerator": 97})

    def test_invalid_clock_parameters_rejected_by_reference_and_generators(self):
        state = player.State.from_physical([0], [0], [0])
        for invalid in (0, -1, True, 1.5, 1 << 31):
            with self.assertRaises(player.PlayerFault):
                player.advance(state, [0], 1, broad_bounds(), clock_hz=invalid)
            with self.assertRaises(player.PlayerFault): player.width_requirements(clock_hz=invalid)
            with self.assertRaises(player.PlayerFault): math_vectors.cases(clock_hz=invalid)
            with self.assertRaises(player.PlayerFault): gate_vectors.generate(1, 1, clock_hz=invalid)

    def test_odd_clock_generator_endpoints_and_signed_half_ties(self):
        clock_hz = 99_999_001
        cases = math_vectors.cases(random_cases=0, clock_hz=clock_hz)
        case = next(case for case in cases if case[0] == "half_even_components")
        _, dt, q, v, a, jerk = case
        qo, vo, ao, overflow = math_vectors.expected(dt, q, v, a, jerk, clock_hz=clock_hz)
        self.assertEqual(dt, clock_hz)
        self.assertEqual(qo[:4], [0, 2, 0, -2])
        self.assertEqual(vo[-3:], [0, 2, -2])
        self.assertEqual(overflow, 0)
        qe, ve, ae = gate_vectors.endpoints(q, v, a, jerk, dt, clock_hz=clock_hz)
        self.assertEqual((tuple(qo), tuple(vo), tuple(ao)), (qe, ve, ae))
        profile_rows, counts = gate_vectors.generate(2601102, 20, clock_hz=clock_hz)
        default_rows, _ = gate_vectors.generate(2601102, 20)
        self.assertNotEqual(profile_rows, default_rows)
        self.assertEqual(counts["allow"]+counts["reject"], 20)

    def test_generator_cli_clock_profile_is_recorded(self):
        import json
        with tempfile.TemporaryDirectory() as temporary:
            temporary = Path(temporary)
            for script, count_option, output, manifest in (
                ("make_player_vectors.py", "--random-cases", temporary/"math.mem", temporary/"math.json"),
                ("make_gate_vectors.py", "--count", temporary/"gate.mem", temporary/"gate.mem.json"),
            ):
                result = subprocess.run([sys.executable, str(ROOT/"sim"/script), "--output", str(output),
                    count_option, "2", "--clock-hz", "99999001"], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(json.loads(manifest.read_text())["clock_hz"], 99_999_001)

    def test_cli_help_and_design_example_have_no_physical_frequency(self):
        script = ROOT/"sim"/"player_reference.py"
        help_result = subprocess.run([sys.executable, str(script), "--help"], capture_output=True, text=True)
        self.assertEqual(help_result.returncode, 0)
        demo = subprocess.run([sys.executable, str(script), "--design-example", "--dt-cycles", "727273", "--steps", "2"], capture_output=True, text=True)
        self.assertEqual(demo.returncode, 0, demo.stderr)
        import json
        result = json.loads(demo.stdout)
        self.assertEqual(result["provenance"], "design_only")
        self.assertIsNone(result["physical_f_exec_hz"])
        profile = subprocess.run([sys.executable, str(script), "--design-example", "--dt-cycles", "727273",
            "--steps", "1", "--clock-hz", "99999001"], capture_output=True, text=True)
        self.assertEqual(profile.returncode, 0, profile.stderr)
        selected = json.loads(profile.stdout)
        self.assertEqual(selected["width_requirements"]["clock_hz"], 99_999_001)
        self.assertEqual(selected["segments"][0]["certificate"]["clock_hz"], 99_999_001)


if __name__ == "__main__": unittest.main()
