import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('compare_grasp_speed_profiles',
                                             Path(__file__).with_name('compare_grasp_speed_profiles.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def saved(values, times=None):
    q = np.zeros((len(values), 6)); q[:, 0] = values
    times = np.arange(len(q))*.2 if times is None else np.array(times)
    plan = module.path_timing.retime(q, times, 5., .5, 1.)
    return dict(sdk_plan=dict(sdk=q.tolist(), rate=5., **plan))


class SpeedComparisonTests(unittest.TestCase):
    def test_short_stopped_edges_are_acceleration_limited(self):
        result = module.compare_saved(saved([0, .1, 0]), module.BASE_PROFILES)
        times = [p['reference_duration_s'] for p in result['profiles']]
        self.assertEqual(times, [times[0]]*3)
        self.assertTrue(all(p['stopped_runs'] == 2 for p in result['profiles']))

    def test_long_path_can_benefit_from_velocity_change(self):
        result = module.compare_saved(saved([0, 10]), module.BASE_PROFILES)
        times = [p['reference_duration_s'] for p in result['profiles']]
        self.assertGreater(times[0], times[1])
        self.assertGreater(times[1], times[2])

    def test_acceleration_change_does_not_remove_stops(self):
        profiles = module.BASE_PROFILES+[('v2_a2', 2., 2.)]
        result = module.compare_saved(saved([0, .1, 0, .1]), profiles)
        self.assertLess(result['profiles'][-1]['reference_duration_s'], result['profiles'][0]['reference_duration_s'])
        self.assertTrue(all(p['internal_stop_count'] == 2 for p in result['profiles']))

    def test_collinear_internal_points_and_dwell_preserved(self):
        result = module.compare_saved(saved([0, .1, .2, .2], [0, .2, .4, 2.4]), module.BASE_PROFILES)
        self.assertEqual(result['preserved_stationary_edges'], 1)
        self.assertTrue(all(p['continuous_internal_points'] == 1 for p in result['profiles']))

    def test_unknown_historical_time_floor_is_not_silently_discarded(self):
        with self.assertRaisesRegex(ValueError, 'not reproduced'):
            module.compare_saved(saved([0, .1], [0, 10]), module.BASE_PROFILES)

    def test_analytical_peaks_bound_dense_existing_profile_samples(self):
        for distance in [.01, .1, 1., 10.]:
            run = module.path_timing._profile(distance, .2, 5., 2., 2.)
            vmax, amax = module.analytical_peaks(run)
            samples = np.array([module.path_timing.scalar_state(run, t)
                                for t in np.linspace(0, run['duration_s'], 2001)])
            self.assertLessEqual(np.max(abs(samples[:, 1])), vmax+1e-9)
            self.assertLessEqual(np.max(abs(samples[:, 2])), amax+1e-9)
            self.assertLess(abs(np.max(abs(samples[:, 1]))-vmax), 1e-4)
            self.assertLess(abs(np.max(abs(samples[:, 2]))-amax), 1e-4)

    def test_output_has_no_executable_geometry(self):
        result = module.compare_saved(saved([0, .1]), module.BASE_PROFILES)
        text = json.dumps(result)
        self.assertNotIn('"sdk"', text)
        self.assertNotIn('"runs"', text)
        self.assertTrue(all(p['executable'] is False and p['hardware_verified'] is False for p in result['profiles']))


if __name__ == '__main__':
    unittest.main()
