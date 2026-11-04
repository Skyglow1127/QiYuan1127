import importlib.util
import io
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('d1_joint_server', Path(__file__).with_name('d1_joint_server.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.state = module.FeedbackState(clock=lambda: self.now, wall=lambda: 1000+self.now)

    def feed(self, data, stamp=None):
        self.state.feed(dict(received_monotonic_s=self.now if stamp is None else stamp,
                             message=dict(data=data)))

    def angles(self, j6=393.3):
        return {f'angle{i}': i*.1 if i < 6 else j6 for i in range(7)}

    def test_initial_is_unavailable_without_fabricated_zero(self):
        result = self.state.snapshot()
        self.assertFalse(result['fresh'])
        self.assertTrue(all(x['position'] is None for x in result['joints']))
        self.assertIsNone(result['power'])

    def test_units_raw_feedback_and_independent_status(self):
        self.feed(self.angles())
        self.feed(dict(power_status=1, enable_status=1, error_status=0))
        result = self.state.snapshot()
        self.assertTrue(result['fresh'])
        self.assertTrue(result['status_fresh'])
        self.assertEqual([x['unit'] for x in result['joints']], ['deg']*6+['raw'])
        self.assertEqual(result['joints'][6]['position'], 393.3)
        self.assertEqual(result['power'], 1)
        self.assertEqual(result['received_at_unix_ms'], 1100000.)

    def test_status_traffic_does_not_refresh_stale_angles(self):
        self.feed(self.angles())
        self.now += 2
        self.feed(dict(power_status=1, enable_status=1, error_status=0))
        result = self.state.snapshot()
        self.assertFalse(result['fresh'])
        self.assertTrue(result['status_fresh'])
        self.assertEqual(result['age_ms'], 2000)
        self.assertIsNone(result['joints'][6]['position'])
        self.assertEqual(result['joints'][6]['last_position'], 393.3)

    def test_reader_exit_invalidates_even_recent_values(self):
        self.feed(self.angles())
        self.feed(dict(power_status=1, enable_status=1, error_status=0))
        result = self.state.snapshot(reader_alive=False)
        self.assertFalse(result['fresh'])
        self.assertEqual(result['unavailable_reason'], 'reader_stopped')
        self.assertIsNone(result['power'])

    def test_delayed_pipe_or_old_packet_cannot_refresh(self):
        self.feed(self.angles(), stamp=self.now-2)
        self.assertFalse(self.state.snapshot()['fresh'])
        self.feed(self.angles(35.9))
        self.feed(self.angles(25), stamp=self.now-1)
        self.assertEqual(self.state.snapshot()['joints'][6]['position'], 35.9)

    def test_partial_nonfinite_boolean_and_future_data_rejected(self):
        self.feed(dict(angle0=1))
        bad = self.angles(); bad['angle3'] = float('nan'); self.feed(bad)
        bad = self.angles(); bad['angle0'] = True; self.feed(bad)
        self.feed(self.angles(), stamp=self.now+10)
        self.assertFalse(self.state.snapshot()['fresh'])
        self.assertEqual(self.state.snapshot()['counters']['invalid'], 4)

    def test_ack_does_not_create_angles_or_status(self):
        self.feed(dict(exec_status=1))
        result = self.state.snapshot()
        self.assertFalse(result['fresh'])
        self.assertFalse(result['status_fresh'])

    def test_proxy_routes_are_fixed_local_camera_reads_only(self):
        self.assertEqual(module.proxy_target('/camera/global/status'), (18761, '/status'))
        self.assertEqual(module.proxy_target('/camera/wrist/color.mjpg'), (18760, '/color.mjpg'))
        for path in ('/camera/global/../../control', '/camera/global/handeye-snapshot',
                     '/camera/other/status', '/camera/global/http://example.com', '/camera/global/status/extra'):
            self.assertIsNone(module.proxy_target(path))

    def test_stream_is_relayed_in_bounded_chunks(self):
        class Stream:
            def __init__(self):
                self.chunks = [b'header', b'frame', b'']
            def read1(self, size):
                self.size = size
                return self.chunks.pop(0)
        source, sink = Stream(), io.BytesIO()
        module.relay_chunks(source, sink)
        self.assertEqual(sink.getvalue(), b'headerframe')
        self.assertEqual(source.size, 65536)


if __name__ == '__main__':
    unittest.main()
