import copy
import csv
import io
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import Recorder, configuration, FIELDS
from board_reader import verify_pl_mapping, ReaderError
from tactile import Debouncer, integration_view
from tactile_client import TactileClient


def conf(**overrides):
    data = dict(source='ssh', host='TEST_ONLY', chip='auto', offsets=[0, 1], rate_hz=100,
                polarity=['low', 'low'], mapping_confirmed=True, sensor_connected=True,
                channel_verified=[True, True], debounce_ms=30, stale_ms=250)
    data.update(overrides)
    return configuration(data)


class ContactTests(unittest.TestCase):
    def setUp(self):
        self.r = Recorder('/nonexistent-test-only')
        self.r.config = conf()
        self.r.csv_file = io.StringIO()
        self.r.writer = csv.DictWriter(self.r.csv_file, fieldnames=FIELDS)
        self.r.accept_pl_mapping(dict(type='pl_mapping', verified=True, chip='/dev/gpiochip7',
                                      offsets=[0, 1], of_node='/axi/gpio@a0000000'))

    def sample(self, seq, t, a=1, b=1):
        with patch('app.time.monotonic', return_value=100+t):
            self.r.ingest(dict(seq=seq, t_s=t, wall_time='TEST_ONLY', source='real',
                               chip='/dev/gpiochip7', offsets=[0, 1], do1=a, do2=b))

    def view(self, t):
        with patch('app.time.monotonic', return_value=100+t): return self.r.tactile_status()

    def stable(self, a=1, b=1):
        for i in range(5): self.sample(i+1, i*.01, a, b)

    def test_unplugged_never_reports_contact(self):
        self.r.config['sensor_connected'] = False
        self.stable(0, 0)
        v = self.view(.04)
        self.assertFalse(v['valid']); self.assertEqual(v['decision'], 'HOLD')
        self.assertIsNone(v['channels'][0]['contact'])

    def test_verified_low_active_through_pl_ps(self):
        self.stable(0, 1)
        v = self.view(.04)
        self.assertTrue(v['valid']); self.assertTrue(v['through_pl_ps'])
        self.assertEqual(v['decision'], 'CONTACT'); self.assertTrue(v['stop_closing'])
        self.assertFalse(v['both_contact']); self.assertFalse(v['grasp_verified'])

    def test_high_polarity_and_both_contact(self):
        self.r.config['polarity'] = ['high', 'high']
        self.stable(1, 1)
        self.assertTrue(self.view(.04)['both_contact'])

    def test_raw_contact_stops_closure_before_debounce(self):
        self.stable()
        self.sample(6, .05, 0, 1)
        v = self.view(.05)
        self.assertFalse(v['channels'][0]['contact'])
        self.assertTrue(v['channels'][0]['raw_contact'])
        self.assertTrue(v['stop_closing'])

    def test_bounce_does_not_create_stable_contact(self):
        self.stable()
        self.sample(6, .05, 0); self.sample(7, .06, 1)
        self.assertFalse(self.view(.06)['channels'][0]['contact'])

    def test_gap_resets_stability(self):
        self.stable(0, 0)
        self.sample(6, .2, 0, 0)
        self.assertEqual(self.r.latest['gap'], 1)
        self.assertFalse(self.view(.2)['valid'])

    def test_stale_or_stopped_hold(self):
        self.stable()
        self.assertEqual(self.view(.04)['decision'], 'NO_CONTACT')
        self.assertEqual(self.view(.5)['decision'], 'HOLD')
        self.r.state = 'stopped'
        self.assertEqual(self.view(.04)['decision'], 'HOLD')

    def test_unknown_or_unverified_channel_holds(self):
        self.r.config['polarity'][1] = 'unknown'; self.stable()
        self.assertFalse(self.view(.04)['valid'])
        self.r.config['polarity'][1] = 'low'; self.r.config['channel_verified'][1] = False
        self.assertFalse(self.view(.04)['valid'])

    def test_no_pl_handshake_or_wrong_order_rejected(self):
        self.r.pl_mapping = None
        with self.assertRaisesRegex(ValueError, 'PL'): self.sample(1, 0)
        with self.assertRaises(ValueError):
            self.r.accept_pl_mapping(dict(type='pl_mapping', verified=True, chip='/dev/gpiochip7',
                                          offsets=[1, 0], of_node='/axi/gpio@a0000000'))

    def test_queue_lag_invalidates_robot_values(self):
        self.stable()
        with patch('app.time.monotonic', return_value=101):
            self.r.ingest(dict(seq=6,t_s=.05,wall_time='TEST_ONLY',source='real',chip='/dev/gpiochip7',offsets=[0,1],do1=0,do2=0))
            v=self.r.tactile_status()
        self.assertFalse(v['fresh']); self.assertTrue(v['stop_closing'])

    def test_demo_cannot_control_robot(self):
        self.stable()
        payload=self.view(.04); payload['source']='simulated'
        self.assertEqual(integration_view(payload)['decision'], 'HOLD')

    def test_consumer_rejects_malformed_channels_and_age(self):
        self.stable(); payload=self.view(.04)
        for change in ({'receive_age_ms':float('nan')}, {'channels':[]}, {'valid':False}, {'through_pl_ps':False}):
            v=copy.deepcopy(payload); v.update(change)
            self.assertEqual(integration_view(v)['decision'], 'HOLD')

    def test_client_never_reuses_cached_value_on_failure(self):
        self.stable(); payload=self.view(.04); payload['session_id']='TEST_ONLY_SESSION'
        client=TactileClient()
        def response(*a, **kw): return io.BytesIO(json.dumps(payload).encode())
        with patch('tactile_client.urlopen', side_effect=response):
            self.assertEqual(client.read()['decision'], 'NO_CONTACT')
            self.assertEqual(client.read()['decision'], 'HOLD')
        with patch('tactile_client.urlopen', side_effect=TimeoutError('timeout')):
            self.assertTrue(client.read()['stop_closing'])

    def test_parameter_validation(self):
        for v in ({'debounce_ms':float('nan')}, {'stale_ms':5}, {'channel_verified':[1,1]}, {'rate_hz':1}, {'enabled_channels':[False,False]}):
            with self.assertRaises(ValueError): conf(**v)

    def test_do1_only_ignores_unconnected_do2(self):
        self.r.config['enabled_channels']=[True,False]
        self.r.config['channel_verified']=[True,False]
        self.r.config['polarity']=['low','unknown']
        self.stable(1,0)
        v=self.view(.04)
        self.assertTrue(v['valid']); self.assertEqual(v['decision'],'NO_CONTACT')
        self.assertIsNone(v['both_contact']); self.assertIsNone(v['channels'][1]['contact'])

    def test_do2_only_ignores_unconnected_do1(self):
        self.r.config['enabled_channels']=[False,True]
        self.r.config['channel_verified']=[False,True]
        self.r.config['polarity']=['unknown','low']
        self.stable(0,0)
        v=self.view(.04)
        self.assertTrue(v['valid']); self.assertEqual(v['decision'],'CONTACT')
        self.assertIsNone(v['both_contact']); self.assertIsNone(v['channels'][0]['contact'])

    def test_debouncer_zero_and_unknown(self):
        d=Debouncer(); self.assertEqual(d.update(1,0,0),1)
        self.assertIsNone(d.update(None,.01,30))


class MappingTests(unittest.TestCase):
    def fixture(self, root):
        dt=root/'dt'; node=dt/'axi/gpio@a0000000'; node.mkdir(parents=True)
        region=dt/'fpga-region'; region.mkdir(); (region/'firmware-name').write_bytes(b'kr260-fsr.bit.bin\0')
        (node/'compatible').write_bytes(b'xlnx,xps-gpio-1.00.a\0')
        (node/'gpio-controller').touch(); (node/'gpio-line-names').write_bytes(b'FSR_DO1\0FSR_DO2\0')
        values={'reg':[0,0xa0000000,0,0x10000],'xlnx,gpio-width':[2],'xlnx,all-inputs':[1],'xlnx,is-dual':[0],'#gpio-cells':[2]}
        for key,v in values.items(): (node/key).write_bytes(struct.pack('>'+'I'*len(v),*v))
        chip=root/'gpio/gpiochip7'; chip.mkdir(parents=True); (chip/'of_node').symlink_to(node)
        (chip/'dev').write_text('0:0'); (root/'dev').mkdir(); (root/'dev/gpiochip7').touch()
        return dt,node

    def test_device_tree_and_device_match_required(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); dt,node=self.fixture(root)
            args=dict(dt_root=dt,gpio_root=root/'gpio',dev_root=root/'dev')
            # Regular files stand in for GPIO character devices only within this test.
            with patch('board_reader.stat.S_ISCHR', return_value=True):
                self.assertTrue(verify_pl_mapping(**args)['verified'])
                (node/'xlnx,all-inputs').write_bytes(struct.pack('>I',0))
                with self.assertRaises(ReaderError): verify_pl_mapping(**args)

    def test_missing_pl_is_not_replaced_by_ps_gpio(self):
        with tempfile.TemporaryDirectory() as td:
            with self.assertRaisesRegex(ReaderError,'PL AXI GPIO'):
                verify_pl_mapping(dt_root=td, gpio_root=td, dev_root=td)


if __name__ == '__main__': unittest.main()
