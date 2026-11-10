import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from check_kr260 import distribution, require_ppp_route, match_boxes


class PppWorkflowTests(unittest.TestCase):
    def test_multiobject_order_change_and_ambiguity(self):
        a={'class':'object','source':'depth_component','box_xywh':[10,10,30,30]}
        b=dict(a,box_xywh=[100,100,30,30])
        self.assertEqual(match_boxes([a,b],[b,a]),{0:1,1:0})
        self.assertEqual(match_boxes([a,a],[a]),{})
        self.assertEqual(match_boxes([a],[a,a]),{})
        self.assertEqual(match_boxes([a],[dict(a,**{'class':'cup'})]),{})
        self.assertEqual(match_boxes([a],[b]),{})

    def test_missing_ppp_does_not_fall_back_to_proxy(self):
        routes=[[],[{'dev':'Meta','gateway':'198.18.0.2'}],[{'dev':'wlp98s0'}]]
        for route in routes:
            result=subprocess.CompletedProcess([],0,stdout=json.dumps(route))
            with patch('check_kr260.subprocess.run',return_value=result):
                with self.assertRaisesRegex(RuntimeError,'PPP route is absent'):
                    require_ppp_route('192.168.28.2')

    def test_existing_ppp_route_is_accepted(self):
        result=subprocess.CompletedProcess([],0,stdout='[{"dev":"ppp0"}]')
        with patch('check_kr260.subprocess.run',return_value=result):
            require_ppp_route('192.168.28.2')

    def test_one_sample_is_not_evidence_of_zero_noise(self):
        self.assertIsNone(distribution([]))
        self.assertIsNone(distribution([285])['stddev'])
        result=distribution([293,295,None,float('nan')])
        self.assertEqual(result['count'],2)
        self.assertEqual(result['median'],294)
        self.assertEqual(result['stddev'],1)


if __name__=='__main__':
    unittest.main()
