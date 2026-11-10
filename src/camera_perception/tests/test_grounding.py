import importlib.util
import unittest
from pathlib import Path

spec = importlib.util.spec_from_file_location('grounding',Path(__file__).resolve().parents[1]/'scripts/qwen_target.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class GroundingTests(unittest.TestCase):
    def test_coordinates(self):
        self.assertEqual(module.parse_box('{"found":true,"bbox_format":"xyxy_1000","box":[100,200,300,400]}'),[100,200,300,400])
        self.assertIsNone(module.parse_box('{"found":false}'))
        self.assertEqual(module.parse_box('{"found":true,"bbox_2d":[100,200,300,400]}'),[100,200,300,400])
        self.assertEqual(module.parse_box('```json\n[{"bbox_2d":[100,200,300,400],"label":"bottle"}]\n```'),[100,200,300,400])
        self.assertIsNone(module.parse_box('[]'))
        with self.assertRaises(ValueError):
            module.parse_box('[{"bbox_2d":[0,0,10,10]},{"bbox_2d":[20,20,30,30]}]')
        for box in ('[0,0,1001,20]','[20,20,10,10]','[0,0,NaN,10]','[false,0,10,10]'):
            with self.assertRaises(ValueError):
                module.parse_box('{"found":true,"bbox_format":"xyxy_1000","box":'+box+'}')
        with self.assertRaises(ValueError):
            module.parse_box('{"found":true,"box":[0,0,100,100]}')

    def test_selection(self):
        candidate={'box_xywh':[64,96,128,96], 'status':'ok','valid':1,'depth_measurement_valid':1}
        box=[100,200,300,400]
        self.assertEqual(module.match_candidate(box,[candidate],640,480)[:2],(0,'matched_geometric_candidate'))
        self.assertEqual(module.match_candidate(box,[candidate,candidate],640,480)[1],'ambiguous_candidates')
        excluded=dict(candidate,status='class_excluded_from_grasp',valid=0)
        self.assertEqual(module.match_candidate(box,[excluded],640,480)[1],'excluded_or_conflicting_candidate')
        missing=dict(candidate,depth_measurement_valid=0)
        self.assertEqual(module.match_candidate(box,[missing],640,480)[1],'candidate_has_no_measured_depth')
        depth_only=dict(candidate,valid=0,status='table_not_found')
        self.assertEqual(module.match_candidate(box,[depth_only],640,480)[:2],(0,'matched_depth_only'))
        self.assertEqual(module.match_candidate([0,0,50,50],[candidate],640,480)[1],'no_matching_visual_candidate')

if __name__ == '__main__':
    unittest.main()
