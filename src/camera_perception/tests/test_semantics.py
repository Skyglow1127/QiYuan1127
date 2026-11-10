"""Semantic/depth boundaries: ambiguity, stale frames, and missing-depth rejection."""
import hashlib
import json
import re
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from grasp_semantics import parse_objects, pixel_box, associate_masks, color_consensus_prompt, requested_color_evidence, duplicate_inventory_masks
from qwen_scene import freeze_snapshot
import qwen_llama


class RuntimeSelectionTests(unittest.TestCase):
    def test_auto_falls_back_when_gpu_startup_fails(self):
        expected = mock.Mock()
        with mock.patch('os.access',return_value=True), mock.patch.object(Path,'is_file',return_value=True), \
             mock.patch.object(qwen_llama,'LlamaRuntime',side_effect=[RuntimeError('GPU unavailable'),expected]) as constructor:
            actual = qwen_llama.create_llama_runtime('model.gguf',device='auto')
        self.assertIs(actual,expected)
        self.assertEqual(actual.startup_fallback_reason,'GPU unavailable')
        self.assertEqual([call.kwargs['device'] for call in constructor.call_args_list],['vulkan','cpu'])

    def test_explicit_gpu_failure_is_not_silently_replaced(self):
        with mock.patch.object(qwen_llama,'LlamaRuntime',side_effect=RuntimeError('GPU unavailable')) as constructor:
            with self.assertRaisesRegex(RuntimeError,'GPU unavailable'):
                qwen_llama.create_llama_runtime('model.gguf',device='vulkan')
        self.assertEqual(constructor.call_count,1)


class ProposalTests(unittest.TestCase):
    def test_explicit_color_requires_image_evidence(self):
        red = np.full((30,30,3),(0,0,180),np.uint8)
        blue = np.full_like(red,(180,0,0)); mask = np.full((30,30),255,np.uint8)
        self.assertEqual(requested_color_evidence('红色方块',red,mask)['status'],'supported')
        self.assertEqual(requested_color_evidence('the red cube',blue,mask)['status'],'requested_color_mismatch')
        self.assertEqual(requested_color_evidence('blue box',blue,mask)['status'],'supported')
        self.assertEqual(requested_color_evidence('a bottle with a red label',blue,mask)['status'],'complex_description_not_checked')
        self.assertEqual(requested_color_evidence('红蓝相间的盒子',red,mask)['status'],'no_single_supported_color')
        self.assertEqual(requested_color_evidence(None,blue,mask)['status'],'not_requested')

    def test_strict_semantic_output(self):
        good = '[{"bbox_2d":[100,200,300,400],"label":"red block"}]'
        self.assertEqual(parse_objects('```json\n'+good+'\n```')[0]['label'], 'red block')
        self.assertEqual(parse_objects('[]'), [])
        for text in [good[:-1], '{"objects":[]}', good.replace('100,', 'true,'),
                     good.replace('100,', 'NaN,'), good.replace('300,', '1001,'),
                     good.replace('red block', ''), good.replace('300,400', '50,400')]:
            with self.subTest(text=text), self.assertRaises((ValueError, json.JSONDecodeError)):
                parse_objects(text)
        self.assertEqual(pixel_box([0, 0, 1000, 1000], 640, 480), [0, 0, 640, 480])

    def test_mask_association_requires_unique_physical_support(self):
        left = np.zeros((40, 80), np.uint8); left[10:20, 10:20] = 255
        right = np.zeros_like(left); right[10:20, 50:60] = 255
        candidates = [{'status': 'incomplete_depth_contour', 'depth_measurement_valid': True}]*2
        matches = associate_masks([left, right], [left, right], candidates)
        self.assertEqual([m['candidate_index'] for m in matches], [0, 1])
        self.assertEqual(associate_masks([left|right], [left, right], candidates)[0]['status'], 'multiple_depth_objects_in_mask')
        duplicate = associate_masks([left, left], [left, right], candidates)
        self.assertTrue(all(m['candidate_index'] is None for m in duplicate))
        self.assertEqual(associate_masks([left], [None], candidates[:1])[0]['status'], 'no_measured_object_support')
        excluded = [{'status': 'class_excluded_from_grasp', 'depth_measurement_valid': True}]
        self.assertIsNone(associate_masks([left], [left], excluded)[0]['candidate_index'])
        self.assertEqual(duplicate_inventory_masks([left,left],[left,right],candidates),{0,1})
        self.assertEqual(duplicate_inventory_masks([left,right],[left,right],candidates),set())
        self.assertEqual(duplicate_inventory_masks([left|right,left|right],[left,right],candidates),set())

    def test_color_consensus_never_truncates_a_tiny_depth_fragment(self):
        color = np.zeros((100,100),np.uint8); color[20:80,20:65]=255
        seed = np.zeros_like(color); seed[30:70,30:60]=255
        leaked = color.copy(); leaked[25:45,65:80]=255
        prompt = color_consensus_prompt(leaked,color,seed)
        self.assertIsNotNone(prompt)
        positive,negative = prompt['points']
        self.assertTrue(seed[positive[1],positive[0]])
        self.assertFalse(color[negative[1],negative[0]])
        tiny = np.zeros_like(color); tiny[30:40,30:40]=255
        self.assertIsNone(color_consensus_prompt(leaked,tiny,tiny))


class MaskDepthIntegration(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        color = np.full((48, 64, 3), 120, np.uint8)
        color[16:32, 24:40] = (0, 0, 220)
        self.depth = np.full((48, 64), 1000, np.uint16)
        self.depth[16:32, 24:40] = 850
        self.mask = np.zeros((48, 64), np.uint8); self.mask[16:32, 24:40] = 255
        cv2.imwrite(str(self.root/'color.png'), color)
        cv2.imwrite(str(self.root/'depth.png'), self.depth)
        cv2.imwrite(str(self.root/'mask.png'), self.mask)
        f = cv2.FileStorage(str(self.root/'snapshot.yml'), cv2.FILE_STORAGE_WRITE)
        values = {'color':'color.png', 'depth':'depth.png', 'depth_unit_m':.001, 'aligned_to_color':1,
                  'width':64, 'height':48, 'host_received_ms':1000., 'depth_host_received_ms':1000.,
                  'color_device_us':1000000., 'depth_device_us':1000000.}
        for key, value in values.items(): f.write(key, value)
        f.write('camera_matrix', np.array([[200.,0,32.],[0,200.,24.],[0,0,1.]]))
        f.write('distortion', np.zeros((1,5), np.float64)); f.release()
        (self.root/'config.yml').write_text('%YAML:1.0\n---\ndetection_mode: "tabletop"\nmax_gripper_opening_m: 0.5\n')
        self.write_proposals()

    def tearDown(self):
        self.temp.cleanup()

    def write_proposals(self):
        sha = lambda name: hashlib.sha256((self.root/name).read_bytes()).hexdigest()
        data = {'source_snapshot':'snapshot.yml', 'source_snapshot_sha256':sha('snapshot.yml'),
                'source_color_sha256':sha('color.png'), 'source_depth_sha256':sha('depth.png'),
                'objects':[{'label':'red block', 'mask':'mask.png', 'mask_sha256':sha('mask.png')}]}
        (self.root/'proposals.json').write_text(json.dumps(data))

    def run_geometry(self,replay=None):
        binary = ROOT/'build/mask_depth'
        if not binary.exists(): self.skipTest('Build mask_depth before integration tests')
        return subprocess.run([str(binary), '--replay', str(replay or self.root/'snapshot.yml'),
            '--config', str(self.root/'config.yml'), '--masks', str(self.root/'proposals.json'),
            '--output', str(self.root/'output')], capture_output=True, text=True)

    def test_moving_observation_cannot_produce_metric_grasp(self):
        with (self.root/'snapshot.yml').open('a') as f:
            f.write('motion_guard: 1\nvisual_motion_state: moving\n')
        self.write_proposals()
        result = self.run_geometry()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('visual motion is not trusted', result.stderr)

    def test_measured_depth_is_not_language_model_distance(self):
        result = self.run_geometry()
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads((self.root/'output/mask_depth_result.json').read_text())
        candidate = data['candidates'][0]
        self.assertTrue(candidate['depth_measurement_valid'])
        self.assertAlmostEqual(candidate['optical_axis_depth_m'], .85, places=5)
        self.assertFalse(data['motion_command_ready'])
        self.assertFalse(data['base_transform_valid'])

    def test_missing_boundary_cannot_become_valid_grasp(self):
        self.depth[14:34, 22:42] = 0
        self.depth[18:30, 26:38] = 850
        cv2.imwrite(str(self.root/'depth.png'), self.depth); self.write_proposals()
        result = self.run_geometry()
        self.assertEqual(result.returncode, 0, result.stderr)
        candidate = json.loads((self.root/'output/mask_depth_result.json').read_text())['candidates'][0]
        self.assertFalse(candidate['valid'])
        self.assertEqual(candidate['status'], 'incomplete_depth_contour')
        self.assertAlmostEqual(candidate['optical_axis_depth_m'], .85, places=5)

    def test_stale_image_and_nonbinary_masks_rejected(self):
        original = (self.root/'color.png').read_bytes()
        cv2.imwrite(str(self.root/'color.png'), np.zeros((48,64,3), np.uint8))
        self.assertNotEqual(self.run_geometry().returncode, 0)
        (self.root/'color.png').write_bytes(original)
        self.mask[17,25] = 1
        cv2.imwrite(str(self.root/'mask.png'), self.mask); self.write_proposals()
        self.assertNotEqual(self.run_geometry().returncode, 0)

    def test_same_pixels_with_different_frame_time_rejected(self):
        metadata = (self.root/'snapshot.yml').read_text()
        changed = re.sub(r'^host_received_ms:.*$', 'host_received_ms: 2000.',metadata,flags=re.MULTILINE)
        self.assertNotEqual(metadata,changed)
        alternate = self.root/'other_frame.yml'; alternate.write_text(changed)
        self.assertNotEqual(self.run_geometry(alternate).returncode,0)

    def test_image_edge_is_not_a_complete_grasp_outline(self):
        self.mask[20:25,:24] = 255
        cv2.imwrite(str(self.root/'mask.png'),self.mask); self.write_proposals()
        result = self.run_geometry()
        self.assertEqual(result.returncode,0,result.stderr)
        candidate=json.loads((self.root/'output/mask_depth_result.json').read_text())['candidates'][0]
        self.assertFalse(candidate['valid'])
        self.assertEqual(candidate['status'],'appearance_cut_by_image_edge')

    def test_freeze_is_independent_of_later_source_changes(self):
        frozen, color, provenance = freeze_snapshot(self.root/'snapshot.yml', self.root/'frozen')
        self.assertTrue(frozen.is_file())
        saved = color.read_bytes()
        (self.root/'color.png').write_bytes(b'changed')
        self.assertEqual(color.read_bytes(), saved)
        self.assertEqual(provenance['frozen_sha256'][str(color.resolve())], hashlib.sha256(saved).hexdigest())


if __name__ == '__main__': unittest.main()
