"""Meaningful regressions for causal chunks, action units, and deployment parity."""
import copy
from dataclasses import asdict
import json
from pathlib import Path
import sys
import unittest
import numpy as np
import torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.policies.contracts import ActionChunk,validate_sample,UNITS,SERIALS
from scripts.policies.prepare import chunks
from scripts.policies.routing import validate_task_spec,route
from scripts.policies.train import tensors
from scripts.policies.act import build_policy,loss_terms
from scripts.policies.shadow import numeric_review


def row(i=0):
    t=10+i*.2
    return dict(schema='d1-causal-command-state-v1',segment_id='run/lift',sample_index=i,
        trainable=True,command_t=t,command_host_monotonic=t,masks={'a':True},labels={'demonstration_success':True},
        action=dict(sdk_target=[.1]*6+[29.6],units=UNITS,commanded_joint_mask=[True]*7,source_line=i+1,source_log='transport.jsonl'),
        observation=dict(q=[0.]*6+[30.6],units=UNITS,q_age_s=.1,q_available_t=t-.1,
            status={name:dict(value=value,age_s=.1) for name,value in [('power_status',1),('enable_status',1),('error_status',0)]},
            cameras={role:dict(path=role+'.jpg',sha256='a'*64,available_t=t-.05,capture_age_estimate_s=.1,serial=serial)
                     for role,serial in SERIALS.items()}))


class DataTests(unittest.TestCase):
    def test_command_and_state_remain_distinct(self):
        r=validate_sample(row(),lambda p,h:None)
        self.assertEqual(r['action']['sdk_target'][6],29.6)
        self.assertEqual(r['observation']['q'][6],30.6)

    def test_partial_command_is_not_filled_from_feedback(self):
        r=row();r['action']['sdk_target'][0]=None;r['action']['commanded_joint_mask'][0]=False
        with self.assertRaises(ValueError):validate_sample(r,lambda p,h:None)

    def test_future_or_stale_or_wrong_camera_rejected(self):
        for field,value in [('available_t',11),('capture_age_estimate_s',.3),('serial','other')]:
            r=row();r['observation']['cameras']['wrist'][field]=value
            with self.assertRaises(ValueError):validate_sample(r,lambda p,h:None)

    def test_future_feedback_rejected(self):
        r=row();r['observation']['q_available_t']=11
        with self.assertRaises(ValueError):validate_sample(r,lambda p,h:None)

    def test_image_hash_failure_propagates(self):
        def check(p,h):raise ValueError('image hash mismatch')
        with self.assertRaises(ValueError):validate_sample(row(),check)

    def test_padding_never_crosses_invalid_rows(self):
        out=chunks([row(0),row(1),None,row(3),row(4)],4)
        self.assertEqual(len(out),2)
        self.assertEqual(out[0]['action_source_lines'],[1,2])
        self.assertEqual(out[0]['is_pad'],[False,False,True,True])

    def test_gap_and_segment_break_chunks(self):
        rows=[row(i) for i in range(4)];rows[2]['segment_id']='different'
        out=chunks(rows,4)
        self.assertEqual(len(out),1)
        self.assertEqual(out[0]['action_source_lines'],[1,2])
        self.assertFalse(chunks([row(0),row(9)],4))

    def test_target_delta_uses_true_command_not_actual_feedback(self):
        r=chunks([row(0),row(1)],2)[0]
        st={k:[0.]*7 for k in ('q_mean','action_mean')};st.update(q_std=[1.]*7,action_std=[1.]*7)
        _,a,_,q,raw=tensors([r],st,'delta_from_q')
        self.assertAlmostEqual(float(a[0,0,6]),-1.,places=5)
        torch.testing.assert_close(a+q[:,None,:],raw)

    def test_fault_outside_limits_rejected(self):
        r=row();r['observation']['q'][4]=-166.6
        with self.assertRaises(ValueError):validate_sample(r,lambda p,h:None)


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):torch.set_num_threads(2)

    def test_seven_dimension_cvae_and_future_free_deployment(self):
        torch.manual_seed(42);m=build_policy({'chunk_size':3,'hidden_dim':32,'heads':4,'layers':1});m.eval()
        q=torch.zeros(2,7);features=torch.randn(2,2,512,2,3)
        with torch.no_grad():
            a,_,latent=m(q,features,None)
            b,_,_=m(q,features,None)
        self.assertEqual(a.shape,(2,3,7));self.assertEqual(latent,[None,None]);torch.testing.assert_close(a,b)
        loss,_,_=loss_terms(m,q,features,torch.zeros(2,3,7),torch.tensor([[False,False,True]]*2))
        loss.backward();self.assertTrue(torch.isfinite(loss));self.assertIsNotNone(m.action_head.weight.grad)

    def test_numeric_review_never_authorizes_physical_motion(self):
        c=ActionChunk([[0.]*7]*3,[1.,1.2,1.4],'obs','a'*64)
        result=numeric_review(c,[0.]*7,5)
        self.assertTrue(result['within_limits']);self.assertFalse(result['live_eligible']);self.assertFalse(result['collision_checked'])

    def test_malformed_chunks_rejected(self):
        for commands,times in [([[float('nan')]*7],[1]),([[0.]*6],[1]),([[0.]*7]*2,[1,1])]:
            with self.assertRaises(ValueError):ActionChunk(commands,times,'obs','a'*64).validate()


class RoutingTests(unittest.TestCase):
    def setUp(self):self.task=dict(target='red_cube',skill_id='pick_lift_return',lift_height_m=.1,return_to_origin=True)
    def test_known_task_routes_shadow_lift_only(self):
        self.assertEqual(route(self.task,'lift')['shadow_provider'],'act')
        self.assertIsNone(route(self.task,'close')['shadow_provider'])
        self.assertEqual(route(self.task,'lift')['executing_provider'],'geometry')
    def test_complete_json_markdown_wrapper_only(self):
        self.assertEqual(validate_task_spec('```json\n'+json.dumps(self.task)+'\n```'),self.task)
        with self.assertRaises(ValueError):validate_task_spec('explanation\n'+json.dumps(self.task))
    def test_qwen_cannot_create_new_skills_or_robot_commands(self):
        for d in [dict(self.task,skill_id='invent_new_grasp'),dict(self.task,lift_height_m=10),dict(self.task,joint0=1)]:
            with self.assertRaises(ValueError):validate_task_spec(json.dumps(d))
        with self.assertRaises(ValueError):route(self.task,'lift','act_live')


if __name__=='__main__':unittest.main()
