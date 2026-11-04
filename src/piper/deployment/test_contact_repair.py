"""Repair-profile and real-codec/two-guard tests, with synthetic feedback only."""
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import launch_act
import launch_act_contact_repair as entry
from act_chunk_timing import ACTActionTiming
from lerobot_robot_piper_executor.guard import ACTION_KEYS,ActionGuard,GuardError,GuardLimits,Readiness
from lerobot_robot_piper_executor.joint_smoothing import smooth_joint_targets
from lerobot_robot_piper_executor.robot import Observation,ROLLOUT_KEYS
from test_executor import ready
import test_policy_target_execution as fixture


class ContactRepairTests(unittest.TestCase):
    def setUp(self):
        self.profile = launch_act.read(entry.PROFILE)
        self.limits = GuardLimits(**self.profile['policy']['limits'])
        self.enterContext(patch('lerobot.robots.utils.logging.warning'))
        self.enterContext(patch('lerobot_robot_piper_executor.robot.LOG.warning'))

    def robot(self, smoothing=True):
        robot,backend,clock = fixture.PolicyTargetExecutionTests().make_robot()
        state = (0.,60.,-50.,0.,30.,0.,.04)
        robot.guard,backend.remote_guard = ActionGuard(self.limits),ActionGuard(self.limits)
        for guard,offset in ((robot.guard,0.),(backend.remote_guard,backend.remote_offset)):
            guard.arm(state,sampled_at=clock()+offset,now=clock()+offset,readiness=ready())
        backend.feedback = state
        backend.config.update(policy_joint_target_margin_deg=2.,policy_gripper_target_margin_m=.001)
        if smoothing:
            backend.config['policy_joint_smoothing_alpha'] = self.profile['policy_joint_smoothing_alpha']
        self.addCleanup(robot.disarm,'synthetic contact repair done')
        return robot,backend,clock

    def fresh(self,robot,backend,clock):
        clock.now += .05
        robot.latest = Observation(dict(zip(ACTION_KEYS,backend.feedback)),clock()-.001)
        robot.policy_action_timing = ACTActionTiming(clock()-.05,clock(),clock(),0,self.profile['n_action_steps'],20.)

    def test_filter_blends_only_joints_and_rejects_bad_alphas(self):
        target=(2.,4.,6.,8.,10.,12.,.08)
        previous=(0.,)*6+(.02,)
        self.assertEqual(smooth_joint_targets(target,previous_target=previous,alpha=1.),target)
        result=smooth_joint_targets(target,previous_target=previous,alpha=.7)
        for actual,desired in zip(result[:6],target[:6]):
            self.assertAlmostEqual(actual,.7*desired)
        self.assertEqual(result[-1],target[-1])
        for alpha in (True,None,'0.7',.4999,1.0001,float('nan'),float('inf')):
            with self.subTest(alpha=alpha),self.assertRaises(GuardError):
                smooth_joint_targets(target,previous_target=previous,alpha=alpha)

    def test_real_codec_guards_and_deadline_with_unfiltered_gripper(self):
        targets=[]
        for smoothing in (False,True):
            robot,backend,clock=self.robot(smoothing)
            self.fresh(robot,backend,clock)
            timing,sample=robot.policy_action_timing,robot.latest.sampled_at
            action=dict(zip(ROLLOUT_KEYS,backend.feedback));action['joint_1.pos']=2.;action['gripper.pos']=.08
            robot.send_action(action)
            host,remote,frames,_=backend.accepted[0]
            self.assertEqual(host.target,remote.target)
            self.assertAlmostEqual(host.expires_at,min(sample+.4,timing.scheduled_at+.4))
            targets.append(host.target)
            if smoothing:
                self.assertEqual(struct.unpack('>ii',frames[0].data)[0],1000)
                self.assertAlmostEqual(host.target[0],1.)
        self.assertEqual(targets[0][-1],targets[1][-1])
        self.assertLess(targets[1][0],targets[0][0])

    def test_two_complete_chunks_and_reversals_remain_guarded(self):
        robot,backend,clock=self.robot()
        count=self.profile['n_action_steps']
        for index in range(2*count):
            clock.now += .012
            sampled=clock()-.001
            robot.latest=Observation(dict(zip(ACTION_KEYS,backend.feedback)),sampled)
            if not index%count:
                source=sampled;clock.now+=.18;generated=clock()
            robot.policy_action_timing=ACTActionTiming(source,generated,clock(),index%count,count,20.)
            raw=tuple(hi+10 if index//count%2 else lo-10 for lo,hi in self.limits.bounds[:6])
            robot.send_action(dict(zip(ROLLOUT_KEYS,raw+(.08 if index//count%2 else .001,))))
            host,remote,_,_=backend.accepted[-1]
            self.assertEqual(host.target,remote.target)
            self.assertAlmostEqual(host.expires_at,min(sampled+.4,source+index%count/20+.4))
        self.assertEqual(len(backend.accepted),2*count)
        self.assertTrue(robot.guard.armed)

    def test_actual_limit_faults_and_missing_prerequisites_still_refuse(self):
        for case in ('actual','stale','timing','mode','cap','rate','null'):
            robot,backend,clock=self.robot()
            self.fresh(robot,backend,clock)
            if case=='actual':
                bad=tuple(backend.feedback[:4])+(70.072,)+tuple(backend.feedback[5:])
                robot.latest=Observation(dict(zip(ACTION_KEYS,bad)),clock()-.001)
            elif case=='stale':
                clock.now+=.5
                robot.latest=Observation(dict(zip(ACTION_KEYS,backend.feedback)),clock()-.001)
            elif case=='timing':
                robot.require_policy_timing=False;robot.policy_action_timing=None
            elif case=='mode':
                del backend.config['policy_joint_target_margin_deg']
            elif case=='cap':
                del backend.config['max_relative_target']
            elif case=='rate':
                del backend.config['policy_target_rate_fraction']
            else:
                backend.config['policy_joint_smoothing_alpha']=None
            with self.subTest(case=case),self.assertRaises(GuardError):
                robot.send_action(dict(zip(ROLLOUT_KEYS,backend.feedback)))
            self.assertEqual(backend.send_attempts,0)

    def test_send_failure_does_not_advance_filter_anchor(self):
        robot,backend,clock=self.robot()
        previous=robot.guard.last_target
        self.fresh(robot,backend,clock)
        action=dict(zip(ROLLOUT_KEYS,backend.feedback));action['joint_1.pos']=2.
        with patch.object(backend,'send_checked',side_effect=GuardError('synthetic transport failure')):
            with self.assertRaisesRegex(GuardError,'synthetic'):
                robot.send_action(action)
        self.assertEqual(robot.guard.last_target,previous)
        self.assertFalse(robot.guard.armed)

    def test_repair_profile_keeps_parent_files_and_restores_shared_selection(self):
        profile=entry.verify_profile()
        self.assertEqual(profile['policy_joint_smoothing_alpha'],.5)
        self.assertEqual(profile['default_duration_s'],0.)
        baseline=launch_act.read(entry.BASELINE/'act_launch_defaults.json')
        changed={key for key in set(profile)|set(baseline) if profile.get(key)!=baseline.get(key)}
        expected={'n_action_steps','default_duration_s','policy_joint_smoothing_alpha','policy_release_chunk_extension'}
        if 'policy_grasp_chunk_extension' in profile:
            expected.add('policy_grasp_chunk_extension')
        for key in ('policy_approach_gripper', 'policy_approach_refresh', 'policy_joint_acceleration_deg_s2', 'policy_release_opening_prefix', 'policy_release_settle'):
            if key in profile:
                expected.add(key)
        self.assertEqual(changed,expected)
        previous=launch_act.DEFAULTS
        with patch.object(launch_act,'main',side_effect=lambda: launch_act.read(launch_act.DEFAULTS)),patch('builtins.print'):
            self.assertEqual(entry.main(),profile)
        self.assertEqual(launch_act.DEFAULTS,previous)

    def test_config_builder_and_wrapper_gate_preserve_all_limits(self):
        base=launch_act.read(self.profile['base_io_config'])
        probe={'identity':dict(base['transport']['expected_identity']),
            'complete_fresh_feedback':True,'motor_faults':[0]*6,'enabled':[True]*6,
            'arm_status_raw':'0100010000000000','gripper_raw':'00009c4000004000',
            'state':[0.,60.,-50.,0.,30.,0.,.04]}
        spec=importlib.util.spec_from_file_location('repair_gate',entry.HERE.parent/'official_adapter/rollout_act_piper.py')
        wrapper=importlib.util.module_from_spec(spec);spec.loader.exec_module(wrapper)
        with tempfile.TemporaryDirectory() as temporary:
            out=Path(temporary)
            launch_act.build_run(self.profile,base,probe,out,'10.42.0.55')
            config=launch_act.read(out/'execute_config.json')
            remote=launch_act.read(out/'act_trial_policy.json')
            self.assertEqual(config['policy_joint_smoothing_alpha'],.5)
            self.assertEqual(config['policy_release_chunk_extension'],self.profile['policy_release_chunk_extension'])
            if 'policy_grasp_chunk_extension' in self.profile:
                self.assertEqual(config['policy_grasp_chunk_extension'], self.profile['policy_grasp_chunk_extension'])
            for key in ('policy_approach_gripper', 'policy_approach_refresh', 'policy_joint_acceleration_deg_s2', 'policy_release_opening_prefix', 'policy_release_settle'):
                if key in self.profile:
                    self.assertEqual(config[key], self.profile[key])
            self.assertEqual(config['guard_limits'],remote['limits'])
            self.assertEqual(config['guard_limits'],self.profile['policy']['limits'])
            args=SimpleNamespace(readiness=out/'readiness.json',readiness_reviewed=True)
            _,missing,_=wrapper.local_gate_status(args,config,Readiness,GuardLimits)
            self.assertEqual(missing,[])
            for alpha in (True,None,.49,1.01,float('nan'),'0.7'):
                with self.subTest(alpha=alpha),self.assertRaisesRegex(ValueError,'policy_joint_smoothing_alpha'):
                    wrapper.local_gate_status(args,{**config,'policy_joint_smoothing_alpha':alpha},Readiness,GuardLimits)
            for release in (None, {}, {**config['policy_release_chunk_extension'], 'extra_steps': 21}):
                with self.subTest(release=release), self.assertRaisesRegex(ValueError,'policy_release_chunk_extension'):
                    wrapper.local_gate_status(args,{**config,'policy_release_chunk_extension':release},Readiness,GuardLimits)
        for alpha in (True,None,.49,1.01,float('nan'),'0.7'):
            with tempfile.TemporaryDirectory() as temporary:
                out=Path(temporary)
                with self.subTest(alpha=alpha),self.assertRaisesRegex(ValueError,'policy_joint_smoothing_alpha'):
                    launch_act.build_run({**self.profile,'policy_joint_smoothing_alpha':alpha},base,probe,out,'10.42.0.55')
                self.assertEqual(list(out.iterdir()),[])


if __name__=='__main__':
    unittest.main()
