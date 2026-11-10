"""Offline timing/boundary regressions; no robot, camera or training imports."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.policies.contracts import ActionChunk
from scripts.policies.continuous_shadow import connection,break_reason,review
from scripts.policies.summarize_continuous_shadow import contextual_row


def point(t,arm=0.,grip=30.):return dict(time=t,q=[arm]+[0.]*5+[grip])


def inspect(target=.001,latency=.03,previous_proposals=None,previous_observations=None,previous_commands=None):
    chunk=ActionChunk([point(0,target)['q'] for _ in range(10)],[10+i*.2 for i in range(10)],'row','a'*64)
    return review(chunk,point(9.95),
        [point(9.75)] if previous_observations is None else previous_observations,
        [point(9.6),point(9.8)] if previous_commands is None else previous_commands,
        [point(9.8),point(10.)] if previous_proposals is None else previous_proposals,10.,latency,5),chunk


class ContinuousReviewTests(unittest.TestCase):
    def test_small_continuous_targets_pass_diagnostics_but_never_authorize_motion(self):
        checked,_=inspect()
        self.assertTrue(checked['diagnostic_pass'])
        self.assertFalse(checked['accepted_for_execution']);self.assertFalse(checked['live_eligible'])
        self.assertTrue(checked['timing']['raw_index0_expired'])
        self.assertEqual(checked['candidate_raw_index'],1)
        self.assertEqual(checked['timing']['time_shift_applied_s'],0.)

    def test_first_step_jump_is_rejected_even_when_block_is_constant(self):
        checked,_=inspect(target=2.)
        self.assertEqual(checked['raw_chunk_internal']['max_speed_deg_s'],0.)
        self.assertIn('observed_entry_speed_exceeded',checked['rejection_reasons'])
        self.assertIn('recorded_command_entry_speed_exceeded',checked['rejection_reasons'])

    def test_block_boundary_jump_is_independently_checked(self):
        checked,_=inspect(previous_proposals=[point(9.8,-1.),point(10.,-1.)])
        self.assertTrue(checked['observed_state_entry']['speed_pass'])
        self.assertIn('raw_proposal_boundary_speed_exceeded',checked['rejection_reasons'])

    def test_boundary_acceleration_detects_reversal_below_speed_limit(self):
        checked,_=inspect(target=-.05,previous_proposals=[point(9.8,-.08),point(10.,0.)])
        self.assertTrue(checked['consecutive_raw_proposal_boundary']['speed_pass'])
        self.assertIn('raw_proposal_boundary_acceleration_exceeded',checked['rejection_reasons'])

    def test_late_prediction_is_rejected_without_retiming_or_clipping(self):
        checked,chunk=inspect(latency=.21)
        self.assertIn('candidate_deadline_missed_no_catchup',checked['rejection_reasons'])
        self.assertEqual(checked['timing']['expired_raw_indices'],[0,1])
        self.assertEqual(checked['candidate']['time'],10.2)
        self.assertEqual(checked['candidate']['q'],chunk.commands[1])

    def test_missing_entry_velocity_stays_unknown_and_is_not_zero(self):
        checked,_=inspect(previous_observations=[],previous_commands=[],previous_proposals=[])
        self.assertIsNone(checked['observed_state_entry']['acceleration_pass'])
        self.assertIsNone(checked['previous_recorded_command_entry']['acceleration_pass'])
        self.assertTrue(checked['unknown_initial_acceleration']);self.assertFalse(checked['diagnostic_pass'])

    def test_real_timestamp_intervals_are_used_instead_of_nominal_rate(self):
        result=connection([point(1.),point(1.1,.08)])
        self.assertAlmostEqual(result['last_speed_deg_s'],.8)
        self.assertFalse(result['speed_pass']);self.assertIsNone(result['acceleration_pass'])

    def test_future_teacher_feedback_and_commands_cannot_anchor_policy_boundary(self):
        for field in ('previous_observations','previous_commands'):
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'future_recorded'):
                inspect(**{field:[point(10.1)]})

    def test_j6_is_raw_sdk_position_not_arm_degree_speed(self):
        result=connection([point(1.,grip=30.),point(1.2,grip=30.6),point(1.4,grip=31.2)])
        self.assertTrue(result['speed_pass']);self.assertTrue(result['acceleration_pass'])
        self.assertAlmostEqual(result['gripper_last_change_sdk_raw'],.6)

    def test_gap_and_segment_boundaries_reset_history(self):
        base=dict(split='val',episode_group='run',segment_id='lift',action_source_log='log',sample_index=1,command_host_monotonic=10.)
        nextrow=dict(base,sample_index=2,command_host_monotonic=10.2)
        self.assertIsNone(break_reason(base,nextrow,5))
        for changes,reason in [({'sample_index':3},'sample_index_gap'),
            ({'command_host_monotonic':10.6},'historical_command_time_gap'),
            ({'segment_id':'other'},'source_segment_or_episode_changed')]:
            row=dict(nextrow,**changes)
            self.assertEqual(break_reason(base,row,5),reason)

    def test_teacher_future_target_is_retrospective_and_uses_same_schedule_for_comparison(self):
        checked,chunk=inspect()
        record=dict(observation_id='row',split='val',raw_policy_action={'commands':chunk.commands},
            historical_observation=point(9.95),review=checked,previous_recorded_commands=[point(9.8)])
        example=dict(id='row',split='val',q=point(9.95)['q'],actions=[point(10.,.08)['q'],point(10.24,.1)['q']],
            is_pad=[False,False],actual_target_times=[10.,10.24],action_source_log='teacher',action_source_lines=[5,6])
        original=copy.deepcopy(record)
        result=contextual_row(record,example)
        self.assertTrue(result['available'])
        self.assertAlmostEqual(result['teacher_index1_same_candidate_time']['required_speed_deg_s'],.4)
        self.assertAlmostEqual(result['teacher_index1_actual_recorded_time']['required_speed_deg_s'],.1/.29)
        self.assertEqual(record,original)
        example['is_pad'][1]=True
        result=contextual_row(record,example)
        self.assertFalse(result['available'])
        self.assertEqual(result['unavailable_reason'],'teacher_index1_missing_or_padded')
        self.assertNotIn('teacher_index1_same_candidate_time',result)


if __name__=='__main__':unittest.main()
