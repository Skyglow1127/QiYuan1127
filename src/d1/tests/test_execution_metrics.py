"""Synthetic measurement tests; no robot, network or historical log fixtures."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

TOOL=Path(__file__).resolve().parents[1]/'tools/analyze_execution.py'
spec=importlib.util.spec_from_file_location('execution_metrics',TOOL)
metrics=importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


def saved_plan(duration=1.):
    return dict(sdk_plan=dict(sdk=[[0.]*6,[0.]*6],ticks=[10*duration],rate=10.,
                             duration_s=duration,timing_profile='edge_quintic_v1'))


def feedback(stamp,angle=2.,accepted=None,rejected=None,joint_times=None):
    event=dict(received_monotonic_s=stamp,merged_angles=[angle]*7,
               accepted_joints=list(range(7)) if accepted is None else accepted,
               rejected_joints=rejected or [],joint_times_s=joint_times or [stamp]*7)
    return dict(t=stamp+.0001,kind='output',line='[feedback-accepted] '+json.dumps(event))


def protocol(t,direction,address,funcode,seq,data):
    return dict(t=t,kind='output',line=f'[{direction}] '+json.dumps(dict(address=address,funcode=funcode,seq=seq,data=data)))


class ExecutionMetricsTest(unittest.TestCase):
    def stage(self,root,plan,result,rows):
        stage=Path(root)/'stage';stage.mkdir()
        (stage/'plan.json').write_text(json.dumps(plan))
        (stage/'result.json').write_text(json.dumps(result))
        (stage/'transport.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
        return stage

    def test_relative_drift_and_actual_submitted_axes(self):
        groups=[dict(source_elapsed_s=.1,group_started_monotonic_s=100.1,group_completed_monotonic_s=100.16,completed=True),
                dict(source_elapsed_s=12.1,group_started_monotonic_s=121.5505,group_completed_monotonic_s=121.61,completed=False)]
        rows=[dict(kind='command',t=t,joint=j,target_sdk_units=1.) for j,t in [(0,100.101),(5,100.151),(1,121.551)]]
        with tempfile.TemporaryDirectory() as root:
            stage=self.stage(root,saved_plan(13.),dict(completed=False,command_group_schedule=groups),rows)
            out,tables=metrics.analyze(stage)
        g=out['group_timing']
        self.assertAlmostEqual(g['reference_phase_span_s'],12.)
        self.assertAlmostEqual(g['actual_call_start_span_s'],21.4505)
        self.assertAlmostEqual(g['relative_cumulative_drift_s'],9.4505)
        self.assertAlmostEqual(tables['groups'][0]['submission_span_s'],.05)
        self.assertEqual(tables['groups'][0]['submitted_active_joints'],[0,5])
        self.assertEqual(g['six_axis_submission_span_s']['count'],0)
        self.assertFalse(out['completed'])

    def test_ack_sequence_pairing_and_rejected_retained_joint(self):
        rows=[dict(t=1.,kind='feedback_filter',enabled=True),feedback(1.),
              dict(t=1.01,kind='command',joint=1,target_sdk_units=3.),
              protocol(1.011,'tx',1,1,42,dict(id=1,angle=3.)),
              protocol(1.012,'rx',3,1,99,dict(recv_status=1)),
              protocol(1.02,'rx',3,1,42,dict(recv_status=1)),
              feedback(1.1,accepted=[0,2,3,4,5,6],rejected=[1],joint_times=[1.1,1.,1.1,1.1,1.1,1.1,1.1]),
              feedback(1.2)]
        commands,fb,rejected,ages,warnings=metrics.parse_transport(rows)
        self.assertEqual(len(fb[1]),2)
        self.assertEqual(len(fb[0]),3)
        self.assertEqual(rejected[1],1)
        self.assertEqual(commands[0]['seq'],42)
        self.assertAlmostEqual(commands[0]['ack_elapsed_s'],.009)
        self.assertEqual(commands[0]['ack_status'],1)

    def test_time_weighted_error_excludes_long_gap(self):
        reference=metrics.Reference(saved_plan())
        samples=[dict(received_monotonic_s=t,angle=2.,joint=0) for t in [100.,100.1,100.2,100.9,101.]]
        out,rows=metrics.weighted_error(samples,reference,100.,0,.3)
        self.assertAlmostEqual(out['covered_s'],.3)
        self.assertAlmostEqual(out['coverage_of_original_duration'],.3)
        self.assertAlmostEqual(out['rmse_deg'],2.)
        self.assertAlmostEqual(out['absolute_error_p95_deg'],2.)
        self.assertEqual(out['maximum_observed_absolute_error_deg'],2.)

    def test_weighted_absolute_percentile_handles_zero_crossing(self):
        # Uniform time on a -2 to +2 linear error has |e| P95=1.9, RMSE=sqrt(4/3).
        reference=metrics.Reference(saved_plan())
        samples=[dict(received_monotonic_s=10.,angle=-2.,joint=0),dict(received_monotonic_s=11.,angle=2.,joint=0)]
        out,_=metrics.weighted_error(samples,reference,10.,0,1.1)
        self.assertAlmostEqual(out['absolute_error_p95_deg'],1.9)
        self.assertAlmostEqual(out['rmse_deg'],(4/3)**.5)

    def test_missing_reference_and_provenance_stay_unknown(self):
        row=dict(t=10.,kind='output',line='[feedback-accepted] '+json.dumps(dict(received_monotonic_s=10.,merged_angles=[2.]*7)))
        with tempfile.TemporaryDirectory() as root:
            stage=self.stage(root,dict(expected_angles=[0.]*6),dict(completed=False),[row])
            out,tables=metrics.analyze(stage)
        self.assertIsNone(out['reference_epoch_monotonic_s'])
        self.assertIsNone(out['joints'][0]['original_timeline_tracking'])
        self.assertEqual(out['joints'][0]['accepted_unique_samples'],0)
        self.assertIsNone(out['group_timing']['reference_phase_span_s'])

    def test_new_output_only_and_input_hashes_unchanged(self):
        with tempfile.TemporaryDirectory() as root:
            stage=self.stage(root,saved_plan(),dict(completed=False,reference_epoch_monotonic_s=100.),[feedback(100.),feedback(100.1)])
            before={p.name:p.read_bytes() for p in stage.iterdir()}
            out,tables=metrics.analyze(stage)
            output=Path(root)/'report'
            metrics.write_report(output,out,tables)
            self.assertTrue((output/'report.html').exists())
            self.assertEqual(set(out['inputs']),{'plan.json','result.json','transport.jsonl'})
            self.assertEqual(out['reference_epoch_basis'],'explicit_original_reference_epoch')
            with self.assertRaises(FileExistsError):metrics.write_report(output,out,tables)
            with self.assertRaises(ValueError):metrics.write_report(stage/'report',out,tables)
            self.assertEqual(before,{p.name:p.read_bytes() for p in stage.iterdir()})

    def test_adjacent_group_boundary_command_belongs_once(self):
        groups=[dict(source_elapsed_s=.1,group_started_monotonic_s=10.,group_completed_monotonic_s=10.1,completed=True),
                dict(source_elapsed_s=.2,group_started_monotonic_s=10.1,group_completed_monotonic_s=10.2,completed=True)]
        rows=[dict(kind='command',t=10.1,joint=0,target_sdk_units=0.)]
        with tempfile.TemporaryDirectory() as root:
            stage=self.stage(root,saved_plan(),dict(completed=False,command_group_schedule=groups),rows)
            _,tables=metrics.analyze(stage)
        self.assertEqual([g['submitted_count'] for g in tables['groups']],[0,1])

    def test_static_residual_requires_sent_final_group_and_receipt(self):
        plan=saved_plan();plan['sdk_plan']['active_joints']=[0]
        ref=metrics.Reference(plan)
        groups=[dict(completed=True,source_elapsed_s=1.,call_completed_monotonic_s=101.01)]
        commands=[dict(joint=0,submitted_monotonic_s=101.,target_sdk_units=0.,ack_status=1,ack_monotonic_s=101.01)]
        fb=[[] for _ in range(7)]
        fb[0]=[dict(received_monotonic_s=101.5+i*.1,angle=1.2) for i in range(21)]
        out,reason=metrics.static_residual(ref,groups,commands,fb,.3)
        self.assertIsNone(reason)
        self.assertAlmostEqual(out['axes'][0]['median_residual_deg'],1.2)
        self.assertFalse(out['hardware_defect_proven'])
        commands[0]['ack_status']=None
        self.assertIsNone(metrics.static_residual(ref,groups,commands,fb,.3)[0])
        commands[0]['ack_status']=1;groups[0]['source_elapsed_s']=.2
        self.assertIsNone(metrics.static_residual(ref,groups,commands,fb,.3)[0])

    def test_feedback_age_does_not_use_future_acceptance(self):
        future=feedback(100.);future['t']=100.5
        rows=[dict(t=100.1,kind='command',joint=0,target_sdk_units=0.),future]
        with tempfile.TemporaryDirectory() as root:
            stage=self.stage(root,saved_plan(),dict(completed=False),rows)
            _,tables=metrics.analyze(stage)
        self.assertIsNone(tables['commands'][0]['same_joint_feedback_receipt_age_s'])


if __name__=='__main__':unittest.main()
