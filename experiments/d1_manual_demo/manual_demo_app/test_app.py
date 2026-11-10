"""Offline workstation checks: all process/network entry points are mocked."""
import csv
import io
import json
from pathlib import Path
import tempfile
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import app as module
from planner import canonical_plan_sha256


class DeferredThread:
    created = []

    def __init__(self, target, args=(), daemon=None, **kwargs):
        self.target, self.args, self.daemon = target, args, daemon
        self.started = False
        self.__class__.created.append(self)

    def start(self):
        self.started = True

    def is_alive(self):
        return self.started

    def join(self, timeout=None):
        pass


class WorkstationOfflineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='d1_app_offline_test_')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        with patch.object(module.Workstation, 'recover_recorder'):
            self.app = module.Workstation(self.root)
        self.name = 'demo_20261025_010203'
        self.path = self.root / self.name
        self.path.mkdir()
        self.devices = [patch.object(module, 'json_request', side_effect=AssertionError('Network access in an offline test')),
                        patch.object(module.subprocess, 'Popen', side_effect=AssertionError('Process access in an offline test')),
                        patch.object(module.subprocess, 'run', side_effect=AssertionError('Process access in an offline test')),
                        patch.object(module.os, 'kill', side_effect=AssertionError('Signal sent in an offline test'))]
        self.device_mocks = [p.start() for p in self.devices]
        for p in self.devices:
            self.addCleanup(p.stop)

    def ended_source(self):
        (self.path/'summary.json').write_text(json.dumps({'state':'stopped', 'elapsed_s':.2, 'subscriber_pipe_drained':True}))
        (self.path/'metadata.json').write_text(json.dumps({'joint_order':[f'J{i}' for i in range(7)], 'joint_units':['deg']*6+['sdk_raw']}))
        (self.path/'events.jsonl').write_text(json.dumps({'label':'开始演示','elapsed_s':0.})+'\n')
        rows=[{'sample':i+1, 'elapsed_s':i*.1, 'dds_received_monotonic_s':100+i*.1,
               'q':[i*.1,0.,0.,0.,0.,0.,20.], 'gap_before':False} for i in range(3)]
        (self.path/'joint_samples.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n')

    def test_concurrent_start_is_exclusive(self):
        DeferredThread.created=[]
        barrier=threading.Barrier(3)
        results=[]
        real_thread=threading.Thread
        def contender():
            barrier.wait()
            try:results.append(('ok',self.app.start()))
            except ValueError as exc:results.append(('rejected',str(exc)))
        with patch.object(module.threading, 'Thread', DeferredThread):
            first=real_thread(target=contender);second=real_thread(target=contender)
            first.start();second.start();barrier.wait();first.join();second.join()
        self.assertEqual([r[0] for r in results].count('ok'),1)
        self.assertEqual([r[0] for r in results].count('rejected'),1)
        self.assertEqual(len(DeferredThread.created),1)
        self.assertEqual(self.app.state,'starting')
        for call in self.device_mocks:call.assert_not_called()

    def test_stop_is_idempotent_without_starting_another_worker(self):
        DeferredThread.created=[]
        self.app.state='recording'
        with patch.object(module.threading,'Thread',DeferredThread):
            first=self.app.stop();second=self.app.stop()
        self.assertEqual(first,second)
        self.assertEqual(first['state'],'stopping')
        self.assertEqual(len(DeferredThread.created),1)
        for call in self.device_mocks:call.assert_not_called()

    def test_canceled_start_worker_does_not_spawn_or_mark(self):
        self.app.state='stopping'
        self.app._start_worker()
        self.assertEqual(self.app.state,'stopping')
        for call in self.device_mocks:call.assert_not_called()

    def test_recovered_starting_recorder_resumes_without_duplicate_marker(self):
        self.ended_source()
        recorder=self.root/'mock_recorder';recorder.mkdir()
        (recorder/'active_session.json').write_text(json.dumps({
            'url':module.CHILD_URL,'pid':12345,'csrf_token':'child-token'}))
        initial={'state':'starting','path':str(self.path),'ready':False,'fresh':False}
        ready={'state':'recording','path':str(self.path),'ready':True,'fresh':True}
        DeferredThread.created=[]
        with patch.object(module,'RECORDER',recorder), \
             patch.object(module,'pid_is_recorder',return_value=True), \
             patch.object(module,'json_request',return_value=initial) as network, \
             patch.object(module.threading,'Thread',DeferredThread):
            recovered=module.Workstation(self.root)
            self.assertEqual(recovered.state,'starting')
            self.assertEqual(len(DeferredThread.created),1)
            self.assertEqual(recovered.active,self.name)
            network.reset_mock();network.return_value=ready
            recovered._resume_worker()
            self.assertEqual(recovered.state,'recording')
            self.assertEqual([call.args[0] for call in network.call_args_list],[module.CHILD_URL+'/api/status'])
        for call in self.device_mocks:call.assert_not_called()

    def test_resume_marks_once_only_when_still_starting(self):
        (self.path/'events.jsonl').write_text('')
        self.app.active=self.name;self.app.child_token='mock-token';self.app.state='starting'
        ready={'state':'recording','path':str(self.path),'ready':True,'fresh':True}
        with patch.object(module,'json_request',return_value=ready) as network:
            self.app._resume_worker()
            self.assertEqual(self.app.state,'recording')
            self.assertEqual([call.args[0] for call in network.call_args_list],
                             [module.CHILD_URL+'/api/status',module.CHILD_URL+'/api/mark'])
            network.reset_mock();self.app.state='stopping'
            self.app._resume_worker()
            network.assert_not_called()
        for call in self.device_mocks:call.assert_not_called()

    def test_session_traversal_and_outside_symlink_are_rejected(self):
        for name in ['../secret','demo_20261025_010203/..','/etc','demo_20261025_010203%2f..']:
            with self.subTest(name=name), self.assertRaises(ValueError):
                module.safe_session(self.root,name)
        with tempfile.TemporaryDirectory(prefix='d1_outside_') as outside:
            (self.root/'demo_20261025_010204').symlink_to(outside,target_is_directory=True)
            with self.assertRaises(ValueError):
                module.safe_session(self.root,'demo_20261025_010204')

    def test_artifact_path_escape_is_rejected_without_http_server(self):
        handler=module.make_handler(self.app,18771)
        request=SimpleNamespace(path=f'/api/artifact?session_id={self.name}&name=../../secret',reply=Mock())
        handler.do_GET(request)
        self.assertEqual(request.reply.call_args.args[1],400)
        with tempfile.TemporaryDirectory(prefix='d1_artifact_outside_') as outside:
            secret=Path(outside)/'secret.csv';secret.write_text('must not be served')
            (self.path/'joint_timeline.csv').symlink_to(secret)
            request=SimpleNamespace(path=f'/api/artifact?session_id={self.name}&name=joint_timeline.csv',reply=Mock())
            handler.do_GET(request)
            self.assertEqual(request.reply.call_args.args[1],400)
            self.assertNotIn('must not be served',str(request.reply.call_args))

    def test_replay_rejects_even_matching_digest_and_explicit_approval(self):
        self.app.plan_cache[self.name]={'sha256':'digest','plan_sha256':'digest','blockers':['controller_hold_not_verified']}
        for digest,approved in [('wrong',True),('digest',False),('digest',True)]:
            result,code=self.app.replay(self.name,digest,approved)
            self.assertEqual(code,409)
            self.assertFalse(result['ok'])
            if digest=='digest' and approved:
                self.assertEqual(result['robot_commands_sent'],0)
        for call in self.device_mocks:call.assert_not_called()

    def test_prepare_preserves_canonical_plan_and_uses_display_copy(self):
        self.ended_source()
        before={p.name:p.read_bytes() for p in self.path.iterdir() if p.is_file()}
        display=self.app.prepare(self.name,1.)
        saved=json.loads((self.path/'analysis/replay_plan.json').read_text())
        self.assertEqual(saved['plan_sha256'],canonical_plan_sha256(saved))
        self.assertEqual(display['sha256'],saved['plan_sha256'])
        self.assertEqual(display['duration_s'],saved['duration'])
        self.assertEqual(display['source_duration_s'],saved['source_duration'])
        self.assertNotIn('sha256',saved)
        self.assertNotIn('duration_s',saved)
        self.assertEqual(before,{p.name:p.read_bytes() for p in self.path.iterdir() if p.is_file()})
        for call in self.device_mocks:call.assert_not_called()

    def test_formal_velocity_excludes_preparation_and_matches_csv_plot_tail(self):
        self.ended_source()
        times=[0.,.1,.2,.3,1.,1.1]
        positions=[-100.,0.,.2,.4,1.,1.2]
        units=['deg']*6+['sdk_raw']
        rows=[]
        for index,(t,q0) in enumerate(zip(times,positions)):
            rows.append({'sample':index+1,'elapsed_s':t,'dds_received_monotonic_s':100+t,
                         'host_unix_s':1000+t,'q':[q0,0.,0.,0.,0.,0.,20.],
                         'dt_s':None if index==0 else t-times[index-1],
                         'gap_before':index==4,
                         # Deliberately contaminated stored velocities must
                         # never pass through into any formal derived output.
                         'velocity':[999.]*7,'velocity_smoothed':[999.]*7})
        (self.path/'joint_samples.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
        (self.path/'events.jsonl').write_text(json.dumps({'label':'开始演示','elapsed_s':.05})+'\n')
        (self.path/'summary.json').write_text(json.dumps({'state':'stopped','elapsed_s':1.2,'sample_count':6}))
        names=['sample','elapsed_s','dds_received_monotonic_s','host_unix_s','dt_s','gap_before']
        names += [f'J{i}_position_{u}' for i,u in enumerate(units)]
        names += [f'J{i}_velocity_{u}_per_s' for i,u in enumerate(units)]
        names += [f'J{i}_velocity_smoothed_{u}_per_s' for i,u in enumerate(units)]
        with (self.path/'joint_timeline.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=names);writer.writeheader()
            for row in rows:
                value={name:row[name] for name in names[:6]}
                for axis,unit in enumerate(units):
                    value[f'J{axis}_position_{unit}']=row['q'][axis]
                    value[f'J{axis}_velocity_{unit}_per_s']=999.
                    value[f'J{axis}_velocity_smoothed_{unit}_per_s']=999.
                writer.writerow(value)
        before={p.name:p.read_bytes() for p in self.path.iterdir() if p.is_file()}
        start,derived=module.segment_rows(self.path)
        self.assertEqual(start,.05)
        self.assertEqual(len(derived),5)
        self.assertEqual(derived[0]['velocity'],[None]*7)
        self.assertEqual(derived[0]['velocity_smoothed'],[None]*7)
        for index in [1,2,4]:
            self.assertAlmostEqual(derived[index]['velocity'][0],2.)
            self.assertAlmostEqual(derived[index]['velocity_smoothed'][0],2.)
        self.assertEqual(derived[3]['velocity_smoothed'],[None]*7)
        axes=[[Mock(),Mock()] for _ in range(7)]
        class PlotAxes:
            def __getitem__(self,key):
                return axes[key[0]][key[1]] if isinstance(key,tuple) else axes[key]
        pyplot=Mock();pyplot.subplots.return_value=(Mock(),PlotAxes())
        matplotlib=ModuleType('matplotlib');matplotlib.use=Mock();matplotlib.pyplot=pyplot
        with patch.dict('sys.modules',{'matplotlib':matplotlib,'matplotlib.pyplot':pyplot}):
            result=module.export_segment(self.path)
        self.assertEqual(result['velocity_estimation']['preparation_samples_used'],False)
        self.assertEqual(axes[0][1].plot.call_args.args[1],[r['velocity_smoothed'][0] for r in derived])
        with (self.path/'analysis/demonstration_timeline.csv').open() as stream:
            exported=list(csv.DictReader(stream))
        self.assertEqual(len(exported),5)
        self.assertEqual(exported[0]['J0_velocity_deg_per_s'],'')
        self.assertEqual(exported[0]['J0_velocity_smoothed_deg_per_s'],'')
        self.assertEqual(exported[0]['segment_dt_s'],'')
        self.assertEqual(float(exported[0]['dt_s']),.1)  # Original interval retained separately.
        self.assertAlmostEqual(float(exported[1]['J0_velocity_smoothed_deg_per_s']),2.)
        self.assertEqual(exported[3]['J0_velocity_smoothed_deg_per_s'],'')
        response=self.app.session(self.name)
        self.assertEqual([r['v'] for r in response['tail']],[r['velocity_smoothed'] for r in derived])
        self.assertEqual(response['velocity_estimation']['id'],result['velocity_estimation']['id'])
        self.assertEqual(before,{p.name:p.read_bytes() for p in self.path.iterdir() if p.is_file()})
        for call in self.device_mocks:call.assert_not_called()


if __name__=='__main__':
    unittest.main()
