#!/usr/bin/env python3
"""Physical PiPER grasp/place/reset verification, exclusively on local Gazebo."""
import json,pathlib,time,urllib.request,urllib.error,os
from datetime import datetime
from zoneinfo import ZoneInfo
ROOT=pathlib.Path(__file__).resolve().parents[1]
PORT=int(os.environ.get('PIPER_PORT',json.loads((ROOT/'config/runtime.json').read_text())['simulation_port']))
URL=f'http://127.0.0.1:{PORT}'
def state():return json.load(urllib.request.urlopen(URL+'/api/state',timeout=5))
def action(name):
 request=urllib.request.Request(URL+'/api/control',data=json.dumps({'action':name}).encode(),headers={'Content-Type':'application/json'})
 try:return json.load(urllib.request.urlopen(request,timeout=15))
 except urllib.error.HTTPError as error:raise RuntimeError(error.read().decode())
def wait_done():
 previous=None;deadline=time.monotonic()+90
 while time.monotonic()<deadline:
  time.sleep(.3);s=state();report['trace'].append(s)
  if s['grasp']['phase']!=previous:
   print(s['sim_time'],s['grasp']['phase'],s['cube']['pose']['position'],flush=True);previous=s['grasp']['phase']
  if not s['grasp']['active']:
   assert s['grasp']['result']=='success',s['grasp'];return s
 raise RuntimeError('Sequence timed out')
report={'passed':False,'simulation_only':True,'started_at':datetime.now(ZoneInfo('Asia/Singapore')).isoformat(),'cycles':[],'trace':[]}
try:
 initial=state();assert 'joint1' in initial['positions'] and 'arm_Joint1' not in initial['positions']
 if initial['hold']:action('resume')
 for cycle in range(2):
  print('Cycle',cycle+1,flush=True)
  action('reset_cube');reset=wait_done()
  action('grasp');grip=wait_done();hold=[];start=grip['sim_time']
  while state()['sim_time']-start<3:
   time.sleep(.3);s=state();hold.append(s)
   assert len(s['cube']['finger_contacts'])==2 and s['cube']['pose']['position'][2]>.08
  action('place');placed=wait_done()
  report['cycles'].append({'reset':reset,'grasp':grip,'hold_3_sim_seconds':hold,'placed':placed})
 action('reset_cube');report['final']=wait_done();report['passed']=True
except Exception as error:
 report['error']=str(error)
 try:action('hold')
 except Exception:pass
finally:
 report['finished_at']=datetime.now(ZoneInfo('Asia/Singapore')).isoformat()
 (ROOT/'grasp-verification.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({'passed':report['passed'],'cycles':len(report['cycles']),'error':report.get('error')}),flush=True)
assert report['passed'],report.get('error')
