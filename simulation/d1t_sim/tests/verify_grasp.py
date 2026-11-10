#!/usr/bin/env python3
"""Physical acceptance: both contacts + lift + stable hold + release + repeatability."""
import json,time,pathlib,urllib.request,urllib.error,xml.etree.ElementTree as ET
from sim_endpoint import BASE_URL
ROOT=pathlib.Path(__file__).resolve().parents[1]
SCENE=json.loads((ROOT/'config/grasp_scene.json').read_text()); expected=SCENE['cube']['position']
def state():return json.load(urllib.request.urlopen(BASE_URL+'/api/state',timeout=8))
def cmd(action):
 req=urllib.request.Request(BASE_URL+'/api/control',data=json.dumps({'action':action}).encode(),headers={'Content-Type':'application/json'})
 try:return json.load(urllib.request.urlopen(req,timeout=15))
 except urllib.error.HTTPError as e:raise AssertionError(e.read().decode())
def wait(label,timeout=180):
 end=time.monotonic()+timeout;last=None;samples=[]
 while time.monotonic()<end:
  s=state();g=s['grasp'];samples.append({'sim_time':s['sim_time'],'phase':g['phase'],'cube':s['cube']['pose'],'contacts':s['cube']['finger_contacts']})
  if g['phase']!=last:print(label,g['phase'],flush=True);last=g['phase']
  assert s['connected'] and s['cube']['connected'],s
  if not g['active']:
   assert g['result']=='success',s
   return s,samples
  time.sleep(.25)
 raise AssertionError('sequence timeout')
r=ET.parse(ROOT/'models/red_cube/model.sdf').getroot();assert r.findtext('.//box/size')=='0.04 0.04 0.04'
assert float(r.findtext('.//inertial/mass'))==.02
assert abs(float(r.findtext('.//ixx'))-.02*.04*.04/6)<1e-12
# The payload has no joints and no artificial attachment plugin.
assert not r.findall('.//joint');assert len(r.findall('.//plugin'))==1
s=state();assert s['cube']['connected'];assert abs(s['cube']['pose']['position'][2]-.02)<.001;assert not ET.parse(ROOT/'worlds/lab.sdf').findall(".//model[@name='pickup_pedestal']");assert ET.parse(ROOT/'generated/d1t.urdf').find("joint[@name='arm_mount']/origin").get('xyz')=='0 0 0.02';assert ET.parse(ROOT/'worlds/lab.sdf').findtext(".//model[@name='mount']/link/collision/geometry/box/size")=='0.15 0.23 0.02';cmd('resume');report={'cube_size_m':[.04]*3,'cube_mass_kg':.02,'cycles':[]}
for i in range(2):
 s=state();z0=s['cube']['pose']['position'][2];cmd('grasp');s,trace=wait(f'cycle {i+1}')
 held=[];end=s['sim_time']+2.
 deadline=time.monotonic()+50
 while state()['sim_time']<end:
  assert time.monotonic()<deadline
  s=state();p=s['cube']['pose']['position'];assert p[2]-z0>.06,s;assert len(s['cube']['finger_contacts'])==2,s
  held.append(p);time.sleep(.2)
 assert max(p[2] for p in held)-min(p[2] for p in held)<.005
 record={'lift_m':s['cube']['pose']['position'][2]-z0,'hold_samples':len(held),'hold_duration_sim_s':2,'hold_height_span_m':max(p[2] for p in held)-min(p[2] for p in held),'success':s['grasp']['last_success'],'trace':trace}
 cmd('place');s,trace=wait('place');p=s['cube']['pose']['position'];assert abs(p[2]-expected[2])<.003 and abs(p[0]-expected[0])<.008 and abs(p[1]-expected[1])<.008,s
 record['placed_position']=p;record['place_trace']=trace;report['cycles'].append(record)
cmd('reset_cube');s,trace=wait('reset');report['reset']=s['cube'];report['passed']=True;report['ground_alignment']={'support_plane_z':0.,'cube_initial_bottom_z':0.,'arm_mount_z':.02,'mount_size_m':[.15,.23,.02],'cube_pedestal_removed':True}
(ROOT/'grasp-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps({'passed':True,'lifts_m':[c['lift_m'] for c in report['cycles']]},ensure_ascii=False),flush=True)
