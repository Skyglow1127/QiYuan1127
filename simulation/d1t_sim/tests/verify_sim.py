#!/usr/bin/env python3
"""End-to-end checks against running Gazebo, not synthetic joint feedback."""
import urllib.request,urllib.error,json,time,pathlib,xml.etree.ElementTree as E,numpy as np
from sim_endpoint import BASE_URL
root=pathlib.Path(__file__).resolve().parents[1];results=[]
def state():return json.load(urllib.request.urlopen(BASE_URL+'/api/state'))
def cmd(data):return json.load(urllib.request.urlopen(urllib.request.Request(BASE_URL+'/api/control',data=json.dumps(data).encode(),headers={'Content-Type':'application/json'})))
def settle(target,timeout=60):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  s=state()
  if s['connected'] and all(abs(s['positions'][k]-v)<(.0005 if 'Joint7_' in k else .003) for k,v in target.items()):return s
  time.sleep(.15)
 raise AssertionError({'expected':target,'actual':state()})
r=E.parse(root/'generated/d1t.urdf').getroot();original=E.parse(root/'upstream/d1_description.urdf').getroot()
for orig in original.findall('link'):
 cur=r.find(f"link[@name='arm_{orig.get('name')}']")
 for tag in ['mass','origin','inertia']:
  a=orig.find('inertial/'+tag);b=cur.find('inertial/'+tag)
  for k,v in a.attrib.items():assert np.allclose([float(x) for x in v.split()],[float(x) for x in b.get(k,'0 0 0').split()],rtol=0,atol=0),orig.get('name')
for link in r.findall('link'):
 inertia=link.find('inertial/inertia')
 if inertia is None:continue
 v={k:float(x) for k,x in inertia.attrib.items()};mat=np.array([[v['ixx'],v['ixy'],v['ixz']],[v['ixy'],v['iyy'],v['iyz']],[v['ixz'],v['iyz'],v['izz']]])
 assert np.all(np.linalg.eigvalsh(mat)>0)
results.append('Official inertial values preserved; all inertia tensors positive definite')
assert state()['connected'];cmd({'action':'resume'});cmd({'action':'home'});settle({f'arm_Joint{i}':0. for i in range(1,7)})
for i,target in enumerate([.25,-.4,.5,.3,-.3,.4],1):
 name=f'arm_Joint{i}';cmd({'values':{name:target}});s=settle({name:target});results.append({'joint':name,'target':target,'actual':s['positions'][name]})
cmd({'values':{'gripper':.7}});s=settle({'arm_Joint7_1':.009,'arm_Joint7_2':-.009});results.append({'gripper':'70%','actual':[s['positions']['arm_Joint7_1'],s['positions']['arm_Joint7_2']]})
cmd({'values':{'gripper':0}});settle({'arm_Joint7_1':.03,'arm_Joint7_2':-.03})
try:cmd({'values':{'arm_Joint1':9}});raise AssertionError('limit violation accepted')
except urllib.error.HTTPError as e:assert e.code==400
results.append('Out-of-range command rejected')
cmd({'action':'hold'});before=state()['positions'];time.sleep(.5);after=state()['positions'];assert max(abs(before[n]-after[n]) for n in before)<.003
try:cmd({'action':'home'});raise AssertionError('hold bypassed')
except urllib.error.HTTPError as e:assert e.code==400
cmd({'action':'resume'});cmd({'action':'ready'});settle({'arm_Joint1':0,'arm_Joint2':-.55,'arm_Joint3':.8,'arm_Joint4':0,'arm_Joint5':-.25,'arm_Joint6':0})
results.append('Hold, resume and ready pose passed')
(root/'verification.json').write_text(json.dumps({'passed':True,'results':results,'state':state()},indent=2))
print(json.dumps(results,indent=2))
