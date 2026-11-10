#!/usr/bin/env python3
"""Regression: official CAD overlap, swept-path rejection, and actual Gazebo motion."""
import sys,pathlib,json,urllib.request,urllib.error,time,random
from sim_endpoint import BASE_URL
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from collision_guard import CollisionGuard
c=CollisionGuard();zero={j['name']:0. for j in c.joints if j['type']!='fixed'};zero.update(arm_Joint7_1=.03,arm_Joint7_2=-.03)
def state():return json.load(urllib.request.urlopen(BASE_URL+'/api/state'))
def cmd(data):return json.load(urllib.request.urlopen(urllib.request.Request(BASE_URL+'/api/control',data=json.dumps(data).encode(),headers={'Content-Type':'application/json'})))
old=json.loads((ROOT/'tests/fixtures/penetrating_pose.json').read_text())['positions'];hits=c.check(old,margin=0,all_hits=True)
assert any(h['pair']==['arm_Link2','arm_Link5'] for h in hits),hits
assert not c.check(zero)
start=state();assert start['connected'] and start['collision_protection'];before=start['targets']
# Use original legal target values, avoiding tiny floating-point overshoots from the physics state.
q=json.loads((ROOT/'tests/fixtures/penetrating_pose.json').read_text())['targets']
try:cmd({'values':{k:v for k,v in q.items() if 'Joint7' not in k}});raise AssertionError('penetrating target accepted')
except urllib.error.HTTPError as e:
 assert e.code==400;err=json.load(e)['error'];assert '碰撞' in err,err
assert state()['targets']==before
# Find a deterministic endpoint-clear trajectory that still crosses a collision.
rng=random.Random(721);swept=None
for _ in range(200):
 end=dict(zero)
 for j in c.joints:
  if j['type']=='revolute':end[j['name']]=rng.uniform(j['limits']['lower'],j['limits']['upper'])
 if c.check(end):continue
 hit=c.check_path(zero,end)
 if hit and 0<hit['fraction']<1:swept={'end':end,'hit':hit};break
assert swept,'No swept regression fixture found'
cmd({'action':'home'});deadline=time.monotonic()+50
while max(abs(state()['positions'][n]-v) for n,v in zero.items())>.01:
 assert time.monotonic()<deadline,'Home did not settle'
 time.sleep(.2)
assert not c.check(state()['positions'],margin=0)
try:cmd({'values':{k:v for k,v in swept['end'].items() if 'Joint7' not in k}});raise AssertionError('swept collision accepted')
except urllib.error.HTTPError as e:assert '碰撞' in json.load(e)['error']
cmd({'action':'ready'});actual=[]
for _ in range(30):
 s=state();assert s['connected'];hit=c.check(s['positions'],margin=0);assert not hit,hit;actual.append(s['positions']);time.sleep(.12)
report={'passed':True,'reproduced_original_overlap':hits,'rejected_message':err,'swept_endpoint_clear':swept,'actual_motion_samples_without_overlap':len(actual),'self_collision_links_enabled':9,'margin_m':.002,'physics_step_s':.0005}
(ROOT/'collision-verification.json').write_text(json.dumps(report,indent=2,ensure_ascii=False));print(json.dumps(report,indent=2,ensure_ascii=False))
