#!/usr/bin/env python3
"""Read-only, isolated RViz review of recorded feedback. Never drives hardware.

Builds an explicitly UNVERIFIED nominal mapping preview, not a calibration.
The operational mapping and vendor model are never written by this program.
"""
import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'ros2_ws/src/d1_pick_place/config/d1.urdf'
sys.path.insert(0, str(ROOT / 'ros2_ws/src/d1_pick_place'))
from d1_pick_place.kinematics import forward


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def fk(q):
    return forward(URDF, {f'Joint{i+1}': float(v) for i, v in enumerate(q)}, tip='Link6')


def fit_rotation(a, b):
    """Proper rotation taking camera relative rotation vectors b into arm a."""
    u, singular, vt = np.linalg.svd(np.asarray(a).T @ np.asarray(b))
    return u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt, singular


def rotation_audit(out):
    pairs=[]
    for j in range(6):
        folder=ROOT / ('logs/j4_visual_probe_20261021' if j==4 else f'logs/axis_checks_20261021/j{j}')
        s=json.loads((folder/'summary.json').read_text())
        v=json.loads((folder/'vision_check.json').read_text())
        def mean_rotation(phase):
            return Rotation.from_matrix([np.asarray(x['T_camera_board'])[:3,:3]
                                         for x in v['poses'] if x['phase']==phase]).mean().as_matrix()
        b=mean_rotation('before') @ mean_rotation('outward').T
        pairs.append((np.deg2rad(s['baseline'][:6]),
                      np.deg2rad(s['endpoints'][0]['all_joints_median'][:6]), b))
    def vectors(signs):
        aa=[fk(q0*signs)[:3,:3].T @ fk(q1*signs)[:3,:3] for q0,q1,_ in pairs]
        bb=[b for _,_,b in pairs]
        return aa,bb,Rotation.from_matrix(aa).as_rotvec(),Rotation.from_matrix(bb).as_rotvec()
    aa,bb,a,b=vectors(np.ones(6))
    train=[1,3,4];heldout=[0,2,5]
    x,singular=fit_rotation(a[train],b[train])
    rows=[]
    for j in range(6):
        error=Rotation.from_matrix(aa[j].T @ x @ bb[j] @ x.T).magnitude()
        rows.append(dict(sdk_joint=j,split='fit' if j in train else 'held_out',
                         model_rotation_deg=float(np.rad2deg(np.linalg.norm(a[j]))),
                         visual_rotation_deg=float(np.rad2deg(np.linalg.norm(b[j]))),
                         rotation_consistency_residual_deg=float(np.rad2deg(error))))
    save(out/'heldout_rotation_diagnostic.json',dict(
        method='Nominal signs +1 and offsets 0. Fit rotation-only end-camera relation on J1/J3/J4; evaluate J0/J2/J5 without fitting them. This is not a mechanical zero or hand-eye calibration.',
        training_joints=train,heldout_joints=heldout,rows=rows,
        fit_covariance_singular_values=singular.tolist(),all_link_orientation_checked=False,
        absolute_tip_pose_checked=False,zero_offset_verified=False,passed=False,
        limitations=['Small rotations and quantized feedback; visual PnP is not independent calibrated angular metrology.',
                     'Summary feedback and images are nearby settled observations, not synchronized formal calibration pairs.',
                     'No independent base/Link6 references. Residual is not a joint zero offset.']))
    ranked=[]
    for signs in itertools.product((-1,1),repeat=6):
        _,_,a,b=vectors(np.asarray(signs));x,_=fit_rotation(a,b)
        err=np.rad2deg(np.linalg.norm(a-b@x.T,axis=1))
        ranked.append(dict(signs=list(signs),rms_deg=float(np.sqrt(np.mean(err**2))),residuals_deg=err.tolist()))
    ranked.sort(key=lambda r:r['rms_deg'])
    save(out/'sign_hypotheses.json',dict(
        method='Exploratory rotation-only ranking at zero nominal offsets using all six recorded probes. Not held-out validation. Does not resolve absolute zero offsets.',
        ranked=ranked,validated=False))


def build(out):
    """Existing endpoint observations; no claim of synchronized calibration pairs."""
    out.mkdir(exist_ok=True, parents=True)
    states = []
    for j in range(6):
        folder = ROOT / ('logs/j4_visual_probe_20261021' if j == 4 else f'logs/axis_checks_20261021/j{j}')
        summary = json.loads((folder/'summary.json').read_text())
        frames = [json.loads(s) for s in (folder/'frames.jsonl').read_text().splitlines()]
        for endpoint in summary['endpoints']:
            phase = endpoint['phase'].replace('_settle', '')
            row = dict(label=f'J{j} {phase} - recorded feedback',
                       joint_sdk_deg=endpoint['all_joints_median'][:6],
                       source=str(folder/'summary.json'), phase=phase,
                       mapping_validated=False)
            for role in ('global', 'wrist'):
                chosen = [f for f in frames if f['role'] == role and f['phase'] == phase+'_endpoint']
                if not chosen:
                    raise ValueError(f'Missing settled image: {folder} {role} {phase}')
                row[role+'_image'] = str(folder/'frames'/chosen[-1]['file'])
            states.append(row)
    current = json.loads((out/'current_capture/capture.json').read_text())
    if not current['complete']:
        raise ValueError('Current stationary capture failed')
    states.insert(0, dict(label='Current stationary capture - recorded feedback',
                         joint_sdk_deg=current['observation']['joint_median'][:6],
                         source=str(out/'current_capture/capture.json'),
                         global_image=str(out/'current_global.jpg'),
                         wrist_image=str(out/'current_capture/color_3.jpg'),
                         mapping_validated=False))
    # This is deliberately not a valid operational mapping file.
    bundle = dict(schema='d1-nominal-review-only-v1', physical_order_user_confirmed=True,
                  order=[f'J{i}->Joint{i+1}' for i in range(6)],
                  sign_hypothesis=[1]*6, offset_hypothesis_rad=[0]*6,
                  mapping_validated=False, zero_offset_validated=False,
                  note='Signs +1 and offsets 0 are display hypotheses only. Images/median feedback are nearby settled observations, not exposure-synchronized pairs. No camera/base registration or independent angle reference.',
                  urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(), states=states)
    save(out/'review_bundle.json', bundle)
    # Show only the six-link arm: finger opening and the nominal TCP are uncalibrated.
    tree = ET.parse(URDF); robot = tree.getroot()
    keep = {'base_link', *[f'Link{i}' for i in range(1,7)]}
    for item in list(robot):
        if ((item.tag == 'link' and item.get('name') not in keep) or
            (item.tag == 'joint' and item.find('child').get('link') not in keep)):
            robot.remove(item)
    for mesh in robot.findall('.//mesh'):
        mesh.set('filename', mesh.get('filename').replace('package://d1_pick_place/',
                 (ROOT/'ros2_ws/src/d1_pick_place').as_uri()+'/'))
    tree.write(out/'review_only.urdf', encoding='utf-8', xml_declaration=True)
    # Exact ambiguity check: an unknown base/board frame absorbs J0's offset;
    # unknown Link6/camera extrinsics absorb J5's offset. More wrist poses alone
    # cannot determine these two absolute offsets.
    qs = np.deg2rad(np.asarray([s['joint_sdk_deg'] for s in states]))
    delta = np.deg2rad(5.)
    for index, side in ((0,'left'), (5,'right')):
        q = qs[0].copy(); shifted=q.copy(); shifted[index]+=delta
        a,b=fk(q),fk(shifted)
        gauge=b@np.linalg.inv(a) if side=='left' else np.linalg.inv(a)@b
        residual=[]
        for q in qs:
            shifted=q.copy();shifted[index]+=delta
            predicted=gauge@fk(q) if side=='left' else fk(q)@gauge
            residual.append(float(np.max(np.abs(fk(shifted)-predicted))))
        bundle[f'J{index}_offset_gauge_max_matrix_error']=max(residual)
    q_range=np.ptp(np.rad2deg(qs),axis=0)
    audit=dict(physical_order_user_confirmed=True,
               sdk_feedback_direction_checks_completed=True,
               urdf_signs_validated=False, zero_offsets_validated=False,
               independent_cross_validation_passed=False,
               pose_count=len(states), observed_joint_ranges_deg=q_range.tolist(),
               j0_gauge_max_matrix_error=bundle['J0_offset_gauge_max_matrix_error'],
               j5_gauge_max_matrix_error=bundle['J5_offset_gauge_max_matrix_error'],
               conclusion='Wrist-board data with unknown base-board and end-camera transforms cannot identify absolute J0/J5 offsets. Current global images have no independently registered base frame or known link fiducials. Nominal RViz replay is not metric cross-validation.',
               needed='Known base and Link6 geometric references, or a manufacturer-defined zero pose with independently measured link orientations; broader independent poses after that reference is established.',
               operational_mapping_written=False, hardware_motion_sent=False)
    save(out/'observability_audit.json',audit)
    rotation_audit(out)
    print(json.dumps(audit,ensure_ascii=False,indent=2))


def replay(out, seconds):
    if os.environ.get('ROS_DOMAIN_ID') != '78':
        raise ValueError('Diagnostic replay requires isolated ROS_DOMAIN_ID=78')
    import cv2
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, Image
    from visualization_msgs.msg import Marker
    import yaml
    bundle=json.loads((out/'review_bundle.json').read_text())
    if bundle['mapping_validated'] is not False:
        raise ValueError('Only explicit unverified review bundles supported')
    rclpy.init();node=Node('d1_recorded_feedback_review')
    joints=node.create_publisher(JointState,'/joint_states',10)
    labels=node.create_publisher(Marker,'/review_label',10)
    images={r:node.create_publisher(Image,f'/review/{r}_image',2) for r in ('global','wrist')}
    params={'robot_state_publisher':{'ros__parameters':{'robot_description':(out/'review_only.urdf').read_text()}}}
    (out/'rsp.yaml').write_text(yaml.safe_dump(params))
    children=[];logs=[]
    try:
        for name,cmd in [('rsp',['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher','--ros-args','--params-file',str(out/'rsp.yaml')]),
                         ('rviz',['/opt/ros/jazzy/bin/rviz2','-d',str(out/'review.rviz')])]:
            log=(out/f'{name}.log').open('w');logs.append(log)
            children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
        start=time.monotonic();last=-1
        while rclpy.ok() and time.monotonic()-start < seconds:
            if any(p.poll() is not None for p in children):
                raise RuntimeError('RViz or state publisher exited; inspect logs')
            # Cycling saved states only. No robot transport or write API exists here.
            idx=int((time.monotonic()-start)//8)%len(bundle['states']);s=bundle['states'][idx]
            stamp=node.get_clock().now().to_msg()
            msg=JointState();msg.header.stamp=stamp
            msg.name=[f'Joint{i+1}' for i in range(6)]
            msg.position=[math.radians(q)*sign+off for q,sign,off in zip(s['joint_sdk_deg'],bundle['sign_hypothesis'],bundle['offset_hypothesis_rad'])]
            joints.publish(msg)
            m=Marker();m.header.frame_id='base_link';m.header.stamp=stamp;m.ns='review';m.id=0
            m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose.orientation.w=1.;m.pose.position.z=.48
            m.scale.z=.018;m.color.r=1.;m.color.g=.7;m.color.a=1.
            m.text='UNVERIFIED MODEL\nRECORDED pose '+str(idx)+' / '+str(len(bundle['states'])-1)+'\nAssumed signs +1, offsets 0'
            labels.publish(m)
            if idx!=last:
                for role,pub in images.items():
                    a=cv2.imread(s[role+'_image'])
                    if a is None:raise ValueError('Missing review image')
                    im=Image();im.header.stamp=stamp;im.header.frame_id=role+'_recorded_optical'
                    im.height,im.width=a.shape[:2];im.encoding='bgr8';im.step=a.shape[1]*3;im.data=a.tobytes();pub.publish(im)
                save(out/'displayed_state.json',dict(index=idx,**s))
                print(s['label'],flush=True);last=idx
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        for p in children:
            if p.poll() is None:p.terminate()
        for p in children:
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for log in logs:log.close()
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['build','replay']);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=600)
    a=p.parse_args()
    if a.mode=='build':build(a.output.resolve())
    else:replay(a.output.resolve(),a.seconds)
