#!/usr/bin/env python3
"""Compare visual relative rotation against actual feedback, not command targets."""
import argparse,base64,json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import kr260_handeye_live as live


def pose(folder, board):
    record=json.loads((folder/'capture.json').read_text())
    if not record.get('complete'):raise ValueError('Incomplete capture: '+str(folder))
    poses=[];errors=[]
    for i in range(4):
        data=json.loads((folder/f'camera_{i}.json').read_text())
        data['image_base64']=base64.b64encode((folder/f'color_{i}.jpg').read_bytes()).decode()
        result=live.inspect(data,board);poses.append(result['pose']);errors.append(result['reprojection_rms_px'])
    rotations=Rotation.from_matrix(np.array(poses)[:,:3,:3]);mean=rotations.mean()
    T=np.eye(4);T[:3,:3]=mean.as_matrix();T[:3,3]=np.mean(np.array(poses)[:,:3,3],axis=0)
    spread=float(np.max(np.rad2deg((mean.inv()*rotations).magnitude())))
    return dict(q=record['observation']['joint_median'],T=T.tolist(),rotation_max_deviation_from_mean_deg=spread,rms_px=errors)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True);p.add_argument('sessions',type=Path,nargs='+')
    args=p.parse_args();board=json.loads(live.DEFAULT_BOARD.read_text())
    observations=[pose(folder,board) for folder in args.sessions];pairs=[]
    for i in range(1,len(observations)):
        a,b=observations[i-1:i+1];A,B=np.array(a['T']),np.array(b['T']);dq=np.array(b['q'])[:6]-np.array(a['q'])[:6]
        main_joint=int(np.argmax(abs(dq)));angle=float(np.rad2deg(Rotation.from_matrix(A[:3,:3]@B[:3,:3].T).magnitude()))
        pairs.append(dict(before=str(args.sessions[i-1]),after=str(args.sessions[i]),
                          feedback_delta_deg=dq.tolist(),dominant_sdk_joint=main_joint,
                          dominant_feedback_rotation_deg=float(abs(dq[main_joint])),
                          visual_rotation_deg=angle,
                          visual_minus_dominant_feedback_deg=angle-float(abs(dq[main_joint])),
                          other_joint_total_absolute_delta_deg=float(sum(abs(dq))-abs(dq[main_joint]))))
    report=dict(schema='d1-relative-motion-diagnostic-v1',observations=observations,pairs=pairs,
                joint_mapping_validated=False,handeye_calibrated=False,
                note='Small within-pose spread is repeatability, not accuracy. A single-joint rotation magnitude should agree independent of constant camera extrinsics; other joint drift, intrinsics, board flatness and mount flex remain possible causes of disagreement.')
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(pairs,indent=2))


if __name__=='__main__':main()
