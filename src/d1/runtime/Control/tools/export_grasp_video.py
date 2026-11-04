#!/usr/bin/env python3
"""Reconstruct a dual-camera archive from one recorded frames.jsonl.

Receipt timestamps preserve elapsed duration. This never records new images,
interpolates motion, or treats missing/stale frames as fresh. Source timestamps
are host receipt times, not proof of simultaneous camera exposure.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

ROLES={'global':'CC1365302K0','wrist':'CC1365301EY'}


def load_frames(path):
    path=Path(path).resolve();frames={role:[] for role in ROLES}
    for line in path.read_text().splitlines():
        if not line.strip():continue
        row=json.loads(line);role=row.get('role')
        if role not in ROLES or row.get('serial')!=ROLES[role]:
            raise ValueError('Unknown role or incorrect camera serial')
        stamp=row.get('host_monotonic')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):
            raise ValueError('Missing finite original host timestamp')
        if frames[role] and stamp<=frames[role][-1]['host_monotonic']:
            raise ValueError('Non-increasing camera timestamps; do not merge clock domains')
        image=(path.parent/row['file']).resolve()
        if image.parent!=path.parent or not image.is_file():
            raise ValueError('Source image must exist beside frames.jsonl')
        row=dict(row,path=str(image),sha256=hashlib.sha256(image.read_bytes()).hexdigest())
        frames[role].append(row)
    if not all(frames.values()):raise ValueError('Both camera streams required')
    return frames


def choose_frame(rows,stamp,max_gap_s):
    index=bisect.bisect_right([row['host_monotonic'] for row in rows],stamp)-1
    if index<0:return None,None
    row=rows[index];age=stamp-row['host_monotonic']
    return (row if age<=max_gap_s else None),age


def export(frames_path,output,archive_label,fps=10.,max_gap_s=.8):
    import cv2
    import numpy as np
    if not archive_label.strip():raise ValueError('An explicit historical recording label is required')
    if not 1<=fps<=60 or not 0<max_gap_s<=5:raise ValueError('Invalid fps/gap bound')
    frames=load_frames(frames_path);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    start=min(rows[0]['host_monotonic'] for rows in frames.values())
    end=max(rows[-1]['host_monotonic'] for rows in frames.values())
    count=math.ceil((end-start)*fps)+1
    target=output/'historical_dual_camera.mp4'
    writer=cv2.VideoWriter(str(target),cv2.VideoWriter_fourcc(*'mp4v'),fps,(1280,540))
    if not writer.isOpened():raise RuntimeError('OpenCV MP4 encoder unavailable; source logs unchanged')
    cache={};missing={role:0 for role in ROLES};used={role:set() for role in ROLES}
    try:
        for number in range(count):
            stamp=start+number/fps
            canvas=np.zeros((540,1280,3),np.uint8)
            cv2.putText(canvas,'ARCHIVE: '+archive_label,(10,22),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,230,255),1)
            cv2.putText(canvas,f'+{stamp-start:.2f}s | RECEIPT TIME ALIGNMENT | recorded snapshots held; no motion interpolation',
                (10,46),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
            for offset,role in enumerate(ROLES):
                row,age=choose_frame(frames[role],stamp,max_gap_s)
                tile=canvas[60:540,offset*640:(offset+1)*640]
                if row is None:
                    missing[role]+=1
                    text=f'{role}: NO RECORDED FRAME' if age is None else f'{role}: RECORDING GAP {age:.3f}s'
                    cv2.putText(tile,text,(12,230),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),1)
                    continue
                filename=row['path']
                if cache.get(role,{}).get('path')!=filename:
                    raw=Path(filename).read_bytes()
                    if hashlib.sha256(raw).hexdigest()!=row['sha256']:raise ValueError('Source image changed while exporting')
                    image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
                    if image is None or image.shape!=(480,640,3):raise ValueError('Expected original 640x480 image')
                    cache[role]={'path':filename,'image':image}
                tile[:]=cache[role]['image'];used[role].add(filename)
                cv2.rectangle(tile,(0,0),(640,24),(0,0,0),-1)
                label=f'{role} frame {row.get("index","?")} | receipt +{row["host_monotonic"]-start:.3f}s | held {age:.3f}s'
                cv2.putText(tile,label,(7,17),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
            writer.write(canvas)
    finally:writer.release()
    # Decode final output: prove video container/frame count, not physical success.
    capture=cv2.VideoCapture(str(target));decoded=0
    while True:
        ok,frame=capture.read()
        if not ok:break
        decoded+=1
    capture.release()
    if decoded!=count:raise RuntimeError(f'Encoded video failed decode check: {decoded}/{count}')
    manifest=dict(schema='d1-historical-dual-video-v1',archive_label=archive_label,
        new_recording=False,robot_commands_sent=0,source_frames=str(Path(frames_path).resolve()),
        source_frames_sha256=hashlib.sha256(Path(frames_path).read_bytes()).hexdigest(),
        clock='recorded host_monotonic receipt times; not synchronized exposure timestamps',
        first_timestamp=start,last_timestamp=end,recorded_span_s=end-start,
        video_duration_s=count/fps,fps=fps,encoded_frames=count,decoded_frames=decoded,
        timeline_quantization_s=1/fps,max_held_frame_age_s=max_gap_s,
        unavailable_video_frames_by_camera=missing,source_frames_count={r:len(v) for r,v in frames.items()},
        unique_images_displayed={r:len(v) for r,v in used.items()},
        source_images=[{'path':row['path'],'sha256':row['sha256']} for rows in frames.values() for row in rows],
        video=str(target.resolve()),video_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        interpretation='Existing logged frames replayed at receipt-time positions; held frames and gaps visibly labelled. No generated or newly captured grasp footage.')
    (output/'video_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({key:manifest[key] for key in ('video','recorded_span_s','video_duration_s','decoded_frames','new_recording')}))
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--archive-label',required=True)
    parser.add_argument('--fps',type=float,default=10.)
    parser.add_argument('--max-gap-s',type=float,default=.8)
    args=parser.parse_args();export(args.frames,args.output,args.archive_label,args.fps,args.max_gap_s)


if __name__=='__main__':main()
