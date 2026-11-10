#!/usr/bin/env python3
"""Bounded USB PPP capture -> verified transfer -> RGB-D replay and stability report.

Run with .venv-models/bin/python. This is sampled offline analysis, not live video.
Uses existing SSH keys/known_hosts, never changes networking or moves a robot.
"""
import argparse
from collections import Counter
from datetime import datetime
import ipaddress
import json
import math
from pathlib import Path
import re
import shlex
import statistics
import subprocess
import tempfile
import time
import uuid
from zoneinfo import ZoneInfo

from import_kr260_capture import convert

ROOT=Path(__file__).resolve().parents[1]

def distribution(values):
    values=sorted(float(x) for x in values if x is not None and math.isfinite(float(x)))
    if not values:
        return None
    return {'count':len(values),'median':statistics.median(values),'minimum':values[0],
            'maximum':values[-1],'stddev':statistics.pstdev(values) if len(values)>1 else None}

def require_ppp_route(address):
    route=subprocess.run(['ip','-j','route','get',address],capture_output=True,text=True,check=True,timeout=10)
    routes=json.loads(route.stdout)
    if not routes or not routes[0].get('dev','').startswith('ppp'):
        raise RuntimeError('USB PPP route is absent. Restore the existing PPP service before capture; no SSH upload was attempted.')

def match_boxes(previous,current,threshold=.4,margin=.15):
    """Conservative adjacent-sample association, not verified physical identity."""
    def iou(a,b):
        if a['class']!=b['class'] or a['source']!=b['source']:return 0.
        x,y,w,h=a['box_xywh'];u,v,s,t=b['box_xywh']
        intersection=max(0,min(x+w,u+s)-max(x,u))*max(0,min(y+h,v+t)-max(y,v))
        union=w*h+s*t-intersection
        return intersection/union if union>0 else 0.
    matrix=[[iou(a,b) for b in previous] for a in current]
    def unique_best(values):
        order=sorted(range(len(values)),key=lambda i:values[i],reverse=True)
        if not order or values[order[0]]<threshold:return None
        if len(order)>1 and values[order[0]]-values[order[1]]<margin:return None
        return order[0]
    matches={}
    for i,row in enumerate(matrix):
        j=unique_best(row)
        if j is not None and unique_best([line[j] for line in matrix])==i:matches[i]=j
    return matches

def analyze_bundle(bundle,output,config,run):
    import cv2
    import numpy as np
    report=json.loads((bundle/'report.json').read_text())
    samples=[]
    for item in report.get('snapshots',[]):
        name=item['directory']
        if not re.fullmatch(r'sample_[0-9]{3}',name):
            raise ValueError('Invalid sample directory in capture report')
        samples.append((bundle/name,float(item['elapsed_s'])))
    samples.append((bundle,None))
    rows=[];previous=None;ious=[];centers=[];tiles=[]
    prior_objects=[];prior_masks=[];tracks={};next_track=0
    for i,(source,elapsed) in enumerate(samples):
        replay=output/f'replay_{i:03d}';result_dir=output/f'result_{i:03d}'
        metadata=convert(source,replay)
        run([str(ROOT/'build/detect'),'--config',str(config),'--replay',str(metadata),
             '--output',str(result_dir)],f'analyze_{i:03d}',90)
        result=json.loads((result_dir/'replay_result.json').read_text())
        measured=[(j,c) for j,c in enumerate(result['candidates']) if c.get('depth_measurement_valid')]
        row={'sample':i,'sample_elapsed_s':elapsed,'host_received_ms':result['host_received_ms'],
             'processing_ms':result['processing_ms'],'candidate_count':len(result['candidates']),
             'statuses':dict(Counter(c['status'] for c in result['candidates']))}
        objects=[];masks=[]
        for j,c in enumerate(result['candidates']):
            obj={key:c[key] for key in ('class','source','box_xywh','status','valid')}
            obj.update(sample=i,host_received_ms=result['host_received_ms'])
            if c.get('depth_measurement_valid'):
                obj.update(z_mm=1000*c['optical_axis_depth_m'],range_mm=1000*c['camera_range_m'],
                           surface_pixel=c['surface_pixel'])
            path=result_dir/f'replay_rgb_contour_{j}.png'
            mask=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE) if path.is_file() else None
            if mask is not None:
                mask=mask>0;ys,xs=np.nonzero(mask)
                if len(xs):obj['rgb_centroid_xy']=[float(xs.mean()),float(ys.mean())]
            objects.append(obj);masks.append(mask)
        matches=match_boxes(prior_objects,objects)
        for j,obj in enumerate(objects):
            if j in matches:
                old=matches[j];obj['track_id']=prior_objects[old]['track_id']
                if masks[j] is not None and prior_masks[old] is not None:
                    union=np.count_nonzero(masks[j]|prior_masks[old])
                    if union:obj['rgb_mask_iou_previous_sample']=float(np.count_nonzero(masks[j]&prior_masks[old])/union)
            else:
                obj['track_id']=next_track;next_track+=1
            tracks.setdefault(obj['track_id'],[]).append(obj)
        row['objects']=objects
        prior_objects,prior_masks=objects,masks
        if len(measured)==1:
            j,c=measured[0];row.update(z_mm=1000*c['optical_axis_depth_m'],
                range_mm=1000*c['camera_range_m'],surface_pixel=c['surface_pixel'],
                missing_boundary_ratio=c['missing_depth_boundary_ratio'])
            mask=cv2.imread(str(result_dir/f'replay_rgb_contour_{j}.png'),cv2.IMREAD_GRAYSCALE)
            if mask is not None:
                mask=mask>0;ys,xs=np.nonzero(mask)
                if len(xs):
                    center=[float(xs.mean()),float(ys.mean())];centers.append(center)
                    row['rgb_centroid_xy']=center
                    if previous is not None:
                        union=np.count_nonzero(mask|previous)
                        iou=float(np.count_nonzero(mask&previous)/union) if union else 0.
                        ious.append(iou);row['rgb_mask_iou_previous_sample']=iou
                    previous=mask
                else:previous=None
            else:previous=None
        else:previous=None
        rows.append(row)
        picture=cv2.imread(str(result_dir/'replay_annotated.png'))
        if picture is not None:
            # Explicitly label the montage as recorded samples, never a live preview.
            top=np.zeros((32,picture.shape[1],3),np.uint8)
            label=f'RECORDED sample {i} | '+(f'+{elapsed:.2f}s' if elapsed is not None else 'last saved pair')
            cv2.putText(top,label,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            tiles.append(np.vstack([top,picture]))
    if tiles:
        if len(tiles)%2:tiles.append(np.zeros_like(tiles[0]))
        montage=np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)])
        cv2.imwrite(str(output/'samples_overview.jpg'),montage)
    fixed=report.get('depth_samples',[])
    track_summaries=[]
    for key,observations in tracks.items():
        track_summaries.append({'track_id':key,'class':observations[0]['class'],
            'sample_count':len(observations),'samples':[o['sample'] for o in observations],
            'z_mm':distribution(o.get('z_mm') for o in observations),
            'rgb_mask_iou_previous_sample':distribution(o.get('rgb_mask_iou_previous_sample') for o in observations),
            'rgb_centroid_x_px':distribution(o['rgb_centroid_xy'][0] for o in observations if 'rgb_centroid_xy' in o),
            'rgb_centroid_y_px':distribution(o['rgb_centroid_xy'][1] for o in observations if 'rgb_centroid_xy' in o),
            'statuses':dict(Counter(o['status'] for o in observations))})
    return {'status':'completed','source_bundle':str(bundle),
        'sequence_scope':report.get('sequence_scope','one_bounded_capture_session'),
        'capture_streams':{k:{field:v.get(field) for field in
            ('frames','unique_indices','observed_fps','arrival_interval_ms')} for k,v in report['streams'].items()},
        'accepted_rgbd_pairs':report.get('accepted_pairs'),'samples':rows,
        'sparse_object_tracks':track_summaries,
        'association_scope':'Adjacent-sample mutual unambiguous box overlap and matching labels/sources; assumes stationary view, not verified identity.',
        'single_measured_candidate_z_mm':distribution(r.get('z_mm') for r in rows),
        'rgb_mask_iou_previous_sample':distribution(ious),
        'rgb_centroid_x_px':distribution(c[0] for c in centers),
        'rgb_centroid_y_px':distribution(c[1] for c in centers),
        'fixed_depth_roi_xywh':report.get('depth_roi_xywh'),
        'fixed_roi_median_depth_mm':distribution(s['median_mm'] for s in fixed),
        'fixed_roi_valid_fraction':distribution(s['valid_fraction'] for s in fixed),
        'limitations':['Sampled offline replay, not real-time PPP video or display FPS.',
            'Mask IoU measures agreement between sampled predictions, not accuracy against reference masks.',
            'Single-candidate comparison does not verify identity or stationarity; variation includes motion.',
            'No ruler ground truth: depth repeatability is not absolute accuracy.',
            'RGB contour estimates do not fill depth or authorize robot motion.']}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='kr260',help='Existing SSH alias with key configured')
    p.add_argument('--address',default='192.168.28.2',help='USB PPP peer address')
    p.add_argument('--remote-root',default='/home/ubuntu/kr260-camera')
    p.add_argument('--seconds',type=float,default=20)
    p.add_argument('--snapshots',type=int,default=3)
    p.add_argument('--depth-roi',type=int,nargs=4,metavar=('X','Y','W','H'))
    p.add_argument('--config',type=Path,default=ROOT/'configs/kr260_tabletop.yml')
    p.add_argument('--output',type=Path,default=ROOT/'output/ppp_checks')
    p.add_argument('--local-bundle',type=Path,help='Analyze an existing capture without contacting the board')
    a=p.parse_args()
    if not math.isfinite(a.seconds) or not 5<=a.seconds<=120 or not 0<=a.snapshots<=10:
        p.error('seconds must be 5..120, snapshots 0..10')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.@-]*',a.host):p.error('invalid SSH host')
    if not re.fullmatch(r'/[A-Za-z0-9_./-]+',a.remote_root) or '..' in Path(a.remote_root).parts:
        p.error('remote-root must be a simple absolute path')
    try:ipaddress.IPv4Address(a.address)
    except ValueError:p.error('address must be an IPv4 address')
    if a.depth_roi and (min(a.depth_roi[:2])<0 or min(a.depth_roi[2:])<=0):p.error('invalid depth ROI')
    config=a.config.resolve()
    if not config.is_file() or not (ROOT/'build/detect').is_file():p.error('build detect and provide a valid config first')
    # Fail before starting a remote capture if the analysis runtime is unavailable.
    import cv2
    import numpy
    token=datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
    output=a.output.resolve()/token;output.mkdir(parents=True)
    (output/'config_used.yml').write_text(config.read_text())
    timings={};summary={'status':'running','output':str(output)}
    def run(command,stage,timeout):
        print(stage,flush=True);start=time.monotonic()
        with (output/(stage+'.log')).open('w') as log:
            log.write(shlex.join(command)+'\n');log.flush()
            subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=timeout,check=True)
        timings[stage]=time.monotonic()-start
    print('OUTPUT',output,flush=True)
    try:
        if a.local_bundle:
            bundle=a.local_bundle.resolve()
        else:
            require_ppp_route(a.address)
            remote_script=a.remote_root+'/capture_'+token+'.py'
            remote_output=a.remote_root+'/ppp_check_'+token
            # Reuse one authenticated SSH connection to reduce PPP setup overhead.
            with tempfile.TemporaryDirectory(prefix='camera-ssh-') as control:
                options=['-o','HostName='+a.address,'-o','BatchMode=yes','-o','ConnectTimeout=15',
                         '-o','StrictHostKeyChecking=yes','-o','ServerAliveInterval=15',
                         '-o','ServerAliveCountMax=2','-o','ControlMaster=auto',
                         '-o','ControlPersist=30','-o','ControlPath='+control+'/socket']
                try:
                    run(['scp',*options,str(ROOT/'scripts/kr260_capture.py'),a.host+':'+remote_script],'upload',120)
                    args=['timeout','--signal=INT',str(int(a.seconds)+45),'env',
                          'LD_LIBRARY_PATH='+a.remote_root+'/lib','python3','-u',remote_script,
                          '--mode','both','--align','--max-pair-skew-ms','25','--seconds',str(a.seconds),
                          '--snapshots',str(a.snapshots),'--output',remote_output]
                    if a.depth_roi:args+=['--depth-roi',*map(str,a.depth_roi)]
                    command='cd '+shlex.quote(a.remote_root)+' && '+shlex.join(args)
                    run(['ssh',*options,a.host,command],'capture',a.seconds+120)
                    bundle=output/'capture'
                    run(['scp',*options,'-r',a.host+':'+remote_output,str(bundle)],'download',300)
                    summary['remote_capture']=remote_output
                finally:
                    subprocess.run(['ssh',*options,'-O','exit',a.host],stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL,timeout=20,check=False)
        remote=summary.get('remote_capture')
        summary=analyze_bundle(bundle,output,config,run)
        summary['remote_capture']=remote
    except (Exception,KeyboardInterrupt) as e:
        summary.update(status='failed',error=f'{type(e).__name__}: {e}')
        print('FAILED; retained logs:',output,flush=True)
    finally:
        summary['stage_seconds']=timings
        (output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    print('SUMMARY',output/'summary.json',flush=True)
    return 0 if summary['status']=='completed' else 1

if __name__=='__main__':
    raise SystemExit(main())
