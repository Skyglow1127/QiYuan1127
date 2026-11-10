"""Record a bounded camera test and summarize observed timing/depth, without moving a robot.

Polling latest.json can miss results: observed output Hz is a lower bound, not
display FPS. Detector console counters separately report capture and inference Hz.
Depth variation includes physical motion; it is not absolute accuracy evidence.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]

def distribution(values):
    values = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not values:
        return None
    def quantile(q):
        index = (len(values)-1)*q
        low = int(index)
        return values[low]+(values[min(low+1,len(values)-1)]-values[low])*(index-low)
    return dict(count=len(values), median=quantile(.5), p05=quantile(.05), p95=quantile(.95),
                minimum=values[0], maximum=values[-1], stddev=statistics.pstdev(values))

def summarize(rows, log, returncode, elapsed_s, warmup_s):
    steady = [row for row in rows if row['observed_elapsed_s'] >= warmup_s]
    metrics = {}
    for field in ('processing_ms','inference_ms','frame_age_at_write_ms','decode_ms'):
        metrics[field] = distribution(row[field] for row in steady if field in row)
    captures, detections = [], []
    counters = re.findall(r'capture=([\d.]+)fps detection=([\d.]+)Hz skew_rejected=(\d+)',log)
    for capture,detection,_ in counters[1:]:  # First console interval includes startup.
        captures.append(float(capture)); detections.append(float(detection))
    metrics['console_capture_fps'] = distribution(captures)
    metrics['console_detection_hz'] = distribution(detections)
    tracks = defaultdict(list)
    for row in steady:
        counts = Counter(c['class'] for c in row['candidates'])
        for c in row['candidates']:
            if counts[c['class']] != 1 or not c.get('depth_measurement_valid'):
                continue
            xyz = c['surface_camera_m']['data']
            tracks[c['class']].append((xyz[2]*1000,c['depth_valid_ratio']))
    depth = {name:{'z_mm':distribution(v[0] for v in values),
                   'depth_valid_ratio':distribution(v[1] for v in values),
                   'scope':'only frames with one instance of this class; identity/motion not verified'}
             for name,values in tracks.items()}
    intervals = [b['observed_elapsed_s']-a['observed_elapsed_s'] for a,b in zip(steady,steady[1:])]
    return dict(status='captured' if returncode==0 and rows else 'failed', returncode=returncode,
        elapsed_s=elapsed_s, observed_results=len(rows), steady_results=len(steady), warmup_s=warmup_s,
        observed_result_interval_ms=distribution(v*1000 for v in intervals),
        observed_output_hz=(len(intervals)/sum(intervals)) if intervals else None,
        observed_unpaired_results=sum(not r.get('rgbd_pair_accepted') for r in steady),
        pairing=dict(Counter(r.get('rgbd_pairing','unknown') for r in steady)),
        metrics=metrics, depth_observations=depth,
        visual_motion_states=dict(Counter(r.get('visual_motion',{}).get('state','not_recorded') for r in steady)),
        motion_processing_ms=distribution(r['visual_motion']['processing_ms'] for r in steady if 'visual_motion' in r),
        candidate_sources=dict(Counter(c.get('source','unknown') for r in steady for c in r['candidates'])),
        appearance_track_depth_leaks=sum(c.get('valid',0) or c.get('depth_measurement_valid',0)
            for r in steady for c in r['candidates'] if c.get('source')=='temporal_appearance'),
        untrusted_motion_depth_leaks=sum(any(c.get('depth_measurement_valid') or c.get('valid') for c in r['candidates'])
            for r in steady if r.get('visual_motion',{}).get('enabled') and r['visual_motion']['state']!='still'),
        limitations=['Polling may miss results; output Hz is not display FPS or exposure latency.',
            'Depth variation includes motion/identity changes and does not establish distance accuracy.',
            'Contour accuracy requires image review and preferably reference masks.',
            'Host-receipt RGB-D pairing supports stationary scenes only.'])

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--mode',choices=['segment','fusion','hybrid','rtdetr','tabletop'],default='segment')
    p.add_argument('--record-interval-ms',type=int,default=0,help='Save sampled RGB-D frames on a separate worker')
    p.add_argument('--gui',action='store_true')
    p.add_argument('--config',type=Path,help='Scene-specific detector configuration')
    p.add_argument('--output',type=Path,default=ROOT/'output/live_checks')
    a=p.parse_args()
    if not math.isfinite(a.seconds) or not 5<=a.seconds<=3600:
        p.error('seconds must be 5..3600')
    # New directory on every run: a disconnected camera cannot reuse an old result.
    directory=a.output.resolve()/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+a.mode)
    directory.mkdir(parents=True)
    command=[str(ROOT/'build/detect'),'--mode',a.mode,'--seconds',str(a.seconds),'--output',str(directory)]
    if a.record_interval_ms:
        if not 200<=a.record_interval_ms<=60000: p.error('record interval must be 200..60000 ms')
        command += ['--record-interval-ms',str(a.record_interval_ms)]
    if a.config:
        command += ['--config',str(a.config.resolve())]
    if not a.gui:
        command.append('--headless')
    rows=[]; seen=None; first=None; start=time.monotonic()
    with (directory/'console.log').open('w') as log, (directory/'observations.jsonl').open('w') as observations:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        def sample():
            nonlocal seen,first
            path=directory/'latest.json'
            try:
                stamp=path.stat().st_mtime_ns
                if stamp==seen:
                    return
                row=json.loads(path.read_text())
            except (FileNotFoundError,json.JSONDecodeError):
                return
            seen=stamp
            if rows and rows[-1]['frame_id']==row['frame_id']:
                return
            if first is None:
                first=time.monotonic()
            row['observed_elapsed_s']=time.monotonic()-first
            rows.append(row)
            observations.write(json.dumps(row,ensure_ascii=False)+'\n')
        try:
            while process.poll() is None:
                sample()
                if time.monotonic()-start>a.seconds+30:
                    process.terminate()
                    break
                time.sleep(.005)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            sample()
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
    log=(directory/'console.log').read_text()
    summary=summarize(rows,log,process.returncode,time.monotonic()-start,2.)
    summary.update(mode=a.mode,gui_requested=a.gui,directory=str(directory),command=command)
    (directory/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(log)
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    if summary['status']=='failed':
        raise SystemExit(1)

if __name__=='__main__':
    main()
