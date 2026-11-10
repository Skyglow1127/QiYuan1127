#!/usr/bin/env python3
"""Verify a KR SDK D2C capture and export the existing Camera replay format.
No robot pose is inferred or generated. An overlay is visual QA, not a measured
registration error. KR SDK timestamps belong to the KR clock, not the AIPC.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np

def prepare(folder):
    report = json.loads((folder/'report.json').read_text())
    if report.get('alignment_mode') not in ('SDK_D2C_HW', 'SDK_D2C_SW'):
        raise ValueError('SDK-aligned capture required')
    if report.get('accepted_pairs', 0) <= 0:
        raise ValueError('No accepted RGB-D pair')
    color_bytes = (folder/'color.jpg').read_bytes()
    depth_bytes = gzip.decompress((folder/'depth_u16le.raw.gz').read_bytes())
    for name, raw in (('color', color_bytes), ('depth', depth_bytes)):
        if hashlib.sha256(raw).hexdigest() != report['streams'][name]['sha256']:
            raise ValueError(name + ' checksum mismatch')
    cm = report['streams']['color']['last_frame']
    dm = report['streams']['depth']['last_frame']
    k = report['camera_parameters']['color_intrinsic']
    d = report['camera_parameters']['color_distortion']
    if report['camera_parameters']['is_mirrored']:
        raise ValueError('Mirrored calibration unsupported')
    shapes = {(m['width'], m['height']) for m in (cm, dm, k)}
    if len(shapes) != 1 or len(depth_bytes) != dm['width']*dm['height']*2:
        raise ValueError('Frame/intrinsic dimensions mismatch')
    color = cv2.imdecode(np.frombuffer(color_bytes, np.uint8), cv2.IMREAD_COLOR)
    if color is None or color.shape[:2] != (cm['height'], cm['width']):
        raise ValueError('Invalid JPEG dimensions')
    depth = np.frombuffer(depth_bytes, '<u2').reshape(dm['height'], dm['width'])
    scale = dm['mm_per_unit']/1000.0
    K = np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1]], np.float64)
    dist = np.array([[d[n] for n in ('k1','k2','p1','p2','k3','k4','k5','k6')]], np.float64)
    if not np.isfinite(K).all() or not np.isfinite(dist).all() or min(k['fx'], k['fy']) <= 0 or not np.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid camera parameters')
    skew = abs(cm['host_received_ms']-dm['host_received_ms'])
    if skew > 40 or min(cm['host_received_ms'],dm['host_received_ms']) <= 0:
        raise ValueError('Unacceptable host-receipt skew')
    # Convert the no-return sentinel to zero; retain original raw scale in YAML.
    saved_depth = depth.copy(); saved_depth[saved_depth == 65535] = 0
    for name, pixels in (('snapshot_color.png', color), ('snapshot_depth.png', saved_depth)):
        if not cv2.imwrite(str(folder/name), pixels): raise IOError(name)
    fs = cv2.FileStorage(str(folder/'snapshot.yml'), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened(): raise IOError('snapshot.yml')
    values = {'color':'snapshot_color.png','depth':'snapshot_depth.png','depth_unit_m':scale,
              'aligned_to_color':1,'width':cm['width'],'height':cm['height'],
              'camera_matrix':K,'distortion':dist,'host_received_ms':float(cm['host_received_ms']),
              'depth_host_received_ms':float(dm['host_received_ms']),
              'color_device_us':float(cm['device_timestamp_us']),'depth_device_us':float(dm['device_timestamp_us']),
              'rgbd_pairing':'host_receipt_approximate_stationary_only',
              'clock_source':'KR260 SDK host receipt; not synchronized with AIPC',
              'robot_stationarity_verified':0,'handeye_sample_ready':0}
    for key,value in values.items(): fs.write(key,value)
    fs.release()
    meters = depth*scale
    valid = (depth>0) & (depth<65535)
    shade = np.uint8(np.clip((meters-.2)/1.0,0,1)*255)
    heat = cv2.applyColorMap(255-shade, cv2.COLORMAP_TURBO); heat[~valid] = 0
    overlay = color.copy()
    mixed = cv2.addWeighted(color,.6,heat,.4,0)
    overlay[valid] = mixed[valid]
    panels = []
    for title,img in [('RGB',color),('SDK aligned depth: 0.2-1.2 m',heat),('RGB + aligned depth (visual QA)',overlay)]:
        panel = cv2.copyMakeBorder(img,36,0,0,0,cv2.BORDER_CONSTANT,value=(25,25,25))
        cv2.putText(panel,title,(10,24),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA)
        panels.append(panel)
    if not cv2.imwrite(str(folder/'alignment-preview.jpg'),np.hstack(panels)): raise IOError('preview')
    result={'dimensions':[cm['width'],cm['height']], 'host_pair_skew_ms':skew,
            'alignment_mode':report['alignment_mode'],'valid_depth_fraction':float(valid.mean()),
            'files_verified':True,'handeye_calibrated':False,'robot_pose_available':False}
    (folder/'snapshot-check.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result))

if __name__ == '__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('folder', type=Path)
    prepare(ap.parse_args().folder)
