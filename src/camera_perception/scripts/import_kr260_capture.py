"""Validate a KR260 capture_orbbec.py bundle and convert it to C++ RGB-D replay.

No proportional depth resize, inferred calibration, or millimetre scale guessing.
Run using .venv-models/bin/python (OpenCV + NumPy).
"""
import argparse
import gzip
import hashlib
import json
import math
import re
from pathlib import Path

def convert(source, destination, max_skew_ms=25.):
    import cv2
    import numpy as np
    report=json.loads((source/'report.json').read_text())
    if report.get('alignment_mode') not in ('SDK_D2C_HW','SDK_D2C_SW'):
        raise ValueError('Bundle does not confirm SDK depth-to-color alignment')
    if report.get('pairing')!='host_receipt_approximate_stationary_only' or report.get('accepted_pairs',0)<1:
        raise ValueError('No accepted RGB-D pair')
    color_meta=report['streams']['color']['last_frame']
    depth_meta=report['streams']['depth']['last_frame']
    params=report['camera_parameters']; intrinsic=params['color_intrinsic']
    if params.get('is_mirrored'):
        raise ValueError('Mirrored calibration unsupported')
    w,h=int(color_meta['width']),int(color_meta['height'])
    if not 1<=w<=4096 or not 1<=h<=4096 or (w,h)!=(depth_meta['width'],depth_meta['height']) or (w,h)!=(intrinsic['width'],intrinsic['height']):
        raise ValueError('Depth/color/intrinsics dimensions disagree')
    times=[float(m['host_received_ms']) for m in (color_meta,depth_meta)]
    if not all(math.isfinite(v) and v>0 for v in times) or abs(times[0]-times[1])>max_skew_ms:
        raise ValueError('Saved pair exceeds allowed host-receipt skew')
    unit=float(depth_meta['mm_per_unit'])/1000
    if not math.isfinite(unit) or unit<=0:
        raise ValueError('Invalid depth unit')
    jpeg=(source/'color.jpg').read_bytes()
    with gzip.open(source/'depth_u16le.raw.gz','rb') as compressed:
        raw=compressed.read(w*h*2+1)
    if len(raw)!=w*h*2:
        raise ValueError('Depth byte count mismatch')
    for name,data in [('color',jpeg),('depth',raw)]:
        if hashlib.sha256(data).hexdigest()!=report['streams'][name]['sha256']:
            raise ValueError('Transfer SHA256 mismatch: '+name)
    color=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    if color is None or color.shape[:2]!=(h,w):
        raise ValueError('Invalid JPEG dimensions')
    depth=np.frombuffer(raw,dtype='<u2').reshape(h,w).astype(np.uint16)
    k=intrinsic
    K=np.array([[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1]],dtype=np.float64)
    d=params['color_distortion']
    distortion=np.array([[d[name] for name in ('k1','k2','p1','p2','k3','k4','k5','k6')]],dtype=np.float64)
    if not np.isfinite(K).all() or not np.isfinite(distortion).all() or min(k['fx'],k['fy'])<=0:
        raise ValueError('Invalid calibration values')
    destination.mkdir(parents=True,exist_ok=True)
    if not cv2.imwrite(str(destination/'color.png'),color) or not cv2.imwrite(str(destination/'depth.png'),depth):
        raise IOError('Cannot write replay images')
    fs=cv2.FileStorage(str(destination/'snapshot.yml'),cv2.FILE_STORAGE_WRITE)
    for key,value in dict(color='color.png',depth='depth.png',depth_unit_m=unit,aligned_to_color=1,
        width=w,height=h,camera_matrix=K,distortion=distortion,host_received_ms=times[0],
        depth_host_received_ms=times[1],rgbd_pairing=report['pairing'],
        color_device_us=float(color_meta['device_timestamp_us']),depth_device_us=float(depth_meta['device_timestamp_us'])).items():
        fs.write(key,value)
    fs.release()
    # OpenCV 5 writes integral-valued doubles without a decimal point. OpenCV 4
    # parses those as 32-bit integers, corrupting epoch timestamps. Force REAL
    # tokens for cross-version replay compatibility.
    path=destination/'snapshot.yml'
    text=path.read_text()
    for key,value in dict(host_received_ms=times[0],depth_host_received_ms=times[1],
                         color_device_us=float(color_meta['device_timestamp_us']),
                         depth_device_us=float(depth_meta['device_timestamp_us'])).items():
        text=re.sub(r'^'+key+r':.*$',key+': '+format(value,'.17e'),text,flags=re.MULTILINE)
    path.write_text(text)
    provenance=dict(source=str(source.resolve()),sdk=report['sdk'],alignment=report['alignment_mode'],
        pair_host_skew_ms=abs(times[0]-times[1]),sha256_verified=True,depth_unit_m=unit,
        note='Stationary-scene approximate pairing, not exposure synchronization. Replay is not live.')
    (destination/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    return destination/'snapshot.yml'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    print(convert(a.source,a.output))

if __name__=='__main__':
    main()
