#!/usr/bin/env python3
"""Run real VPR1 DMA tests, fail on any byte error, and retain measured stages."""
import argparse,hashlib,io,json,platform,sys,urllib.request
from pathlib import Path
from time import perf_counter_ns
import numpy as np
from PIL import Image
from board_transport import BoardTransport
from cpu_resize import NativeCPU
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'software'))
from preprocess import pack_rgbx,resize_floor

def stats(values):
    return {'samples':len(values),'p50_ms':float(np.median(values)/1e6),'p95_ms':float(np.percentile(values,95)/1e6)}

def acquire(url,dest):
    begin=perf_counter_ns()
    with urllib.request.urlopen(url,timeout=5) as response:
        raw=response.read();headers=dict(response.headers)
    received=perf_counter_ns()
    rgb=np.array(Image.open(io.BytesIO(raw)).convert('RGB'))
    decoded=perf_counter_ns()
    dest.write_bytes(raw)
    return rgb,{'url':url,'headers':headers,'jpeg_bytes':len(raw),
        'local_camera_api_fetch_ns':received-begin,'jpeg_decode_ns':decoded-received,
        'sha256':hashlib.sha256(raw).hexdigest(),'acquisition_total_ns':decoded-begin}

def run(args):
    args.output.mkdir(parents=True,exist_ok=False)
    evidence={'hardware':'RUNNING','platform':platform.platform(),'clock_hz':25000000,
        'local_route':'existing KR260 camera service/saved frame -> board CPU -> coherent DMA buffer -> MM2S -> VPR1 -> S2MM -> board CPU',
        'cross_device_input_transfer_ns':0,'cross_device_note':'No added host-to-board round trip on the native board-local route. Existing output delivery is measured separately.',
        'frames':[],'camera_acquisition':[]}
    native=NativeCPU()
    rng=np.random.default_rng(20261027)
    samples=[('pattern_640',rng.integers(0,256,(480,640,4),dtype=np.uint8)),
             ('odd_pattern',rng.integers(0,256,(13,17,4),dtype=np.uint8)),
             ('maximum',rng.integers(0,256,(1080,1920,4),dtype=np.uint8))]
    for path in args.jpeg:
        rgb=np.array(Image.open(path).convert('RGB'));samples.append((path.stem,rgb))
    for i,url in enumerate(args.camera_url):
        rgb,record=acquire(url,args.output/f'camera_{i}.jpg');samples.append((f'live_camera_{i}',rgb))
        evidence['camera_acquisition'].append(record)
    try:
        with BoardTransport() as transport:
            for label,source in samples:
                ih,iw=source.shape[:2]
                sizes=[(iw,ih,True),(max(1,iw//2),max(1,ih//2),False),(1,1,False)]
                if iw>=224 and ih>=224:sizes.append((224,224,False))
                for ow,oh,bypass in sizes:
                    is_rgb=source.shape[2]==3
                    sw_reference=source.copy() if bypass else resize_floor(source,ow,oh)
                    reference=pack_rgbx(sw_reference) if is_rgb else sw_reference
                    times=[];cpu=[];native_times=[];traces=[];max_error=0
                    for trial in range(args.repeats):
                        start=perf_counter_ns()
                        packed=pack_rgbx(source) if is_rgb else source.copy()
                        after_pack=perf_counter_ns()
                        actual,trace=transport.run(packed,ow,oh,bypass)
                        before_unpack=perf_counter_ns()
                        if is_rgb: delivered=actual[:,:,:3].copy()
                        else: delivered=actual
                        finish=perf_counter_ns()
                        trace['rgb_output_unpack_ns']=finish-before_unpack
                        trace['input_owned_copy_ns']=after_pack-start
                        trace['local_pipeline_total_ns']=finish-start
                        traces.append(trace);times.append(finish-start)
                        start=perf_counter_ns()
                        sw=source.copy() if bypass else resize_floor(source,ow,oh)
                        cpu.append(perf_counter_ns()-start)
                        start=perf_counter_ns()
                        native_out=native.resize(source,ow,oh)
                        native_times.append(perf_counter_ns()-start)
                        if not np.array_equal(native_out,sw_reference):raise RuntimeError('Native CPU mismatch')
                        delta=np.abs(actual.astype(np.int16)-reference.astype(np.int16))
                        max_error=max(max_error,int(delta.max(initial=0)))
                        if np.any(delta):raise RuntimeError(f'Hardware output mismatch: {label} {ow}x{oh} trial={trial}')
                        if not np.array_equal(sw,sw_reference):raise RuntimeError('Software reference mismatch')
                    name=f'{label}_{ow}x{oh}_'+('bypass' if bypass else 'resize')
                    actual.tofile(args.output/(name+'.rgbx'))
                    evidence['frames'].append({'name':name,'input':[iw,ih],'output':[ow,oh],
                        'bypass':bypass,'contract':('RGB888 input/output including RGBX packing/unpacking' if is_rgb else 'raw RGBX input/output'),
                        'repeats':args.repeats,'max_abs_byte_error':max_error,
                        'bytes_compared':int(reference.nbytes)*args.repeats,'sha256':hashlib.sha256(actual.tobytes()).hexdigest(),
                        'pl_local_end_to_end':stats(times),'software_same_input_output_contract':stats(cpu),'native_cpu_same_contract':stats(native_times),'measured_stages':traces})
        evidence['hardware']='HARDWARE_PASS'
    except BaseException as exc:
        evidence['hardware']='FAIL';evidence['error']=str(exc);raise
    finally:
        (args.output/'hardware_results.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'hardware':evidence['hardware'],'cases':len(evidence['frames']),
        'repeats':args.repeats,'result':str(args.output/'hardware_results.json')},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--jpeg',type=Path,action='append',default=[])
    p.add_argument('--camera-url',action='append',default=[])
    p.add_argument('--repeats',type=int,default=20);a=p.parse_args()
    if not 1<=a.repeats<=200:p.error('repeats must be 1..200')
    run(a)
