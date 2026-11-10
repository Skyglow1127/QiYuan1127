"""Actual AMD VAIML BF16 vision session, separate from ROCm's dynamic libraries."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import socket
import sys
import time
import numpy as np
import onnxruntime as ort
from common import ROOT, stats, write_json
from protocol import recv_into, send_json, recv_json

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--socket',type=Path,default=ROOT/'cache/vision.sock')
    ap.add_argument('--verify-only',action='store_true')
    ap.add_argument('--evidence',type=Path,default=ROOT/'results/npu_first_compile')
    ap.add_argument('--allow-cpu',action='store_true')
    ap.add_argument('--model',type=Path,default=ROOT/'models/vision_batch1.onnx')
    ap.add_argument('--config',type=Path,default=ROOT/'code/vaiml_tp1.json')
    args=ap.parse_args(); args.evidence.mkdir(parents=True,exist_ok=True)
    # Reuse the earlier audited evidence parser and independent compiler watcher.
    from reused_npu_audit import CompilerEvidenceWatcher, audit_session
    model=args.model.resolve(); config=args.config.resolve()
    identity=hashlib.sha256(model.read_bytes()+config.read_bytes()).hexdigest()[:16]
    # The graph contains stable external tensor names; manifest binds their bytes.
    manifest=json.loads(model.with_suffix('.manifest.json').read_text())
    identity=hashlib.sha256((identity+json.dumps(manifest['tensor_mapping'],sort_keys=True)).encode()).hexdigest()[:16]
    cache=ROOT/'cache/npu'; key='real_pi05_vision_'+identity
    ck=cache/key; ck.mkdir(parents=True,exist_ok=True)
    ep_report=ck/'vitisai_ep_report.json'
    os.environ['XLNX_ONNX_EP_REPORT_FILE']=str(ep_report)
    opt=ort.SessionOptions(); opt.intra_op_num_threads=8; opt.inter_op_num_threads=1
    opt.log_severity_level=2
    opt.graph_optimization_level=ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    opt.enable_profiling=True; opt.profile_file_prefix=str(args.evidence/'ort_vision')
    if not args.allow_cpu: opt.add_session_config_entry('session.disable_cpu_ep_fallback','1')
    options={'config_file':str(config),'cache_dir':str(cache),'cache_key':key,'enable_cache_file_io_in_mem':'0'}
    providers=[('VitisAIExecutionProvider',options)]
    if args.allow_cpu: providers.append('CPUExecutionProvider')
    report=dict(model=str(model),cache=str(ck),provider_options=options,
                ort_version=ort.__version__,available_providers=ort.get_available_providers(),
                allow_cpu=args.allow_cpu,created_utc=__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat())
    write_json(args.evidence/'session.json',report)
    print('Creating real-weight NPU session',flush=True)
    watcher=CompilerEvidenceWatcher(ck); watcher.start(); t=time.perf_counter()
    try:
        session=ort.InferenceSession(str(model),sess_options=opt,providers=providers,enable_fallback=False)
    finally:
        report['compiler_watcher']=watcher.stop()
        report['session_create_seconds']=time.perf_counter()-t
        write_json(args.evidence/'session.json',report)
    session.disable_fallback()
    x=np.load(ROOT/'results/reference/pixels.npy')
    input_name=session.get_inputs()[0].name
    device_batch=session.get_inputs()[0].shape[0]
    if device_batch not in (1,2): raise ValueError(f'Unsupported camera batch {device_batch}')
    from patchify import patchify
    def prepare(pixels): return patchify(pixels) if input_name=='patches' else pixels
    last_ort_seconds=0.; last_concat_seconds=0.
    def run_vision(feed):
        nonlocal last_ort_seconds,last_concat_seconds
        # Keep global, wrist order; static batch-1 avoids the observed batch-2 fault.
        chunks=[];last_ort_seconds=0.;last_concat_seconds=0.
        for i in range(0,2,device_batch):
            t0=time.perf_counter();chunk=session.run(None,{input_name:feed[i:i+device_batch]})[0]
            last_ort_seconds+=time.perf_counter()-t0;chunks.append(chunk)
        if len(chunks)==1: return chunks[0]
        t0=time.perf_counter();result=np.concatenate(chunks,axis=0)
        last_concat_seconds=time.perf_counter()-t0
        return result
    feed=prepare(x)
    t=time.perf_counter(); y=run_vision(feed); cold=time.perf_counter()-t
    np.save(args.evidence/'npu_features.npy',y)
    feature=stats(np.load(ROOT/'results/reference/gpu_features.npy'),y)
    feature['by_camera']={name:stats(np.load(ROOT/'results/reference/gpu_features.npy')[i],y[i])
        for i,name in enumerate(('global','wrist'))}
    write_json(args.evidence/'feature_comparison.json',feature)
    profile=Path(session.end_profiling())
    audit=audit_session(session,ep_report,profile,ck,ck/'compiler_evidence',
        requested_options={'target':'VAIML',**options})
    write_json(args.evidence/'execution_audit.json',audit)
    report.update(cold_run_seconds=cold,input_shape=list(x.shape),output_shape=list(y.shape),
                  input_dtype=str(x.dtype),output_dtype=str(y.dtype),npu_only_proven=audit['npu_only_proven'],
                  device_input_name=input_name,device_input_shape=session.get_inputs()[0].shape,
                  calls_per_camera_pair=2//device_batch,host_patchify=(input_name=='patches'))
    write_json(args.evidence/'session.json',report)
    print(json.dumps(dict(session_create_seconds=report['session_create_seconds'],cold_run_seconds=cold,
        npu_only_proven=audit['npu_only_proven'],feature_comparison=feature)),flush=True)
    if not audit['npu_only_proven']:
        # Mixed execution is permitted only with explicit opt-in and actual HW evidence.
        hw=audit['compiler']['hardware_runner_proven']
        npu=audit['ort']['npu_node_count']>0
        if not (args.allow_cpu and hw and npu):
            raise RuntimeError('Actual NPU execution not proven; see execution_audit.json')
    if args.verify_only: return
    if args.socket.exists(): raise FileExistsError(args.socket)
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    server.bind(str(args.socket)); server.listen(1)
    os.chmod(args.socket,0o600)
    try:
        conn,_=server.accept()
        with conn:
            send_json(conn,report)
            while True:
                message=recv_json(conn)
                if message['command']=='close': break
                if message['command']!='infer': raise ValueError(message)
                t=time.perf_counter(); recv_into(conn,x); received=time.perf_counter()-t
                t=time.perf_counter(); feed=prepare(x); patch_seconds=time.perf_counter()-t
                t=time.perf_counter(); y=run_vision(feed); inference=time.perf_counter()-t
                send_json(conn,{'infer_seconds':inference,'ort_seconds':last_ort_seconds,
                    'batch_concat_seconds':last_concat_seconds,'receive_seconds':received,
                    'patchify_seconds':patch_seconds,'shape':list(y.shape),'dtype':str(y.dtype)})
                conn.sendall(memoryview(y).cast('B'))
    finally:
        server.close(); args.socket.unlink(missing_ok=True)

if __name__=='__main__': main()
