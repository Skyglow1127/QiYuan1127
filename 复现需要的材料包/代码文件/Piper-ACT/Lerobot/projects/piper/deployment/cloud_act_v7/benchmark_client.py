#!/usr/bin/env python3
"""No-motion ACT benchmark over a private persistent SSH stdin/stdout pipe.

Inputs are prerecorded observations. Output predictions are saved, never sent
to a robot. Local encoding time and SSH/GPU roundtrip are reported separately.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import select
import shlex
import struct
import subprocess
import sys
import time

import numpy as np

MAX_FRAME = 4 * 1024 * 1024


def transfer(pipe, payload=None, *, size=None, timeout=30.):
    deadline = time.monotonic() + timeout
    result = bytearray()
    offset = 0
    length = len(payload) if payload is not None else size
    while offset < length:
        left = deadline - time.monotonic()
        if left <= 0:
            raise TimeoutError('Inference transport deadline exceeded')
        readers, writers, _ = select.select([pipe] if payload is None else [],
            [pipe] if payload is not None else [], [], left)
        if payload is not None and writers:
            offset += os.write(pipe.fileno(), payload[offset:offset+65536])
        elif readers:
            chunk = os.read(pipe.fileno(), min(65536, length-offset))
            if not chunk:
                raise EOFError('Inference process exited before completing a frame')
            result.extend(chunk)
            offset += len(chunk)
    return bytes(result)


def response(pipe, *, timeout=30.):
    size = struct.unpack('>I', transfer(pipe, size=4, timeout=timeout))[0]
    if not 0 < size <= MAX_FRAME:
        raise ValueError(f'Invalid response size: {size}')
    value = json.loads(transfer(pipe, size=size, timeout=timeout))
    if not value.get('ok'):
        raise RuntimeError('Inference failed: ' + str(value.get('error')))
    return value


def summary(values):
    return {key:float(value) for key,value in zip(
        ('p50','p95','p99','max'), np.percentile(values, [50,95,99,100]), strict=True)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='root@connect.bjb2.seetacloud.com')
    parser.add_argument('--port', type=int, default=18561)
    parser.add_argument('--control-path', default='/tmp/piper-act-cloud-v7/ssh.sock')
    parser.add_argument('--known-hosts', default='/tmp/piper-act-cloud-v7/known_hosts')
    parser.add_argument('--remote-root', default='/root/autodl-tmp/piper_act_inference_v7')
    parser.add_argument('--local', action='store_true', help='Use the same protocol on the local CPU instead of SSH')
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--request-warmups', type=int, default=0)
    parser.add_argument('--jpeg-quality', type=int,
        help='Encode the two recorded RGB images as JPEG at this quality before each request')
    parser.add_argument('--jpeg-decoder', choices=('pillow','opencv'), default='pillow')
    parser.add_argument('--client-opencv-threads', type=int,
        help='Optional client image-thread limit; default retains OpenCV setting')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('inputs', nargs='+', type=Path)
    args = parser.parse_args()
    if args.samples < 1:
        parser.error('--samples must be positive')
    if args.jpeg_quality is not None and not 1 <= args.jpeg_quality <= 100:
        parser.error('--jpeg-quality must be in [1,100]')
    if args.request_warmups < 0:
        parser.error('--request-warmups must be nonnegative')
    if args.client_opencv_threads is not None and not 1 <= args.client_opencv_threads <= 32:
        parser.error('--client-opencv-threads must be in [1,32]')
    if args.jpeg_quality is not None:
        import cv2
        if args.client_opencv_threads is not None:
            cv2.setNumThreads(args.client_opencv_threads)
    arrays = []
    for path in args.inputs:
        with np.load(path, allow_pickle=False) as data:
            arrays.append({key:np.ascontiguousarray(data[key]) for key in data.files})
    if args.local:
        if args.checkpoint is None:
            parser.error('--local requires --checkpoint')
        command = [sys.executable, '-B', '-u', str(Path(__file__).with_name('benchmark_server.py')),
            '--device','cpu','--checkpoint',str(args.checkpoint),'--warmups','3',
            '--jpeg-decoder',args.jpeg_decoder]
    else:
        root = args.remote_root
        remote = [root+'/venv/bin/python','-B','-u',root+'/benchmark_server.py',
            '--device','cuda','--checkpoint',root+'/model','--warmups','3',
            '--jpeg-decoder',args.jpeg_decoder]
        command = ['ssh','-T','-S',args.control_path,'-o','BatchMode=yes',
            '-o','UserKnownHostsFile='+args.known_hosts,'-o','StrictHostKeyChecking=yes',
            '-p',str(args.port),args.host,shlex.join(remote)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = {'inputs':'recorded training episode, not live control',
        'mode':'local_cpu' if args.local else 'cloud_gpu_ssh',
        'encoding':'lossless_npz' if args.jpeg_quality is None else f'jpeg_quality{args.jpeg_quality}_npz',
        'request_warmups':args.request_warmups,
        'client_opencv_threads':cv2.getNumThreads() if args.jpeg_quality is not None else None,
        'client_loadavg_at_start':list(os.getloadavg()),
        'hardware_motion_commands_sent':0,'command':command,'samples':[]}
    with args.output.with_suffix('.stderr.log').open('wb') as error_log:
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=error_log, bufsize=0)
        try:
            result['server'] = response(process.stdout, timeout=300.)
            for iteration in range(args.samples+args.request_warmups):
                index = iteration-args.request_warmups
                observation = arrays[index % len(arrays)]
                started = time.perf_counter()
                encoded = io.BytesIO()
                if args.jpeg_quality is None:
                    np.savez_compressed(encoded, **observation)
                else:
                    jpeg_arrays = {'observation.state':observation['observation.state']}
                    for key in ('observation.images.global','observation.images.wrist'):
                        ok, image = cv2.imencode('.jpg',
                            cv2.cvtColor(observation[key], cv2.COLOR_RGB2BGR),
                            [cv2.IMWRITE_JPEG_QUALITY,args.jpeg_quality])
                        if not ok:
                            raise ValueError('JPEG encoding failed')
                        jpeg_arrays[key+'.jpeg'] = image.reshape(-1)
                    np.savez(encoded, **jpeg_arrays)
                payload = encoded.getvalue()
                encoded_at = time.perf_counter()
                transfer(process.stdin, struct.pack('>I',len(payload))+payload)
                reply = response(process.stdout)
                finished = time.perf_counter()
                prediction = np.array(reply['predictions'])
                if prediction.shape != (100,7) or not np.isfinite(prediction).all():
                    raise ValueError('Nonfinite or wrongly shaped ACT prediction')
                if index < 0:
                    continue
                record = {'index':index,'input':str(args.inputs[index%len(arrays)].resolve()),
                    'request_sha256':hashlib.sha256(payload).hexdigest(),'request_bytes':len(payload),
                    'encode_ms':(encoded_at-started)*1000,
                    'roundtrip_ms':(finished-encoded_at)*1000,
                    'encode_and_roundtrip_ms':(finished-started)*1000,'response':reply}
                result['samples'].append(record)
                print(json.dumps({key:record[key] for key in ('index','request_bytes','roundtrip_ms','encode_and_roundtrip_ms')}), flush=True)
            result['summary_ms'] = {key:summary([r[key] for r in result['samples']])
                for key in ('encode_ms','roundtrip_ms','encode_and_roundtrip_ms')}
            result['server_pipeline_ms'] = summary([r['response']['timings_ms']['pipeline'] for r in result['samples']])
            result['server_forward_ms'] = summary([r['response']['timings_ms']['forward'] for r in result['samples']])
            args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps({'summary_ms':result['summary_ms'],'server_forward_ms':result['server_forward_ms']},ensure_ascii=False), flush=True)
        except BaseException as error:
            result['error'] = str(error)
            args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            raise
        finally:
            process.stdin.close()
            try:
                process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
    if process.returncode:
        raise RuntimeError(f'Inference process exited with status{process.returncode}')


if __name__ == '__main__':
    main()
