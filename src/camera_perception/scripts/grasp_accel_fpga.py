#!/usr/bin/env python3
"""Exact 2x RGB preprocessing for the grasp CNN; never controls the robot.

The FPGA backend is strict: missing overlay/permission/ABI errors are fatal,
not silently replaced with CPU work. Its MMIO timings include data movement.
The CPU reference and C simulation are explicitly NOT hardware evidence.
"""
from __future__ import annotations
import argparse
import base64
import ctypes
import fcntl
import hashlib
import json
import mmap
import os
from pathlib import Path
import time
from urllib.request import Request, urlopen

CTRL = 0xA0020000
BRAM = 0xA0030000
ABI_TAG = 0x4152454132580001
OVERLAY = 'grasp-preprocess'
SOURCE_SHAPE = (480, 640, 3)


def probe(sysroot='/sys', devroot='/dev'):
    """Read filesystem metadata only; do not read unknown hardware registers."""
    sysroot, devroot = Path(sysroot), Path(devroot)
    state = sysroot / 'class/fpga_manager/fpga0/state'
    overlays = sysroot / 'kernel/config/device-tree/overlays'
    devices = []
    for p in sorted((sysroot / 'class/uio').glob('uio*')):
        try:
            devices.append(dict(name=p.name, logical_name=(p/'name').read_text().strip(),
                addr=int((p/'maps/map0/addr').read_text(), 0),
                size=int((p/'maps/map0/size').read_text(), 0),
                accessible=os.access(devroot/p.name, os.R_OK | os.W_OK)))
        except OSError:
            continue
    names = sorted(p.name for p in overlays.iterdir()) if overlays.exists() else []
    applied = overlays / OVERLAY / 'status'
    ready = (state.exists() and state.read_text().strip() == 'operating'
        and OVERLAY in names and applied.exists() and applied.read_text().strip() == 'applied'
        and all(sum(d['addr'] == a and d['size'] >= 65536 and d['accessible']
                    for d in devices) == 1 for a in (CTRL, BRAM)))
    return dict(schema='grasp-fpga-probe-v1', fpga_state=state.read_text().strip() if state.exists() else None,
                overlays=names, uio=devices, ready=ready,
                hardware_executed=False, expected_overlay=OVERLAY,
                note='An operating FPGA or present bitstream is not execution evidence.')


def require_image(image):
    import numpy as np
    if image.shape != SOURCE_SHAPE or image.dtype != np.uint8:
        raise ValueError('Expected 640x480 BGR uint8 image; no implicit resize allowed')


def cpu_area2x(image):
    """Integer reference matching OpenCV INTER_AREA for fixed 2x uint8 input."""
    import numpy as np
    require_image(image)
    a = image.astype(np.uint16)
    out = ((a[0::2,0::2]+a[0::2,1::2]+a[1::2,0::2]+a[1::2,1::2]+2)//4).astype(np.uint8)
    return np.ascontiguousarray(out[:,:,::-1])


def rgb_to_tensor(rgb):
    import numpy as np
    if rgb.shape != (240,320,3) or rgb.dtype != np.uint8:
        raise ValueError('Expected RGB uint8 [240,320,3]')
    return np.ascontiguousarray(rgb.transpose(2,0,1)[None], dtype=np.float32) / np.float32(255)


class FpgaArea2x:
    def __init__(self):
        self.maps, self.fds = [], []
        self.lock = open('/tmp/grasp-preprocess-fpga.lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.identity = probe()
        if not self.identity['ready']:
            self.close()
            raise RuntimeError('FPGA preprocessing unavailable: ' + json.dumps(self.identity))
        try:
            for addr in (CTRL, BRAM):
                candidates = [d for d in self.identity['uio'] if d['addr'] == addr]
                fd = os.open('/dev/'+candidates[0]['name'], os.O_RDWR | os.O_SYNC)
                self.fds.append(fd)
                self.maps.append(mmap.mmap(fd, 65536, flags=mmap.MAP_SHARED,
                    prot=mmap.PROT_READ | mmap.PROT_WRITE))
            self.reg, self.ram = self.maps
            if not self.read(0) & 4:
                raise RuntimeError('FPGA core is not idle')
        except Exception:
            self.close()
            raise

    def read(self, off):
        return ctypes.c_uint32.from_buffer(self.reg, off).value

    def write(self, off, value):
        ctypes.c_uint32.from_buffer(self.reg, off).value = value

    def run(self, image):
        import numpy as np
        require_image(image)
        start = time.perf_counter()
        pack_start = time.perf_counter()
        packed = np.zeros((480,640,4), np.uint8)
        packed[:,:,:3] = image
        raw = packed.tobytes()
        times = dict(pack=(time.perf_counter()-pack_start)*1000,
                     copy_in=0., launch_and_wait=0., copy_out=0., unpack=0., dma=0.)
        result = bytearray()
        for y in range(0,480,8):
            chunk = raw[y*640*4:(y+8)*640*4]
            t = time.perf_counter()
            self.ram[:len(chunk)] = chunk
            for k in range(4):
                ctypes.c_uint64.from_buffer(self.ram, 0xC000+8*k).value = 0xffffffffffffffff
            times['copy_in'] += (time.perf_counter()-t)*1000
            t = time.perf_counter()
            for reg, addr in ((0x10,BRAM),(0x1c,BRAM+0x8000),(0x28,BRAM+0xC000)):
                self.write(reg, addr); self.write(reg+4,0)
            self.write(0x34,8)
            self.write(0,1)
            deadline=time.monotonic()+.5
            while not self.read(0)&2:
                if time.monotonic()>deadline:
                    raise TimeoutError('FPGA preprocessing timeout; no retry was sent')
                time.sleep(.00002)
            times['launch_and_wait'] += (time.perf_counter()-t)*1000
            t = time.perf_counter()
            stats = tuple(ctypes.c_uint64.from_buffer(self.ram,0xC000+8*k).value for k in range(4))
            if stats != (8,1280,ABI_TAG,0):
                raise RuntimeError('FPGA result ABI/count/status mismatch: '+repr(stats))
            result.extend(self.ram[0x8000:0x8000+1280*4])
            times['copy_out'] += (time.perf_counter()-t)*1000
        t=time.perf_counter()
        rgb=np.ascontiguousarray(np.frombuffer(result,np.uint8).reshape(240,320,4)[:,:,:3])
        times['unpack']=(time.perf_counter()-t)*1000
        times['processing_total']=(time.perf_counter()-start)*1000
        evidence = dict(implementation='FPGA_PL_HLS_AREA2X', hardware_executed=True,
            abi='AREA2X-v1', evidence=dict(overlay=OVERLAY, control_base=hex(CTRL),
            bram_base=hex(BRAM), abi_tag=hex(ABI_TAG), blocks=60,
            driver_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            transport='PS MMIO copy to PL BRAM; no DMA engine',
            bytes_to_pl=1228800, bytes_from_pl=307200,
            timing_scope='copy+register setup+polled completion+copy; not pure core latency'))
        return rgb, times, evidence

    def close(self):
        for m in getattr(self,'maps',[]): m.close()
        for fd in getattr(self,'fds',[]): os.close(fd)
        self.maps, self.fds = [], []
        if not self.lock.closed: self.lock.close()


def process(image, backend='cpu', engine=None):
    if backend == 'fpga':
        if engine is None:
            raise ValueError('FPGA engine must be initialized once; no CPU fallback')
        return engine.run(image)
    if backend not in ('cpu','opencv'):
        raise ValueError('Unknown preprocessing backend')
    t=time.perf_counter()
    if backend=='opencv':
        import cv2
        require_image(image)
        rgb=cv2.cvtColor(cv2.resize(image,(320,240),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB)
    else:
        rgb=cpu_area2x(image)
    return rgb, dict(processing_total=(time.perf_counter()-t)*1000), dict(
        implementation='CPU_OPENCV_AREA2X' if backend=='opencv' else 'CPU_REFERENCE_AREA2X',
        hardware_executed=False, abi='AREA2X-v1', evidence={})


def metadata(rgb, timings, evidence, source):
    result=dict(source)
    result.update(schema='grasp-preprocess-v1', preprocessor=evidence,
        transform=dict(from_size=[640,480], to_size=[320,240],
            interpolation='INTER_AREA_2X_EXACT', layout='HWC', channels='RGB', dtype='uint8'),
        output_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(), timing_ms=timings,
        robot_commands_sent=False)
    return result


def process_remote(encoded_image, url='http://127.0.0.1:18862/preprocess',
                   source=None, require_hardware=True, timeout=3):
    """Send exactly this recorded/live image; count full HTTP transport time."""
    import numpy as np
    source=dict(source or {'source_kind':'replay'})
    source['original_image_sha256']=hashlib.sha256(encoded_image).hexdigest()
    req=Request(url,data=encoded_image,headers={
        'Content-Type':'application/octet-stream',
        'X-Frame-Metadata':json.dumps(source,separators=(',',':'))},method='POST')
    t=time.perf_counter()
    with urlopen(req,timeout=timeout) as r:
        meta=json.loads(r.headers['X-Frame-Metadata']);raw=r.read()
    transport=(time.perf_counter()-t)*1000
    if len(raw)!=240*320*3 or meta['original_image_sha256']!=source['original_image_sha256']:
        raise ValueError('Remote preprocessing dimensions or input identity mismatch')
    if hashlib.sha256(raw).hexdigest()!=meta['output_sha256']:
        raise ValueError('Remote preprocessing output hash mismatch')
    e=meta.get('preprocessor',{})
    if require_hardware and not (e.get('hardware_executed') is True and
            e.get('implementation')=='FPGA_PL_HLS_AREA2X' and
            e.get('evidence',{}).get('abi_tag')==hex(ABI_TAG)):
        raise RuntimeError('Requested FPGA execution but response is not hardware-attested')
    meta['timing_ms']['client_roundtrip']=transport
    meta['timing_ms']['client_network_queue_overhead_estimate']=max(0.,transport-meta['timing_ms']['server_total'])
    meta['timing_ms']['client_timing_scope']='HTTP request+response includes transfer, queueing, decode, preprocessing; not pure FPGA'
    return np.frombuffer(raw,np.uint8).reshape(240,320,3).copy(),meta


def serve(args):
    """Separate service; reads existing snapshot API, never opens a camera SDK."""
    import numpy as np
    import cv2
    cv2.setNumThreads(1)
    from http.server import BaseHTTPRequestHandler, HTTPServer
    engine = FpgaArea2x() if args.backend=='fpga' else None
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_): pass
        def setup(self):
            super().setup()
            self.connection.settimeout(3)
        def reply_result(self, raw, source, timing_prefix=None):
            t0=time.perf_counter()
            t=time.perf_counter()
            image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            if image is None: raise ValueError('Image decode failed')
            decode=(time.perf_counter()-t)*1000
            rgb,times,evidence=process(image,args.backend,engine)
            times.update(timing_prefix or {})
            times.update(decode=decode,server_total=(time.perf_counter()-t0)*1000+
                         times.get('request_body_read',0.))
            source['original_image_sha256']=hashlib.sha256(raw).hexdigest()
            meta=metadata(rgb,times,evidence,source)
            body=rgb.tobytes()
            self.send_response(200);self.send_header('Content-Type','application/octet-stream')
            self.send_header('Content-Length',str(len(body)))
            self.send_header('X-Frame-Metadata',json.dumps(meta,separators=(',',':')))
            self.end_headers();self.wfile.write(body)

        def do_POST(self):
            if self.path!='/preprocess': self.send_error(404);return
            try:
                t0=time.perf_counter()
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=2_000_000: raise ValueError('Encoded image size must be 1..2000000')
                source=json.loads(self.headers.get('X-Frame-Metadata','{}'))
                if source.get('source_kind') not in ('live_wrist','replay'):
                    raise ValueError('source_kind must be live_wrist or replay')
                raw=self.rfile.read(size)
                if len(raw)!=size: raise ValueError('Truncated body')
                if hashlib.sha256(raw).hexdigest()!=source.get('original_image_sha256'):
                    raise ValueError('Encoded input hash mismatch')
                self.reply_result(raw,source,dict(request_body_read=(time.perf_counter()-t0)*1000))
            except Exception as e:
                self.send_error(503,str(e))

        def do_GET(self):
            if self.path=='/health':
                body=json.dumps(dict(backend=args.backend, probe=probe())).encode()
                self.send_response(200); self.send_header('Content-Length',str(len(body)))
                self.end_headers();self.wfile.write(body);return
            if self.path!='/snapshot':
                self.send_error(404);return
            try:
                t0=time.perf_counter()
                with urlopen(args.camera_url,timeout=2) as r: snap=json.load(r)
                capture=(time.perf_counter()-t0)*1000
                if snap['board_frame_age_ms']>250: raise ValueError('Stale frame')
                raw=base64.b64decode(snap['image_base64'],validate=True)
                if hashlib.sha256(raw).hexdigest()!=snap['image_sha256']:
                    raise ValueError('Input image hash mismatch')
                t=time.perf_counter()
                image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
                if image is None: raise ValueError('Image decode failed')
                decode=(time.perf_counter()-t)*1000
                rgb,times,evidence=process(image,args.backend,engine)
                times.update(capture_snapshot=capture,decode=decode,
                             server_total=(time.perf_counter()-t0)*1000)
                if snap['board_frame_age_ms']+times['server_total']>250:
                    raise ValueError('Result exceeds 250ms source frame age budget')
                meta=metadata(rgb,times,evidence,dict(original_image_sha256=snap['image_sha256'],
                    source_kind='live_wrist',frame=snap['frame'],device=snap['device'],
                    board_frame_age_ms_at_fetch=snap['board_frame_age_ms']))
                body=rgb.tobytes()
                self.send_response(200);self.send_header('Content-Type','application/octet-stream')
                self.send_header('Content-Length',str(len(body)))
                self.send_header('X-Frame-Metadata',json.dumps(meta,separators=(',',':')))
                self.end_headers();self.wfile.write(body)
            except Exception as e:
                self.send_error(503,str(e))
    server=HTTPServer(('127.0.0.1',args.port),Handler)
    try: server.serve_forever()
    finally:
        server.server_close()
        if engine: engine.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('probe')
    s=sub.add_parser('serve');s.add_argument('--backend',choices=['cpu','opencv','fpga'],default='fpga')
    s.add_argument('--port',type=int,default=18862)
    s.add_argument('--camera-url',default='http://127.0.0.1:18760/handeye-snapshot')
    a=p.parse_args()
    if a.command=='probe': print(json.dumps(probe(),indent=2))
    else: serve(a)


if __name__=='__main__':main()
