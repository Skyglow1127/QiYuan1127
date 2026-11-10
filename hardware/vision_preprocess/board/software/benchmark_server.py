#!/usr/bin/env python3
"""Loopback-only native board-output benchmark; accepts no uploaded frame or address."""
from http.server import BaseHTTPRequestHandler,HTTPServer
from pathlib import Path
from time import perf_counter_ns
from urllib.parse import parse_qs,urlsplit
import argparse,json,sys
import numpy as np
from PIL import Image
from board_transport import BoardTransport
from cpu_resize import NativeCPU
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'software'))
from preprocess import pack_rgbx,resize_floor

def main():
    p=argparse.ArgumentParser();p.add_argument('--samples',type=Path,required=True);a=p.parse_args()
    samples={path.stem:np.array(Image.open(path).convert('RGB')) for path in a.samples.glob('*.jpg')}
    if not samples:raise RuntimeError('No board-local sample frames')
    native=NativeCPU()
    with BoardTransport() as dma:
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                try:
                    parsed=urlsplit(self.path)
                    if parsed.path=='/status':
                        raw=json.dumps({'service':'vpr-output-benchmark','samples':list(samples),'input':'board-local RGB','port':18765}).encode();self.send_response(200);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw);return
                    if parsed.path!='/process':raise ValueError('Unknown path')
                    q=parse_qs(parsed.query);image=samples[q['sample'][0]]
                    width=int(q['width'][0]);height=int(q['height'][0]);backend=q['backend'][0]
                    bypass=width==image.shape[1] and height==image.shape[0]
                    begin=perf_counter_ns();stages={}
                    if backend=='pl':
                        packed=pack_rgbx(image);packed_at=perf_counter_ns()
                        out4,stages=dma.run(packed,width,height,bypass)
                        output=out4[:,:,:3].copy();stages['rgbx_pack_ns']=packed_at-begin
                    elif backend=='software':output=native.resize(image,width,height)
                    else:raise ValueError('Invalid backend')
                    raw=output.tobytes();duration=perf_counter_ns()-begin
                    self.send_response(200);self.send_header('Content-Type','application/octet-stream')
                    self.send_header('Content-Length',str(len(raw)));self.send_header('X-Server-Work-Ns',str(duration))
                    self.send_header('X-VPR-Stages',json.dumps(stages,separators=(',',':')))
                    self.send_header('X-Width',str(width));self.send_header('X-Height',str(height))
                    self.end_headers();self.wfile.write(raw)
                except (KeyError,ValueError) as e:self.send_error(400,str(e))
                except Exception as e:self.send_error(500,str(e))
            def log_message(self,*args):pass
        server=HTTPServer(('127.0.0.1',18765),Handler)
        print('VPR_BENCHMARK_SERVER_READY 127.0.0.1:18765',flush=True);server.serve_forever()
if __name__=='__main__':main()
