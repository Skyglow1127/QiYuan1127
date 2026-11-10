#!/usr/bin/env python3
"""Run depth preprocessing in PL through two explicitly matched UIO mappings."""
import argparse,array,ctypes,gzip,hashlib,json,mmap,os,struct,sys,time
from pathlib import Path
CTRL=0xA0000000;BRAM=0xA0010000
class DepthGate:
 def __init__(self):
  if Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()!='operating':raise RuntimeError('FPGA is not operating')
  self.maps=[];self.fds=[]
  for addr in (CTRL,BRAM):
   candidates=[p for p in Path('/sys/class/uio').glob('uio*') if int((p/'maps/map0/addr').read_text(),0)==addr and int((p/'maps/map0/size').read_text(),0)>=0x10000]
   if len(candidates)!=1:raise RuntimeError('Expected unique UIO device for '+hex(addr))
   fd=os.open('/dev/'+candidates[0].name,os.O_RDWR|os.O_SYNC);self.fds.append(fd)
   self.maps.append(mmap.mmap(fd,0x10000,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|mmap.PROT_WRITE))
  self.reg,self.ram=self.maps
  if not self.read(0)&4:raise RuntimeError('HLS core not idle; refusing to overwrite its input')
 def read(self,off):return ctypes.c_uint32.from_buffer(self.reg,off).value
 def write(self,off,v):ctypes.c_uint32.from_buffer(self.reg,off).value=v
 def run(self,raw,lower,upper):
  if len(raw)%16 or not 0<=lower<=upper<=65534:raise ValueError('Depth byte count must be a multiple of 16 for aligned MMIO copies; check thresholds')
  output=bytearray();counts=0;total=0;minimum=65535;maximum=0;blocks=0
  start=time.monotonic()
  for pos in range(0,len(raw),8192):
   chunk=raw[pos:pos+8192];self.ram[:len(chunk)]=chunk
   for off in range(0x8000,0x8028,4):ctypes.c_uint32.from_buffer(self.ram,off).value=0xffffffff
   for reg,address in ((0x10,BRAM),(0x1c,BRAM+0x4000),(0x28,BRAM+0x8000)):
    self.write(reg,address);self.write(reg+4,0)
   self.write(0x34,len(chunk)//2);self.write(0x3c,lower);self.write(0x44,upper)
   self.write(0,1);deadline=time.monotonic()+1
   while not self.read(0)&2:
    if time.monotonic()>deadline:raise TimeoutError('HLS ap_done timed out')
    time.sleep(0.00005)
   n,s,lo,hi,status=tuple(ctypes.c_uint64.from_buffer(self.ram,0x8000+i*8).value for i in range(5))
   if status:raise RuntimeError('HLS invalid input status')
   output.extend(self.ram[0x4000:0x4000+len(chunk)])
   counts+=n;total+=s
   if n:minimum=min(minimum,lo);maximum=max(maximum,hi)
   blocks+=1
  return bytes(output),{'implementation':'FPGA_PL_HLS','pixels':len(raw)//2,'blocks':blocks,'valid_count':counts,'sum_raw':total,'min_raw':minimum if counts else 0,'max_raw':maximum,'mean_raw':total/counts if counts else None,'lower_raw':lower,'upper_raw':upper,'elapsed_ms':(time.monotonic()-start)*1000}
 def close(self):
  for m in self.maps:m.close()
  for fd in self.fds:os.close(fd)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--input',required=True);ap.add_argument('--output',default='fpga-result');ap.add_argument('--lower',type=int,default=300);ap.add_argument('--upper',type=int,default=1500);a=ap.parse_args()
 p=Path(a.input);raw=gzip.decompress(p.read_bytes()) if p.suffix=='.gz' else p.read_bytes()
 gate=DepthGate()
 try:
  # Verify mapped BRAM before starting the HLS master.
  pattern=bytes(range(64));gate.ram[:64]=pattern
  if gate.ram[:64]!=pattern:raise RuntimeError('BRAM readback failed')
  filtered,result=gate.run(raw,a.lower,a.upper)
 finally:gate.close()
 values=array.array('H');values.frombytes(raw)
 if sys.byteorder!='little':values.byteswap()
 expected=array.array('H',(v if 0<v<65535 and a.lower<=v<=a.upper else 0 for v in values))
 valid=[v for v in expected if v]
 expected_stats=(len(valid),sum(valid),min(valid,default=0),max(valid,default=0))
 actual_stats=tuple(result[k] for k in ('valid_count','sum_raw','min_raw','max_raw'))
 if sys.byteorder!='little':expected.byteswap()
 if filtered!=expected.tobytes() or actual_stats!=expected_stats:raise RuntimeError('FPGA output differs from independent CPU reference')
 result.update({'cpu_reference_match':True,'input_sha256':hashlib.sha256(raw).hexdigest(),'output_sha256':hashlib.sha256(filtered).hexdigest()})
 out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
 with gzip.open(out/'filtered_u16le.raw.gz','wb') as f:f.write(filtered)
 (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':main()
