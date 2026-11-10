"""VPR1 local coherent DMA transport. Requires the vpr_dma kernel module."""
import fcntl
import mmap
import os
import struct
from time import perf_counter_ns
import numpy as np

BUFFER_BYTES=1920*1080*4
REQUEST=struct.Struct('<12IQ')
RUN_IOCTL=(3<<30)|(REQUEST.size<<16)|(ord('V')<<8)|1

class BoardTransport:
    def __init__(self,device='/dev/vprdma'):
        self.fd=os.open(device,os.O_RDWR|os.O_CLOEXEC)
        try:
            self.tx=mmap.mmap(self.fd,BUFFER_BYTES,offset=0)
            self.rx=mmap.mmap(self.fd,BUFFER_BYTES,offset=BUFFER_BYTES)
        except BaseException:
            if hasattr(self,'tx'):self.tx.close()
            os.close(self.fd);raise
    def close(self):
        self.rx.close();self.tx.close();os.close(self.fd)
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def run(self,rgbx,width,height,bypass=False):
        ih,iw,channels=rgbx.shape
        if rgbx.dtype!=np.uint8 or channels!=4:raise ValueError('expected uint8 RGBX')
        if not (1<=width<=iw<=1920 and 1<=height<=ih<=1080):raise ValueError('invalid VPR1 dimensions')
        if bypass and (width!=iw or height!=ih):raise ValueError('bypass size mismatch')
        begin=perf_counter_ns()
        target=np.ndarray((ih,iw,4),dtype=np.uint8,buffer=self.tx)
        np.copyto(target,rgbx);del target
        copied=perf_counter_ns()
        request=bytearray(REQUEST.pack(iw,ih,width,height,int(bypass),*[0]*8))
        fcntl.ioctl(self.fd,RUN_IOCTL,request,True)
        returned=perf_counter_ns()
        fields=REQUEST.unpack(request)
        result=np.ndarray((height,width,4),dtype=np.uint8,buffer=self.rx).copy()
        end=perf_counter_ns()
        return result,{'input_to_coherent_buffer_copy_ns':copied-begin,
            'ioctl_wall_ns':returned-copied,'dma_reset_submit_and_wait_ns':fields[-1],
            'coherent_buffer_to_owned_output_copy_ns':end-returned,'transport_total_ns':end-begin,
            'core_status':fields[5],'core_errors':fields[6],'input_count':fields[7],
            'output_count':fields[8],'tx_status':fields[9],'rx_status':fields[10],'rx_bytes':fields[11],
            'cache_policy':'Linux coherent DMA mappings (HP0 is non-coherent; no separate userspace cache flush)'}
