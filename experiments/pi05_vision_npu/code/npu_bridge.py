"""Unix socket IPC and a scoped PI05 vision replacement; no LeRobot edits."""
from pathlib import Path
import socket
import subprocess
import time
import types
import numpy as np
from common import ROOT
from protocol import send_json, recv_json, recv_into

class NpuClient:
    def __init__(self, output, model=None, config=None):
        self.output=Path(output)
        self.path=self.output/'vision.sock'
        self.log=(self.output/'npu_worker.log').open('w')
        command=['bash',str(ROOT/'code/run_npu_env.sh'),'python',str(ROOT/'code/npu_worker.py'),
                 '--socket',str(self.path),'--evidence',str(self.output/'npu_session')]
        if model: command.extend(['--model',str(model)])
        if config: command.extend(['--config',str(config)])
        self.command=command
        t=time.perf_counter()
        self.process=subprocess.Popen(command,stdout=self.log,stderr=subprocess.STDOUT)
        try:
            while not self.path.exists():
                if self.process.poll() is not None:
                    raise RuntimeError(f'NPU worker exited {self.process.returncode}; see {self.output}/npu_worker.log')
                if time.perf_counter()-t>900:
                    raise TimeoutError('NPU worker did not become ready; see npu_worker.log')
                time.sleep(.1)
            self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
            self.socket.settimeout(120)
            self.socket.connect(str(self.path))
            self.session=recv_json(self.socket)
            self.start_seconds=time.perf_counter()-t
        except BaseException:
            self.process.terminate(); self.process.wait(timeout=10); self.log.close()
            raise
        self.output_array=np.empty(self.session['output_shape'],dtype=np.float32)
    def infer(self,pixels):
        assert pixels.shape==(2,3,224,224) and pixels.dtype==np.float32 and pixels.flags.c_contiguous
        t=time.perf_counter()
        send_json(self.socket,{'command':'infer'})
        self.socket.sendall(memoryview(pixels).cast('B'))
        meta=recv_json(self.socket)
        assert meta['shape']==list(self.output_array.shape) and meta['dtype']=='float32'
        recv_into(self.socket,self.output_array)
        elapsed=time.perf_counter()-t
        return self.output_array,dict(npu_ort_seconds=meta['ort_seconds'],ipc_roundtrip_seconds=elapsed,
            worker_batch_concat_seconds=meta['batch_concat_seconds'],
            ipc_copy_and_control_seconds=max(0.,elapsed-meta['infer_seconds']-meta['patchify_seconds']),
            worker_patchify_seconds=meta['patchify_seconds'],
            worker_receive_seconds=meta['receive_seconds'])
    def close(self):
        try:
            send_json(self.socket,{'command':'close'}); self.socket.close()
            self.process.wait(timeout=15)
        finally:
            if self.process.poll() is None: self.process.terminate(); self.process.wait(timeout=10)
            self.log.close()

class NpuPrefix:
    def __init__(self,model,client):
        self.model=model; self.client=client
        self.original=model.embed_prefix
        self.records=[]; self.last_features=None
    def install(self):
        outer=self
        def forward(module,images,img_masks,tokens,masks):
            return outer.forward(images,img_masks,tokens,masks)
        self.model.embed_prefix=types.MethodType(forward,self.model)
    def restore(self): self.model.embed_prefix=self.original
    def forward(self,images,img_masks,tokens,masks):
        import torch
        assert len(images)==2 and all(tuple(x.shape)==(1,3,224,224) for x in images)
        torch.cuda.synchronize(); total=time.perf_counter()
        t=time.perf_counter()
        pixels=torch.cat(images,dim=0).contiguous().cpu().numpy()
        torch.cuda.synchronize()
        pack=time.perf_counter()-t
        features,timing=self.client.infer(pixels)
        t=time.perf_counter()
        on_gpu=torch.from_numpy(features).to(device=images[0].device,dtype=images[0].dtype)
        torch.cuda.synchronize(); upload=time.perf_counter()-t
        timing.update(gpu_pack_sync_d2h_seconds=pack,h2d_cast_sync_seconds=upload,
                      vision_roundtrip_seconds=time.perf_counter()-total,
                      input_bytes=pixels.nbytes,output_bytes=features.nbytes)
        self.records.append(timing)
        self.last_features=features.copy()
        vlm=self.model.paligemma_with_expert
        original_image=vlm.embed_image
        index=0
        def consume(module,image):
            nonlocal index
            assert image is images[index], 'Camera order changed'
            result=on_gpu[index:index+1]; index+=1
            return result
        vlm.embed_image=types.MethodType(consume,vlm)
        try:
            result=self.original(images,img_masks,tokens,masks)
            assert index==2
            return result
        finally:
            vlm.embed_image=original_image
