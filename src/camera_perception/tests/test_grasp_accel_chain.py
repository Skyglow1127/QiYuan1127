"""Controlled fakes only: no HTTP, FPGA, NPU or robot requests are issued."""
import argparse
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np

SOURCE=Path(__file__).resolve().parents[1]/'fpga/grasp_preprocess_v1/benchmark_chain.py'
spec=importlib.util.spec_from_file_location('chain',SOURCE)
chain=importlib.util.module_from_spec(spec);spec.loader.exec_module(chain)


class Clock:
    def __init__(self):self.now=0.
    def __call__(self):return self.now
    def advance(self,value):self.now+=value


class FakeFpga:
    def __init__(self,clock,output,bad_pixels=False):
        self.clock=clock;self.output=output;self.bad_pixels=bad_pixels
    def process(self,image,backend):
        return cv2.cvtColor(cv2.resize(image,(320,240),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB),{},{}
    def process_remote(self,raw,*args,**kwargs):
        assert kwargs['require_hardware'] is True
        assert json.loads((self.output/'result.json').read_text())['status']=='RUNNING'
        image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        rgb=self.process(image,'opencv')[0]
        if self.bad_pixels:rgb[0,0,0]^=255
        self.clock.advance(.003)
        return rgb,dict(original_image_sha256=hashlib.sha256(raw).hexdigest(),
            output_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
            preprocessor=dict(hardware_executed=True,implementation='FPGA_PL_HLS_AREA2X'))


class FakeNpu:
    def __init__(self,clock,wrong_model=False,timeout=False,cold=False):
        self.clock=clock;self.wrong_model=wrong_model;self.timeout=timeout;self.cold=cold;self.npu_calls=0
        image=np.zeros((480,640),np.uint8);image[200:280,300:380]=255
        _,png=cv2.imencode('.png',image);self.png=png.tobytes()
    def infer_remote(self,raw,prep=None,endpoint=None,preprocessed=False,**kwargs):
        is_npu=endpoint=='npu'
        delay=.12 if is_npu and self.cold and self.npu_calls==0 else .002
        self.clock.advance(delay)
        if is_npu:self.npu_calls+=1
        if is_npu and self.timeout:raise TimeoutError('simulated timeout; no hardware accessed')
        digest=hashlib.sha256(raw).hexdigest()
        return dict(backend='npu' if is_npu else 'cpu',npu_execution_confirmed=is_npu,
            model_sha256=('b' if is_npu and self.wrong_model else 'a')*64,
            image_sha256=prep['original_image_sha256'] if preprocessed else digest,
            submitted_payload_sha256=digest,mask_png_sha256=hashlib.sha256(self.png).hexdigest(),
            mask_png_base64=base64.b64encode(self.png).decode(),timing_ms={'inference_ms':delay*1000},
            preprocessor_evidence=prep['preprocessor'] if preprocessed else None)


class ChainTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        image=np.full((480,640,3),100,np.uint8)
        image[200:280,300:380]=[0,0,255]
        cv2.imwrite(str(self.root/'image.png'),image)
        self.dataset=self.root/'dataset.json'
        self.dataset.write_text(json.dumps({'samples':[{'image':'image.png','image_sha256':hashlib.sha256((self.root/'image.png').read_bytes()).hexdigest()}]}))
        self.args=argparse.Namespace(dataset=self.dataset,output=self.root/'evidence',rounds=3,warmup=1,
            cpu_url='cpu',npu_url='npu',fpga_url='fpga')
        self.clock=Clock()
    def tearDown(self):self.temp.cleanup()
    def run_fake(self,**npu_kwargs):
        return chain.run_benchmark(self.args,FakeFpga(self.clock,self.args.output),
            FakeNpu(self.clock,**npu_kwargs),clock=self.clock)
    def test_warm_all_and_first_call_are_separate_and_cold_retained(self):
        report=self.run_fake(cold=True)
        self.assertEqual(report['status'],'PASSED')
        self.assertEqual(report['summary']['warm_only']['host_npu']['count'],3)
        self.assertEqual(report['summary']['all_samples_including_warmup']['host_npu']['count'],4)
        self.assertAlmostEqual(report['summary']['all_samples_including_warmup']['host_npu']['max_ms'],120.)
        self.assertAlmostEqual(report['summary']['first_call_per_route']['host_npu']['total_ms'],120.)
        self.assertFalse(report['controlled_cold_comparison'])
        self.assertFalse(report['service_initialization_and_session_warmup_included'])
        self.assertEqual(report['same_model_sha256'],'a'*64)
    def test_existing_output_never_overwritten(self):
        self.args.output.mkdir();marker=self.args.output/'result.json';marker.write_text('old evidence')
        with self.assertRaises(FileExistsError):self.run_fake()
        self.assertEqual(marker.read_text(),'old evidence')
    def test_model_mismatch_fails_and_retains_partial_iteration(self):
        with self.assertRaisesRegex(RuntimeError,'same model'):self.run_fake(wrong_model=True)
        report=json.loads((self.args.output/'result.json').read_text())
        samples=json.loads((self.args.output/'samples.json').read_text())
        self.assertEqual(report['status'],'FAILED')
        self.assertEqual(samples[0]['routes']['host_cpu']['status'],'PASSED')
        self.assertEqual(samples[0]['routes']['host_npu']['status'],'FAILED')
        self.assertEqual(report['summary']['failed_route_calls']['host_npu']['count'],1)
        self.assertIsNone(report['fpga_npu_vs_host_npu_warm_speedup'])
    def test_validation_cost_excluded_from_compute_total(self):
        def slow_equal(a,b):
            self.clock.advance(.05)
            return np.array_equal(a,b)
        report=chain.run_benchmark(self.args,FakeFpga(self.clock,self.args.output),FakeNpu(self.clock),
            clock=self.clock,pixel_equal=slow_equal)
        first=report['summary']['first_call_per_route']['fpga_npu']
        self.assertAlmostEqual(first['total_ms'],5.)
        self.assertAlmostEqual(first['validation_ms'],50.)
        self.assertAlmostEqual(first['attempt_elapsed_ms'],55.)
    def test_timeout_retains_running_and_failed_events(self):
        with self.assertRaises(TimeoutError):self.run_fake(timeout=True)
        report=json.loads((self.args.output/'result.json').read_text())
        events=[json.loads(line) for line in (self.args.output/'events.jsonl').read_text().splitlines()]
        self.assertEqual(report['status'],'FAILED')
        self.assertIn('route_started',[row['event'] for row in events])
        self.assertIn('route_failed',[row['event'] for row in events])
        self.assertEqual(events[-1]['event'],'benchmark_failed')
        call=report['summary']['first_call_per_route']['host_npu']
        self.assertIsNone(call['total_ms']);self.assertGreater(call['attempt_elapsed_ms'],0.)
    def test_bad_pl_pixels_fail_after_timed_compute(self):
        with self.assertRaisesRegex(RuntimeError,'PL pixels'):
            chain.run_benchmark(self.args,FakeFpga(self.clock,self.args.output,True),FakeNpu(self.clock),clock=self.clock)
        sample=json.loads((self.args.output/'samples.json').read_text())[0]
        call=sample['routes']['fpga_npu']
        self.assertEqual(call['status'],'FAILED');self.assertAlmostEqual(call['total_ms'],5.)
        self.assertIsNotNone(call['validation_ms'])


if __name__=='__main__':unittest.main()
