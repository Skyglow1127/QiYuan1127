import ctypes
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('grasp_accel_fpga', ROOT/'scripts/grasp_accel_fpga.py')
fpga = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fpga)


class PreprocessTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        lib = Path(cls.temp.name)/'area2x.so'
        subprocess.run(['g++','-shared','-fPIC','-O2','-Wno-unknown-pragmas',
            str(ROOT/'fpga/grasp_preprocess_v1/area2x.cpp'),'-o',str(lib)],check=True)
        cls.core=ctypes.CDLL(str(lib)).area2x
        cls.core.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_uint32]

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def test_matches_opencv_and_npu_tensor(self):
        rng=np.random.default_rng(42)
        for frame in [rng.integers(0,256,(480,640,3),dtype=np.uint8),
                      np.zeros((480,640,3),np.uint8),np.full((480,640,3),255,np.uint8)]:
            ref=cv2.cvtColor(cv2.resize(frame,(320,240),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB)
            np.testing.assert_array_equal(fpga.cpu_area2x(frame),ref)
            packed=np.zeros((480,640,4),np.uint8);packed[:,:,:3]=frame
            outputs=[]
            for y in range(0,480,8):
                out=np.zeros((4,320,4),np.uint8);stats=np.zeros(4,np.uint64)
                self.core(packed[y:y+8].ctypes.data,out.ctypes.data,stats.ctypes.data,8)
                self.assertEqual(stats.tolist(),[8,1280,fpga.ABI_TAG,0])
                outputs.append(out[:,:,:3])
            actual=np.concatenate(outputs)
            np.testing.assert_array_equal(actual,ref)
            expected=np.ascontiguousarray(ref.transpose(2,0,1)[None],dtype=np.float32)/np.float32(255)
            np.testing.assert_array_equal(fpga.rgb_to_tensor(actual),expected)

    def test_core_invalid_shape_reports_error_without_output(self):
        src=np.zeros((8,640,4),np.uint8);out=np.full((4,320,4),123,np.uint8)
        for rows in [0,1,9,10,4294967295]:
            stats=np.zeros(4,np.uint64)
            self.core(src.ctypes.data,out.ctypes.data,stats.ctypes.data,rows)
            self.assertEqual(int(stats[3]),1)
            self.assertTrue((out==123).all())

    def test_no_silent_resize_or_cpu_fallback(self):
        with self.assertRaises(ValueError): fpga.cpu_area2x(np.zeros((240,320,3),np.uint8))
        with self.assertRaises(ValueError): fpga.process(np.zeros((480,640,3),np.uint8),'fpga')
        _,_,e=fpga.process(np.zeros((480,640,3),np.uint8),'cpu')
        self.assertFalse(e['hardware_executed'])

    def test_probe_does_not_claim_hardware_for_operating_fpga(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'sys/class/fpga_manager/fpga0';p.mkdir(parents=True)
            (p/'state').write_text('operating\n')
            result=fpga.probe(Path(t)/'sys',Path(t)/'dev')
            self.assertFalse(result['ready']);self.assertFalse(result['hardware_executed'])


if __name__=='__main__':unittest.main()
