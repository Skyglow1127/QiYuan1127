"""Offline safety checks for live RGB metadata; never opens a camera or DDS."""
import importlib.util
import ctypes as C
from pathlib import Path
import threading,time,unittest,json
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('kr_live',ROOT/'scripts/kr260_live_server.py')
live=importlib.util.module_from_spec(spec);spec.loader.exec_module(live)
class LiveCalibrationTest(unittest.TestCase):
 def test_sdk_packed_abi(self):
  # Sizes/offsets independently checked with the bundled SDK's C++ header.
  self.assertEqual([C.sizeof(t) for t in (live.Intrinsic,live.Distortion,live.Transform,live.CameraParam)],[20,32,48,153])
  self.assertEqual([C.alignment(t) for t in (live.Intrinsic,live.Distortion,live.Transform,live.CameraParam)],[1,1,1,1])
  self.assertEqual([getattr(live.CameraParam,n).offset for n in ('depthIntrinsic','rgbIntrinsic','depthDistortion','rgbDistortion','transform','isMirrored')],[0,20,40,72,104,152])
 def param(self):
  p=live.CameraParam();p.rgbIntrinsic=live.Intrinsic(500,501,320,240,640,480)
  for i,n in enumerate(('k1','k2','k3','k4','k5','k6','p1','p2')):setattr(p.rgbDistortion,n,i+1)
  return p
 def test_opencv_distortion_order(self):
  c=live.color_calibration(self.param(),{'width':640,'height':480})
  self.assertEqual(c['distortion'],[1,2,7,8,3,4,5,6]);self.assertEqual(c['camera_matrix'][1][1],501)
 def test_reject_wrong_profile_mirrored_and_nan(self):
  with self.assertRaises(ValueError):live.color_calibration(self.param(),{'width':1280,'height':720})
  p=self.param();p.isMirrored=True
  with self.assertRaises(ValueError):live.color_calibration(p,{'width':640,'height':480})
  p=self.param();p.rgbIntrinsic.fx=float('nan')
  with self.assertRaises(ValueError):live.color_calibration(p,{'width':640,'height':480})
 def test_snapshot_rejects_stale_stopped_and_missing_calibration(self):
  c=object.__new__(live.Camera);c.condition=threading.Condition();c.stop=threading.Event();c.error=None;c.device={'serial':'test'};c.calibration=live.color_calibration(self.param(),{'width':640,'height':480});c.calibration_error=None
  c.latest={'color':(b'fixture',{'index':1,'received_mono':time.monotonic(),'width':640,'height':480})}
  result=c.snapshot();self.assertEqual(result['frame']['index'],1);json.dumps(result,allow_nan=False)
  c.latest['color'][1]['received_mono']=time.monotonic()-1
  with self.assertRaises(ValueError):c.snapshot()
  c.latest['color'][1]['received_mono']=time.monotonic();c.calibration=None
  with self.assertRaises(ValueError):c.snapshot()
  c.stop.set()
  with self.assertRaises(ValueError):c.snapshot()
if __name__=='__main__':unittest.main()
