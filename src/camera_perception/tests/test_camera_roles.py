"""Verify role selection cannot silently depend on USB enumeration order."""
import importlib.util,json,tempfile,unittest,threading,time,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('camera_roles_live',ROOT/'scripts/kr260_live_server.py')
live=importlib.util.module_from_spec(spec);spec.loader.exec_module(live)
class RolesTest(unittest.TestCase):
 def test_reordered_missing_and_duplicate(self):
  devices=[{'serial':'GLOBAL','uid':'1-1.2.4-7'},{'serial':'WRIST','uid':'3-1.1.4-6'}]
  for order in [devices,devices[::-1]]:self.assertEqual(live.select_uid(order,'WRIST'),'3-1.1.4-6')
  with self.assertRaises(ValueError):live.select_uid(devices,'MISSING')
  with self.assertRaises(ValueError):live.select_uid(devices+[devices[1]],'WRIST')
 def test_registry_requires_unique_verified_role(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'roles.json';data={'cameras':{'wrist':{'serial':'WRIST','identity_verified_from_image':True}}}
   p.write_text(json.dumps(data));self.assertEqual(live.resolve_role(p,'wrist')[0],'WRIST')
   data['cameras']['global']=dict(data['cameras']['wrist']);p.write_text(json.dumps(data))
   with self.assertRaises(ValueError):live.resolve_role(p,'wrist')
   del data['cameras']['global'];data['cameras']['wrist']['identity_verified_from_image']=False;p.write_text(json.dumps(data))
   with self.assertRaises(ValueError):live.resolve_role(p,'wrist')
 def test_usb_parent_not_device_number(self):
  with tempfile.TemporaryDirectory() as d:
   base=Path(d);root=base/'sysfs';root.mkdir()
   for bus,serial in [('1-1.2','GLOBAL'),('3-1.1','WRIST')]:
    parent=base/bus;parent.mkdir()
    for port,pid in [('2','0557'),('4','0657')]:
     node=parent/(bus+'.'+port);node.mkdir();(node/'idVendor').write_text('2bc5');(node/'idProduct').write_text(pid)
     if port=='2':(node/'serial').write_text(serial)
     (root/node.name).symlink_to(node,target_is_directory=True)
   self.assertEqual(live.usb_serial_for_uid('3-1.1.4-99',root),'WRIST')
   self.assertEqual(live.usb_serial_for_uid('1-1.2.4-101',root),'GLOBAL')
   self.assertEqual(live.usb_serial_for_uid('4-1.1.4-4',root),'')
 def test_global_cannot_supply_wrist_sample(self):
  c=object.__new__(live.Camera);c.role='global'
  with self.assertRaisesRegex(ValueError,'Global observer'):c.snapshot()
 def test_global_reference_has_distinct_schema_and_atomic_metadata(self):
  c=object.__new__(live.Camera);c.role='global';c.condition=threading.Condition()
  c.stop=threading.Event();c.error=None;c.device={'serial':'GLOBAL','role':'global'}
  c.calibration={'width':640,'height':480};c.calibration_error=None
  raw=b'reference-fixture';c.latest={'color':(raw,{'index':7,'received_mono':time.monotonic(),'width':640,'height':480})}
  result=c.color_snapshot()
  self.assertEqual(result['schema'],'kr260-color-reference-v1')
  self.assertEqual(result['device']['serial'],'GLOBAL')
  self.assertEqual(result['frame']['index'],7)
  self.assertEqual(result['image_sha256'],hashlib.sha256(raw).hexdigest())
  with self.assertRaisesRegex(ValueError,'Global observer'):c.snapshot()
  c.latest['color'][1]['received_mono']-=1
  with self.assertRaisesRegex(ValueError,'stale'):c.color_snapshot()
if __name__=='__main__':unittest.main()
