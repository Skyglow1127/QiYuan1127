#!/usr/bin/env python3
"""Bounded Orbbec v1 C-API capture; no firmware changes or robot commands."""
import argparse, array, ctypes as C, gzip, hashlib, json, math, os, sys, time
from collections import deque
from pathlib import Path
P=C.c_void_p; U=C.c_uint32; I=C.c_int
ap=argparse.ArgumentParser()
ap.add_argument('--mode',choices=['probe','depth','color','both'],default='probe')
ap.add_argument('--seconds',type=float,default=12)
ap.add_argument('--output',default='capture')
ap.add_argument('--fpga',action='store_true',help='Run PL depth gate on sampled live frames')
ap.add_argument('--fpga-period',type=float,default=0.2)
ap.add_argument('--align', action='store_true', help='SDK D2C with static-scene RGB-D pairing')
ap.add_argument('--max-pair-skew-ms', type=float, default=40)
a=ap.parse_args()
if a.align and a.mode not in ('both','probe'): ap.error('--align requires both or probe mode')
if not math.isfinite(a.seconds) or not 0<a.seconds<=300: ap.error('seconds must be 0..300')
if not math.isfinite(a.max_pair_skew_ms) or not 0<a.max_pair_skew_ms<=100: ap.error('invalid pair skew')
class Intrinsic(C.Structure):
 _fields_=[(n,C.c_float) for n in ('fx','fy','cx','cy')]+[('width',C.c_int16),('height',C.c_int16)]
class Distortion(C.Structure):
 _fields_=[(n,C.c_float) for n in ('k1','k2','k3','k4','k5','k6','p1','p2')]
class Transform(C.Structure):
 _fields_=[('rot',C.c_float*9),('trans',C.c_float*3)]
class CameraParam(C.Structure):
 _fields_=[('depthIntrinsic',Intrinsic),('rgbIntrinsic',Intrinsic),('depthDistortion',Distortion),('rgbDistortion',Distortion),('transform',Transform),('isMirrored',C.c_bool)]
def fields(obj):return {n:getattr(obj,n) for n,_ in obj._fields_}
root=Path(__file__).resolve().parent
sdk=C.CDLL(str(root/'lib/libOrbbecSDK.so.1.10.18'))
sdk.ob_error_message.argtypes=[P];sdk.ob_error_message.restype=C.c_char_p
sdk.ob_delete_error.argtypes=[P];sdk.ob_delete_error.restype=None

def bind(name,result,*args):
 f=getattr(sdk,name); f.restype=result;f.argtypes=[*args,C.POINTER(P)]
 def call(*values):
  error=P();value=f(*values,C.byref(error))
  if error:
   message=sdk.ob_error_message(error).decode(errors='replace');sdk.ob_delete_error(error)
   raise RuntimeError(name+': '+message)
  return value
 return call
spec={
 'ob_set_logger_to_console':(None,I),'ob_create_pipeline':(P,), 'ob_delete_pipeline':(None,P),
 'ob_pipeline_get_device':(P,P),'ob_device_get_device_info':(P,P),
 'ob_device_info_name':(C.c_char_p,P),'ob_device_info_firmware_version':(C.c_char_p,P),
 'ob_device_info_serial_number':(C.c_char_p,P),'ob_delete_device_info':(None,P),'ob_delete_device':(None,P),
 'ob_pipeline_get_stream_profile_list':(P,P,I),'ob_stream_profile_list_count':(U,P),
 'ob_stream_profile_list_get_profile':(P,P,I),'ob_stream_profile_format':(I,P),
 'ob_video_stream_profile_width':(U,P),'ob_video_stream_profile_height':(U,P),'ob_video_stream_profile_fps':(U,P),
 'ob_delete_stream_profile':(None,P),'ob_delete_stream_profile_list':(None,P),
 'ob_create_config':(P,),'ob_delete_config':(None,P),'ob_config_enable_stream':(None,P,P),
 'ob_pipeline_start_with_config':(None,P,P),'ob_pipeline_stop':(None,P),
 'ob_pipeline_wait_for_frameset':(P,P,U),'ob_frameset_depth_frame':(P,P),'ob_frameset_color_frame':(P,P),
 'ob_frame_data':(P,P),'ob_frame_data_size':(U,P),'ob_frame_format':(I,P),
 'ob_frame_index':(C.c_uint64,P),'ob_frame_time_stamp_us':(C.c_uint64,P),
 'ob_video_frame_width':(U,P),'ob_video_frame_height':(U,P),'ob_depth_frame_get_value_scale':(C.c_float,P),
 'ob_delete_frame':(None,P),
 'ob_get_d2c_depth_profile_list':(P,P,P,I), 'ob_config_set_align_mode':(None,P,I),
 'ob_config_set_depth_scale_require':(None,P,C.c_bool), 'ob_config_set_frame_aggregate_output_mode':(None,P,I),
 'ob_pipeline_enable_frame_sync':(None,P), 'ob_pipeline_get_camera_param':(CameraParam,P),
 'ob_frame_system_time_stamp':(C.c_uint64,P)}
f={name:bind(name,*s) for name,s in spec.items()}
f['ob_set_logger_to_console'](3)
pipeline=f['ob_create_pipeline'](); config=None;started=False;profiles=[];lists=[];gate=None;last_pl=0;pl_results=[]
report={'sdk':'1.10.18','mode':a.mode,'profiles':{},'streams':{},'note':'Color and depth are not asserted to be exposure-synchronized or spatially aligned.'}
try:
 dev=f['ob_pipeline_get_device'](pipeline);info=f['ob_device_get_device_info'](dev)
 report['device']={k:f['ob_device_info_'+fn](info).decode(errors='replace') for k,fn in [('name','name'),('firmware','firmware_version'),('serial','serial_number')]}
 f['ob_delete_device_info'](info);f['ob_delete_device'](dev)
 print('DEVICE',json.dumps(report['device']),flush=True)
 choices={}
 for name,sensor,wanted in [('depth',3,11),('color',2,5)]:
  pl=f['ob_pipeline_get_stream_profile_list'](pipeline,sensor);lists.append(pl);entries=[]
  for i in range(f['ob_stream_profile_list_count'](pl)):
   p=f['ob_stream_profile_list_get_profile'](pl,i);profiles.append(p)
   d={'index':i,'width':f['ob_video_stream_profile_width'](p),'height':f['ob_video_stream_profile_height'](p),'fps':f['ob_video_stream_profile_fps'](p),'format':f['ob_stream_profile_format'](p)}
   entries.append((d,p))
  report['profiles'][name]=[d for d,p in entries]
  candidates=[(d,p) for d,p in entries if d['format']==wanted and d['width']<=640 and d['height']<=480 and 0<d['fps']<=30]
  if not candidates: raise RuntimeError('No compatible '+name+' profile: '+str(report['profiles'][name]))
  choices[name]=max(candidates,key=lambda item:(item[0]['width']*item[0]['height'],item[0]['fps']))
  print('PROFILE',name,json.dumps(choices[name][0]),'available',len(entries),flush=True)
 align_mode=0
 if a.align:
  report['alignment_attempts']=[]
  for mode in (1,2):
   try:
    dl=f['ob_get_d2c_depth_profile_list'](pipeline,choices['color'][1],mode);lists.append(dl);candidates=[]
    for i in range(f['ob_stream_profile_list_count'](dl)):
     p=f['ob_stream_profile_list_get_profile'](dl,i);profiles.append(p)
     d={'index':i,'width':f['ob_video_stream_profile_width'](p),'height':f['ob_video_stream_profile_height'](p),'fps':f['ob_video_stream_profile_fps'](p),'format':f['ob_stream_profile_format'](p)}
     if d['format'] in (8,10,11,12,24,28) and d['fps']==choices['color'][0]['fps']:candidates.append((d,p))
    if candidates:
     choices['depth']=min(candidates,key=lambda x:abs(x[0]['width']-640)+abs(x[0]['height']-480));align_mode=mode;break
   except RuntimeError as e:report['alignment_attempts'].append(str(e))
  if not align_mode:raise RuntimeError('No SDK D2C profile; no resizing fallback permitted')
  report['alignment_mode']={1:'SDK_D2C_HW',2:'SDK_D2C_SW'}[align_mode]
  report['note']='SDK spatial D2C. Host-receipt pairing only; exposure sync and robot stationarity are NOT established.'
  print('ALIGNMENT',report['alignment_mode'],choices['depth'][0],flush=True)
 out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
 if a.mode!='probe':
  if a.fpga:
   if a.mode=='color':raise ValueError('FPGA mode needs depth')
   from run_depth_gate import DepthGate
   gate=DepthGate()
  config=f['ob_create_config']();names=['depth','color'] if a.mode=='both' else [a.mode]
  for name in names:
   f['ob_config_enable_stream'](config,choices[name][1]);report['streams'][name]={'profile':choices[name][0],'frames':0,'unique_indices':set()}
  if a.align:
   f['ob_config_set_align_mode'](config,align_mode)
   f['ob_config_set_depth_scale_require'](config,True)
   f['ob_config_set_frame_aggregate_output_mode'](config,0)
   try:f['ob_pipeline_enable_frame_sync'](pipeline);report['sdk_frame_sync_enabled']=True
   except RuntimeError as e:report['sdk_frame_sync_enabled']=False;report['sync_warning']=str(e)
  f['ob_pipeline_start_with_config'](pipeline,config);started=True
  if a.align:
   param=f['ob_pipeline_get_camera_param'](pipeline);k=param.rgbIntrinsic
   if param.isMirrored:raise RuntimeError('Mirrored camera unsupported')
   if not all(math.isfinite(getattr(k,n)) for n in ('fx','fy','cx','cy')) or min(k.fx,k.fy)<=0:raise RuntimeError('Invalid intrinsics')
   report['camera_parameters']={'color_intrinsic':fields(k),'color_distortion':fields(param.rgbDistortion),'depth_intrinsic':fields(param.depthIntrinsic),'depth_distortion':fields(param.depthDistortion),'depth_to_color_rotation':list(param.transform.rot),'depth_to_color_translation_mm':list(param.transform.trans),'is_mirrored':bool(param.isMirrored)}
   report['pairing']='host_receipt_approximate_stationary_only'
   report['robot_stationarity_verified']=False
   report['accepted_pairs']=0
  colors=deque(maxlen=8);paired=None;used_color=-1;used_depth=-1
  start=time.monotonic();last={};arrivals={name:[] for name in names}
  while time.monotonic()-start<a.seconds:
   fs=f['ob_pipeline_wait_for_frameset'](pipeline,1000)
   if not fs:continue
   try:
    current={}
    for name in names:
     frame=f['ob_frameset_'+name+'_frame'](fs)
     if not frame:continue
     try:
      r=report['streams'][name];r['frames']+=1;r['unique_indices'].add(f['ob_frame_index'](frame));arrivals[name].append(time.monotonic())
      raw=C.string_at(f['ob_frame_data'](frame),f['ob_frame_data_size'](frame))
      meta={'width':f['ob_video_frame_width'](frame),'height':f['ob_video_frame_height'](frame),'format':f['ob_frame_format'](frame),'device_timestamp_us':f['ob_frame_time_stamp_us'](frame),'index':f['ob_frame_index'](frame),'host_received_ms':f['ob_frame_system_time_stamp'](frame)}
      if name=='depth':meta['mm_per_unit']=f['ob_depth_frame_get_value_scale'](frame)
      last[name]=(raw,meta);current[name]=(raw,meta)
      if name=='depth' and gate is not None and time.monotonic()-last_pl>=a.fpga_period:
       scale=meta['mm_per_unit']
       if scale<=0:raise RuntimeError('Invalid depth scale')
       filtered,pl=gate.run(raw,max(1,int(300/scale+0.999999)),min(65534,int(1500/scale)))
       pl.update({'frame_index':meta['index'],'mm_per_unit':scale,'host_monotonic_s':time.monotonic(),'mean_mm':pl['mean_raw']*scale if pl['mean_raw'] is not None else None})
       pl_results.append(pl);last_pl=time.monotonic();print('PL_RESULT',json.dumps(pl),flush=True)
     finally:f['ob_delete_frame'](frame)
    if a.align and 'color' in current and 'depth' in current:
     color=current['color'];depth=current['depth'];dm=depth[1]
     if not colors or colors[-1][1]['index']!=color[1]['index']:colors.append(color)
     candidates=[c for c in colors if c[1]['index']>used_color and c[1]['host_received_ms']>0]
     if candidates and dm['index']>used_depth and dm['host_received_ms']>0 and time.monotonic()-start>=1:
      color=min(candidates,key=lambda c:abs(c[1]['host_received_ms']-dm['host_received_ms']));cm=color[1]
      skew=abs(cm['host_received_ms']-dm['host_received_ms'])
      if skew<=a.max_pair_skew_ms:
       if (cm['width'],cm['height'])!=(dm['width'],dm['height']) or (cm['width'],cm['height'])!=(k.width,k.height):raise RuntimeError('D2C/color/intrinsic dimensions disagree')
       if dm['format'] not in (8,10,11,12,24,28) or len(depth[0])!=dm['width']*dm['height']*2:raise RuntimeError('Invalid aligned depth layout')
       if not math.isfinite(dm['mm_per_unit']) or dm['mm_per_unit']<=0:raise RuntimeError('Invalid depth scale')
       paired={'color':color,'depth':depth};used_color=cm['index'];used_depth=dm['index']
       report['accepted_pairs']+=1;report['saved_pair_host_skew_ms']=skew
   finally:f['ob_delete_frame'](fs)
  elapsed=time.monotonic()-start
  f['ob_pipeline_stop'](pipeline);started=False
  if a.align:
   if paired is None:raise RuntimeError('No acceptable static-scene RGB-D pair')
   last=paired
  for name in names:
   r=report['streams'][name];r['unique_indices']=len(r['unique_indices']);r['elapsed_seconds']=elapsed
   ts=arrivals[name];r['observed_fps']=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0
   if name not in last:raise RuntimeError('No '+name+' frames received')
   raw,meta=last[name];r['last_frame']=meta;r['sha256']=hashlib.sha256(raw).hexdigest();r['bytes']=len(raw)
   if name=='color':
    if not raw.startswith(b'\xff\xd8'):raise RuntimeError('Color data is not JPEG')
    (out/'color.jpg').write_bytes(raw)
   else:
    if meta['format'] not in (8,10,11,12,24,28) or len(raw)!=meta['width']*meta['height']*2:raise RuntimeError('Unexpected depth layout')
    with gzip.open(out/'depth_u16le.raw.gz','wb') as z:z.write(raw)
    pixels=array.array('H');pixels.frombytes(raw)
    if sys.byteorder!='little':pixels.byteswap()
    valid=sorted(v for v in pixels if 0<v<65535)
    r['valid_fraction']=len(valid)/len(pixels)
    r['percentiles_mm']={str(p):valid[min(len(valid)-1,int((len(valid)-1)*p/100))]*meta['mm_per_unit'] for p in (1,10,50,90,99)} if valid else {}
    if not valid:raise RuntimeError('Depth frame has no valid pixels')
   print('RESULT',name,json.dumps(r),flush=True)
 report['fpga_results']=pl_results
 (out/'report.json').write_text(json.dumps(report,indent=2))
 print('SAVED',str(out.resolve()),flush=True)
finally:
 if gate:gate.close()
 if started:f['ob_pipeline_stop'](pipeline)
 if config:f['ob_delete_config'](config)
 for p in profiles:f['ob_delete_stream_profile'](p)
 for p in lists:f['ob_delete_stream_profile_list'](p)
 f['ob_delete_pipeline'](pipeline)
