"""Qwen task grounding -> adapted YOLO -> action-conditioned PREACTION preview.

Reads a saved current image and an already-known planned command. Predicted
images never replace observed frames or enter the robot execution interface.
"""
import os
os.environ.setdefault('YOLO_OFFLINE','true')
os.environ.setdefault('YOLO_CONFIG_DIR','/home/aup/Camera/output/preaction_20261023/yolo_settings')
from pathlib import Path
import argparse,json,math,subprocess,sys,time
import cv2,numpy as np,torch
from PIL import Image
from ultralytics import YOLO
from core import PreAction,sha,current_target,box_of,crop_spec,crop,pack_input,future_box,render_forecast,box_iou
sys.path.insert(0,'/home/aup/Camera/scripts')
from qwen_llama import create_llama_runtime
from grasp_semantics import parse_objects,pixel_box

def positive_horizon(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
        raise ValueError('horizon_s must be a finite positive JSON number')
    return float(value)

def validate_horizon(run,config,command):
    """Use the hash-bound training split, never the requested horizon or labels."""
    pairs_path=run/'pairs.json'
    if not config.get('pairs_sha256') or sha(pairs_path)!=config['pairs_sha256']:
        raise ValueError('Training pairs metadata missing or SHA256 mismatch; horizon cannot be verified')
    pairs=json.loads(pairs_path.read_text())['pairs']
    train=[row for row in pairs if row['split']=='train']
    if not train or len(train)!=config['train_pairs']:
        raise ValueError('Training pair count does not match checkpoint metadata')
    trained=sorted({positive_horizon(row['horizon_s']) for row in train})
    requested=positive_horizon(command['horizon_s'])
    if requested not in trained:
        raise ValueError(f'Requested horizon {requested:g}s was not trained; supported horizons: {trained}')
    return requested,dict(trained_horizons_s=trained,source=str(pairs_path.resolve()),
                          source_sha256=config['pairs_sha256'],split='train')

def validate_npu_model(run,config,model):
    """Bind the quantized model to this run before attributing its horizon."""
    metadata_path=model.with_suffix('.quantization.json')
    metadata=json.loads(metadata_path.read_text())
    if (metadata.get('source_sha256')!=config.get('onnx_sha256') or
        sha(run/'preaction.onnx')!=config.get('onnx_sha256') or
        sha(model)!=metadata.get('model_sha256')):
        raise ValueError('NPU model quantization metadata does not match this training run')
    return dict(source=str(metadata_path.resolve()),source_sha256=sha(metadata_path),
                model_sha256=metadata['model_sha256'],fp32_model_sha256=metadata['source_sha256'])

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--image',type=Path,required=True)
    p.add_argument('--command',type=Path,required=True,help='JSON: q_current[7], delta_q_command[7], horizon_s matching the training run')
    p.add_argument('--output',type=Path,required=True,help='Fresh output directory')
    p.add_argument('--backend',choices=['cpu','npu'],default='npu')
    p.add_argument('--npu-model',type=Path);p.add_argument('--npu-cache',type=Path)
    p.add_argument('--qwen-cache',type=Path,help='Optional semantic result for the exact same image SHA')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(4);cv2.setNumThreads(1);start=time.perf_counter()
    config=json.loads((a.run/'train_config.json').read_text());command=json.loads(a.command.read_text())
    horizon,horizon_evidence=validate_horizon(a.run,config,command)
    npu_binding=None
    if a.backend=='npu':
        if not a.npu_model or not a.npu_cache:raise ValueError('Provide the verified INT8 model and model-bound NPU cache')
        npu_binding=validate_npu_model(a.run,config,a.npu_model)
    image_hash=sha(a.image);bgr=cv2.imread(str(a.image));height,width=bgr.shape[:2]
    if (width,height)!=(640,480):raise ValueError('This prototype expects a saved original 640x480 RGB frame')
    if a.qwen_cache:
        semantic=json.loads(a.qwen_cache.read_text())
        if semantic['image_sha256']!=image_hash:raise ValueError('Semantic cache image mismatch')
    else:
        prompt='Find the visible red cube. Return only a JSON array with bbox_2d [x1,y1,x2,y2] in 0..1000 normalized coordinates and label "red cube". If absent, return [].'
        with create_llama_runtime('/home/aup/Camera/models/qwen3.5-2b-gguf/model-q8_0.gguf',threads=2,
                 device='vulkan',strict_json=True,log_path=str(a.output/'qwen.log')) as runtime:
            runtime.object_limit=1
            with Image.open(a.image) as im:raw,timing=runtime.infer(im.convert('RGB'),prompt,max_tokens=96)
        semantic=dict(image_sha256=image_hash,raw=raw,timing=timing,
            boxes=[pixel_box(o['bbox_2d'],width,height) for o in parse_objects(raw)],weights_updated=False)
    (a.output/'semantic.json').write_text(json.dumps(semantic,ensure_ascii=False,indent=2))
    model=YOLO(config['yolo_weights']);mask,selection=current_target(model,bgr);box=box_of(mask)
    agreement=max([box_iou(box,np.array(b)) for b in semantic['boxes']],default=0.)
    if agreement<.3:
        (a.output/'result.json').write_text(json.dumps(dict(status='NO_SEMANTIC_TARGET_AGREEMENT',
            image_sha256=image_hash,agreement=agreement,robot_commands=0,motion_command_ready=False),indent=2))
        print('NO_SEMANTIC_TARGET_AGREEMENT',flush=True);return
    spec=crop_spec(box);rgb=cv2.cvtColor(crop(bgr,spec),cv2.COLOR_BGR2RGB);cm=crop(mask,spec,True)
    x=pack_input(rgb,cm,command['q_current'],command['delta_q_command'],horizon)[None]
    if list(x.shape)!=config['input_shape']:raise ValueError('Prepared input shape does not match training metadata')
    np.save(a.output/'input.npy',x)
    backend_start=time.perf_counter()
    if a.backend=='npu':
        script=Path('/home/aup/Camera/output/preaction_20261023/npu/infer_preaction_npu.py')
        args=['bash','/home/aup/NPU/ryzenai.sh','python',str(script),'--model',str(a.npu_model),
              '--input',str(a.output/'input.npy'),'--cache',str(a.npu_cache),'--output',str(a.output/'npu_result.json')]
        with (a.output/'npu_process.log').open('w') as log:
            subprocess.run(args,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=180)
        evidence=json.loads((a.output/'npu_result.json').read_text())
        if evidence['status']!='NPU_EXECUTION_CONFIRMED':raise RuntimeError('NPU execution unconfirmed')
        with np.load(a.output/'npu_result.npz',allow_pickle=False) as outputs:raw_pred=outputs['output0']
    else:
        net=PreAction(pretrained=False);net.load_state_dict(torch.load(a.run/'best.pt',weights_only=True));net.eval()
        with torch.no_grad():raw_pred=net(torch.from_numpy(x)).numpy()
        evidence=dict(backend='cpu',npu_used=False)
    if list(raw_pred.shape)!=config['output_shape'] or not np.isfinite(raw_pred).all():
        raise ValueError('Prediction must match the finite training output shape')
    pred=raw_pred.reshape(4)
    backend_seconds=time.perf_counter()-backend_start
    predicted_box=future_box(box,spec,pred);preview,pred_mask=render_forecast(rgb,cm,box,spec,pred)
    Image.fromarray(rgb).save(a.output/'current_roi.png');Image.fromarray(preview).save(a.output/'predicted_roi.png')
    cv2.imwrite(str(a.output/'predicted_mask.png'),np.uint8(np.clip(pred_mask*255,0,255)))
    report=dict(status='EXPERIMENTAL_PREACTION_PREVIEW',image=str(a.image.resolve()),image_sha256=image_hash,
        command_source=str(a.command.resolve()),command_sha256=sha(a.command),command=command,
        current_selection=selection,qwen_yolo_box_iou=agreement,current_bbox=box.tolist(),crop=spec,
        forecast_bbox=predicted_box.tolist(),motion_parameters_scaled=pred.tolist(),horizon_s=horizon,
        horizon_evidence=horizon_evidence,npu_model_binding=npu_binding,
        forecast_kind='affine warp of current observed pixels; no unseen-content generation',
        quality_status='action_effect_not_demonstrated; prototype only',
        backend=a.backend,backend_process_and_session_seconds=backend_seconds,
        pipeline_seconds=time.perf_counter()-start,
        timing_scope='Saved-image single-shot pipeline includes cold model/process starts, not a live-loop benchmark',
        npu_evidence_file=str(a.output/'npu_result.json') if a.backend=='npu' else None,
        prediction_is_observation=False,pose_available=False,motion_command_ready=False,robot_commands=0)
    (a.output/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
