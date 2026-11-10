"""One-command staged detector/PREACTION training on saved causal pairs.

Never promotes a checkpoint or invokes robot/camera interfaces. --npu adds
quantization and measured NPU inference, not NPU gradient training.
"""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys,time

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pairs',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--npu',action='store_true');p.add_argument('--list-only',action='store_true')
    p.add_argument('--detector-mode',choices=('mixed-replay','nearfield-only'),default='mixed-replay',
                   help='Retain original training views by default; nearfield-only is a comparison experiment.')
    a=p.parse_args();scripts=Path(__file__).resolve().parent;out=a.output.resolve();pairs=a.pairs.resolve()
    manifest=json.loads(pairs.read_text());n=len(manifest['pairs'])
    npu=Path('/home/aup/Camera/output/preaction_20261023/npu')
    detector=out/'detector';forecast=out/'forecast'
    detector_script='train_mixed_detector.py' if a.detector_mode=='mixed-replay' else 'train_detector.py'
    commands=[
        [sys.executable,str(scripts/detector_script),'--pairs',str(pairs),'--output',str(detector)],
        [sys.executable,str(scripts/'train.py'),'--pairs',str(pairs),'--output',str(forecast),
         '--yolo',str(detector/'training/weights/best.pt')],
    ]
    # Mixed training writes its fixed-checkpoint old/near/mixed comparisons itself.
    if a.detector_mode=='nearfield-only':
        commands.append([sys.executable,str(scripts/'evaluate_detector.py'),'--run',str(detector)])
    commands.append([sys.executable,str(scripts/'evaluate.py'),'--run',str(forecast)])
    if a.npu:
        commands.extend([
            ['bash','/home/aup/NPU/ryzenai.sh','python',str(npu/'quantize_preaction.py'),
             '--model',str(forecast/'preaction.onnx'),'--calibration',str(forecast/'calib_train.npz'),
             '--output',str(out/'npu/preaction_int8.onnx'),'--op-types','Conv,AveragePool,Concat,Slice,Relu'],
            ['bash','/home/aup/NPU/ryzenai.sh','python',str(npu/'benchmark_preaction.py'),
             '--model',str(out/'npu/preaction_int8.onnx'),'--fp32-model',str(forecast/'preaction.onnx'),
             '--data',str(forecast/'eval_inputs.npz'),'--output',str(out/'npu/benchmark'),
             '--max-samples',str(n),'--warmup','5','--repeat','20'],
            [sys.executable,str(scripts/'evaluate.py'),'--run',str(forecast),
             '--npu-outputs',str(out/'npu/benchmark/outputs.npz')],
        ])
    plan=dict(pairs=str(pairs),pairs_sha256=hashlib.sha256(pairs.read_bytes()).hexdigest(),
              commands=commands,detector_mode=a.detector_mode,
              automatic_promotion=False,robot_commands=0)
    if a.list_only:print(json.dumps(plan,indent=2));return
    out.mkdir(parents=True,exist_ok=False);(out/'pipeline.json').write_text(json.dumps(plan,indent=2))
    completed=[]
    for i,command in enumerate(commands):
        start=time.perf_counter();print(f'Stage {i+1}/{len(commands)}: {Path(command[1]).name}',flush=True)
        with (out/f'stage_{i+1:02d}.log').open('w') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        completed.append(dict(stage=i+1,seconds=time.perf_counter()-start))
        (out/'completed_stages.json').write_text(json.dumps(completed,indent=2))
    print(json.dumps(dict(status='OFFLINE_TRAINING_AND_EVALUATION_COMPLETE',output=str(out),automatic_promotion=False)))

if __name__=='__main__':main()
