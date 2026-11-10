"""Export official RT-DETRv2-S: fixed square RGB /255 -> [1,300,6] xyxy,score,class.

The upstream resize is a stretch (NOT YOLO letterboxing). The class IDs are the
contiguous COCO IDs in deploy mode. Check raw PyTorch/ORT parity on a real image.
"""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--size', type=int, default=416)
    p.add_argument('--source', type=Path, default=ROOT/'experiments/RT-DETR')
    p.add_argument('--checkpoint', type=Path, default=ROOT/'models/rtdetrv2_r18.pth')
    p.add_argument('--image', type=Path, default=ROOT/'output/benchmark_fixture/last_session_color.png')
    a = p.parse_args()
    if a.size < 128 or a.size % 32:
        p.error('size must be >=128 and divisible by 32')
    sys.path.insert(0, str(a.source/'rtdetrv2_pytorch'))
    import cv2
    import numpy as np
    import torch
    import onnx
    import onnxruntime as ort
    from src.core import YAMLConfig
    torch.set_num_threads(4)
    cfg = YAMLConfig(str(a.source/'rtdetrv2_pytorch/configs/rtdetrv2/rtdetrv2_r18vd_120e_coco.yml'))
    cfg.yaml_cfg['PResNet']['pretrained'] = False
    cfg.yaml_cfg['eval_spatial_size'] = [a.size, a.size]
    state = torch.load(a.checkpoint, map_location='cpu', weights_only=True)
    state = state['ema']['module'] if 'ema' in state else state['model']
    # These are cached 640px anchor buffers, not learned weights. The constructor
    # has regenerated them for the requested export resolution.
    buffers = {'decoder.anchors','decoder.valid_mask'}
    state = {key:value for key,value in state.items() if key not in buffers}
    missing,unexpected = cfg.model.load_state_dict(state,strict=False)
    if set(missing)!=buffers or unexpected:
        raise RuntimeError(f'Checkpoint mismatch: {missing}, {unexpected}')

    class Deploy(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.model = cfg.model.deploy()
            self.post = cfg.postprocessor.deploy()
            self.register_buffer('size', torch.tensor([[a.size, a.size]], dtype=torch.int64))

        def forward(self, images):
            labels, boxes, scores = self.post(self.model(images), self.size)
            return torch.cat((boxes, scores.unsqueeze(-1), labels.float().unsqueeze(-1)), -1)

    model = Deploy().eval()
    image = cv2.imread(str(a.image))
    if image is None:
        raise ValueError(f'Cannot read parity image {a.image}')
    data = cv2.dnn.blobFromImage(image, 1/255., (a.size, a.size), swapRB=True)
    output = ROOT/f'models/rtdetrv2_r18_{a.size}.onnx'
    with torch.inference_mode():
        expected = model(torch.from_numpy(data)).numpy()
        torch.onnx.export(model, torch.from_numpy(data), str(output), input_names=['images'],
                          output_names=['detections'], opset_version=17, dynamo=False)
    onnx.checker.check_model(onnx.load(str(output)))
    options = ort.SessionOptions()
    options.intra_op_num_threads = 4
    session = ort.InferenceSession(str(output), options, providers=['CPUExecutionProvider'])
    actual = session.run(None, {'images': data})[0]
    # Ranking of very low-score queries can differ. Compare useful predictions by
    # class and nearest coordinates rather than demanding an identical top-k order.
    errors = []
    for box in expected[0][expected[0, :, 4] >= .35]:
        matches = actual[0][actual[0, :, 5] == box[5]]
        if not len(matches):
            raise AssertionError('Missing class in ONNX export')
        best = matches[np.argmin(np.max(np.abs(matches[:, :4]-box[:4]), axis=1))]
        np.testing.assert_allclose(best[:4], box[:4], atol=.05, rtol=1e-4)
        np.testing.assert_allclose(best[4], box[4], atol=1e-4, rtol=1e-4)
        errors.append(float(np.max(np.abs(best[:4]-box[:4]))))
    # Same COCO IDs, modern spelling (motorcycle/couch/tv), shared with YOLO.
    output.with_suffix('.names').write_text((ROOT/'models/yolo26n-seg.names').read_text())
    report = dict(source='https://github.com/lyuwenyu/RT-DETR',
                  source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=a.source,text=True).strip(),
                  checkpoint_url='https://github.com/lyuwenyu/storage/releases/download/v0.2/rtdetrv2_r18vd_120e_coco_rerun_48.1.pth',
                  checkpoint_sha256=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
                  sha256=hashlib.sha256(output.read_bytes()).hexdigest(), input_size=a.size,
                  preprocessing='stretch RGB float32 /255', output='xyxy in input pixels, score, contiguous COCO class',
                  license='Apache-2.0 (upstream)', parity_image=str(a.image),
                  parity_detections=len(errors), parity_max_box_error_pixels=max(errors,default=None))
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__ == '__main__':
    main()
