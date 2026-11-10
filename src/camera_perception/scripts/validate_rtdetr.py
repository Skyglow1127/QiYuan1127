"""Check C++ RT-DETR coordinates/scores against Python ORT on non-square input."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import onnxruntime as ort

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,default=ROOT/'models/rtdetrv2_r18_416.onnx')
    p.add_argument('--image',type=Path,default=ROOT/'output/benchmark_fixture/last_session_color.png')
    p.add_argument('--result',type=Path,default=ROOT/'output/model_comparison/rtdetr/replay_result.json')
    a=p.parse_args()
    image=cv2.imread(str(a.image))
    if image is None:
        raise ValueError('Cannot read image')
    options=ort.SessionOptions(); options.intra_op_num_threads=4
    session=ort.InferenceSession(str(a.model),options,providers=['CPUExecutionProvider'])
    size=session.get_inputs()[0].shape[2]
    tensor=cv2.dnn.blobFromImage(image,1/255.,(size,size),swapRB=True)
    raw=session.run(None,{session.get_inputs()[0].name:tensor})[0][0]
    names=a.model.with_suffix('.names').read_text().splitlines()
    expected=[]
    for row in raw:
        x1,y1,x2,y2=row[:4]*np.array([image.shape[1],image.shape[0]]*2)/size
        x1,y1=np.floor([x1,y1]); x2,y2=np.ceil([x2,y2])
        x1,x2=np.clip([x1,x2],0,image.shape[1]); y1,y2=np.clip([y1,y2],0,image.shape[0])
        expected.append((names[int(row[5])],float(row[4]),np.array([x1,y1,x2-x1,y2-y1])))
    candidates=json.loads(a.result.read_text())['candidates']
    if not candidates:
        raise AssertionError('Parity test needs at least one actual detection')
    errors=[]
    for c in candidates:
        matches=[e for e in expected if e[0]==c['class'] and abs(e[1]-c['confidence'])<1e-4]
        if not matches:
            raise AssertionError('C++ class/score differs from ONNX reference')
        error=min(float(np.max(np.abs(e[2]-c['box_xywh']))) for e in matches)
        assert error<=1, (c,error)
        errors.append(error)
    report={'checked':len(errors),'max_box_coordinate_error_pixels':max(errors),
            'score_tolerance':1e-4,'image_shape':list(image.shape),'scope':'implementation parity, not detection accuracy'}
    a.result.with_name('rtdetr_parity.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))

if __name__=='__main__':
    main()
