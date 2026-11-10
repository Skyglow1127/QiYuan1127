"""Small action-conditioned ROI predictor. No camera/robot control APIs.

The learned network predicts visible-image translation and scale. Rendering is
an affine forecast of existing pixels, not synthesis of unseen object surfaces.
"""
from pathlib import Path
import hashlib
import cv2
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F

SIZE = 96
PARAM_SCALE = np.array([10., 10., 5., 5.], np.float32)
YOLO_WEIGHTS = Path('/home/aup/Camera/output/fewshot_qwen_yolo_20261023/training/weights/best.pt')
ENCODER_WEIGHTS = Path('/home/aup/NPU/integration/20261023/red_cube_npu/model.pt')

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def box_of(mask):
    ys, xs = np.nonzero(mask)
    if len(xs) < 20:
        raise ValueError('No usable visible target')
    return np.array([xs.min(), ys.min(), xs.max()+1, ys.max()+1], np.float32)

def red_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = (((hsv[:,:,0] <= 10) | (hsv[:,:,0] >= 170)) &
            (hsv[:,:,1] >= 85) & (hsv[:,:,2] >= 45)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2 or max(stats[1:, cv2.CC_STAT_AREA]) < 50:
        raise ValueError('No red component; sample requires review')
    return (labels == (1 + np.argmax(stats[1:, cv2.CC_STAT_AREA]))).astype(np.float32)

def current_target(model, bgr):
    """Current-frame-only target selection, with explicit color fallback."""
    hsv_mask = red_mask(bgr)
    result = model.predict(bgr, imgsz=416, rect=False, conf=.25, device='cpu',
                           verbose=False, retina_masks=True, max_det=10)[0]
    candidates = []
    if result.masks is not None:
        for polygon, confidence in zip(result.masks.xy, result.boxes.conf.cpu().numpy()):
            mask = np.zeros(bgr.shape[:2], np.uint8)
            cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 1)
            area = mask.sum()
            if area and (mask * hsv_mask).sum()/area >= .2:
                candidates.append((float(confidence), mask.astype(np.float32)))
    if candidates:
        confidence, mask = max(candidates, key=lambda x:x[0])
        return mask, dict(source='frozen_fewshot_yolo', confidence=confidence)
    return hsv_mask, dict(source='explicit_hsv_fallback', confidence=None)

def crop_spec(box):
    center = (box[:2]+box[2:])/2
    side = int(np.clip(np.ceil(max(box[2:]-box[:2])*2.0), 160, 640))
    return [int(np.floor(center[0]-side/2)), int(np.floor(center[1]-side/2)), side]

def crop(array, spec, nearest=False):
    x, y, side = spec
    shape = (side,side)+array.shape[2:]
    out = np.zeros(shape, array.dtype)
    x0,y0,x1,y1 = max(0,x),max(0,y),min(array.shape[1],x+side),min(array.shape[0],y+side)
    if x1>x0 and y1>y0:
        out[y0-y:y1-y,x0-x:x1-x] = array[y0:y1,x0:x1]
    return cv2.resize(out, (SIZE,SIZE), interpolation=cv2.INTER_NEAREST if nearest else cv2.INTER_AREA)

def pack_input(rgb_crop, mask_crop, q, delta, horizon):
    q=np.asarray(q,np.float32); delta=np.asarray(delta,np.float32)
    if q.shape!=(7,) or delta.shape!=(7,) or not np.isfinite(np.r_[q,delta,horizon]).all():
        raise ValueError('Finite seven-joint current state and planned command delta required')
    # J6 is uncalibrated SDK position, not degrees or force; separate scaling.
    state=q / np.array([180.]*6+[100.],np.float32)
    action=delta / np.array([5.]*6+[20.],np.float32)
    constants=np.r_[state,action,float(horizon)/2].astype(np.float32)
    planes=np.broadcast_to(constants[:,None,None],(15,SIZE,SIZE))
    return np.concatenate([rgb_crop.transpose(2,0,1).astype(np.float32)/255,
                           mask_crop[None].astype(np.float32),planes],axis=0).copy()

class PreAction(nn.Module):
    def __init__(self, pretrained=True):
        super().__init__()
        self.vision=nn.Sequential(nn.Conv2d(3,8,3,padding=1),nn.ReLU(),
                                  nn.Conv2d(8,8,3,padding=1),nn.ReLU())
        if pretrained:
            state=torch.load(ENCODER_WEIGHTS,map_location='cpu',weights_only=True)
            self.vision.load_state_dict({k:v for k,v in state.items() if k.split('.')[0] in ('0','2')})
        # Preserve the existing small visual encoder; train only the new head.
        for param in self.vision.parameters(): param.requires_grad_(False)
        self.head=nn.Sequential(nn.Conv2d(24,16,3,stride=2,padding=1),nn.ReLU(),
            nn.Conv2d(16,24,3,stride=2,padding=1),nn.ReLU(),
            nn.Conv2d(24,32,3,stride=2,padding=1),nn.ReLU(),
            nn.AvgPool2d(12),nn.Conv2d(32,16,1),nn.ReLU(),nn.Conv2d(16,4,1))
        nn.init.zeros_(self.head[-1].weight);nn.init.zeros_(self.head[-1].bias)

    def forward(self,x):
        features=self.vision(x[:,:3])
        return self.head(torch.cat([features,x[:,3:]],dim=1))

def future_box(box, spec, prediction):
    u=np.asarray(prediction).reshape(4)/PARAM_SCALE
    center=(box[:2]+box[2:])/2 + u[:2]*spec[2]
    size=(box[2:]-box[:2])*np.exp(np.clip(u[2:],-.7,.7))
    return np.r_[center-size/2,center+size/2]

def warp_tensor(current, center01, scaled_prediction):
    u=scaled_prediction.reshape(-1,4)/current.new_tensor(PARAM_SCALE)
    axis=(torch.arange(SIZE,device=current.device,dtype=current.dtype)+.5)/SIZE
    yy,xx=torch.meshgrid(axis,axis,indexing='ij')
    dest=torch.stack([xx,yy],dim=-1)[None]
    center=center01[:,None,None,:]
    source=(dest-center-u[:,:2,None,None].permute(0,2,3,1))/torch.exp(u[:,2:,None,None].permute(0,2,3,1).clamp(-.7,.7))+center
    return F.grid_sample(current,source*2-1,align_corners=False,padding_mode='zeros')

def render_forecast(rgb, mask, box, spec, prediction):
    center=((box[:2]+box[2:])/2-np.array(spec[:2]))/spec[2]
    current=np.concatenate([rgb.transpose(2,0,1).astype(np.float32)/255,mask[None]],axis=0)
    with torch.no_grad():
        rendered=warp_tensor(torch.from_numpy(current[None]),torch.tensor(center[None],dtype=torch.float32),
                             torch.tensor(np.asarray(prediction).reshape(1,4),dtype=torch.float32))[0].numpy()
    return np.uint8(np.clip(rendered[:3].transpose(1,2,0)*255,0,255)),rendered[3]

def box_iou(a,b):
    wh=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]))
    intersection=float(np.prod(wh))
    union=float(np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2])-intersection)
    return intersection/max(union,1e-9)
