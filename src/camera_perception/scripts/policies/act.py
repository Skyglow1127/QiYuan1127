"""Small D1 ACT adapter reusing the pinned author's CVAE and Transformer.

Frozen ImageNet ResNet18 features are cached for CPU training. Both RGB cameras
are always used in fixed order. This is an ACT variant, not a paper reproduction.
"""
from pathlib import Path
import sys
import torch
from torch import nn
from torchvision.models import resnet18, ResNet18_Weights
from torchvision.models._utils import IntermediateLayerGetter

VENDOR=Path(__file__).parent/'vendor/act/detr'
if str(VENDOR) not in sys.path:sys.path.insert(0,str(VENDOR))
from models.detr_vae import DETRVAE
from models.transformer import Transformer, TransformerEncoder, TransformerEncoderLayer
from models.position_encoding import PositionEmbeddingSine
from models.backbone import FrozenBatchNorm2d


class RGBEncoder(nn.Module):
    def __init__(self, pretrained=False):
        super().__init__()
        net=resnet18(weights=ResNet18_Weights.IMAGENET1K_V1 if pretrained else None,norm_layer=FrozenBatchNorm2d)
        self.net=IntermediateLayerGetter(net,return_layers={'layer4':'features'})
        self.requires_grad_(False);self.eval()
        self.register_buffer('mean',torch.tensor([.485,.456,.406]).view(1,3,1,1))
        self.register_buffer('std',torch.tensor([.229,.224,.225]).view(1,3,1,1))

    def forward(self, rgb):
        # B,2,3,H,W RGB float32 in [0,1]; order is global,wrist.
        if rgb.ndim!=5 or rgb.shape[1:3]!=(2,3):raise ValueError('expected two RGB cameras')
        b,c=rgb.shape[:2];x=rgb.flatten(0,1)
        x=self.net((x-self.mean)/self.std)['features']
        return x.reshape(b,c,*x.shape[1:])


class CachedBackbone(nn.Module):
    num_channels=512
    def __init__(self,hidden):
        super().__init__();self.pos=PositionEmbeddingSine(hidden//2,normalize=True)
    def forward(self,feature):return [feature],[self.pos(feature).to(feature.dtype)]


def build_policy(config):
    dim=config.get('hidden_dim',128);heads=config.get('heads',4);layers=config.get('layers',2)
    transformer=Transformer(d_model=dim,nhead=heads,num_encoder_layers=layers,num_decoder_layers=layers,
        dim_feedforward=dim*4,dropout=.1,return_intermediate_dec=False)
    encoder=TransformerEncoder(TransformerEncoderLayer(dim,heads,dim*4,.1),layers)
    model=DETRVAE([CachedBackbone(dim)],transformer,encoder,7,config['chunk_size'],['global','wrist'])
    # Upstream uses 14 in three input projections even when state_dim differs.
    model.input_proj_robot_state=nn.Linear(7,dim)
    model.encoder_action_proj=nn.Linear(7,dim)
    model.encoder_joint_proj=nn.Linear(7,dim)
    return model


def loss_terms(model,q,features,actions,is_pad,beta=10.):
    pred,_,(mu,logvar)=model(q,features,None,actions,is_pad)
    mask=(~is_pad).unsqueeze(-1).expand_as(pred)
    l1=(pred-actions).abs()[mask].mean()
    kl=(-.5*(1+logvar-mu.square()-logvar.exp()).sum(-1)).mean()
    return l1+beta*kl,l1,kl
