#!/usr/bin/env python3
"""Freeze OpenCV reference separately from the HLS compiler's C++ runtime."""
import json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from letterbox import geometry,cpu
def generate(folder):
    folder.mkdir(parents=True,exist_ok=True);cases=[];rng=np.random.default_rng(20261027)
    def add(image,ow,oh,scaleup,pad):
        g=geometry(image.shape[1],image.shape[0],ow,oh,scaleup,pad);ref,_=cpu(image,g);name='case_%02d'%len(cases)
        image.tofile(folder/(name+'.input.rgb'));ref.tofile(folder/(name+'.expected.rgb'))
        cases.append([name,g.iw,g.ih,g.rw,g.rh,g.width,g.height,g.pad])
    for iw,ih in ((640,480),(641,479),(33,25),(1920,1080)):
        image=rng.integers(0,256,(ih,iw,3),dtype=np.uint8);add(image,640,640,True,114)
        if iw<=896:add(image,224,224,True,114);add(image,321,320,False,17)
    for role in ('global','wrist'):
        image=cv2.cvtColor(cv2.imread('/home/aup/Camera/output/preaction_clear_pose_20261025T211706/'+role+'.jpg'),cv2.COLOR_BGR2RGB)
        add(image,640,640,True,114);add(image,320,320,True,114)
    with (folder/'cases.txt').open('w') as f:
        for row in cases:f.write(' '.join(map(str,row))+'\n')
    (folder/'manifest.json').write_text(json.dumps({'opencv':cv2.__version__,'contract':'INTER_LINEAR + centered constant border, Python round geometry','cases':cases},indent=2)+'\n')
if __name__=='__main__':generate(Path(sys.argv[1]))
