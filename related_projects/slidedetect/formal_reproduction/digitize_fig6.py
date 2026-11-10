"""Read selected Fig.6 curve values from the supplied PDF's embedded raster.

Input image is a local inspection intermediate and is not redistributed.
Pixel bands describe reading uncertainty, not statistical confidence intervals.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from PIL import Image

def main():
    p=argparse.ArgumentParser();p.add_argument('image',type=Path);a=p.parse_args()
    im=np.asarray(Image.open(a.image)).astype(int)
    assert im.shape[:2]==(715,928)
    r,g,b=im[:,:,0],im[:,:,1],im[:,:,2]
    blue=(b-r>35)&(g-r>15)&(b-g>15)&(r<170)
    orange=(r-g>20)&(g-b>20)&(r>180)&(b<210)
    orange[:100,750:]=False # Exclude the legend, not the plotted observations.
    rows=[]
    for ms in [15,30,45.53901233508,60,99.22711108802,100,200,400,480]:
        x=546+np.log10(ms/100)/np.log10(2)*(687.5-546);vals={}
        for label,mask in [('STD',blue),('RMSE',orange)]:
            low=90 if label=='STD' else 10
            yy,_=np.where(mask[low:615,int(x)-5:int(x)+6]);yy=yy+low
            if len(yy):
                y=float(np.median(yy));v=lambda py:float(10**(-1-(py-83)/((576-83)/3)))
                vals[label]=dict(nT=v(y),pixel_interval_nT=[v(y+6),v(y-6)],x_px=float(x),y_px=y)
        rows.append(dict(window_ms=ms,**vals))
    result=dict(source='Provided PDF page 7 Fig.6 embedded JPEG object 512; digitized values, not author raw data',
        image_sha256=hashlib.sha256(a.image.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        calibration=dict(x_100ms_px=546,x_200ms_px=687.5,y_0_1nT_px=83,y_0_0001nT_px=576,reading_uncertainty_px=6),rows=rows)
    out=Path(__file__).resolve().parent/'results/noise_audit/fig6_digitized.json'
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(rows,indent=2))

if __name__=='__main__':main()
