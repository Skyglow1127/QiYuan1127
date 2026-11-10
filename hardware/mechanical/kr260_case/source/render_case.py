from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.font_manager import FontProperties
import cadquery as cq
from build_case import P,reference_board
OUT=Path(__file__).resolve().parents[1]
FONT=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family']=['DejaVu Sans']
BASE='#3d6670';GUARD='#dd9551';INK='#182e36';BG='#f2f6f5'

def tris(s):
 v,f=s.val().tessellate(0.12,0.15)
 v=np.array([[a.x,a.y,a.z] for a in v]);return v[np.array(f)]

def add(ax,s,color,shift=0,alpha=1):
 t=tris(s).copy();t[:,:,2]+=shift
 norm=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);norm/=np.maximum(np.linalg.norm(norm,axis=1,keepdims=True),1e-9)
 light=np.array([-.3,-.5,.8]);light/=np.linalg.norm(light)
 shade=.64+.36*np.clip(norm@light,0,1)
 rgb=np.array(matplotlib.colors.to_rgb(color));cols=np.column_stack([np.clip(shade[:,None]*rgb,0,1),np.ones(len(t))*alpha])
 ax._meshes.append((t,cols))

def axes(fig,pos,zmax=65,elev=28,azim=-60):
 ax=fig.add_axes(pos,projection='3d',computed_zorder=True);ax._meshes=[]
 ax.view_init(elev,azim);ax._view=(elev,azim);ax.set_proj_type('ortho');ax.set_xlim(-91,91);ax.set_ylim(-85,85);ax.set_zlim(0,zmax)
 ax.set_box_aspect([182,170,zmax]);ax.set_axis_off();ax.set_facecolor(BG);return ax

def finish(ax):
 # Software depth buffer keeps large coplanar CAD faces from sorting through one another.
 tris=np.concatenate([a for a,b in ax._meshes]);cols=np.concatenate([b for a,b in ax._meshes])
 el,az=np.radians(ax._view);view=np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
 right=np.array([-np.sin(az),np.cos(az),0]);up=np.cross(view,right)
 pts=np.stack([tris@right,tris@up,tris@view],axis=-1)
 pos=ax.get_position();fig=ax.figure;ww=int(fig.get_figwidth()*fig.dpi*pos.width);hh=int(fig.get_figheight()*fig.dpi*pos.height)
 lo=pts[:,:,:2].min((0,1));hi=pts[:,:,:2].max((0,1));scale=min(ww/(hi[0]-lo[0]),hh/(hi[1]-lo[1]))*.90
 mid=(lo+hi)/2;pts[:,:,0]=(pts[:,:,0]-mid[0])*scale+ww/2;pts[:,:,1]=hh/2-(pts[:,:,1]-mid[1])*scale
 rgb=np.empty((hh,ww,3),dtype=np.uint8);rgb[:]=np.array(matplotlib.colors.to_rgb(BG))*255
 depth=np.full((hh,ww),-np.inf)
 for t,c in zip(pts,cols):
  x0=max(0,int(np.floor(t[:,0].min())));x1=min(ww-1,int(np.ceil(t[:,0].max())))
  y0=max(0,int(np.floor(t[:,1].min())));y1=min(hh-1,int(np.ceil(t[:,1].max())))
  if x1<x0 or y1<y0:continue
  x,y=np.meshgrid(np.arange(x0,x1+1)+.5,np.arange(y0,y1+1)+.5)
  den=(t[1,1]-t[2,1])*(t[0,0]-t[2,0])+(t[2,0]-t[1,0])*(t[0,1]-t[2,1])
  if abs(den)<1e-9:continue
  u=((t[1,1]-t[2,1])*(x-t[2,0])+(t[2,0]-t[1,0])*(y-t[2,1]))/den
  v=((t[2,1]-t[0,1])*(x-t[2,0])+(t[0,0]-t[2,0])*(y-t[2,1]))/den
  w=1-u-v;z=u*t[0,2]+v*t[1,2]+w*t[2,2]
  d=depth[y0:y1+1,x0:x1+1];mask=(u>=-1e-6)&(v>=-1e-6)&(w>=-1e-6)&(z>d)
  d[mask]=z[mask];rgb[y0:y1+1,x0:x1+1][mask]=(c[:3]*255).astype(np.uint8)
 ax.set_visible(False);flat=fig.add_axes(pos);flat.imshow(rgb);flat.axis('off')

def txt(fig,x,y,s,size=12,color=INK,weight=None):
 fig.text(x,y,s,fontproperties=FONT,fontsize=size,color=color,weight=weight)

base=cq.importers.importStep(str(OUT/'step'/'01_base.step'))
hood=cq.importers.importStep(str(OUT/'step'/'02_hood_inverted.step'))
cap=cq.importers.importStep(str(OUT/'step'/'03_service_hatch_inverted.step'))
fig=plt.figure(figsize=(14,8),dpi=180,facecolor=BG)
txt(fig,.045,.935,'KR260  ·  V2 防护盒',27)
txt(fig,.047,.885,'完整侧壁与底板  /  窄缝通风顶盖  /  开发板直接放入',12)
ax=axes(fig,[.02,.25,.60,.62],65,31,-56)
add(ax,base,BASE);add(ax,hood,BASE);add(ax,cap,GUARD);finish(ax)
ax2=axes(fig,[.61,.32,.37,.49],63,29,127)
add(ax2,base,BASE);add(ax2,hood,BASE);add(ax2,cap,GUARD);finish(ax2)
txt(fig,.145,.26,'正面：网口 / USB / DP / 电源',12)
txt(fig,.665,.26,'背面：SFP+ / Pmod / 调试口',12)
for x,t,b in [(.05,'01  大幅减少裸露','侧壁和底板包围板卡，顶盖覆盖元件'),(.37,'02  保留散热空间','3.2 mm 通风缝，原装风扇上方留余量'),(.69,'03  无需板壳螺丝','盒体卡合，小盖可取下接扩展接口')]:
 txt(fig,x,.17,t,14);txt(fig,x,.118,b,10)
txt(fig,.045,.045,'实际 CAD 渲染  |  164 × 151 × 56.4 mm  |  PETG  |  待实板试装、卡扣及温升验证',10,color='#587078')
fig.savefig(OUT/'images'/'case_preview_v2.png');plt.close(fig)

fig=plt.figure(figsize=(11,9),dpi=170,facecolor=BG)
txt(fig,.04,.94,'三件式结构',23)
ax=axes(fig,[.01,.08,.73,.82],155,25,-57)
add(ax,base,BASE)
colors={'foot':'#444b4c','pcb':'#447d53','som':'#4f8f64','sink':'#41484a','fan':'#30383b','fan_hub':'#657577','ethernet':'#abb9c0','usb':'#b6c0c8','dp':'#b6c0c8','dc':'#373f43','sfp':'#b3bec2','micro_usb':'#b6c0c8','hat':'#30383b','slvs':'#444c50','pmod':'#30383b','sd':'#bcc9c9'}
for name,s in reference_board(P):add(ax,s,colors[name],shift=17)
add(ax,hood,BASE,shift=66);add(ax,cap,GUARD,shift=91);finish(ax)
txt(fig,.74,.80,'扩展接口小盖',14);txt(fig,.74,.755,'平时盖住，需要时取下',10)
txt(fig,.74,.59,'通风罩壳',14);txt(fig,.74,.545,'侧壁卡扣与底座连接',10)
txt(fig,.74,.35,'整板直接放入',14);txt(fig,.74,.305,'保留原装散热器与脚垫',10)
txt(fig,.74,.16,'实心底座',14)
txt(fig,.04,.035,'板卡为简化占位示意；外壳为实际模型。示意不代表实板及所有插头已验证。',10,color='#587078')
fig.savefig(OUT/'images'/'assembly_exploded_v2.png');plt.close(fig)

# Flat print orientations for user and selective-support annotation.
fig=plt.figure(figsize=(12,6),dpi=170,facecolor=BG)
txt(fig,.04,.93,'打印方向',23)
ax=axes(fig,[.0,.21,.38,.63],30,48,-55);add(ax,base,BASE);finish(ax)
hprint=hood.rotate((0,0,0),(1,0,0),180).translate((0,0,54))
ax2=axes(fig,[.34,.21,.43,.63],60,38,-54);add(ax2,hprint,BASE);finish(ax2)
cprint=cap.rotate((0,0,0),(1,0,0),180).translate((48,-3,56.4))
ax3=axes(fig,[.77,.27,.22,.52],20,50,-60);add(ax3,cprint,GUARD);finish(ax3)
txt(fig,.09,.19,'底座：平底贴热床',12);txt(fig,.405,.19,'罩壳：顶面贴热床，开口朝上',12);txt(fig,.785,.19,'小盖：外表面朝下',12)
txt(fig,.04,.065,'罩壳的四个卡扣倒钩需局部支撑；不要把整片通风格栅填满支撑。STL 已按此方向摆放。',10,color='#587078')
fig.savefig(OUT/'images'/'print_orientation_v2.png');plt.close(fig)
print('V2 previews rendered')
