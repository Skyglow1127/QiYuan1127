"""Two figures from independently repeated, explicitly assumed noise models."""
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/noise_audit'
sys.path.insert(0,str(Path.home()/'.codex/skills/nature-figure/scripts'))
from audit_panel_alignment import require_matplotlib_panel_alignment
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans'],
 'font.size':8,'axes.labelsize':8,'axes.titlesize':9,'legend.fontsize':7,
 'pdf.fonttype':42,'svg.fonttype':'none','axes.spines.top':False,
 'axes.spines.right':False,'legend.frameon':False,'axes.linewidth':.7})

def load(name):return json.loads((OUT/name).read_text())

def export(fig,name):
    fig.canvas.draw()
    require_matplotlib_panel_alignment(fig,json_out=OUT/(name+'.alignment.json'),strict=True)
    fig.savefig(OUT/(name+'.pdf'))
    fig.savefig(OUT/(name+'.svg'))
    fig.savefig(OUT/(name+'.png'),dpi=300)
    plt.close(fig)

def plateau():
    data=load('plateau.json')['rows']
    fig,ax=plt.subplots(figsize=(180/25.4,115/25.4))
    fig.subplots_adjust(left=.13,right=.97,bottom=.26,top=.77)
    for noise,color,mark,label in [(2,'#009E73','s','2 ns jitter'),(20,'#0072B2','o','20 ns jitter'),(940.0609,'#D55E00','^','940 ns jitter')]:
        rows=[r for r in data if r['jitter_ns']==noise]
        x=np.array([r['dwell_s'] for r in rows]);y=np.array([r['record_failures']/r['records']*100 for r in rows])
        ci=np.array([r['record_error_ci95'] for r in rows])*100
        assert np.all(x>0) and np.all(ci[:,1]>=ci[:,0])
        ax.plot(x,y,color=color,marker=mark,lw=1.2,ms=3.5,label=label)
        ax.fill_between(x,ci[:,0],ci[:,1],color=color,alpha=.12,linewidth=0)
    ax.axhline(5,color='#555555',ls='--',lw=.8,label='Proposed 5% upper-bound target')
    ax.set(xscale='log',ylim=(-2,100),xlabel='Time held at each frequency (s)',ylabel='Records with at least one wrong plateau (%)')
    ax.set_xticks([.1,.5,1,2,5,10],['0.1','0.5','1','2','5','10'])
    ax.grid(axis='y',alpha=.18)
    fig.suptitle('Longer plateaus can resolve a 10 microHz step under stationary jitter',y=.97,fontsize=10)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.92),ncol=2)
    fig.text(.13,.15,'320 independent two-level records; shading: Wilson 95% intervals across records.',fontsize=7)
    fig.text(.13,.10,'213-event SFLS window (~100 ms), 21-event hop, 200 MHz quantization; plateau means.',fontsize=7)
    fig.text(.13,.05,'Constant tone with stationary edge jitter. No FID decay or clock drift; not raw-output precision.',fontsize=7)
    export(fig,'plateau_discrimination')

def frontend():
    data=load('causal_frontend.json')['cases'];paper=load('fig6_digitized.json')['rows']
    fig,ax=plt.subplots(figsize=(180/25.4,115/25.4))
    fig.subplots_adjust(left=.13,right=.97,bottom=.26,top=.77)
    for name,color,mark,label in [
        ('bypass_h0002','#D55E00','o','35 dB, no bandpass'),
        ('bp100_h0002','#0072B2','s','35 dB before 100 Hz bandpass'),
        ('bp100_post35_h0002','#CC79A7','^','35 dB after 100 Hz bandpass')]:
        c=next(c for c in data if c['name']==name);rows=c['statistics']
        x=np.array([r['span_ms'] for r in rows]);y=np.array([r['across_record_std_nT'] for r in rows])
        ci=np.array([r['std_ci95_nT'] for r in rows])
        assert np.all(x>0) and np.all(y>0) and np.all(ci>0)
        ax.plot(x,y,color=color,marker=mark,lw=1.1,ms=3.5,label=label)
        ax.fill_between(x,ci[:,0],ci[:,1],color=color,alpha=.12,linewidth=0)
    p=[r for r in paper if 'STD' in r];x=np.array([r['window_ms'] for r in p]);y=np.array([r['STD']['nT'] for r in p]);ci=np.array([r['STD']['pixel_interval_nT'] for r in p])
    ax.plot(x,y,color='#333333',ls='--',marker='.',ms=3,label='Paper Fig. 6, digitized')
    ax.fill_between(x,ci[:,0],ci[:,1],color='#333333',alpha=.10,linewidth=0)
    ax.set(xscale='log',yscale='log',xlabel='Observation span (ms)',ylabel='Across-record field STD (nT)')
    ax.grid(axis='y',alpha=.18)
    fig.suptitle('Causal bandpass filtering helps, but a large precision gap remains',y=.97,fontsize=10)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.92),ncol=2)
    fig.text(.13,.15,'24 independent FID records; colored bands: 95% whole-record bootstrap intervals.',fontsize=7)
    fig.text(.13,.10,'Threshold 0.002 A0; 10 kHz input noise; evaluation 0.2-1.5 s. All are stated assumptions.',fontsize=7)
    fig.text(.13,.05,'Gray band: pixel-reading range, not a confidence interval. Original frontend settings unknown.',fontsize=7)
    export(fig,'causal_frontend_gap')

if __name__=='__main__':plateau();frontend()
