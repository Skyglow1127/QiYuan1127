"""Explicitly conditional simulations: input quality and output rounding."""
import json,sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'results/feasibility'
sys.path.insert(0,str(Path.home()/'.codex/skills/nature-figure/scripts'))
try:
    from audit_panel_alignment import require_matplotlib_panel_alignment
except ImportError:
    # The exported figures are both single-panel. Plotting stays portable;
    # multi-panel geometry requires the full optional audit helper.
    def require_matplotlib_panel_alignment(fig,json_out,strict=True):
        if len(fig.axes)!=1:raise RuntimeError('Install the panel alignment helper for multi-panel figures')
        Path(json_out).write_text(json.dumps({'status':'not_applicable','reason':'Single-panel figure; portable fallback'},indent=2)+'\n')
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans'],
 'font.size':8,'axes.labelsize':8,'legend.fontsize':7,'pdf.fonttype':42,'svg.fonttype':'none',
 'axes.spines.top':False,'axes.spines.right':False,'legend.frameon':False,'axes.linewidth':.7})


def load(name):return json.loads((OUT/name).read_text())


def export(fig,name):
    fig.canvas.draw()
    require_matplotlib_panel_alignment(fig,json_out=OUT/(name+'.alignment.json'),strict=True)
    fig.savefig(OUT/(name+'.pdf'))
    fig.savefig(OUT/(name+'.svg'))
    fig.savefig(OUT/(name+'.png'),dpi=300)
    plt.close(fig)


def conditional():
    fig,ax=plt.subplots(figsize=(180/25.4,115/25.4));fig.subplots_adjust(left=.13,right=.97,bottom=.28,top=.77)
    for case,color,marker in zip(load('frontend.json')['cases'],['#D55E00','#0072B2'],['s','o']):
        rows=case['rows'];x=np.array([r['span_ms'] for r in rows]);y=np.array([r['microhertz_output']['std_nT'] for r in rows])
        ci=np.array([r['microhertz_output']['std_ci95_nT'] for r in rows]);assert np.all(x>0) and np.all(y>0) and np.all(ci>0)
        ax.plot(x,y,color=color,marker=marker,ms=3.5,lw=1.1,label=f"Assumed frontend, {case['input_snr_db']} dB input")
        ax.fill_between(x,ci[:,0],ci[:,1],color=color,alpha=.16,linewidth=0)
    paper=json.loads((ROOT/'results/noise_audit/fig6_digitized.json').read_text())['rows']
    x=np.array([r['window_ms'] for r in paper]);y=np.array([r['STD']['nT'] for r in paper]);ci=np.array([r['STD']['pixel_interval_nT'] for r in paper])
    assert np.all(x>0) and np.all(y>0) and np.all(ci>0)
    ax.plot(x,y,color='#333333',ls='--',lw=1.1,label='Paper Fig. 6, digitized')
    ax.fill_between(x,ci[:,0],ci[:,1],color='#333333',alpha=.12,linewidth=0)
    ax.set(xscale='log',yscale='log',xlabel='Observation span (ms)',ylabel='Field STD (nT)');ax.grid(axis='y',alpha=.18)
    ax.set_xticks([15,30,60,100,200,480],['15','30','60','100','200','480'])
    fig.suptitle('Higher input SNR meets the precision target under an assumed frontend',y=.97,fontsize=9.5)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.92),ncol=2)
    fig.text(.13,.17,'48 independent FIDs; colored bands: pointwise 95% whole-record bootstrap intervals.',fontsize=7)
    fig.text(.13,.12,'100 Hz causal bandpass; threshold 0.0001 A0; 200 MHz clock; 1 microHz output rounding.',fontsize=7)
    fig.text(.13,.07,'Gray band: pixel-reading range. The 80 dB case changes the stated paper conditions.',fontsize=7)
    export(fig,'conditional_precision')


def rounding():
    rows=[r for r in load('bench_margin.json')['rows'] if r['jitter_ns']==.5 and r['window_s']==1]
    x=np.array([r['offset_uHz'] for r in rows]);assert np.all(np.diff(x)>0)
    fig,ax=plt.subplots(figsize=(180/25.4,115/25.4));fig.subplots_adjust(left=.13,right=.97,bottom=.28,top=.77)
    for unit,color,marker,line,label in [(1e-6,'#D55E00','s','-','1 microHz output'),
                                       (1e-9,'#0072B2','o','-','1 nanoHz output'),
                                       (None,'#333333','x',':','Floating reference')]:
        outputs=[next(o for o in r['outputs'] if o['quantum_hz']==unit) for r in rows]
        y=np.array([o['std_pT'] for o in outputs])*1000;ci=np.array([o['std_bootstrap_ci95_pT'] for o in outputs])*1000
        assert np.all(ci[:,0]<=y) and np.all(ci[:,1]>=y)
        ax.errorbar(x,y,yerr=np.stack([y-ci[:,0],ci[:,1]-y]),color=color,marker=marker,ls=line,lw=1.1,ms=4,capsize=2,label=label)
    ax.axhline(8,color='#777777',ls='--',lw=.8,label='Paper 1 s target: 8 fT')
    ax.set(xlabel='Carrier offset within a 1 microHz output interval (microHz)',ylabel='Field STD (fT)',ylim=(0,13),xticks=[0,.25,.5,.75])
    ax.grid(axis='y',alpha=.18)
    fig.suptitle('Coarse output rounding can hide or inflate the same underlying noise',y=.97,fontsize=9.5)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.92),ncol=2)
    fig.text(.13,.17,'4096 independent records per point; error bars: pointwise 95% bootstrap intervals.',fontsize=7)
    fig.text(.13,.12,'Constant tone; 1 s window, 0.5 ns iid edge jitter, random clock phase, 200 MHz quantization.',fontsize=7)
    fig.text(.13,.07,'The nanoHz and floating traces overlap. This timing model is not a physical bench test.',fontsize=7)
    export(fig,'output_rounding')


if __name__=='__main__':conditional();rounding()
