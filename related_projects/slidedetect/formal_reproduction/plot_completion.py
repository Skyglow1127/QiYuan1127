"""Original figures from the recorded simulation suite; no author data replacement."""
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from matplotlib.lines import Line2D
import numpy as np
from scipy.stats import chi2

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/simulation_completion'
sys.path.insert(0,str(Path.home()/'.codex/skills/nature-figure/scripts'))
from audit_panel_alignment import require_matplotlib_panel_alignment

plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans','Arial','Helvetica'],
                     'font.size':8,'axes.labelsize':8,'axes.titlesize':9,'legend.fontsize':7,
                     'pdf.fonttype':42,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,
                     'legend.frameon':False,'axes.linewidth':.7})


def load(name):return json.loads((OUT/name).read_text())


def export(fig,name,**kwargs):
    fig.canvas.draw()
    require_matplotlib_panel_alignment(fig,json_out=OUT/f'{name}.alignment.json',strict=True,**kwargs)
    fig.savefig(OUT/f'{name}.pdf');fig.savefig(OUT/f'{name}.svg');fig.savefig(OUT/f'{name}.png',dpi=300)
    plt.close(fig)


def theory():
    data=load('theory.json');rows=data['rows'];n=data['trials'];x=np.array([r['span_s']*1000 for r in rows])
    fig,ax=plt.subplots(figsize=(180/25.4,105/25.4));fig.subplots_adjust(left=.13,right=.97,bottom=.19,top=.77)
    for name,color,marker,label in [('sfls','#0072B2','o','SFLS'),('ep','#D55E00','s','EP')]:
        y=np.array([r[f'{name}_std_hz'] for r in rows]);pred=np.array([r[f'theory_{name}_std_hz'] for r in rows])
        lo,hi=np.sqrt((n-1)*y[None,:]**2/chi2.ppf([.9995,.0005],n-1)[:,None])
        assert np.all(x>0) and np.all(y>0)
        ax.errorbar(x,y,yerr=[y-lo,hi-y],fmt=marker,color=color,ms=3.5,capsize=2,label=f'{label}: simulation')
        ax.plot(x,pred,'--',color=color,lw=1,label=f'{label}: theory')
    ax.set(xscale='log',yscale='log',xlabel='Observation span (ms)',ylabel='Frequency STD (Hz)')
    ax.grid(axis='y',alpha=.2)
    fig.suptitle('Independent timing-noise check and equal-window EP comparison',y=.97,fontsize=10)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.9),ncol=2)
    fig.text(.13,.07,'2,048 independent trials per window; bars: 99.9% STD intervals.',fontsize=7)
    fig.text(.13,.025,f'20 ns independent timing noise; no quantization. SFLS time exponent: {data["slope"]:.3f}.',fontsize=7)
    export(fig,'theory_comparison')


def sensitivity():
    data=load('frontend.json');raw=load('frontend_raw.json')
    cases=[c for c in data['cases'] if c['bandwidth_hz']==10000]
    fig,axes=plt.subplots(1,2,figsize=(180/25.4,110/25.4));fig.subplots_adjust(left=.10,right=.97,bottom=.23,top=.76,wspace=.4)
    styles={0:('-', 'o'),.002:('--','s'),.02:('-.','^'),.05:(':','D')}
    derived=[]
    for c in cases:
        color='#0072B2' if c['snr_db']==35 else '#D55E00';style,marker=styles[c['threshold_A0']]
        x=np.array([r['span_ms'] for r in c['rows']])
        for ax,stat in zip(axes,['aligned_record_std_nT','rmse_nT']):
            y=np.array([r['sfls'][stat] for r in c['rows']]);interval=[]
            for row in c['rows']:
                if stat=='aligned_record_std_nT':interval.append(row['sfls']['std_ci95_nT'])
                else:
                    e=np.array(raw[c['label']][str(row['m'])]['sfls']);rng=np.random.default_rng(20261113+row['m'])
                    b=e[rng.integers(0,len(e),(300,len(e)))]
                    ci=np.quantile(np.sqrt(np.mean(b*b,axis=(1,2))),[.025,.975]);interval.append(ci)
                    derived.append(dict(case=c['label'],m=row['m'],rmse_ci95_nT=ci.tolist()))
            interval=np.array(interval)
            assert np.all(x>0) and np.all(y>0) and np.all(interval>0)
            ax.plot(x,y,color=color,ls=style,marker=marker,ms=2.2,lw=.9)
            ax.fill_between(x,interval[:,0],interval[:,1],color=color,alpha=.055,linewidth=0)
    for i,ax in enumerate(axes):
        ax.set(xscale='log',yscale='log',xlabel='Observation span (ms)',ylabel='Field error (nT)')
        ax.set_title(['a  Across-record STD','b  Total RMSE'][i],loc='left')
        ax.grid(axis='y',alpha=.2)
    handles=[Line2D([],[],color=c,label=l) for c,l in [('#0072B2','35 dB (paper value)'),('#D55E00','70 dB (sensitivity only)')]]
    handles += [Line2D([],[],color='#555555',ls=s,marker=m,ms=3,label=f'Threshold {h:g} A0') for h,(s,m) in styles.items()]
    fig.suptitle('Noise and comparator assumptions control different error components',y=.98,fontsize=10)
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.93),ncol=3)
    fig.text(.10,.09,'16 independent FID records; bands: 95% whole-record bootstrap intervals.',fontsize=7)
    fig.text(.10,.045,'All eight 10 kHz bandwidth cases shown. 70 dB is not a reported paper condition.',fontsize=7)
    (OUT/'plot_derived_intervals.json').write_text(json.dumps(derived,indent=2)+'\n')
    export(fig,'frontend_sensitivity')


def drift_map():
    rows=load('drift.json')['noisy_maps']
    fig,axes=plt.subplots(1,2,figsize=(180/25.4,105/25.4));fig.subplots_adjust(left=.08,right=.83,bottom=.24,top=.80,wspace=.27)
    values=np.concatenate([r[key] for r in rows for key in ('center_rmse_nT','latest_rmse_nT')]);assert np.all(values>0)
    norm=LogNorm(vmin=float(values.min()),vmax=float(values.max()))
    for ax,key,title in zip(axes,['center_rmse_nT','latest_rmse_nT'],['a  Window-mean reference','b  Latest-event reference']):
        for i,r in enumerate(rows):
            t=np.array(r['times_s']);dx=r['m']/2/(49000*267500000/(2*np.pi)*1e-9)
            edges=np.r_[t-dx/2,t[-1]+dx/2]
            mesh=ax.pcolormesh(edges,[i-.5,i+.5],np.array(r[key])[None,:],cmap='cividis',norm=norm,shading='flat')
        ax.set(xlim=(0,1),ylim=(-.5,len(rows)-.5),xlabel='Window mean time (s)')
        ax.set_yticks(np.arange(len(rows)),[str(r['m']) for r in rows]);ax.set_title(title,loc='left')
    axes[0].set_ylabel('Window length M (events)')
    cax=fig.add_axes([.87,.24,.025,.56]);cb=fig.colorbar(mesh,cax=cax,label='RMSE (nT)')
    ticks=[v for v in (.1,.2,.5,1,2,4,10) if norm.vmin<=v<=norm.vmax]
    cb.set_ticks(ticks);cb.set_ticklabels([str(v) for v in ticks]);cb.ax.minorticks_off()
    fig.suptitle('Linear-drift diagnostic: reference time changes the apparent error',y=.96,fontsize=10)
    fig.text(.08,.095,'16 independent records; 2 nT/s drift; 35 dB; threshold 0.05 A0; 10 kHz bandwidth.',fontsize=7)
    fig.text(.08,.045,'One cell per evaluated frame; shared log color scale. Not a numerical match to Fig. 8.',fontsize=7)
    export(fig,'drift_rmse_map',exclude_axes=[cax])


def step_trace():
    rows=load('step.json')['trace'];t=np.array([r['time_s'] for r in rows])
    fig,ax=plt.subplots(figsize=(180/25.4,105/25.4));fig.subplots_adjust(left=.12,right=.97,bottom=.23,top=.78)
    ax.step(t,[r['truth_minus_base_uHz'] for r in rows],where='post',color='#333333',lw=1.4,label='Input frequency change')
    ax.plot(t,[r['estimate_minus_base_uHz'] for r in rows],color='#0072B2',lw=.55,label='SFLS output (correlated)')
    ax.set(xlabel='Time (s)',ylabel='Frequency change (microHz)',xlim=(0,8))
    ax.grid(axis='y',alpha=.2)
    fig.suptitle('Conditional step tracking: ideal signal with 200 MHz quantization',y=.96,fontsize=10)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.89),ncol=2)
    fig.text(.12,.10,'2,128.87 Hz carrier; 10 microHz steps; 213 events (~99.58 ms); 100 Hz output.',fontsize=7)
    fig.text(.12,.05,'No analog or clock noise in this trace. Independent noisy discrimination is reported separately.',fontsize=7)
    export(fig,'step_trace')


if __name__=='__main__':
    theory();sensitivity();drift_map();step_trace()
