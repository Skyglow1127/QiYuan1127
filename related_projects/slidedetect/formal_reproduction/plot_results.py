#!/usr/bin/env python3
"""Plot recorded simulation results, never substitute author measurements."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results',type=Path,default=Path(__file__).resolve().parent/'results')
    parser.add_argument('--qa-dir',type=Path,default=Path.home()/'.codex/skills/nature-figure/scripts')
    args=parser.parse_args()
    sys.path.insert(0,str(args.qa_dir))
    from audit_panel_alignment import require_matplotlib_panel_alignment
    out=args.results;report=json.loads((out/'summary.json').read_text())
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['DejaVu Sans','Arial','Helvetica'],'font.size':8,'axes.labelsize':8,
                         'axes.titlesize':9,'legend.fontsize':7,'svg.fonttype':'none',
                         'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False,
                         'axes.linewidth':.7,'legend.frameon':False})
    def export(fig,name):
        fig.canvas.draw()
        require_matplotlib_panel_alignment(fig,json_out=out/f'{name}.alignment.json',
                                           tolerance_pt=1.5,gutter_tolerance_pt=1.5,strict=True)
        fig.savefig(out/f'{name}.pdf')
        fig.savefig(out/f'{name}.svg')
        fig.savefig(out/f'{name}.png',dpi=300)
        plt.close(fig)
    # Deterministic series: no sampling uncertainty interval is applicable.
    fig,ax=plt.subplots(figsize=(180/25.4,105/25.4))
    fig.subplots_adjust(left=.12,right=.97,bottom=.17,top=.75)
    rows=report['dynamic'];x=np.array([r['m'] for r in rows])
    assert np.all(x > 0)
    ax.plot(x,[r['peak_mean_time_error_nT'] for r in rows],'o-',color='#0072B2',lw=1.2,ms=4,label='Model: window mean time')
    ax.plot(x,[r['peak_end_time_error_nT'] for r in rows],'s--',color='#D55E00',lw=1.2,ms=4,label='Model: newest event time')
    ax.scatter(x,[r['paper_plot_peak_error_nT'] for r in rows],facecolors='none',edgecolors='#333333',marker='D',s=28,label='Paper: printed error labels',zorder=5)
    ax.set_xscale('log',base=2);ax.set_xticks(x,[str(v) for v in x])
    ax.set(xlabel='Window length M (events)',ylabel='Peak absolute field error (nT)',ylim=(-.1,3.1))
    ax.grid(axis='y',alpha=.2)
    fig.suptitle('Dynamic-window comparison: 10 Hz, 2 nT modulation',y=.97,fontsize=10)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.9),ncol=2)
    fig.text(.12,.025,'Ideal-input model; 49,000 nT background; 0.5 s record; hop = M/4.',fontsize=7)
    export(fig,'dynamic_comparison')
    # Whole-record bootstrap retains the dependence of overlapping frames.
    fig,ax=plt.subplots(figsize=(180/25.4,105/25.4))
    fig.subplots_adjust(left=.12,right=.97,bottom=.19,top=.77)
    rows=report['static'];x=np.array([r['span_ms'] for r in rows])
    for value,ci,label,color,marker in (
        ('rms_across_record_std_nT','std_bootstrap95_nT','Across-record STD','#0072B2','o'),
        ('total_rmse_nT','rmse_bootstrap95_nT','Total RMSE','#D55E00','s')):
        y=np.array([r[value] for r in rows]);interval=np.array([r[ci] for r in rows])
        assert np.all(x > 0) and np.all(y > 0) and np.all(interval > 0)
        ax.plot(x,y,marker+'-',color=color,label=label,lw=1.2,ms=4)
        ax.fill_between(x,interval[:,0],interval[:,1],color=color,alpha=.15,linewidth=0)
    ax.set(xscale='log',yscale='log',xlabel='Observation span (ms)',ylabel='Field error (nT)')
    ax.grid(axis='y',alpha=.2)
    fig.suptitle('Static FID chain: results under declared analog assumptions',y=.97,fontsize=10)
    fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.9),ncol=2)
    fig.text(.12,.07,'32 independent records; bands: 95% whole-record bootstrap intervals.',fontsize=7)
    fig.text(.12,.025,'Initial SNR 35 dB; noise bandwidth 10 kHz; hysteresis thresholds ±0.05 A0.',fontsize=7)
    export(fig,'static_assumptions')


if __name__=='__main__':main()
