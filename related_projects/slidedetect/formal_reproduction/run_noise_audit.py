"""Run noise diagnostics; optional noisy RTL replay and recorded-data figures."""
import argparse,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('--rtl',action='store_true');p.add_argument('--plots',action='store_true');a=p.parse_args()
    def run(name,*extra):subprocess.run([sys.executable,str(ROOT/name),*extra],cwd=ROOT,check=True)
    run('audit_noise.py')
    if a.rtl:
        run('prepare_noisy_replay.py');run('run_rtl.py','--results',str(ROOT/'results/noise_audit/noisy_replay'))
    if a.plots:run('plot_noise_audit.py')
    run('validate_noise_audit.py')
    print('Numerical checks finished. Read results/noise_audit/status.json: a passed diagnostic does not certify full-paper reproduction.')

if __name__=='__main__':main()
