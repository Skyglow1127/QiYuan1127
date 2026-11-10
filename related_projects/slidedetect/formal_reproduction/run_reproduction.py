"""Re-run the numerical suite; optionally include digital simulation and timing."""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rtl',action='store_true',help='Regenerate vectors and run XSim core + full input chain')
    p.add_argument('--timing',action='store_true',help='Run both 200 MHz OOC timing profiles')
    p.add_argument('--plots',action='store_true',help='Render numerical figures (Matplotlib required)')
    args=p.parse_args()
    def run(name,*extra):
        subprocess.run([sys.executable,str(ROOT/name),*extra],cwd=ROOT,check=True)
    run('complete_simulation.py')
    if args.rtl:
        run('reproduce.py');run('run_rtl.py');run('run_sensor.py')
    if args.timing:
        for hop in (0,24):run('run_sensor_timing.py','--hop',str(hop))
    if args.plots:run('plot_completion.py')
    print('Requested runs completed. Inspect results/reproduction_status.json after validate_reproduction.py; do not infer full-paper success from exit status.')


if __name__=='__main__':main()
