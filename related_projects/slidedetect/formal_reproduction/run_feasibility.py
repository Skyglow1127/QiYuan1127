"""Repeat conditional feasibility, with optional plotting and FPGA tools."""
import argparse,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plots',action='store_true');p.add_argument('--rtl',action='store_true');p.add_argument('--timing',action='store_true');args=p.parse_args()
    scripts=['assess_feasibility.py','refine_bench.py']
    if args.rtl:scripts+=['run_precision_rtl.py','run_precision_sensor.py']
    if args.timing:scripts+=['run_nhz_timing.py','run_precision_sensor_timing.py']
    if args.plots:scripts+=['plot_feasibility.py']
    scripts+=['validate_feasibility.py','validate_reproduction.py']
    for script in scripts:subprocess.run([sys.executable,str(ROOT/script)],cwd=ROOT,check=True)


if __name__=='__main__':main()
