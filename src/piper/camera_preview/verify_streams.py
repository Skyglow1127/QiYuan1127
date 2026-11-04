#!/usr/bin/env python3
"""Read-only two-camera soak check using the ACT HTTP camera reader. No CAN TX."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'official_adapter'))
from lerobot_robot_piper_shared.devices import PiperHttpCamera, PiperHttpCameraConfig


def statuses():
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    result={}
    for role,port in [('global',18761),('wrist',18760)]:
        with opener.open(f'http://127.0.0.1:{port}/status',timeout=2) as response:
            result[role]=json.load(response)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=60.)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 1 <= args.seconds <= 600:
        parser.error('seconds must be in [1,600]')
    cameras={
        'global':PiperHttpCamera(PiperHttpCameraConfig()),
        'wrist':PiperHttpCamera(PiperHttpCameraConfig(role='wrist',url='http://127.0.0.1:18760',serial='CC1365301EY'))}
    report={'robot_commands_sent':0,'passed':False,'pairs':0,'max_pair_skew_ms':0.,
        'max_age_ms':0.,'max_acquisition_ms':0.,'duration_s':0.}
    started=time.monotonic()
    try:
        report['before']=statuses()
        for camera in cameras.values():camera.connect()
        started=time.monotonic();last_print=started
        with ThreadPoolExecutor(max_workers=2) as pool:
            while time.monotonic()-started < args.seconds:
                cycle=time.monotonic()
                futures=[pool.submit(c.async_read) for c in cameras.values()]
                for future in futures:future.result()
                for attempt in range(3):
                    times={role:c.last_metadata['estimated_capture_monotonic'] for role,c in cameras.items()}
                    ages={role:(time.monotonic()-captured)*1000 for role,captured in times.items()}
                    skew=(max(times.values())-min(times.values()))*1000
                    if skew <= 60 and all(0<=a<=150 for a in ages.values()):break
                    if attempt==2:raise TimeoutError(f'Camera pair age/skew exceeded: {ages}, {skew}')
                    cameras[min(times,key=times.get)].async_read()
                report['pairs']+=1
                report['max_pair_skew_ms']=max(report['max_pair_skew_ms'],skew)
                report['max_age_ms']=max(report['max_age_ms'],*ages.values())
                report['max_acquisition_ms']=max(report['max_acquisition_ms'],(time.monotonic()-cycle)*1000)
                if time.monotonic()-last_print>=10:
                    print(json.dumps({'elapsed_s':round(time.monotonic()-started,1),'pairs':report['pairs']},ensure_ascii=False),flush=True)
                    last_print=time.monotonic()
                time.sleep(max(0.,.05-(time.monotonic()-cycle)))
        report['after']=statuses()
        for role in cameras:
            if report['after'][role]['generation']!=report['before'][role]['generation']:
                raise RuntimeError(role+' capture generation changed during soak check')
        report['passed']=True
    except Exception as error:
        report['error']={'type':type(error).__name__,'message':str(error)}
    finally:
        report['duration_s']=time.monotonic()-started
        for camera in cameras.values():camera.disconnect()
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['before','after']},ensure_ascii=False),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
