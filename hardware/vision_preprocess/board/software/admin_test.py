#!/usr/bin/env python3
"""Board-admin entry point: verify/build, temporary PL load, real tests, restore.

Run interactively with sudo on the KR260. Never handles passwords. Default
restores the exact previous supported app after tests, including test failures.
"""
import argparse,hashlib,json,os,platform,shutil,signal,subprocess,sys,time
from pathlib import Path
from xmutil_table import parse_listapps

APP='kr260-vision'
HERE=Path(__file__).resolve().parents[1]

def command(args,timeout=120):
    print('+ '+' '.join(map(str,args)),flush=True)
    r=subprocess.run(list(map(str,args)),capture_output=True,text=True,timeout=timeout)
    print(r.stdout,end='',flush=True)
    if r.stderr:print(r.stderr,file=sys.stderr,end='',flush=True)
    if r.returncode:raise RuntimeError(f'command failed rc={r.returncode}: {args[0]}')
    return r.stdout

def active_app():
    rows=parse_listapps(command(['xmutil','listapps'],20))
    active=[r for r in rows if r['active_slots']]
    if len(active)!=1 or active[0]['active_slots']!=[0]:raise RuntimeError('Expected exactly one slot-0 app')
    return active[0]['Accelerator']

def verify_package():
    manifest=json.loads((HERE/'package_manifest.json').read_text())
    if manifest['part']!='xck26-sfvc784-2LV-c' or manifest['clock_hz']!=25000000:raise RuntimeError('Unexpected design')
    for name,digest in manifest['files'].items():
        p=HERE/name
        if p.is_symlink() or not p.is_file() or HERE not in p.resolve().parents:raise RuntimeError('Unsafe package file '+name)
        if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise RuntimeError('Hash mismatch '+name)
    return manifest

def stop_pressure_readers(record):
    # Only the existing, exact stdin pressure reader invocation; no camera/robot process.
    stopped=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdecimal():continue
        try:argv=(p/'cmdline').read_bytes().rstrip(b'\0').split(b'\0')
        except OSError:continue
        if len(argv)>6 and argv[:4]==[b'python3',b'-u',b'-',b'--source'] and argv[4]==b'real' and b'--verify-pl' in argv:
            pid=int(p.name);os.kill(pid,signal.SIGTERM);stopped.append(pid)
    until=time.monotonic()+5
    while any(Path('/proc',str(p)).exists() for p in stopped) and time.monotonic()<until:time.sleep(.1)
    if any(Path('/proc',str(p)).exists() for p in stopped):raise RuntimeError('Pressure reader did not exit; PL not changed')
    record['pressure_reader_pids_stopped']=stopped

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-deps',action='store_true',help='Install required Ubuntu packages before building module')
    parser.add_argument('--camera-url',action='append',default=[])
    parser.add_argument('--repeats',type=int,default=20)
    parser.add_argument('--serve-seconds',type=int,default=0,help='Keep loopback output benchmark available before restoring (0..600)')
    args=parser.parse_args()
    if not 0<=args.serve_seconds<=600:parser.error('serve-seconds must be 0..600')
    if os.geteuid()!=0:raise RuntimeError('Run with sudo in your board terminal; do not send a password to the chat')
    if platform.machine()!='aarch64':raise RuntimeError('KR260 aarch64 only')
    model=Path('/sys/firmware/devicetree/base/model').read_bytes().rstrip(b'\0').decode()
    if 'KR260' not in model:raise RuntimeError('Board model mismatch: '+model)
    manifest=verify_package()
    if args.install_deps:
        command(['apt-get','update'],180)
        command(['apt-get','install','-y','--no-install-recommends','build-essential',
                 'linux-headers-'+platform.release(),'python3-numpy','python3-pil'],600)
    command([sys.executable,'-c','import numpy,PIL; print(numpy.__version__)'],10)
    # Compile before modifying active PL. A failed module build leaves old firmware intact.
    command(['make','-C',HERE/'driver','-j1'],240)
    command(['gcc','-O3','-fPIC','-shared',HERE/'software/cpu_resize.c','-o',HERE/'software/libvpr_cpu.so'],60)
    previous=active_app()
    if previous not in ('kr260-fsr','k26-starter-kits'):raise RuntimeError('Refusing to replace unrecognized active app: '+previous)
    dest=Path('/lib/firmware/xilinx')/APP
    names=['kr260-vision.bit.bin','kr260-vision.dtbo','shell.json']
    if dest.exists():
        for name in names:
            if (dest/name).read_bytes()!=(HERE/'firmware'/name).read_bytes():raise RuntimeError('Different existing firmware: '+str(dest))
    else:
        staging=dest.with_name('.'+APP+'.staging-'+str(os.getpid()));staging.mkdir(mode=0o755)
        for name in names:shutil.copy2(HERE/'firmware'/name,staging/name)
        staging.rename(dest)
    result_dir=HERE/('results_'+time.strftime('%Y%m%d_%H%M%S',time.gmtime()))
    result_dir.mkdir()
    record={'hardware':'NOT_TESTED','model':model,'previous_app':previous,'target_app':APP,
            'manifest_sha256':hashlib.sha256((HERE/'package_manifest.json').read_bytes()).hexdigest(),
            'restored_previous_app':False}
    changed=False;driver=False
    try:
        stop_pressure_readers(record)
        command(['xmutil','unloadapp'],30);changed=True
        command(['xmutil','loadapp',APP],45)
        if active_app()!=APP:raise RuntimeError('New application did not become active')
        command(['insmod',HERE/'driver/vpr_dma.ko'],15);driver=True
        if not Path('/dev/vprdma').exists():raise RuntimeError('VPR DMA driver failed to bind; inspect dmesg')
        test=[sys.executable,HERE/'software/board_test.py','--output',result_dir/'tests','--repeats',str(args.repeats)]
        for path in sorted((HERE/'samples').glob('*.jpg')):test+=['--jpeg',path]
        for url in args.camera_url:test+=['--camera-url',url]
        command(test,600)
        evidence=json.loads((result_dir/'tests/hardware_results.json').read_text())
        if evidence['hardware']!='HARDWARE_PASS':raise RuntimeError('Test result did not pass')
        record['hardware']='HARDWARE_PASS'
        if args.serve_seconds:
            log=(result_dir/'benchmark_server.log').open('w')
            server=subprocess.Popen([sys.executable,str(HERE/'software/benchmark_server.py'),'--samples',str(HERE/'samples')],stdout=log,stderr=subprocess.STDOUT)
            try:
                for _ in range(50):
                    if Path('/proc',str(server.pid)).exists() and 'VPR_BENCHMARK_SERVER_READY' in (result_dir/'benchmark_server.log').read_text():break
                    if server.poll() is not None:raise RuntimeError('Benchmark server failed')
                    time.sleep(.1)
                else:raise RuntimeError('Benchmark server did not become ready')
                print('Benchmark ready on board loopback port 18765 for %d seconds' % args.serve_seconds,flush=True)
                (HERE/'benchmark_ready.json').write_text(json.dumps({'pid':server.pid,'port':18765,'duration_s':args.serve_seconds,'result_directory':str(result_dir)}))
                until=time.monotonic()+args.serve_seconds
                while time.monotonic()<until:
                    if server.poll() is not None:raise RuntimeError('Benchmark server exited early')
                    time.sleep(1)
            finally:
                server.terminate();server.wait(timeout=10);log.close()
                (HERE/'benchmark_ready.json').unlink(missing_ok=True)
    except BaseException as e:
        record['error']=str(e)
        raise
    finally:
        try:
            if driver:command(['rmmod','vpr_dma'],15)
            if changed:
                command(['xmutil','unloadapp'],30)
                command(['xmutil','loadapp',previous],45)
                record['restored_previous_app']=active_app()==previous
                if not record['restored_previous_app']:raise RuntimeError('Previous app restoration failed')
        except BaseException as e:
            record['restore_error']=str(e)
            print('RESTORE FAILED: '+str(e),file=sys.stderr)
        (result_dir/'deployment_result.json').write_text(json.dumps(record,indent=2)+'\n')
        if os.environ.get('SUDO_UID'):
            uid=int(os.environ['SUDO_UID']);gid=int(os.environ['SUDO_GID'])
            for p in [result_dir,*result_dir.rglob('*')]:os.chown(p,uid,gid)
        print('Results: '+str(result_dir),flush=True)
    if not record['restored_previous_app']:raise RuntimeError('Tests finished but prior application was not restored')

if __name__=='__main__':main()
