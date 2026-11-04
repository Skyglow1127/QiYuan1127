#!/usr/bin/env python3
"""Prepare/activate only the D1 controller and its private UART library.

Not run automatically. prepare uploads/builds in a new isolated home directory;
activate alone stops/starts marm_controller.service. It does not send robot
commands, change global libraries, clear angles, or restart companion services.
Failure after stopping leaves the controller stopped, preserves diagnostics,
and never starts the old binary, whose startup clears seven angle counters.
"""
import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

ROOT=Path(__file__).resolve().parent
DEFAULT_SOCKET='/run/user/1000/d1-maint-1000/board.sock'
REMOTE=r'''
import base64,hashlib,io,json,os,pathlib,re,subprocess,sys,tarfile,time,uuid
SERVICE='marm_controller.service'
DROPIN=pathlib.Path('/etc/systemd/system/marm_controller.service.d/90-d1-integrity-fix.conf')
COMPANIONS=['marm_control_node','marm_communication_node','subscriber']
GLOBAL_LIBS=['/usr/local/lib/libfsuartservo.so','/usr/local/lib/libcserialport.so']
request=json.load(sys.stdin)
remote=None;stopped=False;diagnostics={}
def sha(path):return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
def run(args,check=True,input=None):
    p=subprocess.run(args,input=input,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    if check and p.returncode:raise RuntimeError('Command failed: '+repr(args)+'\n'+p.stdout[-12000:])
    return dict(argv=args,returncode=p.returncode,output=p.stdout)
def processes():
    found=[]
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            exe=os.readlink(p/'exe');name=pathlib.Path(exe).name
            if name not in COMPANIONS+['marm_controller_node']:continue
            cgroup=(p/'cgroup').read_text()
            found.append(dict(pid=int(p.name),exe=exe,sha256=sha(p/'exe'),cgroup=cgroup))
        except (OSError,ValueError):pass
    return found
def serial_owners():
    owners=[]
    for p in pathlib.Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            for f in (p/'fd').iterdir():
                try:
                    if os.readlink(f)=='/dev/ttyS4':owners.append(int(p.name))
                except OSError:pass
        except OSError:pass
    return sorted(set(owners))
def snapshot():
    return dict(processes=processes(),serial_owners=serial_owners(),
        global_library_hashes={p:sha(p) for p in GLOBAL_LIBS},
        unit=run(['systemctl','show',SERVICE,'-p','MainPID','-p','ActiveState','-p','ExecStart','-p','DropInPaths','-p','Restart']))
def controller(procs):
    hits=[p for p in procs if pathlib.Path(p['exe']).name=='marm_controller_node' and '/marm_controller.service' in p['cgroup']]
    if len(hits)!=1:raise RuntimeError('Expected exactly one controller service executable; found '+str(len(hits)))
    return hits[0]
def companion_identity(procs):
    return sorted((p['pid'],p['exe'],p['sha256']) for p in procs if pathlib.Path(p['exe']).name in COMPANIONS)
def persist(name,value):
    (remote/name).write_text(json.dumps(value,indent=2)+'\n')
def unit_stop():return run(['sudo','-n','systemctl','stop',SERVICE])
def journal(pid):
    return run(['sudo','-n','journalctl','_PID='+str(pid),'-n','80','--no-pager','-o','cat'],check=False)
try:
    mode=request['mode']
    if mode=='prepare':
        archive=base64.b64decode(request['archive_base64'])
        if hashlib.sha256(archive).hexdigest()!=request['archive_sha256']:raise RuntimeError('Archive SHA mismatch')
        remote=pathlib.Path('/home/ubuntu/d1_integrity_fix_20261027_'+request['archive_sha256'][:12]+'_'+uuid.uuid4().hex[:8])
        remote.mkdir(mode=0o700)
        with tarfile.open(fileobj=io.BytesIO(archive),mode='r:gz') as tar:
            for member in tar.getmembers():
                name=pathlib.PurePosixPath(member.name)
                if member.islnk() or member.issym() or not member.isfile() or name.is_absolute() or '..' in name.parts:
                    raise RuntimeError('Unexpected archive member')
                target=remote/member.name;target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes(tar.extractfile(member).read())
        for name,digest in request['source_hashes'].items():
            if sha(remote/name)!=digest:raise RuntimeError('Source SHA mismatch '+name)
        baseline=snapshot();old=controller(baseline['processes'])
        if baseline['serial_owners']!=[old['pid']]:raise RuntimeError('Expected old controller as sole serial owner')
        library=remote/'library'
        if 'project(D1_FashionStar_Checked' not in (library/'CMakeLists.txt').read_text():
            raise RuntimeError('Private standalone library CMake not present; refusing legacy install-oriented build')
        logs=[]
        logs.append(run(['cmake','-S',str(library),'-B',str(library/'build')]))
        logs.append(run(['cmake','--build',str(library/'build'),'--target','fsuartservo','-j2']))
        lib=library/'build/lib/libfsuartservo.so'
        inc=library/'build/include'
        if not lib.is_file() or not (inc/'FashionStar/UServo/FashionStar_UartServo.h').is_file():
            raise RuntimeError('Private library/header outputs not at expected paths')
        msg=pathlib.Path('/home/ubuntu/marm_code/src/msg')
        msg_names=['ArmString_','PubServoInfo_','SetServoAngle_','SetServoDumping_']
        msg_hashes={str(msg/(n+suffix)):sha(msg/(n+suffix)) for n in msg_names for suffix in ['.cpp','.hpp']}
        project=remote/'controller_project';project.mkdir()
        lines=['cmake_minimum_required(VERSION 3.10)','project(d1_controller_only LANGUAGES CXX)',
            'set(CMAKE_CXX_STANDARD 17)',
            'add_executable(target_only "'+str(remote/'controller/marm_controller_node.cpp')+'" '+
                ' '.join('"'+str(msg/(n+'.cpp'))+'"' for n in msg_names)+')',
            'set_target_properties(target_only PROPERTIES OUTPUT_NAME marm_controller_node)',
            'target_include_directories(target_only PRIVATE "'+str(inc)+'" /home/ubuntu/marm_code/src /usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)',
            'target_link_libraries(target_only PRIVATE "'+str(lib)+'" /usr/local/lib/libunitree_sdk2.a ddsc ddscxx rt pthread cserialport)',
            'set_target_properties(target_only PROPERTIES BUILD_RPATH "'+str(lib.parent)+'")']
        (project/'CMakeLists.txt').write_text('\n'.join(lines)+'\n')
        logs.append(run(['cmake','-S',str(project),'-B',str(remote/'controller_build')]))
        logs.append(run(['cmake','--build',str(remote/'controller_build'),'--target','target_only','-j2']))
        exe=remote/'controller_build/marm_controller_node'
        dynamic=run(['readelf','-d',str(exe)])
        if 'libfsuartservo.so' not in dynamic['output']:raise RuntimeError('Controller does not need the expected private library SONAME')
        after=snapshot()
        if after!=baseline:
            # Unit output includes stable state only; identity/library/owner must remain unchanged.
            if (after['processes']!=baseline['processes'] or after['serial_owners']!=baseline['serial_owners'] or after['global_library_hashes']!=baseline['global_library_hashes']):
                raise RuntimeError('Existing process/library identity changed during prepare')
        prepared=dict(ok=True,mode='prepare',remote_dir=str(remote),source_hashes=request['source_hashes'],
            archive_sha256=request['archive_sha256'],controller_path=str(exe),controller_sha256=sha(exe),
            private_library_path=str(lib),private_library_sha256=sha(lib),msg_hashes=msg_hashes,
            baseline=baseline,readelf=dynamic,
            no_startup_side_effects_reviewed=True,old_binary_rollback_start_prohibited=True)
        persist('compile_log.json',logs);persist('prepared.json',prepared)
        response=prepared
    elif mode=='activate':
        remote=pathlib.Path(request['remote_dir'])
        if not re.fullmatch('/home/ubuntu/d1_integrity_fix_20261027_[a-f0-9]{12}_[a-f0-9]{8}',str(remote)):
            raise RuntimeError('Unexpected remote staging path')
        prepared=json.loads((remote/'prepared.json').read_text())
        if not prepared.get('ok') or prepared['controller_sha256']!=request['controller_sha256']:
            raise RuntimeError('Prepared identity mismatch')
        exe=pathlib.Path(prepared['controller_path']);lib=pathlib.Path(prepared['private_library_path'])
        if sha(exe)!=prepared['controller_sha256'] or sha(lib)!=prepared['private_library_sha256']:
            raise RuntimeError('Prepared binaries changed')
        for name,digest in prepared['source_hashes'].items():
            if sha(remote/name)!=digest:raise RuntimeError('Reviewed source changed '+name)
        before=snapshot();baseline=prepared['baseline'];old=controller(before['processes'])
        if old!=controller(baseline['processes']) or before['global_library_hashes']!=baseline['global_library_hashes']:
            raise RuntimeError('Current old controller/global library identity differs from prepare')
        if companion_identity(before['processes'])!=companion_identity(baseline['processes']):
            raise RuntimeError('Companion process identity differs from prepare')
        if before['serial_owners']!=[old['pid']]:raise RuntimeError('Unexpected serial owner before stop')
        if DROPIN.exists():raise RuntimeError('Deployment drop-in already exists; no overwrite/replay')
        diagnostics['before']=before
        diagnostics['stop']=unit_stop();stopped=True
        deadline=time.monotonic()+5.
        while serial_owners() and time.monotonic()<deadline:time.sleep(.1)
        if serial_owners():raise RuntimeError('Serial port still held after stopping controller service')
        if any('/marm_controller.service' in p['cgroup'] and pathlib.Path(p['exe']).name=='marm_controller_node' for p in processes()):
            raise RuntimeError('Old controller survived stop')
        content='[Service]\nExecStart=\nExecStart=/usr/bin/env LD_LIBRARY_PATH='+str(lib.parent)+' '+str(exe)+'\nRestart=no\n'
        run(['sudo','-n','mkdir','-p',str(DROPIN.parent)])
        run(['sudo','-n','tee',str(DROPIN)],input=content)
        diagnostics['dropin']=dict(path=str(DROPIN),text=content)
        run(['sudo','-n','systemctl','daemon-reload'])
        diagnostics['start']=run(['sudo','-n','systemctl','start',SERVICE])
        deadline=time.monotonic()+8.;new=None;last_journal=None
        while time.monotonic()<deadline:
            try:
                candidate=controller(processes())
                if candidate['exe']==str(exe) and candidate['sha256']==prepared['controller_sha256']:
                    maps=pathlib.Path('/proc/'+str(candidate['pid'])+'/maps').read_text()
                    private_mapped=str(lib) in maps and '/usr/local/lib/libfsuartservo.so' not in maps
                    last_journal=journal(candidate['pid'])
                    frames=[]
                    for line in last_journal['output'].splitlines():
                        try:
                            d=json.loads(line).get('data',{})
                            if d.get('event')=='feedback_scan' and d.get('valid_mask')==127 and d.get('valid_frames',0)>0:frames.append(d)
                        except (ValueError,AttributeError):pass
                    if private_mapped and serial_owners()==[candidate['pid']] and frames:
                        new=candidate;diagnostics['latest_valid_scan']=frames[-1];diagnostics['new_maps']=maps;break
            except (RuntimeError,OSError):pass
            time.sleep(.2)
        diagnostics['new_journal']=last_journal
        if new is None:raise RuntimeError('New controller identity/private map/sole serial owner/fresh valid frame was not confirmed in 8 s')
        after=snapshot();diagnostics['after']=after
        if after['global_library_hashes']!=baseline['global_library_hashes'] or companion_identity(after['processes'])!=companion_identity(baseline['processes']):
            raise RuntimeError('Global library or companion identity changed')
        response=dict(ok=True,mode='activate',remote_dir=str(remote),new_controller=new,
            private_library_sha256=sha(lib),diagnostics=diagnostics,
            physical_motion_command_sent=False,position_accuracy_verified=False,
            old_binary_rollback_start_prohibited=True)
        persist('activation.json',response)
    else:raise RuntimeError('Unknown mode')
except Exception as error:
    if stopped:
        diagnostics['failure_stop']=run(['sudo','-n','systemctl','stop',SERVICE],check=False)
        diagnostics['remaining_serial_owners']=serial_owners()
    response=dict(ok=False,mode=request.get('mode'),remote_dir=str(remote) if remote else None,
        error=str(error),controller_left_stopped=stopped,diagnostics=diagnostics,
        automatic_old_binary_start=False)
    if remote and remote.exists():persist('failure_'+str(request.get('mode'))+'.json',response)
print(json.dumps(response))
sys.exit(0 if response.get('ok') else 1)
'''


def bundle():
    paths=[p for part in ('controller','library') for p in (ROOT/part).rglob('*')
           if p.is_file() and not p.name.startswith('test_') and not any(x in p.relative_to(ROOT/part).parts for x in ('build','original','__pycache__'))]
    if not paths:raise ValueError('No controller/library source payload')
    manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}
    stream=io.BytesIO()
    with tarfile.open(fileobj=stream,mode='w:gz') as archive:
        for path in sorted(paths):
            if path.is_symlink():raise ValueError('Source symlinks not allowed')
            archive.add(path,arcname=str(path.relative_to(ROOT)),recursive=False)
    raw=stream.getvalue()
    return dict(mode='prepare',archive_base64=base64.b64encode(raw).decode(),
                archive_sha256=hashlib.sha256(raw).hexdigest(),source_hashes=manifest)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['prepare','activate'])
    p.add_argument('--socket',default=DEFAULT_SOCKET)
    p.add_argument('--prepared',type=Path,help='Local prepared.json produced by prepare')
    p.add_argument('--output',type=Path,required=True,help='New local evidence directory')
    a=p.parse_args()
    if a.mode=='activate':
        if a.prepared is None:p.error('activate requires --prepared')
        prepared=json.loads(a.prepared.read_text())
        if not prepared.get('ok'):p.error('prepare did not succeed')
        request=dict(mode='activate',remote_dir=prepared['remote_dir'],controller_sha256=prepared['controller_sha256'])
    else:request=bundle()
    # Never silently open a replacement authentication session.
    check=subprocess.run(['ssh','-S',a.socket,'-O','check','ubuntu@192.168.123.100'],capture_output=True,text=True)
    if check.returncode:p.exit(2,'Existing SSH master check failed; no reconnect attempted.\n')
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'request_identity.json').write_text(json.dumps({k:v for k,v in request.items() if k!='archive_base64'},indent=2)+'\n')
    cmd=['ssh','-S',a.socket,'-o','BatchMode=yes','-o','ControlMaster=no','ubuntu@192.168.123.100',
         'python3 -c '+shlex.quote(REMOTE)]
    completed=subprocess.run(cmd,input=json.dumps(request),capture_output=True,text=True)
    (a.output/'ssh_stderr.txt').write_text(completed.stderr)
    (a.output/'ssh_stdout.txt').write_text(completed.stdout)
    try:result=json.loads(completed.stdout.strip().splitlines()[-1])
    except (ValueError,IndexError):p.exit(2,'Remote response unavailable; inspect saved SSH output. Do not retry activation blindly.\n')
    name='prepared.json' if a.mode=='prepare' else 'activation.json'
    (a.output/name).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(ok=result.get('ok'),mode=a.mode,remote_dir=result.get('remote_dir'),
                         evidence=str(a.output.resolve()),error=result.get('error')),ensure_ascii=False))
    return 0 if completed.returncode==0 and result.get('ok') else 1


if __name__=='__main__':raise SystemExit(main())
