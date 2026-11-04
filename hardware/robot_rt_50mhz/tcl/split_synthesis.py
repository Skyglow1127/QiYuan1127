#!/usr/bin/env python3
"""Run each synthesis stage after its Vivado controller exits.

Default prints commands; --execute performs a local build with no board access."""
import argparse,hashlib,json,re,subprocess,tempfile,time
from pathlib import Path

HERE=Path(__file__).resolve().parent

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--phase',choices=['prepare','synth'],required=True)
    ap.add_argument('--repo',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--run',help='Exact frozen-plan synthesis run; e.g. robot_rt_bd_core_0_synth_1 or synth_1')
    ap.add_argument('--core-flatten',choices=['rebuilt','none'],default=None,
                    help='Prepare only: freeze the core synthesis hierarchy policy; default rebuilt')
    ap.add_argument('--core-directive',choices=['runtimeoptimized'],default=None,
                    help='Prepare only: optional core synthesis directive; absent preserves the existing run setting')
    ap.add_argument('--partition-dir',type=Path,help='Prepare only: optional verified child partition directory; implies core-flatten none')
    ap.add_argument('--partition-gate-stable-job-inputs',type=int,choices=(0,1),default=None,
                    help='Prepare-only independent parent expectation; default0 with partition-dir')
    ap.add_argument('--library',type=Path,default=HERE/'build.tcl')
    ap.add_argument('--vivado-root',type=Path,default=Path('/home/aup/Xilinx/2025.2/Vivado'))
    ap.add_argument('--execute',action='store_true',help='Run local synthesis; default prints commands only')
    a=ap.parse_args()
    repo=a.repo.resolve();out=a.out.resolve();lib=a.library.resolve()
    if a.phase=='synth' and not a.run:ap.error('--run is required for synth')
    if a.phase!='prepare' and a.core_flatten is not None:ap.error('--core-flatten is prepare-only; later phases use the frozen plan')
    if a.phase!='prepare' and a.core_directive is not None:ap.error('--core-directive is prepare-only; later phases use the frozen plan')
    if a.phase!='prepare' and a.partition_dir is not None:ap.error('--partition-dir is prepare-only')
    if a.partition_gate_stable_job_inputs is not None and (a.phase!='prepare' or a.partition_dir is None):
        ap.error('--partition-gate-stable-job-inputs requires prepare with --partition-dir')
    if a.partition_dir is not None and a.core_flatten not in (None,'none'):ap.error('--partition-dir requires --core-flatten none')
    # Explicit synthesis-only scope. Route and bit phases remain review prototypes.
    if a.phase=='synth' and not (a.run=='synth_1' or re.fullmatch(r'[A-Za-z0-9_]+_synth_[0-9]+',a.run)):
        ap.error('This minimal external driver accepts only a synthesis run')
    base=[str(a.vivado_root/'bin/vivado'),'-mode','batch','-notrace','-source',str(HERE/'phase_driver.tcl'),'-tclargs',
          '--repo',str(repo),'--out',str(out),'--library',str(lib),'--vivado-root',str(a.vivado_root)]
    def cmd(phase):return base+['--phase',phase]+(['--run',a.run] if a.run else [])+(
        ['--core-flatten',a.core_flatten or ('none' if a.partition_dir else 'rebuilt')]+(
            ['--partition-dir',str(a.partition_dir.resolve()),'--partition-gate-stable-job-inputs',str(0 if a.partition_gate_stable_job_inputs is None else a.partition_gate_stable_job_inputs)] if a.partition_dir else [])+(
            ['--core-directive',a.core_directive] if a.core_directive is not None else []) if phase=='prepare' else [])
    if not a.execute:
        print(json.dumps({'status':'DRY_RUN_NO_EXECUTION','prepare':cmd('prepare')} if a.phase=='prepare' else
              {'status':'DRY_RUN_NO_EXECUTION','before':cmd('scripts'),'external':'complete generated runme.sh after controller exits','after':cmd('verify')},indent=2))
        return
    logs=Path(tempfile.mkdtemp(prefix='robot_rt_split_synthesis_'))
    records=[]
    def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
    def process(label,argv,cwd=None,expected=()):
        log=logs/(label+'.log');started=time.time_ns()
        with log.open('xb') as stream:rc=subprocess.run(argv,cwd=cwd,stdout=stream,stderr=subprocess.STDOUT).returncode
        records.append({'label':label,'argv':argv,'returncode':rc,'started_ns':started,'ended_ns':time.time_ns(),'log':str(log),'log_sha256':sha(log)})
        (logs/'execution.json').write_text(json.dumps({'status':'in_progress','records':records},indent=2)+'\n')
        text=log.read_text(errors='replace')
        if rc:raise RuntimeError(f'{label} failed rc={rc}; {log}')
        if re.search(r'^(?:ERROR|SPLIT_ERROR)(?:\b|:)',text,re.M):
            raise RuntimeError(f'{label} emitted an error despite exit0; {log}')
        for sentinel in expected:
            if sentinel not in text:raise RuntimeError(f'{label} missing expected sentinel {sentinel!r}; {log}')
        return text
    def frozen(label):process(label,['sha256sum','-c',str(out/'split/frozen_inputs.sha256')])
    def insertion_barrier(rd):
        split=out/'split'
        hashes=split/'partition_insertion_hashes.json'
        expected=json.loads(hashes.read_text())
        paths={'original_sha256':split/'core_worker_original.tcl',
               'generated_sha256':rd/'robot_rt_bd_core_0.tcl',
               'patched_sha256':split/'core_worker_patched.tcl',
               'diff_sha256':split/'core_worker_insertion.diff',
               'hook_sha256':split/'assemble_partitions_insert.tcl',
               'runme_sha256':rd/'runme.sh','frozen_manifest_sha256':split/'frozen_inputs.sha256',
               'record_sha256':split/'partition_insertion_record.tcl'}
        actual={key:sha(path) for key,path in paths.items()}
        if expected!=actual or actual['generated_sha256']!=actual['patched_sha256']:
            raise RuntimeError('generated core insertion evidence/hash mismatch')
        return {'hashes_file_sha256':sha(hashes),'hashes':actual}
    try:
        if a.phase=='prepare':
            if out.exists():raise RuntimeError('fresh output required')
            process('prepare',cmd('prepare'),expected=('SPLIT_PREPARED ','PASS SPLIT_PHASE_DONE prepare;'))
            frozen('frozen_after_prepare')
        else:
            frozen('frozen_before_scripts')
            text=process('scripts',cmd('scripts'),expected=('PASS SPLIT_PHASE_DONE scripts;',))
            skip=f'SPLIT_SKIP_CURRENT {a.run}'
            if skip not in text:
                prefix=f'SPLIT_SCRIPT_READY {a.run} '
                hits=[line[len(prefix):].strip() for line in text.splitlines() if line.startswith(prefix)]
                if len(hits)!=1:raise RuntimeError('exact generated run directory not reported')
                rd=Path(hits[0]).resolve()
                if not rd.is_relative_to(out/'robot_rt.runs'):raise RuntimeError('run directory escaped fresh project')
                if not (rd/'runme.sh').is_file():raise RuntimeError('runme.sh missing')
                if list(rd.glob('*.dcp')) or list(rd.glob('.*.end.rst')) or list(rd.glob('.*.error.rst')):
                    raise RuntimeError('fresh run contains prior output/markers; refusing overwrite')
                frozen('frozen_before_external')
                # process() returned only after controller Vivado exited.
                before={'runme_sha256':sha(rd/'runme.sh'),'start_ns':time.time_ns()}
                insertion=None
                if a.run=='robot_rt_bd_core_0_synth_1':
                    hits=[line for line in text.splitlines() if line.startswith('SPLIT_PARTITION_INSERTED ')]
                    if len(hits)>1:raise RuntimeError('duplicate core insertion contract sentinel')
                    if hits:
                        insertion=insertion_barrier(rd)
                        if hits[0].split()[-1]!=insertion['hashes_file_sha256']:
                            raise RuntimeError('controller insertion SHA differs from runner')
                        before['partition_insertion']=insertion
                    elif (out/'split/partition_insertion_hashes.json').exists():
                        raise RuntimeError('insertion record exists without controller acceptance')
                process('external_runme',['bash',str(rd/'runme.sh')],str(rd))
                if insertion is not None and insertion_barrier(rd)!=insertion:
                    raise RuntimeError('core generated script/hook/evidence changed during external worker')
                log=(rd/'runme.log').read_text(errors='replace')
                if 'using a maximum of 1 processes.' not in log:
                    raise RuntimeError('synthesis log does not confirm the single-process limit')
                if not (rd/'.vivado.end.rst').is_file() or list(rd.glob('.*.error.rst')):
                    raise RuntimeError('missing end marker/present error marker; shell exit alone is insufficient')
                dcps=list(rd.glob('*.dcp'))
                if not dcps or any(x.stat().st_size==0 for x in dcps) or re.search(r'^ERROR:',log,re.M) or 'synth_design completed successfully' not in log:
                    raise RuntimeError('actual synthesis log/checkpoint proof failed')
                before['artifact_sha256']={str(x):sha(x) for x in [rd/'.vivado.end.rst',rd/'runme.log',*dcps]}
                (logs/'external_artifacts.json').write_text(json.dumps(before,indent=2)+'\n')
                frozen('frozen_after_external')
            # A completely new controller must recognize real completion/freshness.
            process('verify',cmd('verify'),expected=(f'SPLIT_VERIFIED {a.run}','PASS SPLIT_PHASE_DONE verify;'))
        (logs/'execution.json').write_text(json.dumps({'status':'passed_requested_phase','scope':'synthesis scheduling, not STA or board acceptance','records':records},indent=2)+'\n')
        print('PASS requested split phase; logs='+str(logs))
    except Exception:
        print('FAILED; logs='+str(logs))
        raise

if __name__=='__main__':main()
