#!/usr/bin/env python3
"""Run implementation workers after each controller exits; no board access."""
import argparse,hashlib,json,re,shlex,subprocess,tempfile,time
import tkinter
from pathlib import Path

ERROR=re.compile(r'^(?:ERROR|SPLIT_ERROR)(?:\b|:)',re.M)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def file_state(path):
    path=Path(path)
    if not path.exists():return None
    s=path.stat()
    return {'mtime_ns':s.st_mtime_ns,'ctime_ns':s.st_ctime_ns,'inode':s.st_ino,'device':s.st_dev,'size':s.st_size,'sha256':sha(path)}
def updated(old,new):
    return new is not None and (old is None or any(new[k]!=old[k] for k in ('mtime_ns','ctime_ns','inode','device')))
def bit_snapshot(rd):
    rd=Path(rd);bit=rd/'robot_rt_bd_wrapper.bit'
    if bit.exists():raise RuntimeError('fresh bit barrier: target bit already exists')
    for step in ('.write_bitstream.begin.rst','.write_bitstream.end.rst'):
        if (rd/step).exists():raise RuntimeError('fresh bit barrier: prior write_bitstream marker exists')
    if list(rd.glob('.*.error.rst')) or (rd/'.stop.rst').exists():raise RuntimeError('error/stop marker exists before bit worker')
    return {'end':file_state(rd/'.vivado.end.rst'),'begin':file_state(rd/'.vivado.begin.rst'),
            'log':file_state(rd/'runme.log'),'bit_absent':True}
def check_bit_barrier(rd,before,route_sha,manifest,manifest_sha,signoff,signoff_sha):
    rd=Path(rd);bit=rd/'robot_rt_bd_wrapper.bit'
    if not before['bit_absent'] or not bit.is_file() or not bit.stat().st_size:
        raise RuntimeError('new nonempty bit file missing')
    if not updated(before['end'],file_state(rd/'.vivado.end.rst')):
        raise RuntimeError('stale route end marker cannot prove bit completion')
    if not updated(before['begin'],file_state(rd/'.vivado.begin.rst')):
        raise RuntimeError('new bit worker begin marker missing')
    if list(rd.glob('.*.error.rst')):raise RuntimeError('bit error marker present')
    for name in ('.write_bitstream.begin.rst','.write_bitstream.end.rst'):
        if not (rd/name).is_file():raise RuntimeError('actual write_bitstream step marker missing: '+name)
    log=rd/'runme.log';after=file_state(log)
    if after is None or (before['log'] is not None and after['sha256']==before['log']['sha256']):
        raise RuntimeError('bit run log did not change')
    data=log.read_bytes();old=before['log'];mode='new_or_replaced';delta=data
    if old is not None and len(data)>=old['size'] and hashlib.sha256(data[:old['size']]).hexdigest()==old['sha256']:
        delta=data[old['size']:];mode='appended'
    text=delta.decode(errors='replace')
    if not text or 'write_bitstream' not in text or ERROR.search(text):
        raise RuntimeError('new bit log lacks actual step or contains error')
    route=rd/'robot_rt_bd_wrapper_routed.dcp'
    if sha(route)!=route_sha:raise RuntimeError('route DCP changed after STA')
    if sha(manifest)!=manifest_sha or sha(signoff)!=signoff_sha:
        raise RuntimeError('frozen source/constraint/signoff binding changed')
    return {'new_bit_sha256':sha(bit),'end_marker':file_state(rd/'.vivado.end.rst'),
            'new_log_mode':mode,'new_log_sha256':hashlib.sha256(delta).hexdigest(),
            'routed_dcp_sha256':route_sha,'still_requires_new_controller_write_bitstream_Complete':True}


def check_export_artifacts(out,rd):
    """Validate actual export JSON and four files; never assert board acceptance."""
    out=Path(out);rd=Path(rd);manifest=out/'artifacts.sha256.json'
    data=json.loads(manifest.read_text())
    names={'robot_rt.bit','robot_rt.bit.bin','robot_rt.xsa','addresses.json'}
    if (not isinstance(data,dict) or data.get('schema')!='robot_rt.build.v1' or
        data.get('timing_passed') is not True or data.get('board_tested') is not False or
        not isinstance(data.get('files'),dict) or set(data['files'])!=names):
        raise RuntimeError('actual export manifest schema/flags/four-file set mismatch')
    actual={}
    for name in sorted(names):
        path=out/name;expected=data['files'][name]
        if (not path.is_file() or not path.stat().st_size or not isinstance(expected,str) or
            not re.fullmatch(r'[0-9a-f]{64}',expected)):
            raise RuntimeError('missing/empty export artifact or malformed SHA: '+name)
        actual[name]=sha(path)
        if actual[name]!=expected:raise RuntimeError('export artifact SHA mismatch: '+name)
    generated=rd/'robot_rt_bd_wrapper.bit'
    if not generated.is_file() or sha(generated)!=actual['robot_rt.bit']:
        raise RuntimeError('exported bit differs from the newly generated implementation bit')
    return {'schema':'robot_rt.independent_export_check.v1','manifest':str(manifest),
            'manifest_sha256':sha(manifest),'files_sha256':actual,
            'generated_bit_sha256':sha(generated),'actual_JSON_and_four_artifact_hashes_verified':True,
            'board_tested':False,'FPGA_loaded':False}


def _tcl_chunks(text):
    """Split complete top-level Tcl commands without evaluating any script."""
    interp=tkinter.Tcl();chunks=[];buffer='';offset=0
    for line in text.splitlines(keepends=True):
        buffer+=line
        tail=line.rstrip('\r\n');slashes=len(tail)-len(tail.rstrip('\\'))
        if slashes%2:continue
        if interp.call('info','complete',buffer):
            chunks.append((offset,buffer));offset+=len(buffer);buffer=''
    if buffer.strip():
        if not interp.call('info','complete',buffer):raise RuntimeError('incomplete generated implementation Tcl')
        chunks.append((offset,buffer))
    return interp,chunks

def _implementation_body(text,stage):
    """Read only direct commands and the observed literal vendor executed catch.

    `set rc [catch {BODY} RESULT]` executes BODY as a command substitution.
    Other set/puts data and procedure definitions never prove an executed
    stage or a thread setting. Original Tcl is never evaluated. Unknown
    conditional settings fail closed; possible heavy commands in remaining
    control flow must also follow the literal thread setting. The observed
    profile has no executable semicolon separators; compound chunks reject.
    """
    interp,chunks=_tcl_chunks(text)
    markers=[];bodies=[];settings=[];heavy=[];bad_setting=False;catches=0;post_hooks=[]
    # create_msg_db only opens a message database; the observed worker emits
    # it before maxThreads. Design creation/read/transform/write is covered.
    heavy_head=re.compile(r'^(?:create_(?!msg_db$)\w+|open_\w+|read_\w+|link_\w+|opt_\w+|place_\w+|phys_\w+|route_\w+|write_\w+)$')
    thread_line=re.compile(r'^set_param\s+(?:\{general\.maxThreads\}|general\.maxThreads)(?:\s|$)')
    def executed_semicolon(command):
        # The observed native profile uses one command per complete chunk.
        # Conservatively reject any executed semicolon separator (even a
        # trailing one), but not punctuation inside literal braces/quotes or
        # comments. Bracket substitutions inside quotes are executable, so
        # inspect their script context too; never evaluate any Tcl source.
        def scan(index,substitution=False):
            word_start=True;command_start=True;quoted=False
            while index<len(command):
                char=command[index]
                if char=='\\':
                    index+=2
                    continue
                if quoted:
                    if char=='"':quoted=False
                    elif char=='[':
                        bad,index=scan(index+1,True)
                        if bad:return True,index
                        continue
                    index+=1
                    continue
                if char==']' and substitution:return False,index+1
                if char=='#' and command_start:
                    newline=command.find('\n',index)
                    if newline<0:return False,len(command)
                    index=newline+1;word_start=True;command_start=True
                    continue
                if char==';':return True,index
                if char in ' \t\r\n':
                    word_start=True
                    if char=='\n':command_start=True
                    index+=1
                    continue
                if char=='{' and word_start:
                    depth=1;index+=1
                    while index<len(command) and depth:
                        if command[index]=='\\':index+=2;continue
                        if command[index]=='{':depth+=1
                        elif command[index]=='}':depth-=1
                        index+=1
                    if depth:raise RuntimeError('incomplete braced implementation command')
                    word_start=False;command_start=False
                    continue
                if char=='"' and word_start:
                    quoted=True;word_start=False;command_start=False
                    index+=1
                    continue
                if char=='[':
                    bad,index=scan(index+1,True)
                    if bad:return True,index
                    word_start=False;command_start=False
                    continue
                word_start=False;command_start=False;index+=1
            return False,index
        return scan(0)[0]

    # Only the observed vendor POST source container is additionally allowed.
    # Its exact body and both failure handlers are data, never evaluated Tcl.
    hook_path='/home/aup/Lerobot/projects/piper/robot_rt/tcl/capacity_gate.tcl'
    hook_sha='5d2128b8e8b9fc493867f2008d003788a3a018718d25e52e72f8be9e68ef7ba5'
    def fixed_lines(command):
        return '\n'.join(line.strip() for line in command.splitlines() if line.strip())
    def next_command(sequence,index):
        for _,chunk in sequence[index+1:]:
            command=chunk.strip()
            if command and not command.startswith('#'):return command
        raise RuntimeError('vendor POST source failure handler missing')
    source_body='puts "source '+hook_path+'"\nsource '+hook_path
    source_failure=fixed_lines("""if {$src_rc} {
        set tool_flow [get_property -quiet TOOL_FLOW [current_project -quiet]]
        if { $tool_flow eq {SDx} } {
            send_gid_msg -id 2 -ssname VPL_TCL -severity ERROR $_RESULT
            send_gid_msg -id 3 -ssname VPL_TCL -severity ERROR "sourcing script HOOK failed"
        } else {
            send_msg_id runtcl-1 status "$_RESULT"
            send_msg_id runtcl-2 status "sourcing script HOOK failed"
        }
        return -code error
    }""".replace('HOOK',hook_path))
    outer_failure=fixed_lines("""if {$rc} {
        step_failed phys_opt_design
        return -code error $RESULT
    } else {
        end_step phys_opt_design
        unset ACTIVE_STEP
    }""")
    def inspect(sequence,in_catch=False):
        nonlocal bad_setting,catches
        for index,(_,chunk) in enumerate(sequence):
            cmd=chunk.strip()
            if not cmd or cmd.startswith('#'):continue
            if executed_semicolon(cmd):
                raise RuntimeError('unsupported compound semicolon command in implementation worker')
            match=re.match(r'([^\s;]+)',cmd)
            if not match:continue
            head=match.group(1)
            if head in ('proc','puts'):continue
            # This exact, unconditional top-level substitution is the only
            # additional executable container accepted by this v3 parser.
            if head=='set':
                match=re.fullmatch(r'set[ \t]+rc[ \t]+\[catch[ \t]+(\{.*\})[ \t]+RESULT\][ \t]*',cmd,re.S)
                if match is not None and not in_catch:
                    try: words=interp.splitlist('catch '+match.group(1)+' RESULT')
                    except tkinter.TclError:raise RuntimeError('malformed vendor executed catch')
                    if len(words)!=3 or words[0]!='catch' or words[2]!='RESULT':
                        raise RuntimeError('unsupported vendor executed catch')
                    _,nested=_tcl_chunks(words[1]);catches+=1
                    prior_hooks=len(post_hooks);inspect(nested,True)
                    if len(post_hooks)!=prior_hooks:
                        direct=[chunk.strip() for _,chunk in nested if chunk.strip() and not chunk.strip().startswith('#')]
                        hook_index=next(i for i,item in enumerate(direct) if re.match(r'set[ \t]+src_rc[ \t]+\[catch',item))
                        if direct.count('phys_opt_design')!=1 or direct[:hook_index].count('phys_opt_design')!=1 or fixed_lines(next_command(sequence,index))!=outer_failure:
                            raise RuntimeError('vendor POST must follow phys_opt_design and propagate outer catch error')
                elif in_catch and re.fullmatch(r'set[ \t]+src_rc[ \t]+\[catch[ \t]+(\{.*\})[ \t]+_RESULT\][ \t]*',cmd,re.S):
                    match=re.fullmatch(r'set[ \t]+src_rc[ \t]+\[catch[ \t]+(\{.*\})[ \t]+_RESULT\][ \t]*',cmd,re.S)
                    try:words=interp.splitlist('catch '+match.group(1)+' _RESULT')
                    except tkinter.TclError:raise RuntimeError('malformed vendor POST executed catch')
                    if len(words)!=3 or words[0]!='catch' or words[2]!='_RESULT' or fixed_lines(words[1])!=source_body:
                        raise RuntimeError('vendor POST body must be exactly one fixed puts and literal capacity source')
                    if post_hooks or fixed_lines(next_command(sequence,index))!=source_failure:
                        raise RuntimeError('vendor POST source must be unique with exact unconditional error propagation')
                    if not Path(hook_path).is_file() or sha(hook_path)!=hook_sha:
                        raise RuntimeError('vendor POST canonical capacity hook SHA mismatch')
                    post_hooks.append({'path':hook_path,'sha256':hook_sha,'body_literal':True,'inner_and_outer_errors_propagated':True,
                                       'parser_position':len(markers)+len(bodies)+len(settings)+len(heavy)})
                elif '[' in cmd and re.fullmatch(r'set[ \t]+[^\s;]+[ \t]+\{.*\}[ \t]*',cmd,re.S) is None:
                    raise RuntimeError('unsupported executed set substitution in implementation worker')
                # Literal set data never supplies setting/stage evidence.
                continue
            if head=='source':raise RuntimeError('unsupported implementation source outside fixed vendor POST')
            position=len(markers)+len(bodies)+len(settings)+len(heavy)
            mark=re.fullmatch(r'start_step\s+(\w+)\s*',cmd)
            if mark:markers.append((position,mark.group(1)))
            if head==stage:bodies.append(position)
            if heavy_head.fullmatch(head):heavy.append((position,head))
            if head=='set_param':
                try:words=interp.splitlist(cmd)
                except tkinter.TclError:words=()
                if (len(words)>=2 and words[1]=='general.maxThreads') or thread_line.match(cmd):
                    if len(words)!=3 or words[1]!='general.maxThreads' or words[2]!='1':bad_setting=True
                    else:settings.append(position)
                continue
            # A bounded conservative scan of other vendor control flow does
            # not use it as stage evidence. It catches a conditional override
            # and any possible design operation before the proven setting.
            if head in ('if','catch','foreach','for','while','switch','eval','uplevel'):
                possible=re.findall(r'(?:^|[;{\n])[ \t]*(set_param|[A-Za-z_][A-Za-z_0-9]*)(?=[ \t\n;}]|$)',cmd)
                if 'set_param' in possible and re.search(r'general\.maxThreads',cmd):bad_setting=True
                for token in possible:
                    if heavy_head.fullmatch(token):heavy.append((position,token))
    inspect(chunks)
    relevant=[position for position,name in markers if name==stage]
    if len(relevant)!=1 or len(bodies)!=1:return None
    if post_hooks and sum(name=='phys_opt_design' for _,name in heavy)!=1:
        raise RuntimeError('fixed capacity POST requires globally unique actual phys_opt_design')
    if post_hooks and stage=='route_design' and post_hooks[0]['parser_position']>=bodies[0]:
        raise RuntimeError('fixed capacity POST must execute before route_design')
    if bad_setting:raise RuntimeError('implementation worker has nonliteral/non1 or unknown-control thread override')
    if not settings or not heavy or min(settings)>=min(position for position,name in heavy):
        raise RuntimeError('implementation worker lacks literal maxThreads1 before every heavy design command')
    if relevant[0]>=bodies[0]:raise RuntimeError('implementation stage body does not follow its unique start_step')
    return {'stage':stage,'stage_body_count':len(bodies),'stage_marker_count':len(relevant),
            'explicit_executed_maxThreads':1,'maxThreads_before_all_heavy_commands':True,
            'recognized_literal_vendor_executed_catches':catches,'heavy_command_count':len(heavy),
            'worker_text_not_executed_by_parser':True,
            'recognized_fixed_capacity_POST_hooks':post_hooks}

def check_implementation_worker(rd,stage):
    """Require one real runme-selected implementation body and maxThreads1."""
    if stage not in ('route_design','write_bitstream'):raise RuntimeError('unsupported implementation worker stage')
    rd=Path(rd).resolve();runme=rd/'runme.sh';sources=[]
    logical=re.sub(r'\\\r?\n[ \t]*',' ',runme.read_text())
    for line in logical.splitlines():
        stripped=line.strip()
        if not stripped or stripped.startswith('#'):continue
        if not re.match(r'^(?:EAStep\s+)?(?:[^\s]*/)?vivado(?:\s|$)',stripped):continue
        words=shlex.split(stripped,comments=True,posix=True)
        hits=[index for index,value in enumerate(words) if value=='-source']
        if len(hits)!=1 or hits[0]+1>=len(words):raise RuntimeError('ambiguous implementation runme source')
        source=words[hits[0]+1]
        if any(c in source for c in '$`\n\r'):raise RuntimeError('dynamic implementation Tcl source path unsupported')
        path=(rd/source).resolve()
        if path.parent!=rd or path.suffix!='.tcl' or not path.is_file():raise RuntimeError('implementation Tcl escaped exact run directory')
        sources.append(path)
    if len(sources)!=1:raise RuntimeError('exactly one literal Vivado -source worker is required')
    candidates=[]
    paths=list(rd.glob('*.tcl'))
    if len(paths)>64:raise RuntimeError('unexpected unbounded implementation script inventory')
    for path in paths:
        if path.stat().st_size>2000000:raise RuntimeError('unexpected oversized implementation Tcl')
        result=_implementation_body(path.read_text(),stage)
        if result is not None:candidates.append((path.resolve(),result))
    if len(candidates)!=1 or candidates[0][0]!=sources[0]:
        raise RuntimeError('exactly one actual implementation body must equal runme -source')
    path,result=candidates[0]
    result.update(worker_tcl=str(path),worker_tcl_sha256=sha(path),runme_sha256=sha(runme))
    return result

def worker_unchanged(evidence):
    if sha(evidence['worker_tcl'])!=evidence['worker_tcl_sha256']:
        raise RuntimeError('implementation worker Tcl changed after thread preflight')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--phase',choices=['route','finish','all'],required=True)
    ap.add_argument('--repo',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--library',type=Path);ap.add_argument('--phase-driver',type=Path)
    ap.add_argument('--vivado-root',type=Path,default=Path('/home/aup/Xilinx/2025.2/Vivado'))
    ap.add_argument('--execute',action='store_true',help='Run local implementation; default only prints the phases')
    a=ap.parse_args();repo=a.repo.resolve();out=a.out.resolve()
    lib=(a.library or repo/'tcl/build.tcl').resolve();phase_driver=(a.phase_driver or repo/'tcl/phase_driver.tcl').resolve()
    phases=[]
    if a.phase in ('route','all'):phases+=['scripts impl_1','external route runme.sh','verify impl_1']
    if a.phase in ('finish','all'):phases+=['signoff','bit-scripts','external nextstep runme.sh','verify-bit','export']
    if not a.execute:
        print(json.dumps({'status':'DRY_RUN_NO_EXECUTION','unvalidated_main_implementation_flow':True,'phases':phases},indent=2));return
    logs=Path(tempfile.mkdtemp(prefix='robot_rt_implementation_logs_'))
    protected={str(p):sha(p) for p in (Path(__file__).resolve(),phase_driver,lib)}
    records=[]
    def stable():
        for path,expected in protected.items():
            if sha(path)!=expected:raise RuntimeError('driver/library changed during flow: '+path)
    def process(label,argv,cwd=None,expected=()):
        stable();log=logs/(label+'.log');start=time.time_ns()
        with log.open('xb') as stream:rc=subprocess.run(argv,cwd=cwd,stdout=stream,stderr=subprocess.STDOUT).returncode
        text=log.read_text(errors='replace')
        records.append({'label':label,'argv':argv,'returncode':rc,'start_ns':start,'end_ns':time.time_ns(),'log':str(log),'sha256':sha(log)})
        (logs/'execution.json').write_text(json.dumps({'status':'in_progress','protected':protected,'records':records},indent=2)+'\n')
        if rc or ERROR.search(text):raise RuntimeError(f'{label} failed rc/error; {log}')
        for word in expected:
            if word not in text:raise RuntimeError(f'{label} missing sentinel {word!r}; {log}')
        stable();return text
    def phase(name,run=None,sentinels=()):
        args=[str(a.vivado_root/'bin/vivado'),'-mode','batch','-notrace','-source',str(phase_driver),'-tclargs',
              '--phase',name,'--repo',str(repo),'--out',str(out),'--library',str(lib),'--vivado-root',str(a.vivado_root)]
        if run:args+=['--run',run]
        return process(name,args,expected=(f'PASS SPLIT_PHASE_DONE {name};',*sentinels))
    counter=0
    def frozen():
        nonlocal counter
        counter+=1;process(f'frozen_{counter}',['sha256sum','-c',str(out/'split/frozen_inputs.sha256')])
    def run_dir(text,prefix):
        paths=[x[len(prefix):].strip() for x in text.splitlines() if x.startswith(prefix)]
        if len(paths)!=1:raise RuntimeError('one generated run directory is required')
        rd=Path(paths[0]).resolve()
        if rd!=out/'robot_rt.runs/impl_1' or not (rd/'runme.sh').is_file():raise RuntimeError('unexpected implementation directory/entrypoint')
        return rd
    try:
        frozen()
        rd=out/'robot_rt.runs/impl_1'
        if a.phase in ('route','all'):
            text=phase('scripts','impl_1')
            if 'SPLIT_SKIP_CURRENT impl_1' not in text:
                rd=run_dir(text,'SPLIT_SCRIPT_READY impl_1 ')
                if list(rd.glob('*.dcp')) or list(rd.glob('.*.end.rst')) or list(rd.glob('.*.error.rst')) or (rd/'.stop.rst').exists():
                    raise RuntimeError('fresh route required; refusing previous outputs/markers')
                before={'end':file_state(rd/'.vivado.end.rst'),'begin':file_state(rd/'.vivado.begin.rst'),'log':file_state(rd/'runme.log')}
                route_worker=check_implementation_worker(rd,'route_design')
                (logs/'route_worker_thread_preflight.json').write_text(json.dumps(route_worker,indent=2)+'\n')
                route_runme_sha=sha(rd/'runme.sh')
                frozen();process('external_route',['bash',str(rd/'runme.sh')],str(rd))
                worker_unchanged(route_worker)
                if sha(rd/'runme.sh')!=route_runme_sha:raise RuntimeError('route runme.sh changed during worker')
                route=rd/'robot_rt_bd_wrapper_routed.dcp'
                if not route.is_file() or not route.stat().st_size or list(rd.glob('.*.error.rst')) or not updated(before['end'],file_state(rd/'.vivado.end.rst')):
                    raise RuntimeError('actual route checkpoint/new end/error check failed')
                if not updated(before['begin'],file_state(rd/'.vivado.begin.rst')):raise RuntimeError('new route worker begin marker missing')
                if not (rd/'.route_design.end.rst').is_file():raise RuntimeError('route_design step end marker missing')
                log=(rd/'runme.log').read_text(errors='replace')
                if ERROR.search(log) or 'route_design' not in log:raise RuntimeError('route log proof failed')
                (logs/'route_artifacts.json').write_text(json.dumps({'runme_sha256':route_runme_sha,'routed_dcp_sha256':sha(route),'end_marker':file_state(rd/'.vivado.end.rst'),'run_log_sha256':sha(rd/'runme.log')},indent=2)+'\n')
                frozen()
            phase('verify','impl_1',('SPLIT_VERIFIED impl_1',))
        if a.phase in ('finish','all'):
            # This is a new controller; it runs the centralized unchanged STA,
            # PS/RAMB clock, setup/hold/no_clock/DRC checks and then closes.
            phase('signoff',sentinels=('SPLIT_SIGNOFF_PASSED',))
            manifest=out/'split/frozen_inputs.sha256';signoff=out/'split/route_signoff.tcl'
            route=rd/'robot_rt_bd_wrapper_routed.dcp'
            route_sha=sha(route);manifest_sha=sha(manifest);signoff_sha=sha(signoff)
            bit_snapshot(rd) # forbid an old bit before script generation, too
            text=phase('bit-scripts',sentinels=('SPLIT_BIT_SCRIPT_READY ',))
            rd=run_dir(text,'SPLIT_BIT_SCRIPT_READY ')
            if sha(route)!=route_sha or sha(manifest)!=manifest_sha or sha(signoff)!=signoff_sha:
                raise RuntimeError('script generation changed signed-off inputs')
            bit_worker=check_implementation_worker(rd,'write_bitstream')
            (logs/'bit_worker_thread_preflight.json').write_text(json.dumps(bit_worker,indent=2)+'\n')
            before=bit_snapshot(rd)
            bit_runme_sha=sha(rd/'runme.sh')
            (logs/'bit_barrier_before.json').write_text(json.dumps({'snapshot':before,'route_sha256':route_sha,'manifest_sha256':manifest_sha,'signoff_sha256':signoff_sha,'runme_sha256':sha(rd/'runme.sh')},indent=2)+'\n')
            frozen();process('external_bit',['bash',str(rd/'runme.sh')],str(rd))
            worker_unchanged(bit_worker)
            if sha(rd/'runme.sh')!=bit_runme_sha:raise RuntimeError('bit runme.sh changed during worker')
            evidence=check_bit_barrier(rd,before,route_sha,manifest,manifest_sha,signoff,signoff_sha)
            (logs/'bit_barrier_after.json').write_text(json.dumps(evidence,indent=2)+'\n')
            frozen()
            # Existing phase also compares source-set/CONSTRSET, actual run
            # write_bitstream Complete/100%/NEEDS_REFRESH=0 and route hashes.
            phase('verify-bit')
            if sha(route)!=route_sha or sha(manifest)!=manifest_sha or sha(signoff)!=signoff_sha:raise RuntimeError('binding changed before export')
            phase('export')
            exported=check_export_artifacts(out,rd)
            (logs/'export_artifact_verification.json').write_text(json.dumps(exported,indent=2)+'\n')
            frozen()
        (logs/'execution.json').write_text(json.dumps({'status':'passed_requested_phase','scope':'offline build only; no board validation','protected':protected,'records':records},indent=2)+'\n')
        print('PASS requested implementation phase; logs='+str(logs))
    except Exception:
        print('FAILED; logs='+str(logs));raise
if __name__=='__main__':main()
