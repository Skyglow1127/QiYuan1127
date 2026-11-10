#!/usr/bin/env python3
"""Enable/revert experimental Chinese GUI labels in a Ross-managed Vivado 2025.2.

No Vivado installation files are patched. This is partial UI localization, not a
complete or AMD-supported Chinese edition. Sessions remain open on every outcome.
"""
import argparse
import hashlib
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time

from ross_mcp import ROOT, Ross
from run_mvp import memory_guard, unpack

LOCAL = ROOT / 'localization'
LATEST = ROOT / 'reports/vivado_zh_latest.json'


def ensure_translation_worker(out, vivado_pid):
    """Start the local offline prose translator once per Vivado session."""
    receipt=out/'translation-worker.json'
    script=LOCAL/'translate_worker.py'
    python=LOCAL/'translation-runtime/venv/bin/python'
    if not python.exists():
        raise RuntimeError('离线翻译环境缺失：'+str(python))
    if receipt.exists():
        old=json.loads(receipt.read_text())
        proc=Path('/proc')/str(old.get('pid',0))
        try:
            command=(proc/'cmdline').read_bytes().split(b'\0')
            if str(script).encode() in command and str(out/'translation-queue.jsonl').encode() in command:
                return old
        except (FileNotFoundError,PermissionError):
            pass
    command=[str(python),'-I',str(script),'--queue',str(out/'translation-queue.jsonl'),
             '--output',str(out/'machine-translations.tsv'),'--status',str(out/'translation-status.json'),
             '--owner-pid',str(vivado_pid)]
    with (out/'translation-worker.log').open('a') as log:
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    info={'pid':child.pid,'vivado_pid':vivado_pid,'offline':True,'quality_reviewed':False,'command':command}
    receipt.write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n')
    time.sleep(0.15)
    if child.poll() is not None:
        raise RuntimeError('离线翻译服务启动失败，请查看 '+str(out/'translation-worker.log'))
    return info


def checked(ross, name, arguments=None):
    result = unpack(ross.call(name, arguments or {}))
    if result.get('exit_code', 0) != 0 or result.get('status') in ('error', 'failed'):
        raise RuntimeError(json.dumps(result, ensure_ascii=False))
    return result


def build_agent():
    classes = LOCAL / 'classes'
    classes.mkdir(exist_ok=True)
    sources = [LOCAL / n for n in ('VivadoZhAgent.java', 'VivadoZhRenderers.java', 'VivadoZhStyledLabel.java', 'VivadoZhBorders.java', 'VivadoZhIpTables.java', 'VivadoZhOverlay.java', 'VivadoZhHtml.java', 'VivadoZhSmartConnectOverlay.java', 'VivadoZhBootstrap.java', 'AttachVivadoZh.java')]
    digest=hashlib.sha256(b''.join(p.read_bytes() for p in sources+[LOCAL/'MANIFEST.MF'])).hexdigest()[:12]
    # Unique immutable paths avoid the target JVM's cached JAR entries on reload.
    jar = LOCAL / f'vivado-zh-{digest}.jar'
    if not jar.exists():
        subprocess.run(['javac', '--release', '11', '-encoding', 'UTF-8', '-d', str(classes), *map(str,sources)], check=True)
        subprocess.run(['jar', 'cfm', str(jar), str(LOCAL/'MANIFEST.MF'), '-C', str(classes), '.'], check=True)
    return jar


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', action='store_true', help='start a new empty GUI through Ross (8 GiB RAM gate)')
    parser.add_argument('--session-id', help='choose an existing local Ross session explicitly')
    parser.add_argument('--disable', action='store_true', help='restore original UI labels without closing Vivado')
    args = parser.parse_args()
    if args.start and (args.disable or args.session_id):
        parser.error('--start cannot be combined with --disable or --session-id')
    jar = build_agent()
    with Ross() as ross:
        sessions = checked(ross, 'vivado_list_sessions').get('sessions', {}) or {}
        if isinstance(sessions,list):
            sessions = {s['session_id']: s for s in sessions}
        if args.start:
            memory_guard(8)
            out = ROOT / 'reports' / datetime.now().strftime('vivado_zh_%Y%m%d_%H%M%S')
            out.mkdir(parents=True,exist_ok=True)
            session = checked(ross, 'vivado_start', {'working_dir':str(out), 'session_type':'general', 'gui_mode':False, 'display_mode':'native'})
            # Record the receipt immediately; never lose a launched session on error.
            (out/'session.json').write_text(json.dumps(session,ensure_ascii=False,indent=2)+'\n')
        else:
            selected = args.session_id
            if not selected and LATEST.exists():
                candidate=json.loads(LATEST.read_text()).get('session_id')
                if candidate in sessions and sessions[candidate].get('process_running',True) and sessions[candidate].get('deployment_type','local')=='local':
                    selected=candidate
            if not selected:
                local = [s for s in sessions.values() if s.get('deployment_type','local')=='local' and s.get('process_running',True)]
                if len(local)!=1:
                    raise RuntimeError('请用 --session-id 选择唯一的本地会话，或用 --start 新建空白会话。当前会话：'+', '.join(sessions))
                selected=local[0]['session_id']
            if selected not in sessions:
                raise RuntimeError('指定会话不存在；没有修改任何其他会话。')
            session=sessions[selected]
            previous=json.loads(LATEST.read_text()) if LATEST.exists() else {}
            if previous.get('session_id')==selected and previous.get('evidence_directory'):
                out=Path(previous['evidence_directory'])
            else:
                out=ROOT/'reports'/('vivado_zh_session_'+selected)
            out.mkdir(parents=True,exist_ok=True)
        sid=session['session_id']
        if session.get('deployment_type','local')!='local' or session.get('gui_surface')=='ide-webview':
            raise RuntimeError('此原型只支持本机桌面 Vivado 2025.2 GUI。')
        pid=int(session['process_id'])
        proc=Path('/proc')/str(pid)
        import os
        if proc.stat().st_uid!=os.getuid() or Path(os.readlink(proc/'exe')).name!='vivado':
            raise RuntimeError('进程身份校验失败；未加载界面原型。')
        identity=checked(ross,'vivado_execute',{'session_id':sid,
            'command':f'list [version -short] [pid] {time.time_ns()}',
            'verbosity':'full'}).get('output','').strip().split()
        if len(identity)!=3 or identity[1]!=str(pid):
            raise RuntimeError('Vivado 进程身份返回不符：'+repr(identity))
        version=identity[0]
        if version!='2025.2':
            raise RuntimeError('此原型仅验证了 Vivado 2025.2，当前版本为 '+version)
        if not args.disable and not session.get('is_gui_mode',False):
            checked(ross,'vivado_execute',{'session_id':sid,'command':'start_gui','verbosity':'full'})
        options='disable' if args.disable else f'enable|{LOCAL / "vivado-2025.2-zh.tsv"}|{out}|{jar}'
        if not args.disable:
            ensure_translation_worker(out,pid)
        subprocess.run(['java','-cp',str(jar),'AttachVivadoZh',str(pid),str(jar),options],check=True)
        summary={'session_id':sid,'process_id':pid,'vivado_version':version,
                 'state':'disabled' if args.disable else 'enabled',
                 'scope':'experimental partial GUI localization', 'session_close_policy':'manual',
                 'evidence_directory':str(out),'installation_files_modified':False}
        LATEST.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        print('已恢复英文界面。' if args.disable else '已启用实验性中文界面；尚未实现完整汉化。')
        print('Vivado 会话保持打开。会话：'+sid)
        print('记录：'+str(out))


if __name__=='__main__':
    main()
