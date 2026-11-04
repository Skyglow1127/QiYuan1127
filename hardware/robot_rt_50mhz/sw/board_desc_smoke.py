#!/usr/bin/env python3
"""DESC_ONLY stationary MMIO smoke planner/collector. Default: OFFLINE --check.

Examples:
  python sw/board_desc_smoke.py --check --build BUILD --input stationary.json \
      --lead-cycles 20000000 --window-cycles 100000000 --output-dir /tmp/check_new
  # MANUAL TARGET-BOARD ONLY, not executed by --check:
  sudo python sw/board_desc_smoke.py --run --build BUILD --input stationary.json \
      --lead-cycles 20000000 --window-cycles 100000000 --output-dir /tmp/run_new \
      --platform-manifest-sha256 ACTUAL_MANIFEST_SHA --device-mmio-mapping

Input: explicit sw/pl_bank.py JSON plus source_kind='synthetic_stationary'.
Only zero v/a/j and first-bank data; no default frequency. Input absolute time
fields are explicitly REPLACED by locally generated synthetic PL-clock times.
Limits/rate/lease/source-age/stop-timeout/force remain explicit and unchanged.

--run requires a signed-off DESC_ONLY build, matching initialized PS platform,
Linux/AArch64 root, Device MMIO mapping, operating fpga_manager and existing cc.
It never loads a bitstream or opens CAN/robot/camera/SDK interfaces. An invented
stationary seed is not physical feedback; descriptor bytes must not be forwarded
to CAN. MMIO completion/readback is NOT an observed AXI BRESP or independent
latency measurement. All output directories are fresh; no automatic retry.
"""
from __future__ import annotations
import argparse
import ctypes
import hashlib
import json
import mmap
import os
from pathlib import Path
import platform
import shutil
import stat
import struct
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from sw.desc_smoke import (CAPTURE_SCHEMA, CMDS, REGS, canonical, make_plan,
                           register_words, read_json, verify_capture)
from sw.pl_bank import build_bank,uint
from sim.tick_reference import DDSConfig

EXPECTED={'schema':'robot_rt.addresses.v1','board_part':'xilinx.com:kr260_som:part0:1.1',
 'part':'xck26-sfvc784-2LV-c','backend':'DESC_ONLY','cfg_base':0xA0000000,'cfg_bytes':4096,
 'chunk_base':0xA0020000,'chunk_bytes':131072,'expected_id':0x5254504c}
HELPER=r'''#include <stdint.h>
#include <stddef.h>
#if !defined(__aarch64__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Requires little-endian AArch64"
#endif
static inline void fence(void){__asm__ volatile("dsb sy" ::: "memory");}
uint32_t mmio_read32(void *p,size_t o){fence();uint32_t v=*(volatile uint32_t*)((char*)p+o);fence();return v;}
void mmio_write32(void *p,size_t o,uint32_t v){fence();*(volatile uint32_t*)((char*)p+o)=v;fence();}
void mmio_fence(void){fence();}
'''


def require(condition,message):
    if not condition:raise ValueError(message)


def sha(path):
    require(stat.S_ISREG(path.stat().st_mode),f'input artifact must be a regular file: {path}')
    return hashlib.sha256(path.read_bytes()).hexdigest()

def dump(path,value):
    with path.open('xb') as handle:handle.write(canonical(value))


def check_build(build):
    manifest=read_json(build/'artifacts.sha256.json');addresses=read_json(build/'addresses.json')
    require(isinstance(manifest,dict) and manifest.get('schema')=='robot_rt.build.v1' and
            manifest.get('timing_passed') is True,'no successful final build timing manifest')
    files=manifest.get('files',{})
    for name in ('robot_rt.bit','robot_rt.bit.bin','robot_rt.xsa','addresses.json'):
        require(name in files and sha(build/name)==files[name],f'build artifact SHA mismatch: {name}')
    for key,value in EXPECTED.items():
        require(type(addresses.get(key)) is type(value) and addresses[key]==value,f'address/design contract mismatch: {key}')
    clock=addresses.get('clock_hz')
    require(type(clock) is int and 0<clock<(1<<31),'invalid nominal CLOCK_HZ')
    return {'manifest_sha256':sha(build/'artifacts.sha256.json'),'addresses':addresses,
            'timing_manifest_passed':True,'files_sha256':{n:files[n] for n in ('robot_rt.bit','robot_rt.bit.bin','robot_rt.xsa','addresses.json')}}


def rebase_input(data,origin,now,lead):
    """Synthetic test generation in the PL clock; no sensor/source retimestamp."""
    origin,now,lead=map(uint,(origin,now,lead))
    require(lead>0 and now>=origin,'explicit positive lead and current cycle>=origin required')
    cfg=DDSConfig(data['f_num'],data['f_den'],data['clock_hz'])
    first=(uint(now+lead)-origin)*cfg.f_num//cfg.threshold+1
    generated=dict(data,origin_cycle=origin,first_tick_index=uint(first),observation_cycle=now,
                   produced_cycle=now,validation_at_cycle=now,seed_cycle=now,
                   normal_deadline_cycle=uint(now+data['source_max_age_cycles']))
    return generated,build_bank(generated)


def check_input(data,build,lead,window):
    require(isinstance(data,dict) and data.get('source_kind')=='synthetic_stationary' and
            data.get('design_only') is True,'explicit design_only synthetic_stationary input required')
    require(data.get('clock_hz')==build['addresses']['clock_hz'],'input clock differs from compiled profile')
    require(data.get('previous_sequence')==0 and data.get('map_error_cycles')==0 and
            data.get('max_map_error_cycles')==0,'first-bank and explicit same-PL synthetic time only')
    require(all(v==0 for key in ('v','a') for v in data['initial_state'][key]),'stationary seed required')
    require(len(data['normal_jerks'])==2 and len(data['brake_jerks'])==2 and
            all(v==0 for row in data['normal_jerks']+data['brake_jerks'] for v in row),'exactly two zero-jerk normal and two brake records required')
    lead,window=uint(lead),uint(window)
    require(window>0,'explicit positive capture window required')
    generated,artifact=rebase_input(data,0,0,lead)
    cfg=DDSConfig(data['f_num'],data['f_den'],data['clock_hz'])
    require(window>cfg.tick_cycle(generated['first_tick_index']+3)+112,
            'capture window must exceed lead plus the complete stationary brake output')
    require(window<min(data['lease_cycles'],data['stop_timeout_cycles']),
            'capture window exceeds explicit lease/stop horizon')
    return artifact


class NativeMMIO:
    """AArch64 volatile LE32+dsb sy. Caller owns validated Device mapping."""
    def __init__(self,build,out,platform_sha,device_mapping):
        require(os.geteuid()==0,'--run requires target-board root')
        require(platform.system()=='Linux' and platform.machine()=='aarch64' and sys.byteorder=='little',
                'native MMIO is only supported on little-endian Linux/AArch64')
        require(platform_sha==build['manifest_sha256'] and device_mapping,
                'explicit matching PS-platform manifest SHA and Device MMIO mapping acknowledgement required')
        require(Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()=='operating',
                'fpga_manager is not operating; this script does not load a design')
        cc=shutil.which('cc')
        require(cc is not None,'existing cc required for reviewed volatile/barrier helper; no installation/fallback')
        source=out/'mmio32.c';source.write_text(HELPER);library=out/'mmio32.so'
        command=[cc,'-shared','-fPIC','-O2','-std=c11',str(source),'-o',str(library)]
        proc=subprocess.run(command,capture_output=True,text=True,timeout=30)
        (out/'helper_compile.stdout.log').write_text(proc.stdout);(out/'helper_compile.stderr.log').write_text(proc.stderr)
        dump(out/'helper_compile.json',dict(argv=command,returncode=proc.returncode,source_sha256=sha(source),
            library_sha256=sha(library) if proc.returncode==0 else None))
        require(proc.returncode==0,'MMIO helper compile failed; no device opened')
        self.lib=ctypes.CDLL(str(library));self.lib.mmio_read32.argtypes=[ctypes.c_void_p,ctypes.c_size_t]
        self.lib.mmio_read32.restype=ctypes.c_uint32
        self.lib.mmio_write32.argtypes=[ctypes.c_void_p,ctypes.c_size_t,ctypes.c_uint32]
        self.lib.mmio_write32.restype=None;self.lib.mmio_fence.argtypes=[];self.lib.mmio_fence.restype=None
        self.maps={};self.fd=None;self.sizes={'cfg':4096,'ram':131072}
        try:
            self.fd=os.open('/dev/mem',os.O_RDWR|os.O_SYNC)
            for name,field in (('cfg','cfg_base'),('ram','chunk_base')):
                self.maps[name]=mmap.mmap(self.fd,self.sizes[name],flags=mmap.MAP_SHARED,
                    prot=mmap.PROT_READ|mmap.PROT_WRITE,offset=build['addresses'][field])
        except BaseException:
            self.close();raise
    def _ptr(self,region,offset):
        require(region in self.maps and type(offset) is int and offset%4==0 and
                0<=offset<=self.sizes[region]-4,'MMIO aligned-window guard')
        return ctypes.addressof(ctypes.c_uint32.from_buffer(self.maps[region])),offset
    def read32(self,region,offset):return int(self.lib.mmio_read32(*self._ptr(region,offset)))
    def write32(self,region,offset,value):self.lib.mmio_write32(*self._ptr(region,offset),uint(value,32))
    def barrier(self):self.lib.mmio_fence()
    def close(self):
        for view in self.maps.values():view.close()
        self.maps={}
        if self.fd is not None:os.close(self.fd);self.fd=None


class Session:
    def __init__(self,io,timeout_s,sleep=time.sleep):
        require(type(timeout_s) in (int,float) and 0<timeout_s<=30,'poll timeout must be in (0,30] seconds')
        self.io=io;self.timeout=timeout_s;self.sleep=sleep;self.receipts=[]
    def r(self,address):return uint(self.io.read32('cfg',address),32)
    def w(self,address,value):self.io.write32('cfg',address,uint(value,32))
    def u64(self,address):
        for _ in range(16):
            hi=self.r(address+4);lo=self.r(address);hi2=self.r(address+4)
            if hi==hi2:return (hi<<32)|lo
        raise ValueError('torn live uint64 register')
    def cycle(self):return self.u64(0x20)
    def status(self):return {'STATE':self.r(12),'STATUS':self.r(20),'FAULT_CODE':self.r(24),'REJECT_CODE':self.r(28)}
    def wait(self,predicate,label):
        deadline=time.monotonic()+self.timeout
        while True:
            value=predicate()
            if value:return value
            require(time.monotonic()<deadline,f'timeout: {label}')
            self.sleep(0.001)
    def state(self,state,status,allow_fault=False):
        def poll():
            value=self.status()
            require(allow_fault or value['FAULT_CODE']==0 and value['REJECT_CODE']==0,'PL fault/reject during smoke')
            return value if value['STATE']==state and value['STATUS']==status else None
        return poll
    def cmd(self,name,state,status):
        before=self.cycle();self.w(16,CMDS[name]);self.io.barrier()
        rb=self.wait(self.state(state,status,allow_fault=name=='DISARM'),name);after=self.cycle()
        require(before<after,'non-increasing PL cycle command interval')
        receipt=dict(kind='mmio_readback',command=name,address=16,value=CMDS[name],wstrb=15,
            write_completed=True,access_width_bits=32,readback=rb,cycle_before=before,cycle_after=after)
        self.receipts.append(receipt);return receipt
    def writes(self,writes):
        for address,value in writes:self.w(address,value)
        self.io.barrier()
        for address,value in writes:require(self.r(address)==value,'shadow/mailbox write readback mismatch')
    def mailbox(self,address,value):self.writes([(address,value&0xffffffff),(address+4,value>>32)])
    def snapshot(self,name):
        before=self.cycle();self.w(16,CMDS['SNAPSHOT']);self.io.barrier()
        words={key:self.r(int(key,16)) for key in register_words()}
        rb=self.status();after=self.cycle()
        ack=dict(kind='mmio_readback',command='SNAPSHOT',address=16,value=16,wstrb=15,write_completed=True,access_width_bits=32,
                 readback=rb,cycle_before=before,cycle_after=after)
        return dict(name=name,live_cycle_before=before,command_ack=ack,words=words,live_cycle_after=after)


def run_transactions(io,data,build,lead,window,out,timeout_s=2,sleep=time.sleep,*,hardware_access=False):
    """Injectable transport; tests provide only fake memory. Never loads PL."""
    require(out.is_dir(),'fresh output directory must already exist')
    for name in ('generated_stage2_input.json','bank0.bin','configuration_readback.json','plan.json','capture.json','result.json','execution.json'):
        require(not (out/name).exists(),f'existing output cannot be overwritten: {name}')
    stage1=check_input(data,build,lead,window)
    session=Session(io,timeout_s,sleep);started=False;capture=None;failure=None;result=None
    status=session.status()
    require(session.r(0)==EXPECTED['expected_id'] and session.r(4)==0x10000 and session.r(8)==1 and
            session.u64(0xa8)==data['clock_hz'] and status==dict(STATE=0,STATUS=0,FAULT_CODE=0,REJECT_CODE=0),
            'DESC_ONLY identity/version/clock/idle status mismatch; no writes made')
    try:
        started=True
        # Static CRC includes 28 reserved words that the current sparse RTL
        # ties internally to zero. They have no host register mapping: even
        # writing zero (or reading them) returns SLVERR on the real board.
        static=[(a,v) for a,v in stage1.register_writes if not 0x400<=a<0x44c]
        dump(out/'static_config_writes.json',dict(writes=static,source_kind='synthetic_stationary',seed_excluded=True))
        session.writes(static)
        config=session.cmd('CONFIG_COMMIT',0,256)
        origin=session.u64(0xa0);now=session.cycle()
        require(config['cycle_before']<origin<config['cycle_after'] and now>=origin,'CONFIG origin/cycle inconsistent')
        generated,artifact=rebase_input(data,origin,now,lead)
        require(artifact.report['config_crc32']==stage1.report['config_crc32'],'rebasing changed static configuration CRC')
        dump(out/'generated_stage2_input.json',generated)
        with (out/'bank0.bin').open('xb') as handle:handle.write(artifact.image)
        dump(out/'configuration_readback.json',dict(origin_cycle=origin,current_cycle=now,
            config_crc32=artifact.report['config_crc32'],source_kind='synthetic_stationary',
            note='Locally generated stationary test data in PL time; not measured robot feedback'))
        session.writes([(a,v) for a,v in artifact.register_writes if 0x400<=a<0x44c])
        used=256*(len(generated['normal_jerks'])+len(generated['brake_jerks']))
        for start,end in ((256,256+used),(0,256)):
            for address in range(start,end,4):io.write32('ram',address,struct.unpack_from('<I',artifact.image,address)[0])
        io.barrier()
        for address in range(0,256+used,4):
            require(io.read32('ram',address)==struct.unpack_from('<I',artifact.image,address)[0],'bank readback mismatch')
        session.mailbox(0x80,generated['bank_sequence']);session.cmd('COMMIT_BANK0',0,258)
        cfg=DDSConfig(data['f_num'],data['f_den'],data['clock_hz'])
        first_cycle=origin+cfg.tick_cycle(generated['first_tick_index'])
        require(session.cycle()<first_cycle,'bank admission missed first tick; no ARM')
        session.mailbox(0x98,1);session.cmd('KICK',0,258)
        session.mailbox(0x90,generated['first_tick_index']);arm=session.cmd('ARM',1,259)
        session.cmd('STOP',1,259);begin=session.snapshot('begin')
        dump(out/'begin_snapshot.json',begin)
        begin_cycle=begin['words']['0x500']|(begin['words']['0x504']<<32)
        end_target=uint(begin_cycle+window)
        require(end_target<min(now+data['lease_cycles'],now+data['stop_timeout_cycles']),
                'actual capture window no longer fits explicit lease/stop horizon')
        def reached_end():
            rb=session.status()
            require(rb['FAULT_CODE']==0 and rb['REJECT_CODE']==0,'PL fault/reject during capture window')
            return session.cycle()>=end_target
        session.wait(reached_end,'capture cycle window')
        end=session.snapshot('end')
        dump(out/'end_snapshot.json',end)
        end_cycle=end['words']['0x500']|(end['words']['0x504']<<32)
        plan,_=make_plan(generated,arm_by_cycle=arm['cycle_after'],begin_cycle=begin_cycle,
                         end_cycle=end_cycle,seed_after_config=True)
        capture=dict(schema=CAPTURE_SCHEMA,plan_id=plan['plan_id'],source_kind='external_capture' if hardware_access else 'synthetic',
            capture_source=('AArch64 volatile LE32+dsb sy MMIO' if hardware_access else 'fake-MMIO synthetic test')+' completion/readback; AXI BRESP NOT observed',
            run_id=out.name,source_label='synthetic_stationary',control_receipts=session.receipts[:5],snapshots=[begin,end])
        dump(out/'plan.json',plan)
    except BaseException as exc:
        failure=exc
    finally:
        if started:
            try:
                cleanup=session.cmd('DISARM',0,0)
                if capture is not None:capture['cleanup']=dict(command_ack=cleanup,state_after=0,status_after=0)
            except BaseException as cleanup_error:
                if failure is None:failure=cleanup_error
        if capture is not None:dump(out/'capture.json',capture)
    if failure is None:
        try:result=verify_capture(plan,capture)
        except BaseException as exc:failure=exc
    dump(out/'execution.json',dict(status='failed' if failure else 'capture_consistent',
        error=None if failure is None else str(failure),receipts=session.receipts,
        source_kind='synthetic_stationary',axi_bresp_observed=False,hardware_access=hardware_access,
        scope='DESC_ONLY MMIO only; not CAN or motion'))
    if failure is not None:raise failure
    result.update(source_label='synthetic_stationary',build_manifest_sha256=build['manifest_sha256'],hardware_access=hardware_access,
        tool_source_sha256={name:sha(ROOT/name) for name in ('sw/board_desc_smoke.py','sw/desc_smoke.py','sw/pl_bank.py','spec/regmap.yaml')})
    dump(out/'result.json',result)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    mode=parser.add_mutually_exclusive_group();mode.add_argument('--check',action='store_true');mode.add_argument('--run',action='store_true')
    parser.add_argument('--build',type=Path,required=True);parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--lead-cycles',type=int,required=True);parser.add_argument('--window-cycles',type=int,required=True)
    parser.add_argument('--output-dir',type=Path,required=True);parser.add_argument('--poll-timeout-s',type=float,default=2.0)
    parser.add_argument('--platform-manifest-sha256');parser.add_argument('--device-mmio-mapping',action='store_true')
    args=parser.parse_args(argv)
    try:
        build=check_build(args.build);data=read_json(args.input)
        artifact=check_input(data,build,args.lead_cycles,args.window_cycles)
        require(type(args.poll_timeout_s) is float and 0<args.poll_timeout_s<=30,'invalid bounded poll timeout')
        args.output_dir.mkdir(parents=True,exist_ok=False)
        dump(args.output_dir/'input.json',data);dump(args.output_dir/'checked_build.json',build)
        if not args.run:
            result=dict(status='offline_check_only',hardware_access=False,source_label='synthetic_stationary',
                axi_bresp_observed=False,board_acceptance=False,physical_f_exec_hz=None,
                static_config_crc32=artifact.report['config_crc32'],build_manifest_sha256=build['manifest_sha256'],
                stages=['static CONFIG exactly once','read ACTUAL origin/cycle','rebase synthetic seed/bank only',
                        'READY/KICK/ARM/STOP','two fresh SNAPSHOT','DISARM'],
                tool_source_sha256={name:sha(ROOT/name) for name in ('sw/board_desc_smoke.py','sw/desc_smoke.py','sw/pl_bank.py','spec/regmap.yaml')},
                native_backend='Requires existing cc and little-endian Linux/AArch64 Device mapping only on explicit --run')
            dump(args.output_dir/'check.json',result)
        else:
            io=NativeMMIO(build,args.output_dir,args.platform_manifest_sha256,args.device_mmio_mapping)
            try:result=run_transactions(io,data,build,args.lead_cycles,args.window_cycles,args.output_dir,args.poll_timeout_s,hardware_access=True)
            finally:io.close()
        print(json.dumps(result,indent=2));return 0
    except (ValueError,TypeError,KeyError,OSError,subprocess.SubprocessError) as exc:
        parser.exit(2,f'DESC_ONLY board smoke: {exc}\n')


if __name__=='__main__':raise SystemExit(main())
