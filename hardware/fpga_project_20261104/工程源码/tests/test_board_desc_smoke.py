"""Fake MMIO only; no native helper compilation, /dev/mem or hardware calls."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from sw.board_desc_smoke import (EXPECTED,Session,check_build,check_input,run_transactions)
from sw.desc_smoke import make_plan,verify_capture
from sw.pl_bank import example

CLOCK=99999001
LEAD=5000000
WINDOW=10000000


def input_data():
    data=example(275,2,0,8,clock_hz=CLOCK)
    data['source_kind']='synthetic_stationary'
    data.update(lease_cycles=50000000,source_max_age_cycles=50000000,stop_timeout_cycles=50000000)
    return data


def build_files(path):
    addresses=dict(EXPECTED,clock_hz=CLOCK)
    (path/'addresses.json').write_text(json.dumps(addresses))
    for name in ('robot_rt.bit','robot_rt.bit.bin','robot_rt.xsa'):(path/name).write_bytes(b'SYNTHETIC TEST ONLY; NOT A REAL BITSTREAM')
    files={name:hashlib.sha256((path/name).read_bytes()).hexdigest() for name in
           ('robot_rt.bit','robot_rt.bit.bin','robot_rt.xsa','addresses.json')}
    (path/'artifacts.sha256.json').write_text(json.dumps(dict(schema='robot_rt.build.v1',timing_passed=True,files=files)))
    return check_build(path)


class FakeMMIO:
    """Independent register/bank mock, with raw descriptor bytes and CRC checks.

    Read advances invented PL cycles. No actual software/RTL timing is modeled.
    """
    def __init__(self,*,bad_id=False,reject_commit=False,fault_arm=False,torn=False):
        self.reg=[0]*1024;self.ram=bytearray(131072);self.now=100000;self.origin=None
        self.events=[];self.armed=False;self.stopped=False;self.reject_commit=reject_commit
        self.fault_arm=fault_arm;self.torn=torn;self.high_reads=0
        for a,v in ((0,0 if bad_id else 0x5254504c),(4,0x10000),(8,1),(0xa8,CLOCK)):
            self.reg[a//4]=v
        # Match the shipped sparse RTL: reserved CRC words are internal
        # constants, and their addresses reject both host reads and writes.
        self.unmapped={0x100+axis*64+off for axis in range(7) for off in range(48,64,4)}
    def _put64(self,address,value):
        self.reg[address//4]=value&0xffffffff;self.reg[address//4+1]=value>>32
    def _u64(self,address):return self.reg[address//4]|(self.reg[address//4+1]<<32)
    def _tick_cycle(self,index):return self.origin+(index*CLOCK*2+274)//275
    def _update_state(self):
        if self.armed:
            if self.now>=self._tick_cycle(self.first+3)+112:self.reg[3]=4
            elif self.now>=self._tick_cycle(self.first+1):self.reg[3]=3
            elif self.now>=self._tick_cycle(self.first):self.reg[3]=2
    def read32(self,region,address):
        assert type(address) is int and address%4==0 and 0<=address<(4096 if region=='cfg' else 131072)
        assert region!='cfg' or address not in self.unmapped,'SLVERR: unmapped reserved register read'
        self.now+=1000;self._update_state()
        if region=='ram':value=struct.unpack_from('<I',self.ram,address)[0]
        elif address==0x20:value=self.now&0xffffffff
        elif address==0x24:
            self.high_reads+=1;value=(self.high_reads&1) if self.torn else self.now>>32
        elif address==0x28:value=0 if self.origin is None else (self.now-self.origin)*275//(CLOCK*2)
        elif address==0x2c:value=0
        else:value=self.reg[address//4]
        self.events.append(('read',region,address,value,self.now));return value
    def write32(self,region,address,value):
        assert type(value) is int and 0<=value<=0xffffffff and address%4==0
        assert region!='cfg' or address not in self.unmapped,'SLVERR: unmapped reserved register write'
        self.now+=100;self._update_state();self.events.append(('write',region,address,value,self.now))
        if region=='ram':
            assert self.reg[5] in (0,256),'write to frozen bank'
            struct.pack_into('<I',self.ram,address,value);return
        if address!=16:self.reg[address//4]=value;return
        if value==1:
            assert self.reg[5]==0
            raw=bytearray(struct.pack('<1024I',*self.reg));crc=self.reg[0x70//4];raw[0x70:0x74]=bytes(4)
            assert zlib.crc32(raw[0x30:0x74]+raw[0x100:0x2c0]+raw[0x460:0x468])==crc
            self.origin=self.now;self._put64(0xa0,self.origin);self.reg[5]=256
        elif value==32:
            assert self.reg[5]==256
            header=bytearray(self.ram[:256]);crc=struct.unpack_from('<I',header,100)[0];header[100:104]=bytes(4)
            assert zlib.crc32(header)==crc
            assert zlib.crc32(self.ram[256:1280])==struct.unpack_from('<I',header,96)[0]
            raw=struct.pack('<1024I',*self.reg)
            assert zlib.crc32(raw[0x400:0x448])==self.reg[0x448//4]
            self.first=struct.unpack_from('<Q',header,72)[0]
            self.reg[5]=260 if self.reject_commit else 258
            if self.reject_commit:self.reg[7]=3
        elif value==256:assert self.reg[5]==258 and self._u64(0x98)==1
        elif value==2:
            assert self._u64(0x90)==self.first and self.now<self._tick_cycle(self.first)
            assert self._u64(0x400)<=self.now
            self.armed=True;self.reg[3]=1;self.reg[5]=259
            if self.fault_arm:self.reg[6]=1;self.reg[3]=5
        elif value==4:self.stopped=True
        elif value==16:self._snapshot()
        elif value==128:self.armed=False;self.reg[3]=0;self.reg[5]=0
        else:raise AssertionError('unplanned CMD')
    def barrier(self):self.events.append(('barrier',None,None,None,self.now))
    def _snapshot(self):
        cycle=self.now;ticks=(cycle-self.origin)*275//(CLOCK*2)
        emitted=max(0,(cycle-112-self.origin)*275//(CLOCK*2)-self.first+1)
        counts=[cycle,ticks,emitted,0,0,0 if self.stopped else 1,0,112 if emitted else 0,
                112 if emitted else 0,emitted,0]
        for i,value in enumerate(counts):self._put64(0x500+8*i,value)
        raw=bytes(96)
        if emitted:
            tick=self.first+emitted-1;flag=0 if emitted<=2 else 1 if emitted<=4 else 2
            raw=struct.pack('<QQQII',emitted,self._tick_cycle(tick),tick,1,flag)
            for ident,payload in ((0x155,bytes(8)),(0x156,bytes(8)),(0x157,bytes(8)),
                                  (0x159,struct.pack('>iHBB',50003,1000,1,0))):
                raw+=struct.pack('<HBB8sI',ident,8,0,payload,0)
        for i,(value,) in enumerate(struct.iter_unpack('<I',raw)):self.reg[(0x600+4*i)//4]=value


class BoardSmokeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='fake_desc_board_');self.path=Path(self.tmp.name)
        self.build=self.path/'build';self.build.mkdir();self.info=build_files(self.build)
        self.data=input_data()
    def tearDown(self):self.tmp.cleanup()
    def execute(self,io,name='run'):
        out=self.path/name;out.mkdir()
        result=run_transactions(io,self.data,self.info,LEAD,WINDOW,out,sleep=lambda _:None)
        return out,result
    def test_actual_origin_rebase_single_config_seed_late_and_full_capture(self):
        io=FakeMMIO();out,result=self.execute(io)
        commands=[e[3] for e in io.events if e[:3]==('write','cfg',16)]
        self.assertEqual(commands,[1,32,256,2,4,16,16,128])
        config_index=next(i for i,e in enumerate(io.events) if e[:4]==('write','cfg',16,1))
        seed_writes=[i for i,e in enumerate(io.events) if e[0]=='write' and e[1]=='cfg' and 0x400<=e[2]<0x44c]
        self.assertTrue(seed_writes and min(seed_writes)>config_index)
        generated=json.loads((out/'generated_stage2_input.json').read_text())
        self.assertEqual(generated['origin_cycle'],io.origin)
        self.assertGreater(generated['seed_cycle'],io.origin)
        self.assertEqual(generated['observation_cycle'],generated['seed_cycle'])
        self.assertEqual(generated['source_kind'],'synthetic_stationary')
        self.assertFalse(result['axi_bresp_observed']);self.assertFalse(result['hardware_access'])
        self.assertFalse(result['latency_check']['independent_measurement'])
        self.assertFalse(result['board_acceptance']);self.assertIsNone(result['physical_f_exec_hz'])
        capture=json.loads((out/'capture.json').read_text())
        for ack in capture['control_receipts']+[s['command_ack'] for s in capture['snapshots']]+[capture['cleanup']['command_ack']]:
            self.assertEqual(ack['kind'],'mmio_readback');self.assertNotIn('bresp',ack)
        self.assertEqual(io.reg[5],0)
    def test_ram_payload_header_barrier_readback_and_freeze_order(self):
        io=FakeMMIO();self.execute(io)
        writes=[e[2] for e in io.events if e[0]=='write' and e[1]=='ram']
        self.assertEqual(writes,list(range(256,1280,4))+list(range(0,256,4)))
        last_ram=max(i for i,e in enumerate(io.events) if e[0]=='write' and e[1]=='ram')
        commit=next(i for i,e in enumerate(io.events) if e[:4]==('write','cfg',16,32))
        middle=io.events[last_ram+1:commit]
        self.assertTrue(any(e[0]=='barrier' for e in middle))
        self.assertEqual({e[2] for e in middle if e[:2]==('read','ram')},set(range(0,1280,4)))
    def test_bad_identity_never_writes_and_rejection_or_fault_always_disarms(self):
        for kwargs in ({'bad_id':True},{'reject_commit':True},{'fault_arm':True}):
            io=FakeMMIO(**kwargs);out=self.path/next(iter(kwargs));out.mkdir()
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                run_transactions(io,self.data,self.info,LEAD,WINDOW,out,sleep=lambda _:None)
            writes=[e for e in io.events if e[0]=='write']
            if 'bad_id' in kwargs:self.assertEqual(writes,[])
            else:
                self.assertEqual([e[3] for e in writes if e[1:3]==('cfg',16)][-1],128)
                self.assertEqual(io.reg[3],0);self.assertEqual(io.reg[5],0)
                self.assertEqual(json.loads((out/'execution.json').read_text())['status'],'failed')
    def test_mmio_cannot_fabricate_bresp_and_raw_axi_stays_compatible(self):
        io=FakeMMIO();out,_=self.execute(io)
        plan=json.loads((out/'plan.json').read_text());capture=json.loads((out/'capture.json').read_text())
        capture['control_receipts'][0]['bresp']=0
        with self.assertRaisesRegex(ValueError,'fabricate'):verify_capture(plan,capture)
        capture=json.loads((out/'capture.json').read_text())
        capture['control_receipts'][0]['readback']['REJECT_CODE']=3
        with self.assertRaisesRegex(ValueError,'fault/reject'):verify_capture(plan,capture)
        capture['control_receipts'][0]['readback']['REJECT_CODE']=False
        with self.assertRaisesRegex(ValueError,'fault/reject'):verify_capture(plan,capture)
        # Import only the independent old fixture function, not its tests.
        source=ROOT/'tests/test_desc_smoke.py'
        spec=importlib.util.spec_from_file_location('raw_fixture',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        legacy,_=make_plan(example(275,2,1000,8),arm_by_cycle=2000,begin_cycle=2200,end_cycle=9000000)
        result=verify_capture(legacy,module.synthetic_capture(legacy))
        self.assertTrue(result['axi_bresp_observed'])
    def test_invalid_input_build_and_existing_results_refuse_before_io(self):
        for change in ('moving','unknown_source','clock','frequency','short_window'):
            data=copy.deepcopy(self.data)
            if change=='moving':data['normal_jerks'][0][0]=1
            if change=='unknown_source':data['source_kind']='robot_feedback'
            if change=='clock':data['clock_hz']=100000000
            if change=='frequency':data.pop('f_num')
            with self.subTest(change=change),self.assertRaises((ValueError,KeyError)):
                check_input(data,self.info,LEAD,1 if change=='short_window' else WINDOW)
        (self.build/'robot_rt.bit.bin').write_bytes(b'changed')
        with self.assertRaises(ValueError):check_build(self.build)
        io=FakeMMIO();out=self.path/'existing';out.mkdir();(out/'result.json').write_text('retain')
        with self.assertRaises(ValueError):run_transactions(io,self.data,self.info,LEAD,WINDOW,out)
        self.assertEqual(io.events,[]);self.assertEqual((out/'result.json').read_text(),'retain')
    def test_torn_live_counter_is_bounded(self):
        io=FakeMMIO(torn=True)
        with self.assertRaisesRegex(ValueError,'torn'):Session(io,2).cycle()
        self.assertEqual(io.high_reads,32)
    def test_default_check_cli_is_pure_and_fresh_without_native_compile(self):
        source=self.path/'input.json';source.write_text(json.dumps(self.data));out=self.path/'check'
        args=[sys.executable,str(ROOT/'sw/board_desc_smoke.py'),'--build',str(self.build),'--input',str(source),
              '--lead-cycles',str(LEAD),'--window-cycles',str(WINDOW),'--output-dir',str(out)]
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
        proc=subprocess.run(args,capture_output=True,text=True,env=env,timeout=10)
        self.assertEqual(proc.returncode,0,proc.stderr)
        report=json.loads((out/'check.json').read_text());self.assertFalse(report['hardware_access'])
        self.assertFalse((out/'mmio32.c').exists());self.assertFalse((out/'mmio32.so').exists())
        before={p.name:p.read_bytes() for p in out.iterdir()}
        proc=subprocess.run(args,capture_output=True,text=True,env=env,timeout=10)
        self.assertNotEqual(proc.returncode,0)
        self.assertEqual(before,{p.name:p.read_bytes() for p in out.iterdir()})
        proc=subprocess.run([sys.executable,str(ROOT/'sw/board_desc_smoke.py'),'--help'],capture_output=True,text=True,env=env,timeout=10)
        self.assertEqual(proc.returncode,0);self.assertIn('--platform-manifest-sha256',proc.stdout)


if __name__=='__main__':unittest.main()
