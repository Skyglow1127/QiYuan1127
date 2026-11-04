import copy
import json
from pathlib import Path
import struct
import sys
import unittest
import zlib

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sw.pl_bank import build_bank,example


class BankPackingTests(unittest.TestCase):
    def setUp(self):
        self.data=example(275,2,0,8)

    def test_layout_header_and_payload_crcs_cover_exact_used_bytes(self):
        result=build_bank(self.data);image=result.image
        self.assertEqual(len(image),65536)
        self.assertEqual(image[:16],b'RTBK'+struct.pack('<III',1,256,256))
        header=bytearray(image[:256]);stored=struct.unpack_from('<I',header,100)[0]
        header[100:104]=bytes(4)
        self.assertEqual(zlib.crc32(header),stored)
        self.assertEqual(zlib.crc32(image[256:256+4*256]),struct.unpack_from('<I',image,96)[0])
        self.assertEqual(image[256+4*256:],bytes(65536-256-4*256))
        mutated=bytearray(image);mutated[256+40]^=1
        self.assertNotEqual(zlib.crc32(mutated[256:256+4*256]),struct.unpack_from('<I',image,96)[0])

    def test_static_config_crc_and_no_control_commands(self):
        result=build_bank(self.data);cfg=bytearray(result.config)
        crc=struct.unpack_from('<I',cfg,0x70)[0];cfg[0x70:0x74]=bytes(4)
        self.assertEqual(crc,zlib.crc32(cfg[0x30:0x74]+cfg[0x100:0x2c0]+cfg[0x460:0x468]))
        self.assertEqual(crc,0x4196f834)
        addresses={address for address,_ in result.register_writes}
        self.assertNotIn(0x10,addresses) # No CMD/ARM/KICK write in artifacts.
        self.assertNotIn(0x98,addresses)
        self.assertEqual(struct.unpack_from('<I',cfg,0x448)[0],zlib.crc32(cfg[0x400:0x448]))

    def test_noninteger_ticks_and_origin_preserved(self):
        result=build_bank(self.data)
        intervals=[]
        for i in range(4):
            start,end,index,kind=struct.unpack_from('<QQQI',result.image,256+i*256)
            self.assertEqual(index,8+i);self.assertEqual(kind,1 if i<2 else 2)
            intervals.append(end-start)
        self.assertTrue(set(intervals)<={727272,727273})
        shifted=copy.deepcopy(self.data)
        for key in ('origin_cycle','observation_cycle','produced_cycle','validation_at_cycle','normal_deadline_cycle','seed_cycle'):
            shifted[key]+=1000
        image=build_bank(shifted).image
        self.assertEqual(struct.unpack_from('<Q',image,256)[0],struct.unpack_from('<Q',result.image,256)[0]+1000)

    def test_actual_ps_nominal_clock_controls_record_timing(self):
        clock=99_999_001
        data=example(275,2,0,8,clock_hz=clock)
        result=build_bank(data)
        self.assertEqual(result.report['clock_hz'],clock)
        for i in range(4):
            start,end,index=struct.unpack_from('<QQQ',result.image,256+i*256)
            self.assertEqual(start,((index-1)*clock*2+274)//275)
            self.assertEqual(end,(index*clock*2+274)//275)
        self.assertNotEqual(result.image,build_bank(self.data).image)
        self.assertNotIn(0xa8,{address for address,_ in result.register_writes})

    def test_invalid_clock_rejected(self):
        for clock in (0,-1,True,2**31):
            data=copy.deepcopy(self.data);data['clock_hz']=clock
            with self.subTest(clock=clock),self.assertRaises(ValueError):build_bank(data)

    def test_missing_or_nonzero_stop_tail_rejected(self):
        for mutate in ('none','moving','capacity'):
            data=copy.deepcopy(self.data)
            if mutate=='none':data['brake_jerks']=[]
            elif mutate=='moving':data['brake_jerks'][-1][0]=1
            else:data['normal_jerks']=[[0]*7]*255
            with self.subTest(mutate=mutate),self.assertRaises(ValueError):build_bank(data)

    def test_expiry_mapping_and_invalid_session_rejected(self):
        for key,value in (('normal_deadline_cycle',100),('produced_cycle',1),('session_nonce',0),('map_error_cycles',1),('lease_cycles',1<<63)):
            data=copy.deepcopy(self.data);data[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):build_bank(data)

    def test_no_implicit_gripper_enable_or_hardware_mode(self):
        for key,value in (('gripper_explicit_reviewed',False),('gripper_set_zero',1),('gripper_force_mn',0),('design_only',False)):
            data=copy.deepcopy(self.data);data[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):build_bank(data)

    def test_mapping_interval_cannot_cross_negative_time_or_uint64_wrap(self):
        data=copy.deepcopy(self.data)
        data.update(map_error_cycles=1,max_map_error_cycles=1,produced_cycle=2,validation_at_cycle=3,
                    normal_deadline_cycle=199999999)
        with self.assertRaisesRegex(ValueError,'negative source time'):build_bank(data)
        data=copy.deepcopy(self.data)
        for key in ('origin_cycle','observation_cycle','produced_cycle','validation_at_cycle','seed_cycle'):
            data[key]=(1<<64)-100
        data['normal_deadline_cycle']=(1<<64)-1
        with self.assertRaises(ValueError):build_bank(data)


if __name__=='__main__':unittest.main()
