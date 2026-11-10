import csv
import json
from pathlib import Path
import tempfile
import unittest
from recorder import Journal

class JournalTests(unittest.TestCase):
    def test_time_velocity_gap_and_stop(self):
        with tempfile.TemporaryDirectory() as temp:
            now = [100.0]
            j = Journal(Path(temp)/'test', clock=lambda:now[0])
            def feed(t,q):
                now[0]=t
                j.feed({'received_monotonic_s':t,'message':{'address':2,'funcode':1,
                        'data':{f'angle{i}':v for i,v in enumerate(q)}}})
            feed(100.0,[0.,1.,2.,3.,4.,5.,20.])
            feed(100.1,[1.,2.,3.,4.,5.,6.,21.])
            self.assertAlmostEqual(j.previous['velocity'][0],10.)
            feed(100.1,[9.]*7)
            self.assertEqual(j.nonmonotonic_count,1)
            feed(100.2,[0.]*6+[float('nan')])
            self.assertEqual(j.invalid_count,1)
            feed(101.,[2.]*7)
            self.assertTrue(j.previous['gap_before'])
            self.assertEqual(j.previous['velocity'],[None]*7)
            self.assertEqual(j.sample_count,3)
            j.stop('unit_test_end')
            feed(101.1,[3.]*7)
            self.assertEqual(j.sample_count,3)
            self.assertEqual(j.stop('again')['state'],'stopped')
            with (j.path/'joint_timeline.csv').open() as f:
                rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),3)
            self.assertEqual(json.loads((j.path/'summary.json').read_text())['robot_commands_sent'],0)
            self.assertEqual(json.loads((j.path/'summary.json').read_text())['replay_executed'],False)

if __name__=='__main__': unittest.main()
