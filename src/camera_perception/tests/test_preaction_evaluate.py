"""Evaluation reports must describe the saved training data, including horizon."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/preaction'))
from evaluate import evaluation_metadata,sha


class EvaluationMetadataTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out=Path(self.temp.name)
        self.pairs=[dict(id='a',split='train',stage='approach',episode='one',horizon_s=8.),
                    dict(id='b',split='test',stage='lift',episode='one',horizon_s=8.),
                    dict(id='c',split='test',stage='retreat',episode='two',horizon_s=8.)]
        self.config=dict(train_pairs=1,val_pairs=0,test_pairs=2,input_shape=[1,19,96,96])
        self.save_metadata()
        self.data={'id':np.array(['a','b','c']),'split':np.array(['train','test','test']),
                   'x':np.zeros((3,19,96,96),np.float32)}
        self.data['x'][:,18]=4.

    def save_metadata(self):
        path=self.out/'pairs.json'
        path.write_text(json.dumps({'pairs':self.pairs}))
        self.config['pairs_sha256']=sha(path)
        (self.out/'train_config.json').write_text(json.dumps(self.config))

    def test_report_uses_real_count_stages_episodes_and_horizons(self):
        result=evaluation_metadata(self.out,self.data,self.pairs,self.config)
        self.assertEqual(result['trained_horizons_s'],[8.])
        self.assertEqual(result['test_scope']['samples'],2)
        self.assertEqual(result['test_scope']['stages'],['lift','retreat'])
        self.assertEqual(result['test_scope']['episodes'],['one','two'])
        self.assertEqual(result['test_scope']['horizons_s'],[8.])

    def test_reordered_data_rows_cannot_receive_wrong_pair_metadata(self):
        self.data['id']=self.data['id'][::-1]
        with self.assertRaisesRegex(ValueError,'row IDs/splits'):
            evaluation_metadata(self.out,self.data,self.pairs,self.config)

    def test_tensor_must_contain_the_recorded_horizon(self):
        self.data['x'][1,18]=1.
        with self.assertRaisesRegex(ValueError,'horizon channel'):
            evaluation_metadata(self.out,self.data,self.pairs,self.config)

    def test_untrained_evaluation_horizon_is_rejected(self):
        self.pairs[1]['horizon_s']=2.
        self.save_metadata()
        with self.assertRaisesRegex(ValueError,'not represented'):
            evaluation_metadata(self.out,self.data,self.pairs,self.config)

    def test_config_count_must_match_evaluated_rows(self):
        self.config['test_pairs']=5
        with self.assertRaisesRegex(ValueError,'split counts'):
            evaluation_metadata(self.out,self.data,self.pairs,self.config)


if __name__=='__main__':
    unittest.main()
