"""Reject unsupported or misattributed forecast horizons before inference."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/preaction'))
from predict import sha,validate_horizon,validate_npu_model


class ForecastContractTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run=Path(self.temp.name)

    def training_metadata(self,horizon):
        path=self.run/'pairs.json'
        path.write_text(json.dumps({'pairs':[
            {'split':'train','horizon_s':horizon,'label_actual_horizon_s':horizon-.05},
            {'split':'val','horizon_s':3.}]}))
        return {'pairs_sha256':sha(path),'train_pairs':1}

    def test_old_two_second_and_eight_second_runs_use_recorded_requested_horizon(self):
        for horizon in (2.,8.):
            with self.subTest(horizon=horizon):
                config=self.training_metadata(horizon)
                actual,evidence=validate_horizon(self.run,config,{'horizon_s':horizon})
                self.assertEqual(actual,horizon)
                self.assertEqual(evidence['trained_horizons_s'],[horizon])

    def test_validation_only_horizon_is_not_a_trained_horizon(self):
        config=self.training_metadata(8.)
        for horizon in (2.,3.,7.95):
            with self.subTest(horizon=horizon),self.assertRaisesRegex(ValueError,'was not trained'):
                validate_horizon(self.run,config,{'horizon_s':horizon})

    def test_command_horizon_must_be_a_finite_positive_number(self):
        config=self.training_metadata(8.)
        for horizon in (True,'8',0,-8,float('nan'),float('inf')):
            with self.subTest(horizon=horizon),self.assertRaisesRegex(ValueError,'finite positive'):
                validate_horizon(self.run,config,{'horizon_s':horizon})

    def test_modified_manifest_cannot_relabel_checkpoint_horizon(self):
        config=self.training_metadata(2.)
        self.training_metadata(8.)
        with self.assertRaisesRegex(ValueError,'SHA256 mismatch'):
            validate_horizon(self.run,config,{'horizon_s':8.})

    def test_missing_metadata_does_not_silently_default_to_two_seconds(self):
        config=self.training_metadata(2.)
        del config['pairs_sha256']
        with self.assertRaisesRegex(ValueError,'horizon cannot be verified'):
            validate_horizon(self.run,config,{'horizon_s':2.})

    def test_training_count_mismatch_is_rejected(self):
        config=self.training_metadata(8.)
        config['train_pairs']=2
        with self.assertRaisesRegex(ValueError,'pair count'):
            validate_horizon(self.run,config,{'horizon_s':8.})

    def test_npu_checkpoint_cannot_borrow_another_runs_horizon(self):
        source=self.run/'preaction.onnx';source.write_bytes(b'run A')
        model=self.run/'int8.onnx';model.write_bytes(b'quantized run A')
        metadata=model.with_suffix('.quantization.json')
        metadata.write_text(json.dumps({'source_sha256':sha(source),'model_sha256':sha(model)}))
        config={'onnx_sha256':sha(source)}
        self.assertEqual(validate_npu_model(self.run,config,model)['model_sha256'],sha(model))
        source.write_bytes(b'run B')
        config['onnx_sha256']=sha(source)
        with self.assertRaisesRegex(ValueError,'does not match'):
            validate_npu_model(self.run,config,model)


if __name__=='__main__':
    unittest.main()
