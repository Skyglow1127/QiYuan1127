"""Lossless production-client boundaries, synthetic HTTP only; no robot IO."""
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import unittest

import numpy as np
import client

HERE = Path(__file__).resolve().parent
_, SETTINGS, _ = client.service_settings()
CODEC_PATH = Path(client.service_settings()[0]['runtime_config']).parent/'wire_codec.py'
CODEC = client.wire_codec_module(str(CODEC_PATH))


def sample():
    with np.load(CODEC_PATH.parent/'canonical_input.npz', allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


class SyntheticConnection:
    def __init__(self, failure=None):
        self.calls, self.closed, self.failure = [], 0, failure
    def request(self, method, path, *, body, headers):
        self.calls.append((method, path, body, headers))
    def getresponse(self):
        _, _, payload, headers = self.calls[-1]
        fields = CODEC.decode_payload(payload, headers['X-Image-Codec'])
        result = dict(protocol_version=client.PROTOCOL, request_id=headers['X-Request-ID'],
            input_sha256=hashlib.sha256(payload).hexdigest(), model_sha256=client.MODEL_SHA256,
            startup_id='synthetic-service', model_id=SETTINGS['model_id'], action_shape=[50,7],
            units=['degree']*6+['metre'], representation='absolute joint targets and gripper width',
            image_codec=client.PNG_CODEC, decoded_fields_sha256=CODEC.decoded_fields_sha256(fields),
            actions=np.zeros((50,7)).tolist(), server_timing_s={key:.001 for key in
                ['decode','preprocess','model','postprocess','total']})
        if self.failure == 'semantic':
            result['decoded_fields_sha256'] = '0'*64
        elif self.failure == 'codec':
            result['image_codec'] = client.RAW_CODEC
        elif self.failure == 'request':
            result['request_id'] = 'wrong-request'
        elif self.failure == 'shape':
            result['actions'] = [[0.]*7]
        elif self.failure == 'model':
            result['model_sha256'] = '0'*64
        class Response:
            status = 200
            def read(self, limit):
                return json.dumps(result).encode()
        return Response()
    def close(self):
        self.closed += 1


class TransportTests(unittest.TestCase):
    def test_production_png_preserves_canonical_fields_and_rgb_channels(self):
        data=sample()
        # Strong asymmetric channels expose an RGB/BGR swap independently of noise.
        data['observation.images.global'][0] = 255
        data['observation.images.global'][1] = 23
        data['observation.images.global'][2] = 7
        args=(data['observation.state'],data['observation.images.global'],
              data['observation.images.wrist'],client.TASK)
        raw,_,_=client.encode_observation(*args)
        png,_,_,semantic=client.prepare_wire_observation(*args,
            image_codec=client.PNG_CODEC,codec_path=CODEC_PATH)
        decoded=CODEC.decode_payload(png,client.PNG_CODEC)
        with np.load(io.BytesIO(raw),allow_pickle=False) as reference:
            self.assertEqual(set(reference.files),set(decoded))
            for key in reference.files:
                np.testing.assert_array_equal(reference[key],decoded[key])
        self.assertEqual(CODEC.decoded_fields_sha256(decoded),semantic)

    def test_real_client_accepts_matching_semantic_bytes_and_retains_timeout(self):
        data=sample();connection=SyntheticConnection()
        http=client.InferenceClient(timeout_s=.4)
        http.health={'metadata':{'startup_id':'synthetic-service','model_id':SETTINGS['model_id']}}
        http.connection=connection;http.open=lambda:http.health
        result=http.infer(data['observation.state'],data['observation.images.global'],
                          data['observation.images.wrist'],client.TASK)
        self.assertEqual(result['actions'].shape,(50,7))
        self.assertEqual(http.timeout_s,.4)
        self.assertEqual(len(connection.calls),1)
        self.assertEqual(connection.calls[0][3]['X-Image-Codec'],client.PNG_CODEC)
        self.assertIn('wait_response_headers',result['timing_s'])

    def test_mismatched_source_or_decoded_bytes_close_once_without_retry(self):
        data=sample()
        for failure in ['semantic','codec','request','shape','model']:
            with self.subTest(failure=failure):
                connection=SyntheticConnection(failure);http=client.InferenceClient(timeout_s=.4)
                http.health={'metadata':{'startup_id':'synthetic-service','model_id':SETTINGS['model_id']}}
                http.connection=connection;http.open=lambda:http.health
                with self.assertRaises(RuntimeError):
                    http.infer(data['observation.state'],data['observation.images.global'],
                               data['observation.images.wrist'],client.TASK)
                self.assertEqual(connection.closed,1)
                self.assertEqual(len(connection.calls),1)
                self.assertIsNone(http.connection)


if __name__ == '__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(TransportTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    (HERE/'lossless_bridge_verification_result.json').write_text(json.dumps({
        'success':result.wasSuccessful(),'tests_run':result.testsRun,
        'failures':len(result.failures),'errors':len(result.errors),
        'network_calls':0,'robot_commands_sent':0,
        'client_sha256':hashlib.sha256((HERE/'client.py').read_bytes()).hexdigest(),
        'wire_codec_sha256':hashlib.sha256(CODEC_PATH.read_bytes()).hexdigest(),
        'scope':'Exact existing quantized224RGB/state bytes and strict synthetic HTTP response boundaries'
    },indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
