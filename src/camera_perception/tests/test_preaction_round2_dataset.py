"""Data-contract regression tests; no camera, transport or training imports."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts/preaction'/(name+'.py'))
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


export = load('prepare_cycle_pairs')
freeze = load('freeze_round2_dataset')


def pair(name,split,run=None):
    return dict(id=name,split=split,episode=run or name,horizon_s=8.,
                image_t=dict(sha256=name+'_current'),image_t1=dict(sha256=name+'_future'))


class Round2DatasetTests(unittest.TestCase):
    def test_partial_single_axis_keeps_uncommanded_targets_null(self):
        mask=[True,False,False,False,False,False,False]
        plan=dict(commanded_joint_mask=mask,action_semantics=export.PARTIAL_ACTION_SEMANTICS,
                  requested_gripper_target_sdk=None)
        self.assertEqual(export.plan_command_mask(plan),mask)
        current,target,planned,delta=export.action_vectors([10,20,30,40,50,60],
            [11,21,31,41,51,61],[9.8,19,29,39,49,59,13],None,mask)
        self.assertEqual(current,[10,None,None,None,None,None,None])
        self.assertEqual(target,[11,None,None,None,None,None,None])
        self.assertEqual(planned,[1,0,0,0,0,0,0])
        self.assertAlmostEqual(delta[0],1.2);self.assertEqual(delta[1:],[0]*6)

    def test_partial_prefix_requires_exact_logged_mask(self):
        mask=[False,False,False,False,True,False,False]
        action=dict(sdk_target=[None,None,None,None,5,None,None],commanded_joint_mask=mask)
        self.assertTrue(export.command_matches_plan(action,[9,8,7,6,5,4],mask))
        action['commanded_joint_mask']=[True]*7
        self.assertFalse(export.command_matches_plan(action,[9,8,7,6,5,4],mask))
        action['commanded_joint_mask']=mask;action['sdk_target'][0]=9
        self.assertFalse(export.command_matches_plan(action,[9,8,7,6,5,4],mask))

    def test_partial_contract_cannot_update_gripper_or_imply_unsent_targets(self):
        mask=[True,False,False,False,False,False,False]
        with self.assertRaisesRegex(ValueError,'invalid_explicit'):
            export.plan_command_mask(dict(commanded_joint_mask=mask))
        with self.assertRaisesRegex(ValueError,'must_not_request_gripper'):
            export.plan_command_mask(dict(commanded_joint_mask=mask,action_semantics=export.PARTIAL_ACTION_SEMANTICS,
                                          requested_gripper_target_sdk=12))
        self.assertFalse(export.command_matches_plan(dict(sdk_target=[1,None,None,None,None,None,None]),[1,2,3,4,5,6],None))

    def test_full_axis_contract_retains_existing_targets_and_deltas(self):
        self.assertIsNone(export.plan_command_mask({}))
        current,target,planned,delta=export.action_vectors([1,2,3,4,5,6],[2,3,4,5,6,7],[0]*7,12,None)
        self.assertEqual(current,[1,2,3,4,5,6,12]);self.assertEqual(target,[2,3,4,5,6,7,12])
        self.assertEqual(planned,[1,1,1,1,1,1,0]);self.assertEqual(delta,target)

    def test_unknown_context_stays_unknown(self):
        value = export.interaction_context(None)
        self.assertIsNone(value['object_held']);self.assertIsNone(value['contact_verified'])

    def test_empty_gripper_declaration_does_not_become_contact_success(self):
        value = export.interaction_context(dict(object_held=False,contact_verified=False,
                                                state_source='operator_declared_empty_gripper'))
        self.assertIs(value['object_held'],False);self.assertIs(value['contact_verified'],False)
        self.assertNotIn('demonstration_success',value)
        with self.assertRaises(ValueError):export.interaction_context(dict(object_held='false'))

    def test_stages_manifest_context_reaches_discovered_stage(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory);stage = root/'train_run'/'j0_plus';stage.mkdir(parents=True)
            (stage/'transport.jsonl').write_text('')
            manifest = root/'stages.json'
            manifest.write_text(json.dumps(dict(interaction_context=dict(object_held=False,
                contact_verified=False,state_source='operator_declared_empty_gripper'),
                stages=[dict(path=str(stage),run=str(stage.parent),split='train')])) )
            rows = export.discover(SimpleNamespace(cycle=None,stages_json=manifest))
            context = rows[0]['interaction_context']
            self.assertIs(context['object_held'],False)
            self.assertEqual(context['declaration_source']['sha256'],export.sha(manifest))

    def test_whole_run_split_rejects_temporal_leakage(self):
        rows = [pair('a','train','same_run'),pair('b','val','same_run')]
        with self.assertRaisesRegex(ValueError,'crosses_splits'):freeze.validate_rows(rows,False)

    def test_replay_keeps_only_old_train(self):
        fresh = dict(pairs=[pair(s,s) for s in freeze.SPLITS])
        old = dict(pairs=[pair('old_'+s,s) for s in freeze.SPLITS])
        rows,omitted = freeze.merge_replay(fresh,old,False)
        self.assertEqual(len(rows),4)
        self.assertEqual(rows[-1]['id'],'historical_replay__old_train')
        self.assertEqual(omitted['historical_validation_or_test'],2)

    def test_aborted_fallback_uses_new_train_and_unchanged_old_holdouts(self):
        fresh=dict(pairs=[dict(pair('fresh_a','train','new_run'),segment_id='a'),
                          dict(pair('fresh_b','train','new_run'),segment_id='b')])
        old=dict(pairs=[pair('old_'+s,s) for s in freeze.SPLITS])
        rows,omitted=freeze.merge_aborted_regression(fresh,old,2,False)
        self.assertEqual([r['split'] for r in rows],['train','train','val','test'])
        self.assertEqual(omitted['historical_train_excluded'],1)
        for actual,expected in zip(rows[2:],old['pairs'][1:]):
            actual=dict(actual);self.assertEqual(actual.pop('collection_role'),'historical_holdout_regression')
            self.assertEqual(actual,expected)
        fresh['pairs'][1]['split']='val';fresh['pairs'][1]['episode']='other_run'
        with self.assertRaisesRegex(ValueError,'remain_train'):
            freeze.merge_aborted_regression(fresh,old,2,False)

    def test_replay_rejects_alias_of_old_holdout(self):
        fresh = dict(pairs=[pair(s,s) for s in freeze.SPLITS])
        old_train = pair('old_train','train');old_test = pair('old_test','test')
        old_train['image_t']['sha256'] = old_test['image_t']['sha256']
        with self.assertRaisesRegex(ValueError,'historical_train_contains_holdout'):
            freeze.merge_replay(fresh,dict(pairs=[old_train,old_test]),False)

    def test_new_capture_cannot_relabel_old_image(self):
        fresh = dict(pairs=[pair(s,s) for s in freeze.SPLITS])
        old = dict(pairs=[pair('old_train','train')])
        fresh['pairs'][0]['image_t']['sha256'] = old['pairs'][0]['image_t']['sha256']
        with self.assertRaisesRegex(ValueError,'reuses_historical'):freeze.merge_replay(fresh,old,False)


if __name__ == '__main__':unittest.main()
