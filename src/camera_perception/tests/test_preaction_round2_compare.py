"""Tests for counterfactual action control; no inference or physical devices."""
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/preaction'))
from compare_round2 import cross_direction_permutation, direction,validate_regression_scope


class CrossDirectionTests(unittest.TestCase):
    def test_shuffle_preserves_marginals_and_changes_every_direction(self):
        splits = np.array(['train']*16+['val']*16+['test']*16)
        directions = (['j0+']*4+['j0-']*4+['j4+']*4+['j4-']*4)*3
        permutation = cross_direction_permutation(splits,directions,42)
        self.assertEqual(len(set(permutation)),48)
        self.assertTrue(np.array_equal(permutation,cross_direction_permutation(splits,directions,42)))
        for i,j in enumerate(permutation):
            self.assertEqual(splits[i],splits[j]);self.assertNotEqual(directions[i],directions[j])

    def test_unbalanced_single_direction_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'cannot_permute'):
            cross_direction_permutation(['train']*3+['val']*2+['test']*2,
                                        ['j0+']*3+['j0+','j0-']*2)

    def test_return_to_zero_uses_delta_not_anchor(self):
        row = dict(action=dict(planned_delta_sdk=[-1.2,0,0,0,0,0,0]),
                   interaction_context=dict(commanded_joint=0,anchor_offset_deg=0))
        self.assertEqual(direction(row),'j0-')

    def test_fallback_shuffles_train_only_without_misnaming_holdout_control(self):
        splits=['train']*4+['val']*2+['test']*2
        directions=['j0+','j0-','j4+','j4-']+['j0+']*4
        values=cross_direction_permutation(splits,directions,42,('train',))
        self.assertTrue(all(directions[i]!=directions[j] for i,j in enumerate(values[:4])))
        self.assertEqual(values[4:].tolist(),list(range(4,8)))

    def test_regression_cannot_modify_historical_holdout(self):
        old=[dict(id='v',split='val',horizon_s=8),dict(id='t',split='test',horizon_s=8)]
        rows=[dict(r,collection_role='historical_holdout_regression') for r in old]
        rows.append(dict(id='new',split='train',collection_role='new_action_collection',interaction_context=dict(object_held=False)))
        validate_regression_scope(rows,old)
        rows[0]['horizon_s']=2
        with self.assertRaisesRegex(ValueError,'modified'):validate_regression_scope(rows,old)


if __name__ == '__main__':unittest.main()
