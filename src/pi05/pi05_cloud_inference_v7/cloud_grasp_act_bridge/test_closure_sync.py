"""Offline assertions for the closure coordination candidate; no hardware calls."""
from dataclasses import FrozenInstanceError, asdict, replace
import json
import unittest

from closure_sync import (ClosureSyncConfig, ClosureSyncState, shape_closure,
                          validate_config)


ANCHOR = (-19.865,98.807,-55.245,5.451,44.688,-29.506)
PLACE = (30.,125.,-90.,0.,60.,-55.)
RAW = ANCHOR+(.02,)

# Captured seq104..123 from trace SHA256 f078e409ac8b4b29a7c1012e92d9e54af289fb9d474cbdc6e3088d23d7c4594e.
LATEST_104_123 = (((-21.863752365112305, 96.94979095458984, -55.455841064453125, 6.591035842895508, 44.42316818237305, -30.35987091064453, 0.059254929423332214), 0.0665), ((-21.848493576049805, 95.43053436279297, -55.5522346496582, 6.464255332946777, 44.50255584716797, -30.3873291015625, 0.053870249539613724), 0.0665), ((-21.69145965576172, 96.44893646240234, -55.64273452758789, 6.476163864135742, 44.660064697265625, -30.509578704833984, 0.0445660725235939), 0.0655), ((-21.82073402404785, 96.64539337158203, -56.02252197265625, 6.455385208129883, 44.48666763305664, -30.617515563964844, 0.03503754362463951), 0.0644), ((-21.87755584716797, 96.03386688232422, -55.66611862182617, 6.286603927612305, 44.58738708496094, -30.752399444580078, 0.02612103708088398), 0.0633), ((-21.715078353881836, 95.98765563964844, -55.07195281982422, 6.090921401977539, 44.168758392333984, -30.770465850830078, 0.018525730818510056), 0.0622), ((-22.03420066833496, 95.46427154541016, -55.21770095825195, 6.305421829223633, 44.47488784790039, -30.684783935546875, 0.013312293216586113), 0.0612), ((-21.86296844482422, 95.48802947998047, -55.537940979003906, 6.13224983215332, 44.81840896606445, -30.982315063476562, 0.01106070913374424), 0.0601), ((-22.197105407714844, 94.77764129638672, -55.68825912475586, 6.108928680419922, 44.919334411621094, -30.751220703125, 0.009986702352762222), 0.0591), ((-22.376441955566406, 93.81854248046875, -56.05094909667969, 6.083467483520508, 45.35067367553711, -31.20639419555664, 0.00991920568048954), 0.0581), ((-22.53528594970703, 93.34698486328125, -55.91965103149414, 5.847084045410156, 45.85316848754883, -31.980804443359375, 0.009894948452711105), 0.0569), ((-22.24765968322754, 92.66535186767578, -56.490638732910156, 5.763932228088379, 46.116661071777344, -32.44292068481445, 0.010478582233190536), 0.0559), ((-22.49175453186035, 91.24097442626953, -56.666481018066406, 5.886857986450195, 47.37518310546875, -34.148441314697266, 0.010132050141692162), 0.054799999999999995), ((-22.055509567260742, 90.01026153564453, -56.7612419128418, 5.65751838684082, 47.5541877746582, -34.93938446044922, 0.010307356715202332), 0.0537), ((-21.998659133911133, 87.97392272949219, -57.2473030090332, 5.889918327331543, 48.65778350830078, -37.87824249267578, 0.01077895425260067), 0.0527), ((-21.475788116455078, 85.52696990966797, -57.94423294067383, 6.193586349487305, 49.67144775390625, -39.825355529785156, 0.010621555149555206), 0.0516), ((-20.309375762939453, 83.2951889038086, -58.3652229309082, 6.249120712280273, 50.75047302246094, -42.731773376464844, 0.010733891278505325), 0.0506), ((-19.95124053955078, 94.11756896972656, -52.95867156982422, 5.222929954528809, 46.260032653808594, -31.75753402709961, 0.052974458783864975), 0.049499999999999995), ((-20.144460678100586, 93.45700073242188, -52.741024017333984, 5.133208274841309, 45.96485900878906, -31.02933120727539, 0.05467534065246582), 0.0479), ((-20.666349411010742, 93.06495666503906, -52.78582763671875, 4.876014709472656, 45.33848571777344, -30.509376525878906, 0.05687155947089195), 0.0501))


def snapshot(observed, source, force=-.08):
    return {'source_ns':source,'state':list(observed),'healthy':True,
            'status':{'ctrl_mode':1,'arm_status':0,'mode_feedback':1,
                      'teach_status':0,'err_code':0,'gripper_mode':0,
                      'gripper_status':0x40,'gripper_force_n':force}}


class ClosureSyncTests(unittest.TestCase):
    def step(self, phase, *, now=10., observed=ANCHOR+(.0445,), raw=RAW,
             source=1000, ack=None, closure_latched=False, force=-.08,
             **kwargs):
        args=dict(now=now,observed_source_ns=source,
                  snapshot=snapshot(observed,source,force),snapshot_at=now-.01,
                  raw_source_at=now-.1,previous_ack_target7=ack or ANCHOR+(.0415,),
                  closure_latched=closure_latched)
        args.update(kwargs)
        return shape_closure(raw,observed,phase,**args)

    def ready(self):
        phase=ClosureSyncState()
        for i in range(8):
            target,phase,diag=self.step(phase,now=10.+i*.05,source=1000+i,
                                        closure_latched=i==0)
        self.assertEqual(phase.phase,'contact_ready')
        return phase

    def complete(self):
        phase=self.ready()
        target,phase,diag=self.step(phase,now=10.5,source=1100,new_prediction=True,
                                    raw_source_at=10.4)
        self.assertEqual(phase.phase,'completed')
        return phase

    def test_config_complete_mapping_and_immutable_state(self):
        config=ClosureSyncConfig()
        self.assertEqual(validate_config(json.loads(json.dumps(asdict(config)))),config)
        self.assertEqual(config.to_dict(),asdict(config))
        with self.assertRaises(FrozenInstanceError):
            config.stability_s=0.
        with self.assertRaises(FrozenInstanceError):
            ClosureSyncState().phase='closing'

    def test_config_rejects_missing_extra_nonfinite_boolean_and_bad_bounds(self):
        values=ClosureSyncConfig().to_dict()
        for edit in ({'max_closing_s':float('nan')},{'stability_s':True},
                     {'max_closing_s':5.},{'width_min_m':.05},
                     {'state_gripper_tolerance_m':1e-5},
                     {'placement_joint_bounds_deg':((0.,1.),)*6}):
            with self.subTest(edit=edit),self.assertRaises(ValueError):
                validate_config(values|edit)
        with self.assertRaises(ValueError): validate_config({'max_closing_s':4.})
        with self.assertRaises(ValueError): validate_config(values|{'typo':1})

    def test_idle_transparent_and_only_explicit_visual_latch_starts(self):
        phase=ClosureSyncState()
        target,next_phase,diag=self.step(phase)
        self.assertEqual(target,RAW)
        self.assertEqual(next_phase,phase)
        target,next_phase,diag=self.step(phase,closure_latched=True)
        self.assertEqual(next_phase.phase,'closing')
        self.assertEqual(next_phase.anchor,ANCHOR)
        self.assertEqual(next_phase.anchor_source_ns,1000)
        self.assertEqual(target,ANCHOR+(.02,))

    def test_latest_104_123_raw_reopen_rejected_and_six_axes_frozen(self):
        anchor=ANCHOR
        phase=ClosureSyncState();last_grip=float('inf')
        for i,(raw,width) in enumerate(LATEST_104_123):
            # Counterfactual feedback keeps axes at the proposed frozen anchor;
            # original failed-run widths/raw proposals are retained unchanged.
            observed=anchor+(width,)
            target,phase,diag=self.step(phase,now=10.+i*.05,source=2000+i,
                raw=raw,observed=observed,closure_latched=i==0,ack=observed)
            self.assertEqual(phase.phase,'closing')
            self.assertEqual(target[:6],anchor)
            self.assertLessEqual(target[6],last_grip)
            last_grip=target[6]
        self.assertGreater(raw[6],.055)
        self.assertLess(last_grip,.02)
        minimum_index=min(range(20),key=lambda i:LATEST_104_123[i][0][6])
        self.assertEqual(phase.target_model_source_at,10.+minimum_index*.05-.1)

    def test_real_unfrozen_trace_must_fail_on_anchor_departure(self):
        _,phase,_=self.step(ClosureSyncState(),observed=ANCHOR+(.0665,),
                           raw=ANCHOR+(.059,),closure_latched=True)
        moved=(ANCHOR[0],ANCHOR[1]-3.001,*ANCHOR[2:],.06)
        target,phase,diag=self.step(phase,now=10.05,source=1001,observed=moved)
        self.assertIsNone(target)
        self.assertIn('anchor',diag.failure_reason)

    def test_ack_pressure_required_not_merely_small_raw_target(self):
        phase=ClosureSyncState()
        for i in range(9):
            _,phase,diag=self.step(phase,now=10.+i*.05,source=1000+i,
                closure_latched=i==0,ack=ANCHOR+(.044,))
        self.assertEqual(phase.phase,'closing')
        self.assertEqual(phase.samples,())
        self.assertAlmostEqual(diag.pressure_m,.0005)

    def test_stable_width_and_ack_pressure_become_candidate_not_proven(self):
        phase=self.ready()
        target,next_phase,diag=self.step(phase,now=10.4,source=1099)
        self.assertEqual(target[:6],ANCHOR)
        self.assertTrue(diag.request_replan)
        self.assertFalse(diag.contact_or_grasp_proven)
        self.assertFalse(diag.force_is_calibrated_contact_evidence)
        self.assertAlmostEqual(diag.gripper_force_n,-.08)
        self.assertEqual(next_phase.phase,'contact_ready')

    def test_continuous_no_load_closing_does_not_confirm_and_empty_fails(self):
        phase=ClosureSyncState()
        for i,width in enumerate((.06,.055,.05,.046,.045,.043,.04,.036)):
            _,phase,diag=self.step(phase,now=10.+i*.05,source=1000+i,
                observed=ANCHOR+(width,),closure_latched=i==0,force=0.)
            self.assertEqual(phase.phase,'closing')
        target,phase,diag=self.step(phase,now=10.4,source=1008,
                                    observed=ANCHOR+(.0349,),force=0.)
        self.assertIsNone(target)
        self.assertEqual(phase.phase,'failed')
        self.assertIn('Empty',diag.failure_reason)

    def test_duplicate_sources_never_advance_stability(self):
        _,phase,_=self.step(ClosureSyncState(),closure_latched=True)
        original=phase.samples
        for offset in (.02,.04,.06):
            _,phase,diag=self.step(phase,now=10.+offset,source=1000,snapshot_at=9.99)
        self.assertEqual(phase.samples,original)
        self.assertFalse(diag.unique_sample)
        self.assertFalse(diag.request_replan)

    def test_long_sample_gap_and_width_variation_reset_stability(self):
        _,phase,_=self.step(ClosureSyncState(),closure_latched=True)
        _,phase,_=self.step(phase,now=10.1,source=1001)
        _,phase,diag=self.step(phase,now=10.3,source=1002)
        self.assertEqual(len(phase.samples),1)
        _,phase,diag=self.step(phase,now=10.35,source=1003,observed=ANCHOR+(.0435,))
        self.assertEqual(len(phase.samples),1)
        self.assertEqual(diag.stable_elapsed_s,0.)

    def test_stale_future_snapshot_mismatched_source_or_values_fail(self):
        for patch in ({'snapshot_at':9.89},{'snapshot_at':10.01},
                      {'observed_source_ns':999},
                      {'snapshot':snapshot(ANCHOR+(.043,),1000)},
                      {'snapshot':snapshot(ANCHOR+(.0445,),1000,float('nan'))}):
            with self.subTest(patch=patch):
                target,phase,diag=self.step(ClosureSyncState(),closure_latched=True,**patch)
                self.assertIsNone(target)
                self.assertEqual(phase.phase,'failed')

    def test_float32_tolerance_accepts_but_larger_opening_mismatch_fails(self):
        obs=ANCHOR+(.0445,)
        snap=snapshot(tuple(v+1e-6 for v in ANCHOR)+(.044500001,),1000)
        _,phase,_=self.step(ClosureSyncState(),snapshot=snap,closure_latched=True)
        self.assertEqual(phase.phase,'closing')
        snap['state'][6]+=.000001
        target,phase,_=self.step(ClosureSyncState(),snapshot=snap,closure_latched=True)
        self.assertIsNone(target)

    def test_health_enabled_mode_faults_and_invalid_vectors_fail(self):
        modifications=({'healthy':False},{'status':{'gripper_status':0}},
                       {'status':{'gripper_status':0x41}},{'status':{'gripper_mode':1}},
                       {'status':{'err_code':1}},{'status':{'gripper_force_n':None}})
        for change in modifications:
            snap=snapshot(ANCHOR+(.0445,),1000)
            for key,value in change.items():
                if key=='status': snap[key].update(value)
                else: snap[key]=value
            with self.subTest(change=change):
                target,phase,_=self.step(ClosureSyncState(),snapshot=snap,closure_latched=True)
                self.assertIsNone(target)
        for raw in ((1.,)*6,ANCHOR+(float('inf'),),ANCHOR+(True,)):
            self.assertIsNone(self.step(ClosureSyncState(),raw=raw,closure_latched=True)[0])

    def test_four_second_timeout_and_time_source_reversal_fail(self):
        _,phase,_=self.step(ClosureSyncState(),closure_latched=True)
        for patch in ({'now':14.,'source':1002},{'now':9.9,'source':1002},
                      {'now':10.05,'source':999}):
            with self.subTest(patch=patch):
                target,next_phase,diag=self.step(phase,**patch)
                self.assertIsNone(target)
                self.assertEqual(next_phase.phase,'failed')

    def test_proposed_state_is_not_committed_without_ack(self):
        initial=ClosureSyncState()
        _,proposed,_=self.step(initial,closure_latched=True)
        self.assertEqual(initial.phase,'idle')
        self.assertEqual(proposed.phase,'closing')
        target,next_phase,_=self.step(initial,now=10.05,source=1001,raw=ANCHOR+(.06,))
        self.assertEqual(next_phase,initial)
        self.assertEqual(target[-1],.06)

    def test_replan_required_after_readiness_and_must_be_fresh(self):
        phase=self.ready()
        target,held,diag=self.step(phase,now=10.4,source=1099)
        self.assertEqual(held.phase,'contact_ready')
        self.assertTrue(diag.request_replan)
        self.assertEqual(target[:6],ANCHOR)
        target,failed,diag=self.step(phase,now=10.5,source=1100,new_prediction=True,
                                   raw_source_at=10.2)
        self.assertIsNone(target)
        self.assertIn('after contact',diag.failure_reason)
        self.complete()

    def test_fresh_replan_reopening_at_anchor_fails(self):
        target,phase,diag=self.step(self.ready(),now=10.5,source=1100,new_prediction=True,
                                   raw_source_at=10.4,raw=ANCHOR+(.06,))
        self.assertIsNone(target)
        self.assertIn('reopening',diag.failure_reason)

    def test_fixed_carry_ceiling_rejects_first_small_reopening(self):
        ready=self.ready()
        self.assertAlmostEqual(ready.carry_ceiling_m,.0425)
        # 45mm is less than observed44.5+1mm, yet already removes the ACK's
        # inward pressure. The fixed confirmation-time ceiling rejects it.
        target,phase,diag=self.step(ready,now=10.5,source=1100,new_prediction=True,
            raw_source_at=10.4,raw=ANCHOR+(.045,))
        self.assertIsNone(target)
        self.assertIn('reopening',diag.failure_reason)
        target,phase,diag=self.step(self.complete(),now=10.6,source=1101,
                                    raw=ANCHOR+(.045,))
        self.assertIsNone(target)

    def test_submillimetre_reopening_cannot_accumulate_carry_ceiling(self):
        phase=self.complete()
        fixed=phase.carry_ceiling_m
        # Every increment is only .9mm and stays far below actual+1mm.
        for i,width in enumerate((.040,.0409,.0418)):
            target,phase,diag=self.step(phase,now=10.6+i*.05,source=1101+i,
                                        raw=ANCHOR+(width,))
            self.assertEqual(phase.phase,'completed')
            self.assertEqual(phase.carry_ceiling_m,fixed)
        target,phase,diag=self.step(phase,now=10.75,source=1104,raw=ANCHOR+(.0427,))
        self.assertIsNone(target)
        self.assertEqual(phase.phase,'failed')

    def test_replan_may_increase_goal_while_remaining_below_fixed_ceiling(self):
        ready=self.ready()
        self.assertEqual(ready.target_gripper_m,.02)
        target,phase,diag=self.step(ready,now=10.5,source=1100,new_prediction=True,
            raw_source_at=10.4,raw=ANCHOR+(.030,))
        self.assertEqual(target[-1],.030)
        self.assertEqual(phase.phase,'completed')
        self.assertAlmostEqual(phase.carry_ceiling_m,.0425)

    def test_completed_empty_width_fails_but_released_is_unrestricted(self):
        completed=self.complete()
        target,failed,diag=self.step(completed,now=10.6,source=1101,
            observed=ANCHOR+(.0349,),raw=ANCHOR+(.02,))
        self.assertIsNone(target)
        self.assertIn('lost',diag.failure_reason)
        _,released,_=self.step(completed,now=10.6,source=1101,
            observed=PLACE+(.0445,),raw=PLACE+(.06,))
        target,phase,diag=self.step(released,now=10.65,source=1102,
            observed=ANCHOR+(.02,),raw=ANCHOR+(.03,))
        self.assertEqual(target[-1],.03)
        self.assertEqual(phase.phase,'released')

    def test_completed_lift_allowed_early_reopen_fails_placement_releases(self):
        phase=self.complete()
        lifted=tuple(v+5. for v in ANCHOR)+(.0445,)
        target,phase,diag=self.step(phase,now=10.6,source=1101,observed=lifted)
        self.assertEqual(phase.phase,'completed')
        target,failed,diag=self.step(phase,now=10.65,source=1102,observed=lifted,raw=ANCHOR+(.06,))
        self.assertIsNone(target)
        target,released,diag=self.step(phase,now=10.65,source=1102,
                                      observed=PLACE+(.0445,),raw=PLACE+(.06,))
        self.assertEqual(released.phase,'released')
        self.assertEqual(target[-1],.06)
        target,next_phase,diag=self.step(released,now=10.7,source=1103,
                                        observed=lifted,raw=ANCHOR+(.07,))
        self.assertEqual(target[-1],.07)
        self.assertEqual(next_phase.phase,'released')

    def test_visual_reset_cannot_cancel_closing_or_authorize_early_reopen(self):
        _,phase,_=self.step(ClosureSyncState(),closure_latched=True)
        self.assertIsNone(self.step(phase,now=10.05,source=1001,reset=True)[0])
        phase=self.complete()
        self.assertIsNone(self.step(phase,now=10.6,source=1101,reset=True,raw=ANCHOR+(.06,))[0])

    def test_target_source_is_retained_while_fresh_feedback_advances(self):
        _,phase,_=self.step(ClosureSyncState(),raw_source_at=9.8,raw_source_ns=800,closure_latched=True)
        _,next_phase,diag=self.step(phase,now=10.05,source=1001,raw=ANCHOR+(.06,),raw_source_at=10.)
        self.assertEqual(next_phase.target_model_source_at,9.8)
        self.assertEqual(next_phase.target_model_source_ns,800)
        self.assertEqual(next_phase.anchor_source_ns,1000)
        self.assertEqual(diag.snapshot_source_ns,1001)


if __name__=='__main__':
    unittest.main()
