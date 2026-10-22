#!/usr/bin/env python3
"""Independent clearance-floor, policy admission and frozen-comparison controls."""
import copy
import math
import unittest

from check_obstruction_stops import certify_trajectory
from check_refined_lateral import certificates, materialize, specifications, verify_policy
from check_road_width import verify_targets


def config():
    return dict(route=dict(points=[dict(x=0.,y=0.),dict(x=100.,y=0.)],half_width=5.5),
                vehicle=dict(radius=1.25),lateral_candidate_margin_m=.35,refine_lateral_candidates=True)


def near_miss():
    return dict(estimate=dict(pose=dict(yaw=0.)),
                trajectory=dict(points=[dict(position=dict(x=0.,y=0.),speed=5.,time=0.),
                                        dict(position=dict(x=20.,y=0.),speed=5.,time=4.)]),
                predictions=[dict(id=1,positions=[dict(x=10.,y=2.1)]*41,radius=.5,dt=.2)])


class RefinedLateralTests(unittest.TestCase):
    def test_a_legacy_accepted_near_miss_cannot_satisfy_the_refined_one_meter_floor(self):
        out=near_miss()
        self.assertTrue(certify_trajectory(out,1.)['certified'])
        with self.assertRaises(ValueError):certify_trajectory(out,1.,minimum_clearance_m=1.)

    def test_the_independent_floor_never_replaces_larger_transverse_reserves(self):
        out=near_miss();out['trajectory']['points']=[dict(position=dict(x=0.,y=0.),speed=0.,time=t) for t in [0.,8.]]
        out['predictions'][0]['positions']=[dict(x=2.6,y=4.-i*.2) for i in range(41)]
        with self.assertRaises(ValueError):certify_trajectory(out,1.,minimum_clearance_m=1.)

    def test_a_growing_interior_radius_cannot_hide_between_clear_hold_endpoints(self):
        out=near_miss();out['trajectory']['points']=[dict(position=dict(x=0.,y=0.),speed=0.,time=t) for t in [0.,8.]]
        out['predictions']=[dict(id=1,positions=[dict(x=4.,y=0.)]*3,radius=.5,dt=4.,additional_radius_m=[0.,2.,0.])]
        with self.assertRaises(ValueError):certify_trajectory(out,1.,minimum_clearance_m=1.)
        for bad in [-1.,math.nan,math.inf]:
            with self.assertRaises(ValueError):certify_trajectory(out,1.,minimum_clearance_m=bad)

    def test_exact_intermediate_targets_require_the_explicit_fitted_policy(self):
        c=config();tick=dict(expected=dict(trajectory=dict(lateral_target=3.0625,points=[dict(position=dict(x=5.,y=3.0625))])))
        self.assertEqual(verify_targets(c,[tick])['side_control_ticks'],1)
        for mutation in ['lost','wrong-target','wrong-type','lost-width']:
            changed=copy.deepcopy(c);sample=copy.deepcopy(tick)
            if mutation=='lost':changed.pop('refine_lateral_candidates')
            elif mutation=='wrong-target':sample['expected']['trajectory']['lateral_target']=.875
            elif mutation=='wrong-type':changed['refine_lateral_candidates']='true'
            else:changed.pop('lateral_candidate_margin_m')
            with self.assertRaises(ValueError):verify_targets(changed,[sample])

    def test_frozen_policy_admission_rejects_an_omitted_true_policy(self):
        frozen=dict(duration=45.,refine_lateral_candidates=True,lateral_candidate_margin_m=.35)
        with self.assertRaises(ValueError):verify_policy(dict(duration=45.,lateral_candidate_margin_m=.35),config(),frozen)
        with self.assertRaises(ValueError):verify_policy(frozen,dict(config(),refine_lateral_candidates=False),frozen)

    def test_full_emergency_braking_cannot_be_removed_from_an_unavailable_path(self):
        out=dict(trajectory=dict(mode='Emergency',lateral_target=0.,points=[]),health=[],command=dict(acceleration=0.,steering=0.))
        with self.assertRaises(ValueError):certificates(config(),[dict(expected=out)])

    def test_paired_corridor_inputs_change_only_the_new_policy_and_name(self):
        rows=specifications(['reference-2d','rne-dynamic'],'corridor')
        self.assertEqual(len(rows),16)
        for spec in rows:
            if not spec['refined']:continue
            other=next(s for s in rows if not s['refined'] and all(s[k]==spec[k] for k in ['plant','world','seed']))
            _,current=materialize(spec);_,control=materialize(other)
            for value in [current,control]:value.pop('name');value.pop('refine_lateral_candidates')
            self.assertEqual(current,control)
            self.assertEqual(current['duration'],45.)

    def test_parent_sampling_requires_its_existing_width_calibration_and_keeps_eight_missions(self):
        rows=specifications(['reference-2d','rne-dynamic'],'regression')
        self.assertEqual(len(rows),28)
        self.assertEqual(sum(s['world']=='mission' for s in rows),8)
        for spec in rows:
            _,value=materialize(spec)
            self.assertIsNotNone(value['lateral_candidate_margin_m'])
            if spec['world']=='mission':self.assertEqual(value['duration'],65.)

    def test_a_safe_partial_hold_cannot_be_presented_as_a_complete_refinement(self):
        c=config();c['route']['lengths']=[0.,100.]
        out=dict(estimate=dict(pose=dict(position=dict(x=0.,y=3.0625),yaw=0.)),
                 trajectory=dict(mode='Yield',lateral_target=3.0625,points=[dict(position=dict(x=0.,y=3.0625),speed=0.,time=t) for t in [0.,8.]]),
                 health=[],command=dict(acceleration=-.5,steering=0.),predictions=[])
        with self.assertRaises(ValueError):certificates(c,[dict(expected=out)])


if __name__=='__main__':unittest.main()
