#!/usr/bin/env python3
"""Frozen cohort, independent correlation and raw normal-equation controls."""
import copy
from pathlib import Path
import tempfile
import unittest
from check_relocalization import POLICY, freeze, hypotheses, materialize, raw_fit, specifications, verify_candidate, verify_search
from check_gnss_map_history import MapOnlyOracle
import check_relocalization_map_only as map_only


class RelocalizationTests(unittest.TestCase):
    def test_all_28_inputs_are_fixed_and_disabled_controls_only_remove_policy(self):
        specs=specifications(['reference-2d','rne-dynamic'])
        self.assertEqual(len(specs),28)
        self.assertEqual(sum(s['mode']=='recover' for s in specs),12)
        self.assertEqual(len(specifications(['reference-2d'],True)),10)
        for s in (s for s in specs if s['mode']=='disabled'):
            _,disabled=materialize(s);_,enabled=materialize(dict(s,mode='recover'))
            disabled.pop('name');enabled.pop('name');disabled['expected']='goal'
            self.assertEqual(enabled.pop('relocalization'),POLICY);self.assertEqual(enabled,disabled)
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);frozen=freeze(root,specs);self.assertEqual(freeze(root,specs),frozen)
            (root/frozen['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):freeze(root,specs)

    def test_complete_correlation_keeps_the_exact_translation_peak_and_rejects_a_changed_vote(self):
        scan=[dict(x=x,y=y) for x,y in [(0.,0.),(1.,0.),(0.,1.)]]
        center=dict(position=dict(x=0.,y=0.),yaw=0.)
        seeds=hypotheses(scan,scan,center,POLICY)
        peak=next(s for s in seeds if s['pose']==center)
        self.assertEqual(peak['votes'],3)
        search=dict(center=center,neighbor_checks=100,correlation_comparisons=45,scan_points_used=3,seeds_tested=len(seeds),hypotheses=seeds,candidates=[],error='no accepted bounded regional fit',proposed_pose=None,proposed_covariance=None)
        self.assertFalse(verify_search(search,dict(lidar=dict(points=scan)),scan,POLICY,{}))
        bad=copy.deepcopy(search);bad['hypotheses'][0]['votes']+=1
        with self.assertRaises(ValueError):verify_search(bad,dict(lidar=dict(points=scan)),scan,POLICY,{})
        bad=copy.deepcopy(search);bad['neighbor_checks']=POLICY['max_neighbor_checks']+1
        with self.assertRaises(ValueError):verify_search(bad,dict(lidar=dict(points=scan)),scan,POLICY,{})

    def test_joint_raw_equations_retain_covariance_correlation_and_refuse_a_forged_pose(self):
        scan=[dict(x=1.+i*.37,y=(i*i%11)*.43) for i in range(16)]
        pose=dict(position=dict(x=0.,y=0.),yaw=0.)
        proof=raw_fit(scan,scan,pose,dict(min_pairs=3))
        self.assertEqual(proof['rms_m'],0.)
        self.assertEqual(proof['inlier_count'],12)
        self.assertAlmostEqual(proof['covariance'][0][0],.01)
        self.assertNotEqual(proof['covariance'][0][2],0.)
        bad=copy.deepcopy(pose);bad['position']['x']=.1
        with self.assertRaises(ValueError):verify_candidate(scan,scan,pose,dict(pose=bad),dict(min_pairs=3))

    def test_seeded_convergence_requires_the_bounded_last_step_and_exact_published_pose(self):
        scan=[dict(x=1.+i*.37,y=(i*i%11)*.43) for i in range(16)]
        pose=dict(position=dict(x=0.,y=0.),yaw=0.)
        seed=copy.deepcopy(pose);seed['position']['x']=.01
        config=dict(min_pairs=3);candidate=dict(pose=pose)
        with self.assertRaises(ValueError):verify_candidate(scan,scan,seed,candidate,config,max_iterations=1)
        proof=verify_candidate(scan,scan,seed,candidate,config,max_iterations=2)
        self.assertEqual(proof['rms_m'],0.)

    def test_map_only_cohort_freezes_all_24_inputs_and_keeps_a_matching_default_control(self):
        specs = map_only.specifications(['reference-2d', 'rne-dynamic'])
        self.assertEqual(len(specs), 24)
        self.assertEqual(sum(s['mode'] == 'handoff' for s in specs), 12)
        self.assertEqual(len(map_only.specifications(['reference-2d'], True)), 8)
        for spec in (s for s in specs if s['mode'] == 'default'):
            _, default = map_only.materialize(spec)
            _, enabled = map_only.materialize(dict(spec, mode='handoff'))
            default.pop('name'); enabled.pop('name')
            self.assertEqual(enabled['relocalization'].pop('map_only_grace_s'), 5.)
            self.assertEqual(enabled, default)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = map_only.freeze(root, specs)
            self.assertEqual(map_only.freeze(root, specs), manifest)
            (root/manifest['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError): map_only.freeze(root, specs)

    def test_map_only_oracle_requires_post_reset_map_and_never_slides_the_deadline(self):
        def output(status, stamp, expiry=15.):
            return dict(relocalization=dict(map_only_support=dict(reset_stamp=10., expires_at=expiry,
                        last_accepted_map_stamp=stamp, status=status)))
        oracle = MapOnlyOracle(10., 5.)
        self.assertFalse(oracle.verify(output('WaitingForMap', 10.), 10., 10., True, None))
        self.assertTrue(oracle.verify(output('Active', 10.1), 10.1, 10.1, True, None))
        self.assertFalse(oracle.verify(output('WaitingForMap', 10.1), 10.35, 10.1, True, None))
        self.assertTrue(oracle.verify(output('Active', 15.), 15., 15., True, None))
        with self.assertRaises(ValueError):
            oracle.verify(output('Active', 15.05, expiry=20.05), 15.05, 15.05, True, None)
        self.assertFalse(oracle.verify(output('Expired', 15.1), 15.1, 15.1, True, None))
        self.assertFalse(oracle.verify(output('GnssAccepted', 15.2), 15.2, 15.2, True, 15.2))

    def test_map_only_oracle_refuses_forged_activity_and_latches_a_real_interruption(self):
        def output(status):
            return dict(relocalization=dict(map_only_support=dict(reset_stamp=10., expires_at=15.,
                        last_accepted_map_stamp=10.1, status=status)))
        with self.assertRaises(ValueError):
            MapOnlyOracle(10., 5.).verify(output('Active'), 10.1, 10.1, False, None)
        with self.assertRaises(ValueError):
            MapOnlyOracle(10., 5.).verify(output('Active'), 10.1, 10.1, True, None, healthy=False)
        oracle = MapOnlyOracle(10., 5.)
        self.assertFalse(oracle.verify(output('Interrupted'), 10.1, 10.1, True, None, interrupted=True))
        self.assertFalse(oracle.verify(output('Interrupted'), 10.15, 10.1, True, None))


if __name__=='__main__':unittest.main()
