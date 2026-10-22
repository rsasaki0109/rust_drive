#!/usr/bin/env python3
"""Independent road geometry, target corruption and fixed paired-protocol checks."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from check_road_width import compare, freeze, project, road_margin, specifications, swept_clearance, verify_targets


def point(x,y=0.): return dict(x=x,y=y)
def config(): return dict(route=dict(points=[point(0.),point(100.)],half_width=4.5),
                          vehicle=dict(radius=1.25),lateral_candidate_margin_m=.35)
def tick(target=2.9,position=None):
    return dict(expected=dict(trajectory=dict(lateral_target=target,points=[dict(position=position or point(40.,2.9))])))


class RoadWidthTests(unittest.TestCase):
    def test_unsafe_swept_contacts_are_scored_without_erasing_the_remaining_metrics(self):
        run=dict(vehicle=dict(radius=.25),summary=dict(min_clearance=-.5),frames=[
            dict(truth=dict(pose=dict(position=point(x))),objects=[dict(id=1,position=point(0.),radius=.25)]) for x in [-3.,3.]])
        result=swept_clearance(run)
        self.assertEqual(result['minimum_swept_clearance_m'],-.5); self.assertEqual(result['contact_intervals'],1)
        run['summary']['min_clearance']=0.
        with self.assertRaises(ValueError): swept_clearance(run)
    def test_projection_is_rotation_invariant_and_uses_finite_endpoints(self):
        self.assertEqual(project(point(5.,-2.),[point(0.),point(10.)]),(2.,5.,-2.))
        self.assertEqual(project(point(2.,5.),[point(0.),point(0.,10.)]),(2.,5.,-2.))
        self.assertEqual(project(point(12.,0.),[point(0.),point(10.)]),(2.,10.,0.))
        with self.assertRaises(ValueError): project(point(0.),[point(0.),point(0.)])
        with self.assertRaises(ValueError): project(point(float('nan')),[point(0.),point(10.)])

    def test_full_circular_footprint_detects_road_and_endpoint_excursions(self):
        run=dict(route=config()['route'],vehicle=config()['vehicle'],frames=[dict(truth=dict(pose=dict(position=point(20.,2.9))))])
        self.assertAlmostEqual(road_margin(run)['minimum_actual_road_margin_m'],.35)
        run['frames'][0]['truth']['pose']['position']=point(20.,3.3)
        self.assertEqual(road_margin(run)['violations'],1)
        run['frames'][0]['truth']['pose']['position']=point(104.)
        self.assertEqual(road_margin(run)['violations'],1)

    def test_an_old_offset_or_an_interior_path_excursion_cannot_pass_the_width_audit(self):
        self.assertEqual(verify_targets(config(),[tick()])['side_control_ticks'],1)
        for target in [3.5,float('nan')]:
            with self.assertRaises(ValueError): verify_targets(config(),[tick(target)])
        with self.assertRaises(ValueError): verify_targets(config(),[tick(position=point(25.,3.3))])
        c=config(); c['route']['half_width']=1.5
        with self.assertRaises(ValueError): verify_targets(c,[tick(.1,point(25.))])

    def test_protocol_has_64_unique_rows_and_pairs_differ_only_in_the_candidate_reserve(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); specs=specifications(['reference-2d','rne-dynamic']); self.assertEqual(len(specs),64)
            self.assertEqual(len({(s['plant'],s['world'],s['seed'],s['mode']) for s in specs}),64)
            manifest=freeze(root,specs)
            for a,b in zip(manifest['episodes'][::2],manifest['episodes'][1::2]):
                left=json.loads((root/a['scenario']).read_text()); right=json.loads((root/b['scenario']).read_text())
                self.assertEqual(right.pop('lateral_candidate_margin_m'),.35); self.assertEqual(left,right)
            (root/manifest['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError): freeze(root,specs)

    def test_comparison_rejects_new_accuracy_failures_incomplete_replay_and_changed_wide_outputs(self):
        base=dict(plant='reference-2d',world='wide',seed=7,mode='fixed',failed_gates=[],functional_passed=True,
                  replay=dict(verified=True,ticks=2),replay_exit_code=0,run_exit_code=0,summary=dict(steps=2,passed=True),
                  storage={n:dict(sha256='same') for n in ['replay/outputs.jsonl','run/summary.json']})
        fit=copy.deepcopy(base); fit['mode']='fitted'
        self.assertTrue(compare([base,fit])[0]['passed'])
        for kind in ['accuracy','replay','output']:
            bad=copy.deepcopy(fit)
            if kind=='accuracy': bad['failed_gates']=['localization_accuracy']
            if kind=='replay': bad['replay']['ticks']=1
            if kind=='output': bad['storage']['replay/outputs.jsonl']['sha256']='different'
            self.assertFalse(compare([base,bad])[0]['passed'])


if __name__=='__main__': unittest.main()
