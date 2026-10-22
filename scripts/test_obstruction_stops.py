#!/usr/bin/env python3
"""Adversarial collision-certificate and post-stop contract tests."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from check_obstruction_stops import audit_contract, certify_trajectory, freeze, specifications


def point(x,y=0.):return dict(x=x,y=y)
def sample(x,speed,time):return dict(position=point(x),speed=speed,time=time)
def prediction(x=100.):return dict(id=1,radius=1.,dt=.2,positions=[point(x)]*41)
def output(time=0.,mode='Yield',path=None,command=-.5):
    return dict(time=time,health=[],estimate=dict(pose=dict(position=point(0.),yaw=0.),speed=0.),
                command=dict(acceleration=command),predictions=[prediction()],
                trajectory=dict(mode=mode,points=path or [sample(0.,0.,0.),sample(0.,0.,8.)]))
def config():return dict(vehicle=dict(radius=1.25),route=dict(points=[point(0.),point(100.)],lengths=[0.,100.]))
def ticks(outputs):return [dict(expected=o) for o in outputs]
def armed():
    o=output(mode='Emergency');o['trajectory']['points']=[];return o


class ObstructionTests(unittest.TestCase):
    def test_clear_accelerated_motion_is_certified_with_full_interval_bounds(self):
        out=output(path=[sample(0.,0.,0.),sample(16.,8.,4.)])
        proof=certify_trajectory(out,1.25)
        self.assertTrue(proof['certified']);self.assertGreater(proof['interval_bounds'],0)

    def test_an_interior_crossing_is_rejected_despite_separated_endpoint_samples(self):
        out=output();out['predictions']=[dict(id=1,radius=.2,dt=.5,positions=[point(0.,5.),point(0.,-5.),point(0.,5.)])]
        with self.assertRaises(ValueError):certify_trajectory(out,.2)
        out['trajectory']['points']=[sample(0.,0.,0.)]
        with self.assertRaises(ValueError):certify_trajectory(out,.2)

    def test_acceleration_integrals_and_growing_radii_cannot_be_replaced_by_static_endpoints(self):
        out=output(path=[sample(0.,0.,0.),sample(4.,4.,2.)]);out['predictions']=[prediction(1.)]
        with self.assertRaises(ValueError):certify_trajectory(out,.1)
        out=output();out['predictions']=[dict(id=1,radius=1.,dt=8.,positions=[point(15.),point(15.)],additional_radius_m=[0.,20.])]
        with self.assertRaises(ValueError):certify_trajectory(out,1.)

    def test_a_complete_clear_path_releases_but_an_accelerating_partial_stop_is_rejected(self):
        hold=output(.1);release=output(.2,path=[sample(0.,0.,0.),sample(40.,8.,10.)],command=2.)
        result=audit_contract(config(),ticks([armed(),hold,release]))
        self.assertTrue(result['passed']);self.assertEqual(result['held_ticks'],1);self.assertEqual(result['releases'],[.2])
        partial=output(.2,path=[sample(0.,0.,0.),sample(1.,2.,1.),sample(2.,0.,2.),sample(2.,0.,10.)],command=2.)
        result=audit_contract(config(),ticks([armed(),hold,partial]))
        self.assertFalse(result['passed']);self.assertEqual(result['violations'][0]['reason'],'restarted_on_partial_blocked_candidate')

    def test_the_high_speed_approach_is_preserved_until_a_real_slow_sample(self):
        start=armed();start['estimate']['speed']=4.
        approach=output(.1,path=[sample(0.,4.,0.),sample(10.,0.,5.),sample(10.,0.,13.)],command=1.)
        approach['estimate']['speed']=4.
        self.assertTrue(audit_contract(config(),ticks([start,approach]))['passed'])
        approach['estimate']['speed']=.1
        self.assertFalse(audit_contract(config(),ticks([start,approach]))['passed'])

    def test_health_braking_does_not_withdraw_the_preceding_collision_stop(self):
        fault=output(.1);fault['health']=['StaleGnss'];fault['command']['acceleration']=-6.
        bad=output(.2,command=2.)
        self.assertFalse(audit_contract(config(),ticks([armed(),fault,bad]))['passed'])
        fault['command']['acceleration']=-.5
        result=audit_contract(config(),ticks([armed(),fault]))
        self.assertEqual(result['violations'][0]['reason'],'health_fault_brake')

    def test_frozen_release_pairs_differ_only_in_policy_and_reject_input_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);manifest=freeze(root,specifications(['reference-2d','rne-dynamic']))
            self.assertEqual(len(manifest['episodes']),56)
            releases=[r for r in manifest['episodes'] if r['world'].startswith('release')]
            for left,right in zip(releases[::2],releases[1::2]):
                a=json.loads((root/left['scenario']).read_text());b=json.loads((root/right['scenario']).read_text())
                self.assertTrue(b.pop('settle_obstruction_stops'));self.assertEqual(a,b)
            (root/manifest['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):freeze(root,specifications(['reference-2d','rne-dynamic']))


if __name__=='__main__':unittest.main()
