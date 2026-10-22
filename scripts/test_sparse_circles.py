#!/usr/bin/env python3
"""Analytic sparse-arc and corrupted-measurement checks for the independent oracle."""
import copy
import math
import unittest

from sparse_circle_oracle import detections, reconstruct
from check_sparse_circles import baseline, materialize, parent, specifications, verify_policy


def arc(count, facing=math.pi, half_span=.5):
    return [dict(x=40.+math.cos(facing-half_span+2*half_span*i/(count-1)),
                 y=math.sin(facing-half_span+2*half_span*i/(count-1))) for i in range(count)]


def fixture():
    config = dict(fit_sparse_circles=True, enclose_track_measurements=True)
    tick = dict(input=dict(time=0., lidar=dict(stamp=0., points=arc(3))),
                expected=dict(tracking_frame=dict(position=dict(x=0.,y=0.),yaw=0.),
                              estimate=dict(pose=dict(position=dict(x=0.,y=0.),yaw=0.)),
                              health=[], track_measurements=[dict(id=1,stamp=0.,center=dict(x=40.,y=0.),radius=1.12)]))
    return config, [tick]


class SparseCircles(unittest.TestCase):
    def test_analytic_visible_arcs(self):
        for n in [3, 4, 5, 8]:
            d = detections(arc(n), True)[0]
            self.assertTrue(d['fitted'])
            self.assertLess(math.dist(d['center'], (40., 0.)), 1e-9)
            self.assertAlmostEqual(d['radius'], 1.12)

    def test_legacy_sparse_proxy_is_not_the_circular_center(self):
        for n in [3, 4]:
            d = detections(arc(n), False)[0]
            self.assertFalse(d['fitted'])
            self.assertGreater(math.dist(d['center'], (40., 0.)), .85)

    def test_flat_short_and_far_surface_arcs_keep_the_proxy(self):
        flat = [dict(x=40.,y=y) for y in [-.5,0.,.5]]
        for points in [flat, arc(3,half_span=.2), arc(4,facing=0.)]:
            self.assertEqual(detections(points,True),detections(points,False))

    def test_fourth_contradictory_return_is_not_discarded(self):
        points = arc(4); points[1]['x'] += .5
        self.assertEqual(detections(points,True),detections(points,False))

    def test_fresh_complete_geometry_is_reconstructed(self):
        config,ticks=fixture(); result=reconstruct(config,ticks)
        self.assertTrue(result['passed']);self.assertEqual(result['sparse_fits'],1)
        self.assertFalse(result['physical_shape_confidence_claimed'])

    def test_center_and_radius_corruption_are_rejected(self):
        for field in ['center','radius','stamp']:
            config,ticks=fixture();measured=ticks[0]['expected']['track_measurements'][0]
            if field=='center':measured['center']['x']-=.3
            else:measured[field]+=.3
            with self.assertRaisesRegex(ValueError,'geometry differs'): reconstruct(config,ticks)

    def test_missing_cluster_and_lost_policy_are_rejected(self):
        config,ticks=fixture(); ticks[0]['expected']['track_measurements']=[]
        with self.assertRaisesRegex(ValueError,'cluster set'): reconstruct(config,ticks)
        config,ticks=fixture();config['fit_sparse_circles']=False
        with self.assertRaisesRegex(ValueError,'geometry differs'): reconstruct(config,ticks)

    def test_dense_arcs_do_not_change(self):
        for n in [5,8,20]:
            self.assertEqual(detections(arc(n),True),detections(arc(n),False))

    def test_unsupported_and_nonboolean_policies_are_rejected(self):
        for changes in [dict(fit_sparse_circles='true'),dict(lidar3d={'mode':'xy'})]:
            config,ticks=fixture();config.update(changes)
            with self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_frame_changes_do_not_enter_the_sensor_fit(self):
        config,ticks=fixture();tick=ticks[0]
        tick['expected']['tracking_frame']=dict(position=dict(x=7.,y=-2.),yaw=.7)
        tick['expected']['estimate']['pose']=copy.deepcopy(tick['expected']['tracking_frame'])
        self.assertTrue(reconstruct(config,ticks)['passed'])

    def test_all_frozen_inputs_change_only_the_declared_fit_policy(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),32)
        for spec in specs:
            _,frozen=materialize(spec);original=copy.deepcopy(frozen);original.pop('fit_sparse_circles')
            self.assertEqual(original,parent()['frozen_inputs'][baseline(spec)['scenario']]['input'])
            verify_policy(dict(fit_sparse_circles=True),frozen,spec)

    def test_world_deadline_and_motion_budget_changes_are_rejected(self):
        spec=specifications(['reference-2d'])[0]
        for key in ['duration','forecast_envelope']:
            _,frozen=materialize(spec)
            if key=='duration':frozen[key]+=1.
            else:frozen[key]['position_error_m']*=2.
            with self.assertRaisesRegex(ValueError,'physical input or error budget'):
                verify_policy(dict(fit_sparse_circles=True),frozen,spec)

    def test_missing_operational_policy_is_rejected(self):
        spec=specifications(['reference-2d'])[0];_,frozen=materialize(spec)
        with self.assertRaisesRegex(ValueError,'fitting lost'):verify_policy({},frozen,spec)


if __name__=='__main__':unittest.main()
