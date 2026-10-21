import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from check_measurement_enclosure import baseline,freeze,materialize,specifications
from measurement_enclosure_oracle import certify,reconstruct
from track_initialization_oracle import reconstruct as filter_oracle
from check_track_initialization import verify_tracker_policies


def fixture():
    ticks=[]
    for time,stamp,x,radius,measured_x,measured_radius,velocity in [
        (0.,0.,0.,1.,0.,1.,0.),(.1,.1,1.5,2.,2.,1.5,2.4),(.15,.1,1.5,2.,2.,1.5,2.4)]:
        ticks.append(dict(input=dict(time=time,lidar=dict(stamp=stamp,points=[])),
                          expected=dict(time=time,health=[],predictions=[],track_measurements=[dict(id=1,stamp=stamp,
                                        center=dict(x=measured_x,y=0.),radius=measured_radius)],
                                        tracks=[dict(id=1,position=dict(x=x,y=0.),velocity=dict(x=velocity,y=0.),radius=radius,last_seen=stamp)])))
    return dict(enclose_track_measurements=True),ticks


class EnclosureTests(unittest.TestCase):
    def test_standalone_matrix_keeps_every_original_world_without_initial_motion_confirmation(self):
        specs=specifications(['reference-2d','rne-dynamic'],initialization='legacy')
        self.assertEqual(len(specs),52)
        for spec in specs:
            baseline(spec)
            _,value=materialize(spec)
            self.assertNotIn('confirm_track_initialization',value)
            self.assertTrue(value['enclose_track_measurements'])
            result_row=dict(spec,initialization=dict(passed=True,confirmations=[]),initialization_mode='legacy')
            self.assertEqual(materialize(result_row),materialize(spec))
        with tempfile.TemporaryDirectory() as directory:
            manifest=freeze(Path(directory),specs)
            self.assertEqual(manifest['gates']['initialization'],'legacy')
            with self.assertRaises(ValueError):freeze(Path(directory),specifications(['reference-2d','rne-dynamic']))

    def test_standalone_coverage_baseline_cannot_hide_the_original_failed_forecast(self):
        spec=specifications(['reference-2d'])[0]
        self.assertNotIn('forecast_coverage',baseline(spec)['failed_gates'])
        self.assertIn('forecast_coverage',baseline(dict(spec,initialization='legacy'))['failed_gates'])

    def test_header_policy_loss_cannot_be_masked_by_another_enabled_tracker_feature(self):
        verify_tracker_policies(dict(enclose_track_measurements=True),dict(enclose_track_measurements=True))
        declared=dict(confirm_track_initialization=True,enclose_track_measurements=True)
        verify_tracker_policies(declared,declared)
        for config in [dict(enclose_track_measurements=True),dict(confirm_track_initialization=True),
                       dict(confirm_track_initialization=1,enclose_track_measurements=True)]:
            with self.subTest(config=config),self.assertRaises(ValueError):verify_tracker_policies(config,declared)
        with self.assertRaises(ValueError):verify_tracker_policies({}, {})

    def test_invalid_or_mixed_initialization_modes_are_rejected_before_freezing(self):
        with self.assertRaises(ValueError):specifications(['reference-2d'],initialization='unrecognized')
        spec=specifications(['reference-2d'])[0]
        with self.assertRaises(ValueError):materialize(dict(spec,initialization='unrecognized'))
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):freeze(Path(directory),[spec,dict(spec,initialization='legacy')])

    def test_closed_form_growing_disc_and_independent_filter_without_bootstrap(self):
        config,ticks=fixture();r=reconstruct(config,ticks)
        self.assertEqual((r['acquired_measurements'],r['associated_updates'],r['cached_tracks']),(2,1,1))
        self.assertTrue(filter_oracle(config,ticks)['passed'])
        self.assertEqual(certify((1.5,0.),2.,dict(center=dict(x=2.,y=0.),radius=1.5),1.1),(0.,0.))

    def test_shrinking_or_ballooning_radius_is_rejected(self):
        for radius in [1.9,2.1]:
            with self.subTest(radius=radius),self.assertRaises(ValueError):
                certify((1.5,0.),radius,dict(center=dict(x=2.,y=0.),radius=1.5),1.1)

    def test_smoothed_disc_can_be_the_active_minimum_with_coincident_centers(self):
        self.assertEqual(certify((0.,0.),2.45,dict(center=dict(x=0.,y=0.),radius=.25),2.45),(0.,0.))
        with self.assertRaises(ValueError):certify((0.,0.),.25,dict(center=dict(x=0.,y=0.),radius=.25),2.45)

    def test_boundary_certificate_is_frame_invariant(self):
        for yaw in [0.,.7,-2.]:
            c,s=math.cos(yaw),math.sin(yaw)
            center=(100.+1.5*c,-70.+1.5*s);m=dict(center=dict(x=100.+2*c,y=-70.+2*s),radius=1.5)
            deficit,slack=certify(center,2.,m,1.1)
            self.assertLess(abs(deficit),1e-12);self.assertLess(abs(slack),1e-12)

    def test_lost_policy_evidence_or_cached_acquisition_cannot_be_hidden(self):
        config,ticks=fixture()
        with self.assertRaises(ValueError):reconstruct({},ticks)
        for mutation in ['radius','stamp','absent','unowned','removed','duplicate']:
            changed=copy.deepcopy(ticks)
            if mutation=='radius':changed[2]['expected']['tracks'][0]['radius']=2.1
            elif mutation=='stamp':changed[1]['expected']['track_measurements'][0]['stamp']=0.
            elif mutation=='absent':changed[1]['expected'].pop('track_measurements')
            elif mutation=='unowned':changed[1]['expected']['track_measurements'][0]['id']=2
            elif mutation=='removed':changed[1]['expected']['track_measurements']=[]
            else:changed[1]['expected']['track_measurements']*=2
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):reconstruct(config,changed)

    def test_nonfinite_and_negative_geometry_is_rejected(self):
        m=dict(center=dict(x=0.,y=0.),radius=1.)
        for radius in [-1.,math.nan,math.inf]:
            with self.subTest(radius=radius),self.assertRaises(ValueError):certify((0.,0.),radius,m,1.)

    def test_all_52_fixed_worlds_change_only_the_new_flag_and_freezing_is_immutable(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),52)
        for spec in specs:
            old=baseline(spec);self.assertEqual((old['plant'],old['world'],old['seed']),(spec['plant'],spec['world'],spec['seed']))
            self.assertTrue(materialize(spec)[1]['enclose_track_measurements'])
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);manifest=freeze(root,specs)
            (root/manifest['episodes'][0]['scenario']).write_text(json.dumps({}))
            with self.assertRaises(ValueError):freeze(root,specs)


if __name__=='__main__':unittest.main()
