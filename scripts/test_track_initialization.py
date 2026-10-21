import copy
import json
from pathlib import Path
import tempfile
import unittest

from check_track_initialization import baseline, compare, freeze, materialize, specifications
from track_initialization_oracle import fitted, reconstruct


def fixture():
    ticks=[]
    for time,position,velocity in [(0.,0.,0.),(.1,.15,0.),(.2,.4,2.),(.25,.4,2.)]:
        acquired=min(time,.2)
        m=dict(id=1,stamp=acquired,center=dict(x=2*acquired,y=0.),radius=1.)
        ticks.append(dict(input=dict(time=time,lidar=dict(stamp=acquired,points=[])),expected=dict(time=time,health=[],
                     track_measurements=[m],tracks=[dict(id=1,last_seen=acquired,position=dict(x=position,y=0.),
                                                        velocity=dict(x=velocity,y=0.),radius=1.)],predictions=[])))
    return dict(confirm_track_initialization=True),ticks


class InitializationTests(unittest.TestCase):
    def test_optional_coverage_does_not_abort_recording_and_new_failures_still_fail(self):
        row=dict(summary=dict(passed=True,steps=1),run_exit_code=0,replay_exit_code=0,
                 replay=dict(verified=True,ticks=1),failed_gates=[],coverage=None)
        old=dict(failed_gates=[],coverage=None)
        self.assertTrue(compare(row,old)['regression_passed'])
        self.assertEqual(compare(row,old)['coverage'],{})
        row['failed_gates']=['forecast_coverage']
        self.assertFalse(compare(row,old)['regression_passed'])
        row['failed_gates']=[];row['replay']['verified']=False
        self.assertFalse(compare(row,old)['regression_passed'])

    def test_closed_form_three_observations_and_unrenewed_cache(self):
        config,ticks=fixture();r=reconstruct(config,ticks)
        self.assertEqual((r['births'],r['updates'],r['cached_ticks']),(1,2,1))
        self.assertEqual(len(r['confirmations']),1)
        self.assertAlmostEqual(r['confirmations'][0]['speed_m_s'],2.)

    def test_raw_time_radius_position_velocity_and_missing_tracks_are_checked(self):
        config,ticks=fixture()
        for kind in ['time','radius','position','velocity','missing','measurement']:
            changed=copy.deepcopy(ticks)
            if kind=='time':changed[1]['expected']['track_measurements'][0]['stamp']=.11
            elif kind=='radius':changed[1]['expected']['tracks'][0]['radius']=2.
            elif kind=='position':changed[2]['expected']['tracks'][0]['position']['x']+=.01
            elif kind=='velocity':changed[2]['expected']['tracks'][0]['velocity']['x']+=.01
            elif kind=='missing':changed[2]['expected']['tracks']=[]
            else:changed[2]['expected']['track_measurements']=[]
            with self.subTest(kind=kind),self.assertRaises(ValueError):reconstruct(config,changed)

    def test_cached_measurements_cannot_be_renewed_or_changed(self):
        config,ticks=fixture()
        for kind in ['time','center','removed']:
            changed=copy.deepcopy(ticks)
            if kind=='time':changed[3]['expected']['track_measurements'][0]['stamp']=.25
            elif kind=='center':changed[3]['expected']['track_measurements'][0]['center']['x']+=.01
            else:changed[3]['expected']['track_measurements']=[]
            with self.subTest(kind=kind),self.assertRaises(ValueError):reconstruct(config,changed)

    def test_every_intervening_measurement_contributes_to_fit_residual(self):
        history=[dict(stamp=t,center=dict(x=x,y=0.)) for t,x in [(0.,0.),(.1,1.),(.2,.4)]]
        self.assertIsNone(fitted(history))
        history[1]['center']['x']=.2
        self.assertAlmostEqual(fitted(history)[1][0],2.)

    def test_omitted_policy_and_diagnostic_are_rejected(self):
        config,ticks=fixture()
        with self.assertRaises(ValueError):reconstruct({},ticks)
        ticks[0]['expected'].pop('track_measurements')
        with self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_frozen_matrix_changes_only_declared_flag_and_retains_all_52_baselines(self):
        specs=specifications(['reference-2d','rne-dynamic'])
        self.assertEqual(len(specs),52)
        for spec in specs:
            old=baseline(spec)
            self.assertEqual(old['episode']['plant'],spec['plant'])
            self.assertEqual(old['episode']['world'],spec['world'])
            self.assertTrue(materialize(spec)[1]['confirm_track_initialization'])
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory);manifest=freeze(output,specs)
            (output/manifest['episodes'][0]['scenario']).write_text(json.dumps({}))
            with self.assertRaises(ValueError):freeze(output,specs)


if __name__=='__main__':unittest.main()
