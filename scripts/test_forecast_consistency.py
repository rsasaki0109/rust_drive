#!/usr/bin/env python3
"""Closed-form geometry and corrupted evidence controls for forecast braking."""
import copy
import unittest

from forecast_consistency_oracle import blank, reconstruct, witness


def fixture():
    config = dict(forecast_consistency_guard=True, enclose_track_measurements=True,
                  forecast_envelope=dict(position_error_m=.2, velocity_error_m_s=.3, acceleration_error_m_s2=.1))
    ticks = [];report = blank()
    for i, now in enumerate([0., .1, .15, .2, .3, .4]):
        stamp = .1 if i == 2 else now;x = 0. if i == 0 else .4
        if i != 2:
            report.update(last_scan_acquisition_s=stamp, evaluated_acquisition_s=stamp, evaluated_at_control_s=now,
                          checked_measurements=int(i > 0), unmonitored_measurements=int(i == 0),
                          complete_scene_check=i > 0, retained_references=report['retained_references']+1,
                          misses=[], missed_measurements=0)
        if i == 1:
            report.update(state='Holding', fault='MeasuredMiss', fault_control_time_s=.1, recovery_after_control_s=.1,
                          held_identities=[7], missed_measurements=1, misses=[dict(id=7, reference_origin_s=0.,
                          reference_acquisition_s=0., measurement_acquisition_s=.1, predicted_center=dict(x=0., y=0.),
                          predicted_radius_m=.7, measured_center=dict(x=.4, y=0.), measured_radius_m=.5, shortfall_m=.2)])
        elif i in [3, 4]:
            report.update(state='Confirming', recovery_samples=i-2, recovery_first_acquisition_s=.2)
        elif i == 5:
            report.update(state='Observing', fault=None, fault_control_time_s=None, recovery_after_control_s=None,
                          held_identities=[], recovery_samples=0, recovery_first_acquisition_s=None)
        blocked = report['fault'] is not None
        out = dict(time=now, tracks=[dict(id=7, position=dict(x=x, y=0.), velocity=dict(x=0., y=0.), radius=.5, last_seen=stamp)],
                   predictions=[dict(id=7, positions=[dict(x=x, y=0.)]*9, radius=.7, dt=.2)],
                   track_measurements=[dict(id=7, center=dict(x=x, y=0.), radius=.5, stamp=stamp)],
                   health=['InconsistentForecast'] if blocked else [], emergency=blocked,
                   command=dict(acceleration=-6. if blocked else 0., steering=0.), forecast_consistency=copy.deepcopy(report))
        ticks.append(dict(input=dict(time=now, lidar=None if i == 2 else dict(stamp=stamp, points=[]), lidar_failed=False), expected=out))
    return config, ticks


class ForecastConsistencyTests(unittest.TestCase):
    def test_qualification_changes_only_the_guard_and_excludes_undeclared_envelopes(self):
        from check_forecast_consistency import baseline, materialize, parent, specifications
        specs = specifications(['reference-2d', 'rne-dynamic'])
        self.assertEqual(len(specs), 32)
        for spec in specs:
            _, value = materialize(spec);self.assertTrue(value.pop('forecast_consistency_guard'))
            self.assertEqual(value, parent()['frozen_inputs'][baseline(spec)['scenario']]['input'])

    def test_closed_form_boundary_uses_the_whole_measured_disc_and_interpolated_radius(self):
        prediction = dict(positions=[dict(x=0., y=0.), dict(x=2., y=0.)], radius=.5, dt=1., additional_radius_m=[0., .2])
        result = witness((0., 0., prediction), dict(id=1, stamp=.5, center=dict(x=1., y=.4), radius=.3))
        self.assertAlmostEqual(result['predicted_radius_m'], .6)
        self.assertAlmostEqual(result['shortfall_m'], .1)
        self.assertEqual(result['predicted_center'], dict(x=1., y=0.))

    def test_reference_origin_and_horizon_cannot_be_extended(self):
        prediction = dict(positions=[dict(x=0., y=0.)]*2, radius=.5, dt=1.)
        for stamp in [.1, 1.3]:
            with self.assertRaises(ValueError):witness((.2, 0., prediction), dict(id=1, stamp=stamp, center=dict(x=0., y=0.), radius=.3))

    def test_a_miss_cached_scan_and_three_fresh_acquisitions_have_a_known_outcome(self):
        config, ticks = fixture();result = reconstruct(config, ticks)
        self.assertEqual(result['checked_measurements'], 4)
        self.assertEqual(result['unmonitored_measurements'], 1)
        self.assertEqual(result['missed_measurements'], 1)
        self.assertEqual(result['braking_control_ticks'], 4)
        self.assertEqual(result['recoveries'], [dict(hold_start_control_s=.1, release_control_s=.4, first_acquisition_s=.2, last_acquisition_s=.4)])

    def test_missing_capture_changed_budget_policy_or_fabricated_reference_is_rejected(self):
        for mutation in ['flag', 'enclosure', 'capture', 'origin', 'acquisition', 'radius', 'center', 'shortfall', 'identity']:
            config, ticks = fixture();report = ticks[1]['expected']['forecast_consistency']
            if mutation == 'flag':config['forecast_consistency_guard'] = False
            elif mutation == 'enclosure':config['enclose_track_measurements'] = False
            elif mutation == 'capture':ticks[1]['expected'].pop('forecast_consistency')
            elif mutation == 'origin':report['misses'][0]['reference_origin_s'] = .1
            elif mutation == 'acquisition':report['misses'][0]['measurement_acquisition_s'] = .15
            elif mutation == 'radius':report['misses'][0]['predicted_radius_m'] = .8
            elif mutation == 'center':report['misses'][0]['predicted_center']['x'] = .1
            elif mutation == 'shortfall':report['misses'][0]['shortfall_m'] = 0.
            else:report['misses'][0]['id'] = 8
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):reconstruct(config, ticks)

    def test_cached_delivery_and_recovery_counters_cannot_be_fabricated(self):
        for mutation in ['cached-vote', 'early-release', 'first', 'last', 'fault-cutoff', 'references', 'new-field']:
            config, ticks = fixture();report = ticks[2]['expected']['forecast_consistency']
            if mutation == 'cached-vote':report['recovery_samples'] = 1
            elif mutation == 'early-release':report['state'] = 'Observing';report['fault'] = None
            elif mutation == 'first':ticks[4]['expected']['forecast_consistency']['recovery_first_acquisition_s'] = .1
            elif mutation == 'last':report['last_scan_acquisition_s'] = .15
            elif mutation == 'fault-cutoff':report['recovery_after_control_s'] = 0.
            elif mutation == 'references':report['retained_references'] += 1
            else:report['extra'] = True
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):reconstruct(config, ticks)

    def test_lost_fault_identity_or_an_unmeasured_active_identity_cannot_clear_a_hold(self):
        for mutation in ['disappeared', 'replacement', 'unobserved']:
            config, ticks = fixture()
            for tick in ticks[3:]:
                out = tick['expected']
                if mutation == 'disappeared':out['tracks'] = [];out['predictions'] = [];out['track_measurements'] = []
                elif mutation == 'replacement':
                    for key in ['tracks', 'predictions', 'track_measurements']:out[key][0]['id'] = 8
                else:
                    for key in ['tracks', 'predictions']:
                        extra = copy.deepcopy(out[key][0]);extra['id'] = 8;out[key].append(extra)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):reconstruct(config, ticks)

    def test_removed_same_cycle_braking_emergency_or_health_is_rejected(self):
        for mutation in ['acceleration', 'steering', 'emergency', 'health']:
            config, ticks = fixture();out = ticks[1]['expected']
            if mutation == 'acceleration':out['command']['acceleration'] = 0.
            elif mutation == 'steering':out['command']['steering'] = .1
            elif mutation == 'emergency':out['emergency'] = False
            else:out['health'] = []
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):reconstruct(config, ticks)

    def test_upstream_faults_preclude_recovery_and_downstream_faults_keep_their_own_duties(self):
        config, ticks = fixture();ticks[3]['expected']['health'].append('StaleGnss')
        with self.assertRaises(ValueError):reconstruct(config, ticks)
        config, ticks = fixture()
        for tick in ticks:tick['expected']['health'].append('InvalidNavigation')
        self.assertEqual(reconstruct(config, ticks)['final_state'], 'Observing')

    def test_rigid_coordinate_changes_preserve_geometry(self):
        config, ticks = fixture()
        for tick in ticks:
            out = tick['expected'];out['tracking_frame'] = dict(position=dict(x=17., y=-3.), yaw=0.)
            for t in out['tracks']:t['position']['x'] += 17.;t['position']['y'] -= 3.
            for p in out['predictions']:
                p['positions'] = [dict(x=q['x']+17., y=q['y']-3.) for q in p['positions']]
        self.assertTrue(reconstruct(config, ticks)['passed'])

    def test_duplicate_and_nonfinite_raw_measurements_are_rejected(self):
        for mutation in ['duplicate', 'nan', 'future', 'wrong-track']:
            config, ticks = fixture();out = ticks[1]['expected']
            if mutation == 'duplicate':out['track_measurements'] *= 2
            elif mutation == 'nan':out['track_measurements'][0]['radius'] = float('nan')
            elif mutation == 'future':out['track_measurements'][0]['stamp'] = .2
            else:out['tracks'][0]['last_seen'] = 0.
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):reconstruct(config, ticks)


if __name__ == '__main__':unittest.main()
