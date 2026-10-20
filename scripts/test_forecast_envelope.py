#!/usr/bin/env python3
"""Independent geometric and adversarial tests for forecast error propagation."""
import copy
import math
from pathlib import Path
import tempfile
import unittest

from check_forecast_envelope import coverage, freeze, specifications
from forecast_envelope import audit_envelope, error_at, validate
from test_prediction_evidence import audit, fixture

BOUNDS = dict(position_error_m=.2, velocity_error_m_s=.3, acceleration_error_m_s2=.1)


def envelope_rows(age=.1):
    rows = fixture(False)
    rows[0]['header']['config']['forecast_envelope'] = BOUNDS.copy()
    for row in rows[1:]:
        tick = row['tick'];tick['input']['time'] += age
        current = tick['expected']['tracks'][0];forecast = tick['expected']['predictions'][0]
        for i, p in enumerate(forecast['positions']):
            for axis in ['x', 'y']:
                p[axis] = current['position'][axis]+current['velocity'][axis]*(age+i*.2)
        forecast['additional_radius_m'] = [error_at(BOUNDS, age+i*.2) for i in range(41)]
    return rows


class EnvelopeEvidenceTests(unittest.TestCase):
    def test_acquisition_origin_and_cached_age_are_independently_reconstructed(self):
        for age in [0., .1, .3]:
            self.assertEqual(audit(envelope_rows(age))['enveloped_forecasts'], 7)

    def test_missing_shrunken_shifted_or_ballooned_envelopes_are_rejected(self):
        for kind in ['missing', 'short', 'negative', 'nan', 'shrunk', 'shifted', 'ballooned', 'reset_age', 'policy_removed']:
            rows = envelope_rows();p = rows[-1]['tick']['expected']['predictions'][0]
            if kind == 'missing':p.pop('additional_radius_m')
            elif kind == 'short':p['additional_radius_m'].pop()
            elif kind == 'shifted':p['positions'][20]['y'] += .1
            elif kind == 'reset_age':p['additional_radius_m'] = [error_at(BOUNDS, i*.2) for i in range(41)]
            elif kind == 'policy_removed':rows[0]['header']['config'].pop('forecast_envelope')
            else:p['additional_radius_m'][20] = {'negative':-1., 'nan':math.nan, 'shrunk':0., 'ballooned':100.}[kind]
            with self.subTest(kind=kind), self.assertRaises(ValueError):audit(rows)

    def test_two_discs_require_both_containment_and_minimum_radius_in_any_world_rotation(self):
        for yaw in [0., 1.57, -.7]:
            direction = (math.cos(yaw), math.sin(yaw))
            track = dict(last_seen=0., position=dict(x=0., y=0.), velocity=dict(x=4*direction[0], y=4*direction[1]))
            bounds = dict(position_error_m=2., velocity_error_m_s=0., acceleration_error_m_s2=0.)
            p = dict(dt=1., positions=[dict(x=0., y=0.), dict(x=3*direction[0], y=3*direction[1])], additional_radius_m=[2., 3.])
            audit_envelope(bounds, track, 0., p, [(0., 0.)]*2, [])
            p['positions'][1] = dict(x=4*direction[0], y=4*direction[1])
            with self.assertRaises(ValueError):audit_envelope(bounds, track, 0., p, [(0., 0.)]*2, [])

    def test_cv_error_disc_does_not_erase_the_old_uncertain_slow_disc(self):
        track = dict(last_seen=0., position=dict(x=0., y=0.), velocity=dict(x=4., y=0.))
        bounds = dict(position_error_m=0., velocity_error_m_s=0., acceleration_error_m_s2=0.)
        p = dict(dt=1., positions=[dict(x=0., y=0.), dict(x=2., y=0.)], additional_radius_m=[0., 2.])
        audit_envelope(bounds, track, 0., p, [(0., 0.), (2., 0.)], [0., 2.])
        p['positions'][1]['x'] = 4.;p['additional_radius_m'][1] = 0.
        with self.assertRaises(ValueError):audit_envelope(bounds, track, 0., p, [(0., 0.), (2., 0.)], [0., 2.])

    def test_invalid_unrecognized_and_nonfinite_budgets_are_not_silently_used(self):
        for change in [dict(position_error_m=-1.), dict(velocity_error_m_s=16.), dict(acceleration_error_m_s2=math.inf), dict(position_error_m=True), dict(confidence=.99)]:
            with self.subTest(change=change), self.assertRaises(ValueError):validate(dict(BOUNDS, **change))

    def test_all_paired_inputs_keep_original_worlds_and_freezing_rejects_later_edits(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory);specs = specifications(['reference-2d'])
            manifest = freeze(output, specs)
            self.assertEqual(len(manifest['episodes']), 20)
            import json
            for a, b in zip(manifest['episodes'][::2], manifest['episodes'][1::2]):
                left = json.loads((output/a['scenario']).read_text());right = json.loads((output/b['scenario']).read_text())
                self.assertEqual(right.pop('forecast_envelope'), BOUNDS)
                self.assertEqual(left, right)
            (output/manifest['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):freeze(output, specs)

    def test_unmatched_actors_stay_in_the_coverage_denominator_and_censoring_is_explicit(self):
        frames = [dict(time=i*.05, objects=[dict(id=0, radius=.5, position=dict(x=10., y=0.))]) for i in range(201)]
        t = dict(id=1, last_seen=0., position=dict(x=10., y=0.), velocity=dict(x=0., y=0.), radius=.5)
        p = dict(id=1, radius=.7, dt=.2, positions=[dict(x=10., y=0.)]*41)
        ticks = [dict(expected=dict(tracks=[t], predictions=[p])) for _ in frames]
        self.assertEqual(coverage(dict(frames=frames), ticks)['horizons']['1.0']['covered'], 1)
        changed = copy.deepcopy(ticks);changed[0]['expected']['tracks'][0]['position']['x'] = 100.
        result = coverage(dict(frames=frames), changed)['horizons']['1.0']
        self.assertEqual((result['eligible'], result['unmatched'], result['covered']), (1, 1, 0))


if __name__ == '__main__':unittest.main()
