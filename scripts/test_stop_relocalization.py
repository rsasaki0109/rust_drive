#!/usr/bin/env python3
import copy
import tempfile
import unittest
from pathlib import Path

from check_stop_relocalization import freeze, materialize, physical_recovered_holds, physical_stops, specifications, verify_stops


def config():
    return dict(route=dict(points=[dict(x=0., y=0.), dict(x=100., y=0.)], half_width=3.5),
                vehicle=dict(radius=1.25), stop_signs=[dict(id='s', route_s_m=35.)])


def tick(now, front=33., phase='Holding', held=0., reset=None, healthy=True):
    status = dict(stops=[dict(id='s', phase=phase, held_s=held)], stop_s_m=None if phase in ['Released', 'Passed'] else 32.75)
    if reset is not None:status['localization_reset'] = reset
    return dict(input=dict(time=now, odometry=dict(stamp=now, speed=0., yaw_rate=0.)),
                expected=dict(estimate=dict(pose=dict(position=dict(x=front-1.25, y=0.), yaw=0.), speed=0.),
                              health=[] if healthy else ['StaleGnss'], command=dict(acceleration=-.5), stop_signs=status))


def reset(now, front, retained=None, rearmed=None, bad=None):
    return dict(stamp=now, progress_s_m=front-1.25, distance_to_route_m=0., yaw_error_rad=0.,
                admitted=not bad, retained_ids=retained or [], rearmed_ids=rearmed or [], unserved_too_close_ids=bad or [])


class StopRecovery(unittest.TestCase):
    def test_frozen_32_cases_retain_calibration_and_explicit_stop_policy(self):
        specs = specifications(['reference-2d', 'rne-dynamic'])
        self.assertEqual(len(specs), 32);self.assertEqual(len(specifications(['reference-2d'], True)), 12)
        for spec in specs:
            _, value = materialize(spec)
            self.assertEqual(value['cruise_speed'], 4.);self.assertEqual(value['duration'], 65.)
            self.assertEqual(len(value['localization_map']['points']), 4320)
            self.assertNotIn('navigation', value)
            if spec['mode'] != 'healthy':
                loss = {'during-hold':11.2, 'released':12.9, 'after-passed':16.}.get(spec['mode'],8.)
                self.assertEqual(value['sensor_timing']['odometry_dropout_windows'], [{'from':loss, 'until':loss+3.}])
                self.assertEqual(value['gnss_dropout_windows'], [{'from':loss, 'until':loss+8.}])
            if spec['mode'] == 'disabled':self.assertNotIn('relocalization', value)
            else:self.assertTrue(value['relocalization']['preserve_stop_signs'])
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);a = freeze(root, specs);self.assertEqual(freeze(root, specs), a)
            (root/a['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):freeze(root, specs)

    def test_reset_never_credits_old_hold_or_cached_motion(self):
        c = config();r = reset(5., 33.)
        ticks = [tick(i*.05, held=i*.05) for i in range(21)]
        ticks.append(tick(5., phase='Approaching', reset=r, healthy=False))
        ticks += [tick(5.+i*.05, phase='Released' if i == 41 else 'Holding', held=(i-1)*.05, reset=r) for i in range(1, 42)]
        result = verify_stops(c, ticks, 5.)
        self.assertAlmostEqual(result['releases'][0]['stamp'], 7.05)
        self.assertAlmostEqual(result['releases'][0]['start'], 5.05)
        for change in ['old_credit', 'cached', 'geometry', 'release', 'brake']:
            altered = copy.deepcopy(ticks)
            if change == 'old_credit':altered[23]['expected']['stop_signs']['stops'][0]['held_s'] += 1.
            elif change == 'cached':altered[23]['input']['odometry'] = None
            elif change == 'geometry':altered[21]['expected']['stop_signs']['localization_reset']['progress_s_m'] += 1.
            elif change == 'release':altered[30]['expected']['stop_signs']['stops'][0]['phase'] = 'Released'
            else:altered[23]['expected']['command']['acceleration'] = -.49
            with self.assertRaises(ValueError):verify_stops(c, altered, 5.)

    def test_completed_stop_is_retained_behind_and_rearmed_ahead(self):
        for front in [33., 40.]:
            ticks = [tick(i*.05, held=i*.05, phase='Released' if i == 40 else 'Holding') for i in range(41)]
            ticks.append(tick(2.05, front=36., phase='Passed', held=2.))
            r = reset(3., front, retained=['s'] if front == 40. else [], rearmed=['s'] if front == 33. else [])
            ticks.append(tick(3., front=front, phase='Passed' if front == 40. else 'Approaching', held=2. if front == 40. else 0., reset=r, healthy=False))
            result = verify_stops(config(), ticks, 3.)
            self.assertEqual(result['final_stops'][0]['phase'], 'Passed' if front == 40. else 'Approaching')
            altered = copy.deepcopy(ticks);altered[-1]['expected']['stop_signs']['stops'][0]['held_s'] = 1.
            with self.assertRaises(ValueError):verify_stops(config(), altered, 3.)

    def test_unserved_crossing_admission_fault_stays_braked(self):
        r = reset(0., 36., bad=['s'])
        ticks = [tick(i*.05, front=36., phase='Approaching', reset=r) for i in range(4)]
        for t in ticks:t['expected'].update(health=['InvalidStopSignRecovery'], command=dict(acceleration=-6.))
        self.assertTrue(verify_stops(config(), ticks, 0.)['passed'])
        for change in ['brake', 'health', 'admission']:
            altered = copy.deepcopy(ticks)
            if change == 'brake':altered[2]['expected']['command']['acceleration'] = -.5
            elif change == 'health':altered[2]['expected']['health'] = []
            else:altered[0]['expected']['stop_signs']['localization_reset']['admitted'] = True
            with self.assertRaises(ValueError):verify_stops(config(), altered, 0.)

    def test_physical_scoring_uses_route_progress_and_ignores_driver_claims(self):
        for vertical in [False, True]:
            def pose(front):return dict(position=dict(x=0. if vertical else front-1.25, y=front-1.25 if vertical else 0.), yaw=0.)
            frames = [dict(time=i*.05, truth=dict(pose=pose(33.), speed=0.)) for i in range(42)]
            frames.append(dict(time=2.1, truth=dict(pose=pose(36.), speed=1.)))
            run = dict(scenario=dict(stop_signs=[dict(id='s', route_s_m=35.)]), route=dict(points=[dict(x=0.,y=0.), dict(x=0. if vertical else 100.,y=100. if vertical else 0.)]),
                       vehicle=dict(radius=1.25), frames=frames, summary=dict(stop_sign_min_margin_m=2.))
            self.assertTrue(physical_stops(run)['crossings'][0]['satisfied'])
            release = [dict(id='s',start=0.,stamp=2.,after_reset=True)]
            self.assertEqual(physical_recovered_holds(run, release)[0]['duration_s'], 2.)
            for change in ['rolling', 'distant', 'short']:
                altered = copy.deepcopy(run);candidate = copy.deepcopy(release)
                if change == 'rolling':altered['frames'][20]['truth']['speed'] = .2
                elif change == 'distant':altered['frames'][20]['truth']['pose'] = pose(30.)
                else:candidate[0]['start'] = .05
                with self.assertRaisesRegex(ValueError, 'physical two-second'):physical_recovered_holds(altered, candidate)
            altered = copy.deepcopy(run);altered['summary']['stop_sign_violations'] = 1
            with self.assertRaisesRegex(ValueError, 'physical stop scoring'):physical_stops(altered)
            altered = copy.deepcopy(run);altered['frames'][20]['truth']['speed'] = .2
            with self.assertRaisesRegex(ValueError, 'physical stop scoring'):physical_stops(altered)


if __name__ == '__main__':unittest.main()
