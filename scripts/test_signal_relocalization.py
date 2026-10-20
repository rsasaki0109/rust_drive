#!/usr/bin/env python3
import copy
import tempfile
import unittest
from pathlib import Path

from check_signal_relocalization import freeze, materialize, physical_signals, specifications, verify_signals


def config():
    return dict(route=dict(points=[dict(x=0., y=0.), dict(x=100., y=0.)], half_width=3.5),
                vehicle=dict(radius=1.25), stop_lines=[dict(id='s', route_s_m=35.)])


def snapshot(stamp, color='Green'):
    return dict(stamp=stamp, states=[dict(id='s', color=color)])


def reset(front=33., retained=None, rearmed=None, bad=None):
    return dict(stamp=5., progress_s_m=front-1.25, distance_to_route_m=0., yaw_error_rad=0.,
                admitted=not bad, retained_ids=retained or [], rearmed_ids=rearmed or [], unserved_too_close_ids=bad or [])


def tick(now, observation=None, last=None, color='Unknown', passed=False, front=33., evidence=None, fault=False, rejected=False):
    status = dict(last_accepted_stamp=last, fault=fault, signals=[dict(id='s', color=color, passed=passed)],
                  stop_s_m=32.75 if not passed and (color != 'Green' or rejected) else None)
    if evidence is not None:status['localization_reset'] = evidence
    health = (['InvalidTrafficSignal'] if fault else [])+(['InvalidTrafficSignalRecovery'] if rejected else [])
    return dict(input=dict(time=now, traffic_signal=observation), expected=dict(traffic_controls=status,
                estimate=dict(pose=dict(position=dict(x=front-1.25, y=0.), yaw=0.), speed=0.),
                health=health, command=dict(acceleration=-6. if health else -.5)))


class SignalRecovery(unittest.TestCase):
    def test_frozen_two_plants_seven_compositions_and_curves(self):
        specs = specifications(['reference-2d', 'rne-dynamic']);self.assertEqual(len(specs), 28)
        for spec in specs:
            _, value = materialize(spec)
            self.assertEqual(value['duration'], 65.);self.assertEqual(value['cruise_speed'], 4.)
            self.assertEqual(len(value['localization_map']['points']), 4320)
            self.assertNotIn('navigation', value)
            if spec['mode'] == 'disabled':self.assertNotIn('relocalization', value)
            else:self.assertTrue(value['relocalization']['preserve_traffic_signals'])
            if spec['mode'] == 'mixed-stop':self.assertTrue(value['relocalization']['preserve_stop_signs'])
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);a = freeze(root, specs);self.assertEqual(a, freeze(root, specs))
            (root/a['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):freeze(root, specs)

    def test_old_and_reset_time_green_cannot_supply_new_permission(self):
        r = reset();ticks = [tick(0., snapshot(0.), 0., 'Green'), tick(5., snapshot(5.), evidence=r),
                             tick(5.05, snapshot(5.), evidence=r), tick(5.1, snapshot(5.1, 'Red'), 5.1, 'Red', evidence=r),
                             tick(5.15, snapshot(5.05), 5.1, 'Red', evidence=r), tick(5.2, snapshot(5.2), 5.2, 'Green', evidence=r),
                             tick(5.75, snapshot(5.2), 5.2, evidence=r)]
        self.assertEqual(verify_signals(config(), ticks, 5.)['first_post_reset_acquisition_s'], 5.1)
        for change in ['clock', 'color', 'passage', 'geometry', 'admission', 'endpoint']:
            altered = copy.deepcopy(ticks);s = altered[1]['expected']['traffic_controls']
            if change == 'clock':s['last_accepted_stamp'] = 5.
            elif change == 'color':s['signals'][0]['color'] = 'Green'
            elif change == 'passage':s['signals'][0]['passed'] = True
            elif change == 'geometry':s['localization_reset']['progress_s_m'] += 1.
            elif change == 'admission':s['localization_reset']['admitted'] = False
            else:s['stop_s_m'] = None
            with self.assertRaises(ValueError):verify_signals(config(), altered, 5.)

    def test_passage_is_retained_behind_and_rearmed_ahead(self):
        for front in [33., 40.]:
            retained = front == 40.
            r = reset(front=front, retained=['s'] if retained else [], rearmed=[] if retained else ['s'])
            ticks = [tick(0., snapshot(0.), 0., 'Green', True, 36.), tick(5., front=front, passed=retained, evidence=r)]
            result = verify_signals(config(), ticks, 5.)
            self.assertTrue(result['reset_ownership']['s']);self.assertEqual(result['records'][-1]['passed']['s'], retained)
            altered = copy.deepcopy(ticks);altered[1]['expected']['traffic_controls']['signals'][0]['passed'] = not retained
            with self.assertRaises(ValueError):verify_signals(config(), altered, 5.)

    def test_unserved_crossing_is_latched_despite_new_green(self):
        r = reset(front=36., bad=['s'])
        ticks = [tick(5., front=36., evidence=r, rejected=True), tick(5.2, snapshot(5.2), 5.2, 'Green', front=36., evidence=r, rejected=True),
                 tick(6., snapshot(6.), 6., 'Green', front=36., evidence=r, rejected=True)]
        self.assertTrue(verify_signals(config(), ticks, 5.)['passed'])
        for change in ['brake', 'health', 'passed']:
            altered = copy.deepcopy(ticks)
            if change == 'brake':altered[-1]['expected']['command']['acceleration'] = -.5
            elif change == 'health':altered[-1]['expected']['health'] = []
            else:altered[-1]['expected']['traffic_controls']['signals'][0]['passed'] = True
            with self.assertRaises(ValueError):verify_signals(config(), altered, 5.)

    def test_malformed_snapshot_requires_later_valid_acquisition(self):
        r = reset();bad = dict(stamp=5.1, states=[])
        ticks = [tick(5., evidence=r), tick(5.1, bad, evidence=r, fault=True),
                 tick(5.15, snapshot(5.1), evidence=r, fault=True), tick(5.2, snapshot(5.2), 5.2, 'Green', evidence=r)]
        self.assertTrue(verify_signals(config(), ticks, 5.)['passed'])
        altered = copy.deepcopy(ticks);altered[2]['expected']['traffic_controls']['fault'] = False
        with self.assertRaises(ValueError):verify_signals(config(), altered, 5.)

    def test_another_failed_stop_duty_cannot_commit_signal_passage(self):
        t = tick(5., snapshot(5.), 5., 'Green', front=36.)
        t['expected'].update(health=['InvalidStopSignRecovery'], command=dict(acceleration=-6.))
        self.assertTrue(verify_signals(config(), [t])['passed'])
        altered = copy.deepcopy(t);altered['expected']['traffic_controls']['signals'][0]['passed'] = True
        with self.assertRaises(ValueError):verify_signals(config(), [altered])

    def test_physical_crossing_needs_true_and_acquired_green_in_either_axis(self):
        for vertical in [False, True]:
            def pos(front):return dict(x=0. if vertical else front-1.25, y=front-1.25 if vertical else 0.)
            run = dict(scenario=dict(traffic_signals=[dict(stop_line=dict(id='s', route_s_m=35.), phases=[dict(**{'from':0., 'color':'Red'}), dict(**{'from':2., 'color':'Green'})])]),
                       route=dict(points=[dict(x=0., y=0.), dict(x=0. if vertical else 100., y=100. if vertical else 0.)]),
                       vehicle=dict(radius=1.25), summary=dict(signal_min_stopline_margin_m=2.),
                       frames=[dict(time=0., truth=dict(pose=dict(position=pos(33.)), speed=0.)),
                               dict(time=2., truth=dict(pose=dict(position=pos(36.)), speed=1.))])
            records = [dict(stamp=0., colors={'s':'Red'}, last_accepted_stamp=0.), dict(stamp=2., colors={'s':'Green'}, last_accepted_stamp=2.)]
            self.assertEqual(len(physical_signals(run, records)['crossings']), 1)
            for change in ['observed', 'true', 'summary']:
                altered = copy.deepcopy(run);candidate = copy.deepcopy(records)
                if change == 'observed':candidate[1]['colors']['s'] = 'Unknown'
                elif change == 'true':altered['scenario']['traffic_signals'][0]['phases'][1]['from'] = 3.
                else:altered['summary']['signal_min_stopline_margin_m'] = 1.9
                with self.assertRaises(ValueError):physical_signals(altered, candidate)


if __name__ == '__main__':unittest.main()
