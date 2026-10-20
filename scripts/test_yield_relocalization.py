#!/usr/bin/env python3
import copy
import tempfile
import unittest
from pathlib import Path

from check_yield_relocalization import blockers_for, freeze, loss_window, materialize, physical_yields, specifications, verify_yields


def config():
    return dict(route=dict(points=[dict(x=0., y=0.), dict(x=100., y=0.)], half_width=3.5), vehicle=dict(radius=1.25),
                yield_intersections=[dict(stop_line=dict(id='z', route_s_m=35.),
                                         conflict_bounds=dict(min=dict(x=42., y=-3.5), max=dict(x=48., y=3.5)), exit_s_m=48.)])


def reset(front=33., committed=False, passed=False, rearmed=False, bad=False):
    return dict(stamp=5., progress_s_m=front-1.25, distance_to_route_m=0., yaw_error_rad=0., admitted=not bad,
                retained_committed_ids=['z'] if committed else [], retained_passed_ids=['z'] if passed else [],
                rearmed_ids=['z'] if rearmed else [], unserved_too_close_ids=['z'] if bad else [])


def tick(now, front=33., phase='Waiting', committed=False, since=None, stamp=None, evidence=None, health=None):
    health = health or []
    status = dict(zones=[dict(id='z', phase=phase, blocking_tracks=[], clear_since=since, committed=committed)], stop_s_m=32.75 if phase == 'Waiting' else None)
    if evidence is not None:status['localization_reset'] = evidence
    return dict(input=dict(time=now, lidar=dict(stamp=now if stamp is None else stamp, points=[]), lidar_failed=False),
                expected=dict(estimate=dict(pose=dict(position=dict(x=front-1.25, y=0.), yaw=0.), speed=0.),
                              health=health, command=dict(acceleration=-6. if health else -.5), predictions=[], intersections=status))


def clear_prefix():
    return [tick(i*.05, phase='Proceeding' if i == 20 else 'Waiting', since=0.) for i in range(21)]


class YieldRecovery(unittest.TestCase):
    def test_frozen_two_plants_two_seeds_and_all_compositions(self):
        specs = specifications(['reference-2d', 'rne-dynamic']);self.assertEqual(len(specs), 32)
        for spec in specs:
            _, v = materialize(spec);self.assertEqual(v['duration'], 95.)
            self.assertEqual(v['cruise_speed'], 4.);self.assertEqual(v['curve_amplitude'], 0.)
            if spec['mode'] != 'disabled':self.assertTrue(v['relocalization']['preserve_yield_intersections'])
            else:self.assertNotIn('relocalization', v)
            if spec['mode'] in ['committed', 'passed']:
                self.assertTrue(all(abs(o['lateral']) == 7.5 for o in v['objects'] if o['s'] in [22, 30]))
                self.assertEqual(v['yield_intersections'][0]['conflict_bounds']['min']['y'], -3.5)
            if spec['mode'] == 'passed':self.assertEqual(next(o['lateral'] for o in v['objects'] if o['s'] == 83), 7.5)
        self.assertEqual(loss_window('committed'), [11., 14.])
        self.assertEqual(loss_window('committed', 'clear-landmarks-v2'), [14., 17.])
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder);freeze(root, specs);freeze(root, specs)
            (root/'frozen/000.json').write_text('{}')
            with self.assertRaises(ValueError):freeze(root, specs)

    def test_reset_time_cached_and_reordered_scans_cannot_supply_new_clear_time(self):
        r = reset(rearmed=True);ticks = clear_prefix()+[tick(5., evidence=r)]
        ticks += [tick(5.+i*.05, stamp=5., evidence=r) for i in range(1, 4)]
        ticks += [tick(5.2+i*.05, phase='Proceeding' if i == 20 else 'Waiting', since=5.2, evidence=r) for i in range(21)]
        result = verify_yields(config(), ticks, 5.);self.assertEqual(result['releases'][-1]['first_acquisition_s'], 5.2)
        for field, value in [('clear_since', 4.), ('phase', 'Proceeding'), ('committed', True)]:
            altered = copy.deepcopy(ticks);altered[21]['expected']['intersections']['zones'][0][field] = value
            with self.assertRaises(ValueError):verify_yields(config(), altered, 5.)
        altered = copy.deepcopy(ticks);altered[-1]['input']['lidar']['stamp'] = 5.1
        with self.assertRaises(ValueError):verify_yields(config(), altered, 5.)

    def test_owned_entry_is_retained_inside_passed_behind_and_rearmed_ahead(self):
        for front, phase, committed, passed, rearmed in [(45., 'Proceeding', True, False, False), (52., 'Passed', False, True, False), (33., 'Waiting', False, False, True)]:
            r = reset(front, committed, passed, rearmed)
            ticks = clear_prefix()+[tick(1.05, front=36., phase='Proceeding', committed=True, since=0.),
                                    tick(5., front=front, phase=phase, committed=not rearmed, evidence=r)]
            self.assertTrue(verify_yields(config(), ticks, 5.)['reset_ownership'][0]['committed'])
            altered = copy.deepcopy(ticks);altered[-1]['expected']['intersections']['localization_reset']['retained_committed_ids'] = ['fake']
            with self.assertRaises(ValueError):verify_yields(config(), altered, 5.)

    def test_unserved_entry_cannot_be_pardoned_by_measured_position_or_fresh_scans(self):
        for front in [34.6, 36., 52.]:
            r = reset(front, bad=True)
            ticks = [tick(5.+i*.05, front=front, evidence=r, health=['InvalidIntersectionRecovery']) for i in range(31)]
            self.assertFalse(verify_yields(config(), ticks, 5.)['reset']['admitted'])
            for change in ['health', 'brake', 'ownership']:
                altered = copy.deepcopy(ticks)
                if change == 'health':altered[-1]['expected']['health'] = []
                elif change == 'brake':altered[-1]['expected']['command']['acceleration'] = -.5
                else:altered[-1]['expected']['intersections']['zones'][0]['committed'] = True
                with self.assertRaises(ValueError):verify_yields(config(), altered, 5.)

    def test_another_control_fault_cannot_establish_clear_permission(self):
        ticks = [tick(i*.05, health=['InvalidStopSignRecovery']) for i in range(31)]
        self.assertTrue(verify_yields(config(), ticks)['passed'])
        altered = copy.deepcopy(ticks);altered[-1]['expected']['intersections']['zones'][0].update(phase='Proceeding', clear_since=0.)
        with self.assertRaises(ValueError):verify_yields(config(), altered)

    def test_forecast_sweep_handles_partial_tail_expanding_radius_and_malformed_data(self):
        z = config()['yield_intersections'][0]
        p = dict(id=8, dt=3., radius=1., positions=[dict(x=0., y=8.), dict(x=0., y=8.), dict(x=0., y=8.), dict(x=90., y=8.)])
        self.assertEqual(blockers_for(z, [p]), [])
        p['additional_radius_m'] = [0., 0., 5., 5.]
        self.assertEqual(blockers_for(z, [p]), [8])
        p['additional_radius_m'] = [0.]
        self.assertEqual(blockers_for(z, [p]), [8])
        p['positions'] = [];self.assertEqual(blockers_for(z, [p]), [8])

    def test_independent_actual_priority_overlap_is_detected_despite_permissive_diagnostics(self):
        c = config();frames = [dict(time=t, truth=dict(pose=dict(position=dict(x=x, y=0.)), speed=1.),
                                   objects=[dict(id=1, position=dict(x=45., y=0.), radius=1.)]) for t, x in [(0., 30.), (1., 45.), (2., 52.)]]
        run = dict(scenario=dict(yield_intersections=c['yield_intersections']), route=c['route'], vehicle=c['vehicle'], frames=frames,
                   summary=dict(intersection_min_gap_s=-1., intersection_violations=1))
        records = [dict(stamp=t, zones=[dict(phase='Proceeding')]) for t in [0., 1., 2.]]
        self.assertEqual(physical_yields(run, records)['violations'], 1)
        run['summary']['intersection_violations'] = 0
        with self.assertRaises(ValueError):physical_yields(run, records)


if __name__ == '__main__':unittest.main()
