#!/usr/bin/env python3
import copy
import tempfile
import unittest
from pathlib import Path

from check_navigation_handover import freeze, goal_improvement, materialize, specifications
from check_navigation_relocalization import materialize as prior_materialize, verify_navigation
from test_navigation_relocalization import handover_ticks


def held_ticks():
    config, ticks = handover_ticks()
    config['navigation']['hold_for_handover'] = True
    for i in [0, 1, 3, 4]:
        ticks[i]['expected']['navigation']['handover_hold'] = True
        ticks[i]['expected']['command']['acceleration'] = -.5
    return config, ticks


class BrakedHandover(unittest.TestCase):
    def test_frozen_inputs_only_enable_braked_confirmation(self):
        specs = specifications(['reference-2d', 'rne-dynamic'])
        self.assertEqual(len(specs), 32)
        self.assertEqual(sum(s['mode'] == 'late-closure' for s in specs), 12)
        for spec in specs:
            source, value = materialize(spec)
            before_source, before = prior_materialize(spec)
            self.assertTrue(value['navigation'].pop('hold_for_handover'))
            self.assertEqual((source, value), (before_source, before))
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            first = freeze(root, specs)
            self.assertEqual(freeze(root, specs), first)
            (root/first['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):
                freeze(root, specs)

    def test_closed_form_hold_covers_cached_motion_and_reset_clears_old_votes(self):
        config, ticks = held_ticks()
        result = verify_navigation(config, ticks, .1)
        self.assertEqual(result['verified_handover_hold_ticks'], 4)
        self.assertEqual(result['switch_events'][0]['stamp'], .25)
        for corruption in ['lost_brake', 'invented_hold', 'cached_vote', 'unhealthy_hold', 'early_release']:
            changed = copy.deepcopy(ticks)
            if corruption == 'lost_brake':
                changed[3]['expected']['command']['acceleration'] = -.49
            elif corruption == 'invented_hold':
                changed[2]['expected']['navigation']['handover_hold'] = True
            elif corruption == 'cached_vote':
                changed[3]['input']['odometry'] = None
            elif corruption == 'unhealthy_hold':
                changed[0]['expected']['health'] = ['StaleLidar']
            else:
                changed[4]['expected']['navigation'].pop('handover_hold')
            with self.assertRaises(ValueError):
                verify_navigation(config, changed, .1)

    def test_default_policy_cannot_report_a_hold(self):
        config, ticks = held_ticks()
        config['navigation'].pop('hold_for_handover')
        with self.assertRaisesRegex(ValueError, 'unproved handover'):
            verify_navigation(config, ticks, .1)

    def test_fault_then_still_closed_snapshot_cannot_release_an_existing_hold(self):
        config, source = held_ticks()
        ticks = []
        for i, (revision, closed, phase) in enumerate([
            (7, ['main'], 'Braking'), (8, ['unknown'], 'Fault'),
            (8, ['main', 'detour'], 'Blocked'), (9, [], 'Following'),
        ]):
            tick = copy.deepcopy(source[0]);now = i*.05;out = tick['expected'];nav = out['navigation']
            tick['input'].update(time=now, odometry=dict(stamp=now, speed=0., yaw_rate=0.), navigation_update=dict(stamp=now, revision=revision, closed_edges=closed))
            out['health'] = ['InvalidNavigation'] if i == 1 else []
            out['command']['acceleration'] = -6. if i == 1 else (-.5 if i < 3 else 2.)
            nav.update(revision=7 if i == 1 else revision, phase=phase,
                       closed_edges=['main'] if i == 1 else closed,
                       pending_edges=['head', 'detour', 'tail'] if i == 0 else [],
                       stop_s=None if i == 3 else 66.75, handover_hold=i < 3)
            ticks.append(tick)
        self.assertTrue(verify_navigation(config, ticks)['passed'])
        for i in [1, 2]:
            changed = copy.deepcopy(ticks);changed[i]['expected']['navigation']['handover_hold'] = False
            with self.assertRaisesRegex(ValueError, 'unproved handover'):
                verify_navigation(config, changed)
        ticks[2]['expected']['command']['acceleration'] = 2.
        with self.assertRaisesRegex(ValueError, 'failed to brake'):
            verify_navigation(config, ticks)

    def test_pairing_requires_both_goals_and_actual_two_second_gain(self):
        before = dict(reached_goal=True, simulated_seconds=44.)
        self.assertTrue(goal_improvement(before, dict(reached_goal=True, simulated_seconds=42.))['passed'])
        self.assertFalse(goal_improvement(before, dict(reached_goal=True, simulated_seconds=42.05))['passed'])
        with self.assertRaises(ValueError):
            goal_improvement(before, dict(reached_goal=False, simulated_seconds=20.))


if __name__ == '__main__':
    unittest.main()
