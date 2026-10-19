#!/usr/bin/env python3
import json
from pathlib import Path
import unittest

from check_navigation_speeds import map_plan
from check_lane_resume_speeds import interval_excess

REPO = Path(__file__).resolve().parents[1]


class NavigationSpeeds(unittest.TestCase):
    def scenario(self, case):
        return json.loads((REPO/f'scenarios/navigation-speed-{case}.json').read_text())

    def test_closure_search_rebuilds_the_longer_detour_profile_and_blocks_unreachable_goals(self):
        nav = self.scenario('handover')['navigation']
        direct = map_plan(nav, [])
        detour = map_plan(nav, ['main'])
        self.assertEqual(detour['edge_ids'], ['approach', 'detour', 'east-exit'])
        self.assertGreater(detour['zones'][1]['end_s_m'], direct['zones'][1]['end_s_m'])
        self.assertEqual(detour['zones'][1]['speed_m_s'], 2.)
        self.assertIsNone(map_plan(nav, ['main', 'detour']))

    def test_using_the_old_route_misses_the_new_centimeter_speed_limit(self):
        nav = self.scenario('thin')['navigation']
        direct, detour = map_plan(nav, []), map_plan(nav, ['main'])
        truth = {'pose': {'position': {'x': 100.005, 'y': 20.}}, 'speed': 4.}
        wrong, _ = interval_excess(direct['points'], direct['zones'], truth, truth, 1.25)
        correct, _ = interval_excess(detour['points'], detour['zones'], truth, truth, 1.25)
        self.assertEqual(wrong, 0.)
        self.assertAlmostEqual(correct, 2.5)
        thin = next(z for z in detour['zones'] if z['speed_m_s'] == 1.5)
        self.assertLess(thin['end_s_m']-thin['start_s_m'], .011)

    def test_uncapped_detour_drops_the_old_main_limit_and_rotation_preserves_arc_lengths(self):
        nav = self.scenario('uncapped')['navigation']
        detour = map_plan(nav, ['main'])
        self.assertEqual(len(detour['zones']), 2)
        self.assertTrue(all(z['speed_m_s'] == 4. for z in detour['zones']))
        east = map_plan(self.scenario('handover')['navigation'], ['main'])
        north = map_plan(self.scenario('north')['navigation'], ['main'])
        self.assertAlmostEqual(east['length_m'], north['length_m'])


if __name__ == '__main__': unittest.main()
