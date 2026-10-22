#!/usr/bin/env python3
import copy
import json
import unittest
from check_lane_change_speeds import CASES, REPO, blend, reference
from check_lane_speed_zones import check_policy


class ConnectorSpeedTests(unittest.TestCase):
    def test_casteljau_midpoint_and_actual_arc_exceed_longitudinal_distances(self):
        case=CASES[0];spec=json.loads((REPO/case['map']).read_text());points,zones=reference(spec,case)
        self.assertEqual(blend(.5),.5)
        self.assertEqual(points[61],{'x':40.,'y':1.75})
        self.assertGreater(zones[-1]['end_s_m'],120.2)
        self.assertLess(zones[-1]['end_s_m'],121.)
        self.assertEqual(zones[1]['speed_m_s'],2.)

    def test_source_limit_cannot_release_at_unadjusted_longitudinal_change_end(self):
        case=CASES[1];spec=json.loads((REPO/case['map']).read_text());_,zones=reference(spec,case)
        wrong=[{'start_s_m':0.,'end_s_m':55.,'speed_m_s':1.5},{'start_s_m':55.,'end_s_m':zones[-1]['end_s_m'],'speed_m_s':4.}]
        with self.assertRaises(ValueError):check_policy(wrong,zones)
        correct=copy.deepcopy(wrong);correct[0]['end_s_m']=correct[1]['start_s_m']=zones[1]['end_s_m'];check_policy(correct,zones)

    def test_thin_tail_and_directed_permission_remain_visible(self):
        case=CASES[-1];spec=json.loads((REPO/case['map']).read_text());_,zones=reference(spec,case)
        thin=next(z for z in zones if z['speed_m_s']==1.5)
        self.assertAlmostEqual(thin['end_s_m']-thin['start_s_m'],.01,places=10)
        spec['lanes'][0]['left']['change_allowed']=False
        with self.assertRaises(ValueError):reference(spec,case)


if __name__=='__main__':unittest.main()
