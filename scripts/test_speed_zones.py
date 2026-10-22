#!/usr/bin/env python3
"""Arithmetic adversaries for the independent physical speed evaluator."""
import unittest
from check_speed_zones import evaluate

class SpeedIntervalTests(unittest.TestCase):
    def run_case(self, speed, radius=0., vertical=False):
        point=lambda s:{'x':0. if vertical else s,'y':s if vertical else 0.}
        return {'scenario':{'speed_zones':[{'start_s_m':10.4,'end_s_m':10.41,'speed_m_s':2.}]},
                'vehicle':{'radius':radius},'route':{'points':[point(0.),point(100.)]},
                'frames':[{'truth':{'pose':{'position':point(s)},'speed':speed}} for s in [10.,11.]],'summary':{'steps':2}}
    def test_a_short_zone_between_frames_is_not_erased_and_rotation_preserves_scoring(self):
        for vertical in [False,True]:
            result=evaluate(self.run_case(6.,vertical=vertical));self.assertFalse(result['passed']);self.assertEqual(result['violation_intervals'],1);self.assertEqual(result['maximum_excess_m_s'],4.)
    def test_radius_and_fixed_tolerance_are_physical_evaluation_inputs(self):
        self.assertTrue(evaluate(self.run_case(2.14,radius=1.25))['passed'])
        bad=evaluate(self.run_case(2.16,radius=1.25));self.assertFalse(bad['passed']);self.assertEqual(bad['violation_intervals'],2)

if __name__=='__main__':unittest.main()
