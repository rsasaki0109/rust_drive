#!/usr/bin/env python3
"""Independent covariance corruption, acquisition ownership and protocol checks."""
import copy
import math
from pathlib import Path
import tempfile
import unittest

from check_absolute_heading import HeadingFilter, freeze, integration_passed, materialize, specifications, verify_covariance, verify_unactivated_heading_hold
from check_road_width import materialize as road_input


def config(yaw=0.): return dict(initial_pose=dict(position=dict(x=0.,y=0.),yaw=yaw))
def sample(stamp,yaw=0.,variance=1e-4): return dict(stamp=stamp,yaw_rad=yaw,variance_rad2=variance)


class HeadingTests(unittest.TestCase):
    def test_wrap_crossing_conditions_the_angle_without_changing_gnss_clocks(self):
        f=HeadingFilter(config(math.pi-.01)); issues=f.observe_heading(sample(0.,-math.pi+.01),0.)
        self.assertEqual(issues,[]); self.assertLess(abs(f.x[2]-(-math.pi+.01)),1e-4)
        self.assertAlmostEqual(f.p[2][2],.03*1e-4/.0301)
        self.assertIsNone(f.accepted_stamp); self.assertEqual(f.accepted,0)

    def test_old_and_invalid_deliveries_cannot_withdraw_a_rejection_or_refresh_age(self):
        f=HeadingFilter(config()); f.observe_heading(sample(0.),0.)
        self.assertEqual(f.observe_heading(sample(.05,.8),.05),['HeadingInnovationHold'])
        self.assertEqual(f.observe_heading(sample(.05),.1),['HeadingInnovationHold'])
        self.assertEqual(f.heading['last_accepted_stamp'],0.)
        self.assertEqual(f.observe_heading(sample(.4,variance=-1.),.4),['InvalidAbsoluteHeading','StaleAbsoluteHeading'])
        self.assertEqual(f.observe_heading(sample(.45),.45),[])
        self.assertEqual(f.heading['accepted_samples'],2)

    def test_all_covariance_entries_are_verified_even_when_the_trace_is_unchanged(self):
        expected=[[1.,.1,.02],[.1,1.,-.01],[.02,-.01,.03]]
        self.assertEqual(verify_covariance(expected,expected),0.)
        for bad in [None,[[1.,0.],[0.,1.]],[[1.,0.,0.],[0.,1.,0.],[0.,0.,-.01]]]:
            with self.assertRaises(ValueError): verify_covariance(bad,expected)
        bad=copy.deepcopy(expected); bad[0][0]+=.01; bad[1][1]-=.01
        with self.assertRaises(ValueError): verify_covariance(bad,expected)
        bad=copy.deepcopy(expected); bad[0][2]+=.001; bad[2][0]+=.001
        with self.assertRaises(ValueError): verify_covariance(bad,expected)

    def test_the_58_case_protocol_adds_sensing_without_changing_base_worlds_or_priors(self):
        specs=specifications(['reference-2d','rne-dynamic']); self.assertEqual(len(specs),58)
        self.assertEqual(len({(r['plant'],r['world'],r['mode'],r['seed']) for r in specs}),58)
        for spec in [r for r in specs if r['kind']=='base']:
            source,value=materialize(spec); self.assertTrue(value.pop('absolute_heading_fusion'))
            self.assertEqual(value.pop('heading_sensor'),dict(period_ticks=4,variance_rad2=1e-4))
            self.assertEqual((source,value),road_input(dict(spec,mode='fitted')))
            self.assertIsNone(value.get('initial_pose_covariance'))
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory); manifest=freeze(output,specs); self.assertEqual(len(manifest['episodes']),58)
            (output/manifest['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError): freeze(output,specs)

    def test_negative_controls_still_require_complete_replay_and_real_quality_failure(self):
        row=dict(kind='bias',world='straight',run_exit_code=0,replay_exit_code=0,summary=dict(passed=True,steps=2),replay=dict(verified=True,ticks=2),failed_gates=['localization_accuracy'])
        self.assertTrue(integration_passed(row))
        bad=copy.deepcopy(row); bad['failed_gates']=[]; self.assertFalse(integration_passed(bad))
        bad=copy.deepcopy(row); bad['replay']['ticks']=1; self.assertFalse(integration_passed(bad))
        bad=copy.deepcopy(row); bad['audit_error']='corrupt raw stream'; self.assertFalse(integration_passed(bad))

    def test_an_absent_provider_uses_sensor_fault_acceptance_and_stays_absent_at_termination(self):
        _,value=materialize(dict(plant='reference-2d',world='straight',seed=171,mode='empty',kind='empty'))
        self.assertEqual(value['expected'],'fault'); self.assertEqual(value['duration'],12.)
        self.assertTrue(value['absolute_heading_fusion']); self.assertNotIn('heading_sensor',value)

    def test_a_no_profile_fault_still_checks_every_command_and_actual_speed(self):
        run=dict(frames=[dict(truth=dict(speed=0.))])
        tick=dict(input={},expected=dict(emergency=True,health=['StaleAbsoluteHeading'],trajectory=dict(mode='Emergency'),command=dict(acceleration=-6.,steering=0.)))
        self.assertEqual(verify_unactivated_heading_hold(run,[tick])['verified_emergency_ticks'],1)
        for key,value in [('emergency',False),('health',[]),('command',dict(acceleration=0.,steering=0.))]:
            bad=copy.deepcopy(tick); bad['expected'][key]=value
            with self.assertRaises(ValueError): verify_unactivated_heading_hold(run,[bad])
        moving=copy.deepcopy(run); moving['frames'][0]['truth']['speed']=.1
        with self.assertRaises(ValueError): verify_unactivated_heading_hold(moving,[tick])


if __name__=='__main__': unittest.main()
