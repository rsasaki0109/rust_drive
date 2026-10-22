#!/usr/bin/env python3
"""Pose-occupancy geometry, malformed inputs and independent provenance checks."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from check_hazards import check_motion_predictions
from check_pose_forecasts import materialize,original,specifications,verify_policy
from pose_forecast_oracle import factor,nominal_output,padding,validate_sigma
from tracking_frame import continuous_observations

POSE=dict(position=dict(x=0.,y=0.),yaw=0.)
COV=[[.04,0.,0.],[0.,.09,0.],[0.,0.,0.]]
BOUNDS=dict(position_error_m=.2,velocity_error_m_s=.3,acceleration_error_m_s2=.1)


def output():
    forecast=dict(id=1,positions=[dict(x=10.,y=0.) for _ in range(41)],radius=1.12,dt=.2,
        additional_radius_m=[.9+.2+.3*(i*.2)+.05*(i*.2)**2 for i in range(41)])
    return dict(time=0.,estimate=dict(pose=copy.deepcopy(POSE),speed=0.),pose_covariance=copy.deepcopy(COV),
        tracking_frame=copy.deepcopy(POSE),predictions=[forecast],
        tracks=[dict(id=1,position=dict(x=10.,y=0.),velocity=dict(x=0.,y=0.),radius=1.12,last_seen=0.)])


class PoseForecastTests(unittest.TestCase):
    def test_translation_and_zero_uncertainty_have_exact_independent_bounds(self):
        for point in [dict(x=0.,y=0.),dict(x=45.,y=-9.)]:
            self.assertAlmostEqual(padding(point,POSE,COV,3.),.9)
            self.assertEqual(padding(point,POSE,[[0.]*3 for _ in range(3)],3.),0.)

    def test_heading_uncertainty_scales_with_range_and_includes_rotation_remainder(self):
        p=[[0.,0.,0.],[0.,0.,0.],[0.,0.,.0001]]
        self.assertAlmostEqual(padding(dict(x=45.,y=0.),POSE,p,3.),1.37025)
        self.assertAlmostEqual(padding(dict(x=0.,y=-90.),POSE,p,3.),2*1.37025)

    def test_joint_correlation_can_cancel_the_linear_displacement(self):
        p=[[1.,0.,.02],[0.,0.,0.],[.02,0.,.0004]]
        self.assertAlmostEqual(padding(dict(x=0.,y=50.),POSE,p,3.),.09)

    def test_exact_pose_rotations_fit_the_bound_without_a_linearized_truth_model(self):
        l=[[.15,0.,0.],[.04,.11,0.],[.002,-.003,.006]]
        p=[[sum(l[i][k]*l[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        for q in [(45.,0.),(-20.,30.)]:
            bound=padding(dict(zip(['x','y'],q)),POSE,p,3.)
            for i in range(61):
                a=i*math.tau/60;unit=[math.cos(a)/math.sqrt(2),math.sin(a)/math.sqrt(2),1/math.sqrt(2)]
                d=[3*sum(l[r][k]*unit[k] for k in range(3)) for r in range(3)]
                actual=[d[0]+math.cos(d[2])*q[0]-math.sin(d[2])*q[1],d[1]+math.sin(d[2])*q[0]+math.cos(d[2])*q[1]]
                self.assertLessEqual(math.dist(actual,q),bound+1e-12)

    def test_malformed_covariance_cannot_justify_a_small_margin(self):
        for p in [None,[[1.]],[[1.,0.,0.],[0.,-1.,0.],[0.,0.,1.]],
                  [[1.,2.,0.],[2.,1.,0.],[0.,0.,1.]],[[1.,.2,0.],[.3,1.,0.],[0.,0.,1.]],
                  [[1.,.9,.9],[.9,1.,-.9],[.9,-.9,1.]],[[math.nan,0.,0.],[0.,1.,0.],[0.,0.,1.]]]:
            with self.subTest(p=p),self.assertRaises(ValueError):factor(p)

    def test_multiplier_requires_a_declared_finite_number(self):
        for value in [True,None,0.,.9,5.1,math.inf,math.nan,'3']:
            with self.subTest(value=value),self.assertRaises(ValueError):validate_sigma(value)

    def test_unexpansion_preserves_centers_times_and_original_objects(self):
        out=output();before=copy.deepcopy(out);restored=nominal_output(out,3.)
        self.assertEqual(out,before);self.assertEqual(restored['tracks'],out['tracks'])
        self.assertEqual(restored['predictions'][0]['positions'],out['predictions'][0]['positions'])
        self.assertAlmostEqual(restored['predictions'][0]['additional_radius_m'][0],.2)
        self.assertEqual(continuous_observations(out,3.)[0][0]['last_seen'],0.)
        self.assertEqual(continuous_observations(out)[1][0]['additional_radius_m'][0],1.1)

    def test_full_motion_checker_rejects_missing_extra_or_forged_pose_padding(self):
        config=dict(forecast_envelope=BOUNDS,forecast_pose_sigma=3.)
        with tempfile.TemporaryDirectory() as scratch:
            path=Path(scratch)/'sensors.jsonl'
            for mutation in ['none','under','over','wrong-covariance','wrong-multiplier']:
                out=output();c=copy.deepcopy(config)
                if mutation=='under':out['predictions'][0]['additional_radius_m'][0]-=.01
                if mutation=='over':out['predictions'][0]['additional_radius_m'][0]+=.01
                if mutation=='wrong-covariance':out['pose_covariance'][1][1]*=2
                if mutation=='wrong-multiplier':c['forecast_pose_sigma']=2.
                path.write_text(json.dumps(dict(header=dict(config=c)))+'\n'+json.dumps(dict(kind='tick',tick=dict(input=dict(time=0.),expected=out)))+'\n')
                if mutation=='none':self.assertTrue(check_motion_predictions(path)['verified'])
                else:
                    with self.subTest(mutation=mutation),self.assertRaises(ValueError):check_motion_predictions(path)

    def test_fixed_protocol_support_is_selected_before_looking_at_scores(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),48)
        for spec in specs:
            _,value=materialize(spec);self.assertEqual(value.pop('forecast_pose_sigma'),3.)
            self.assertEqual(value,original(spec));self.assertTrue(value['absolute_heading_fusion'])

    def test_world_sensor_budget_or_multiplier_changes_cannot_be_accepted_as_a_fix(self):
        spec=specifications(['reference-2d'])[0];_,frozen=materialize(spec)
        config=dict(forecast_pose_sigma=3.,forecast_envelope=copy.deepcopy(frozen['forecast_envelope']))
        verify_policy(config,frozen,spec)
        for mutation in ['sigma','budget','deadline','sensor']:
            c=copy.deepcopy(config);f=copy.deepcopy(frozen)
            if mutation=='sigma':c['forecast_pose_sigma']=2.
            if mutation=='budget':c['forecast_envelope']['position_error_m']=.4
            if mutation=='deadline':f['duration']+=1.
            if mutation=='sensor':f['heading_sensor']['variance_rad2']*=.5
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):verify_policy(c,f,spec)


if __name__=='__main__':unittest.main()
