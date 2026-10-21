#!/usr/bin/env python3
"""Independent cold-support clocks, reachable geometry and qualification provenance."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from check_cold_motion import materialize,original,specifications,verify_policy
from check_hazards import check_motion_predictions
from cold_motion_oracle import ColdMotionOracle,blank,fitted
from forecast_envelope import audit_envelope,error_at

BOUNDS=dict(position_error_m=.2,velocity_error_m_s=.3,acceleration_error_m_s2=.1)
CONFIG=dict(forecast_envelope=BOUNDS,forecast_cold_max_speed_m_s=15.,fit_recent_track_motion=True,
    enclose_track_measurements=True,forecast_consistency_guard=True)


def sample(stamp,speed=2.5,radius=1.):
    return dict(id=1,stamp=stamp,center=dict(x=10.+speed*stamp,y=0.),radius=radius)


def report(times,confirmed=False,residual=None,last=None):
    r=blank(1)
    if times:r.update(first_acquisition_s=times[0],last_acquisition_s=times[-1],acquisition_count=len(times),
        last_observed_acquisition_s=times[-1] if last is None else last,velocity_residual_m_s=residual,
        confirmed=confirmed,confirmation_acquisition_s=times[-1] if confirmed else None)
    return r


def tick(now,stamp,support,speed=2.5,scan=True,failed=False,radius=1.):
    m=sample(stamp,radius=radius)
    track=dict(id=1,position=m['center'],velocity=dict(x=speed,y=0.),radius=radius,last_seen=stamp)
    source=dict(time=now,lidar=dict(stamp=stamp,points=[]) if scan else None,lidar_failed=failed)
    out=dict(time=now,health=['AcquisitionFailed'] if failed else [],tracks=[track],track_measurements=[m],
        predictions=[],cold_motion=dict(maximum_speed_m_s=15.,tracks=[support]))
    return dict(input=source,expected=out)


def observe(oracle,t):oracle.observe(t,t['expected']['tracks'])


class ColdMotionTests(unittest.TestCase):
    def test_three_real_acquisitions_confirm_and_latch_the_first_support(self):
        oracle=ColdMotionOracle(CONFIG)
        for i,t in enumerate([0.,.1,.2]):
            observe(oracle,tick(t,t,report([0.,.1,.2][:i+1],i==2,0. if i==2 else None)))
        observe(oracle,tick(.3,.3,report([0.,.1,.2],True,0.,last=.3)))
        self.assertEqual(oracle.confirmations,1)
        self.assertIsNone(oracle.floor(dict(id=1),{},.3))

    def test_cached_acquisitions_never_become_extra_votes(self):
        oracle=ColdMotionOracle(CONFIG);observe(oracle,tick(0.,0.,report([0.])))
        for time in [.05,.1,.2]:observe(oracle,tick(time,0.,report([0.]),scan=False))
        with self.assertRaises(ValueError):observe(oracle,tick(.25,0.,report([0.,.1,.2],True,0.),scan=False))

    def test_failures_consume_the_old_scene_and_require_a_new_prefix(self):
        oracle=ColdMotionOracle(CONFIG);observe(oracle,tick(0.,0.,report([0.])))
        observe(oracle,tick(.05,0.,blank(1),scan=False,failed=True))
        observe(oracle,tick(.1,0.,blank(1),scan=False,failed=True))
        for i,time in enumerate([.2,.3,.4]):
            observe(oracle,tick(time,time,report([.2,.3,.4][:i+1],i==2,0. if i==2 else None)))
        self.assertEqual(oracle.confirmations,1)

    def test_acquisition_gap_rearms_and_ids_cannot_inherit_old_support(self):
        oracle=ColdMotionOracle(CONFIG)
        for i,t in enumerate([0.,.1,.2]):observe(oracle,tick(t,t,report([0.,.1,.2][:i+1],i==2,0. if i==2 else None)))
        observe(oracle,tick(.56,.2,blank(1),scan=False))
        observe(oracle,tick(.6,.6,report([.6])))
        empty=tick(.65,.6,report([.6]),scan=False);empty['expected'].update(tracks=[],track_measurements=[],cold_motion=dict(maximum_speed_m_s=15.,tracks=[]))
        observe(oracle,empty);self.assertEqual(oracle.states,{})

    def test_inconsistent_radius_or_velocity_cannot_confirm(self):
        for shape in [False,True]:
            oracle=ColdMotionOracle(CONFIG)
            for i,t in enumerate([0.,.1,.2]):
                observe(oracle,tick(t,t,report([0.,.1,.2][:i+1],residual=2.5 if i==2 and not shape else None),
                    speed=0. if not shape else 2.5,radius=1.+(.2 if i==1 and shape else 0.)))
            self.assertFalse(oracle.states[1]['report']['confirmed'])

    def test_speed_above_the_declared_prior_cannot_be_clipped_into_confirmation(self):
        oracle=ColdMotionOracle(CONFIG)
        for i,t in enumerate([0.,.1]):
            value=tick(t,t,report([0.,.1][:i+1]));value['expected']['track_measurements']=[sample(t,20.)];observe(oracle,value)
        value=tick(.2,.2,report([0.,.1,.2]));value['expected']['track_measurements']=[sample(.2,20.)]
        with self.assertRaisesRegex(ValueError,'exceeds'):observe(oracle,value)
        self.assertAlmostEqual(fitted([sample(t,20.) for t in [0.,.1,.2]])[0],20.)

    def test_reachable_disc_keeps_acquisition_age_and_all_directions_between_knots(self):
        oracle=ColdMotionOracle(CONFIG);value=tick(.1,0.,report([0.]));observe(oracle,value)
        track=value['expected']['tracks'][0]
        p=dict(dt=.2,positions=[dict(x=10.+2.5*(.1+i*.2),y=0.) for i in range(41)])
        floor=oracle.floor(track,p,.1)
        for step in range(160):
            t=step*.05;index=min(39,int(t/.2));fraction=t/.2-index
            center=[p['positions'][index][k]*(1-fraction)+p['positions'][index+1][k]*fraction for k in ['x','y']]
            radius=floor[index]*(1-fraction)+floor[index+1]*fraction
            for j in range(16):
                a=math.tau*j/16;point=[10.+15.*(.1+t)*math.cos(a),15.*(.1+t)*math.sin(a)]
                self.assertLessEqual(math.dist(center,point)+.2,radius+1e-9)

    def test_extra_cold_radius_cannot_hide_a_changed_nominal_center(self):
        t=dict(last_seen=0.,position=dict(x=0.,y=0.),velocity=dict(x=0.,y=0.))
        p=dict(dt=.2,positions=[dict(x=0.,y=0.),dict(x=0.,y=0.)],additional_radius_m=[.2,3.2])
        audit_envelope(BOUNDS,t,0.,p,[(0.,0.)]*2,[],[.2,3.2])
        p['positions'][1]['x']=.5
        with self.assertRaises(ValueError):audit_envelope(BOUNDS,t,0.,p,[(0.,0.)]*2,[],[.2,3.2])

    def test_full_motion_checker_rejects_forged_radius_age_clock_or_confirmation(self):
        value=tick(.1,0.,report([0.]),speed=0.);track=value['expected']['tracks'][0]
        p=dict(id=1,dt=.2,radius=1.,positions=[dict(x=10.,y=0.) for _ in range(41)],
            additional_radius_m=[max(error_at(BOUNDS,.1+i*.2),.2+15.*(.1+i*.2)) for i in range(41)])
        value['expected']['predictions']=[p]
        with tempfile.TemporaryDirectory() as scratch:
            path=Path(scratch)/'sensors.jsonl'
            for mutation in ['none','under','over','age','clock','confirmation']:
                changed=copy.deepcopy(value);out=changed['expected']
                if mutation=='under':out['predictions'][0]['additional_radius_m'][20]-=.01
                if mutation=='over':out['predictions'][0]['additional_radius_m'][20]+=.01
                if mutation=='age':out['predictions'][0]['additional_radius_m'][0]=.2
                if mutation=='clock':out['cold_motion']['tracks'][0]['last_observed_acquisition_s']=.1
                if mutation=='confirmation':out['cold_motion']['tracks'][0]['confirmed']=True
                path.write_text(json.dumps(dict(header=dict(config=CONFIG)))+'\n'+json.dumps(dict(kind='tick',tick=changed))+'\n')
                if mutation=='none':self.assertTrue(check_motion_predictions(path)['verified'])
                else:
                    with self.subTest(mutation=mutation),self.assertRaises(ValueError):check_motion_predictions(path)
        self.assertEqual(track['last_seen'],0.)

    def test_prerequisites_and_speed_are_explicit_and_finite(self):
        for value in [None,True,.69,15.1,math.inf,math.nan,'15']:
            with self.subTest(value=value),self.assertRaises(ValueError):ColdMotionOracle(dict(CONFIG,forecast_cold_max_speed_m_s=value))
        for key in ['fit_recent_track_motion','enclose_track_measurements','forecast_consistency_guard']:
            with self.subTest(key=key),self.assertRaises(ValueError):ColdMotionOracle(dict(CONFIG,**{key:False}))

    def test_complete_parent_cohort_and_original_worlds_are_preserved_before_scoring(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),48)
        for spec in specs:
            _,value=materialize(spec);self.assertEqual(value.pop('forecast_cold_max_speed_m_s'),15.)
            self.assertEqual(value,original(spec))

    def test_world_sensor_deadline_or_old_budgets_cannot_be_changed_to_pass(self):
        spec=specifications(['reference-2d'])[0];_,frozen=materialize(spec)
        verify_policy(frozen,frozen,spec)
        for mutation in ['speed','motion','pose','sensor','deadline','prerequisite']:
            c=copy.deepcopy(frozen);f=copy.deepcopy(frozen)
            if mutation=='speed':c['forecast_cold_max_speed_m_s']=14.
            if mutation=='motion':c['forecast_envelope']['position_error_m']=.4
            if mutation=='pose':c['forecast_pose_sigma']=4.
            if mutation=='sensor':f['heading_sensor']['variance_rad2']*=.5
            if mutation=='deadline':f['duration']+=1.
            if mutation=='prerequisite':c['fit_recent_track_motion']=False
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):verify_policy(c,f,spec)


if __name__=='__main__':unittest.main()
