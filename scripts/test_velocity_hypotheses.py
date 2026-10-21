#!/usr/bin/env python3
"""Acquisition ownership, point-set geometry and unchanged qualification gates."""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from check_hazards import check_motion_predictions
from check_velocity_hypotheses import materialize,original,specifications,verify_policy
from test_cold_motion import CONFIG as COLD_CONFIG,report as cold_report,tick as cold_tick
from velocity_hypotheses_oracle import VelocityHypothesesOracle

CONFIG=dict(COLD_CONFIG,forecast_recent_velocity_hypotheses=True)


def status(times,ident=1):
    return dict(id=ident,sample_count=len(times),first_acquisition_s=times[0] if times else None,
        last_acquisition_s=times[-1] if times else None)


def value(now,stamp,times,*,velocity=(1.,0.),supported=True,failed=False,scene=None):
    track=dict(id=1,position=dict(x=10.,y=0.),velocity=dict(zip(['x','y'],velocity)),radius=1.,last_seen=stamp)
    out=dict(time=now,health=['AcquisitionFailed'] if failed else [],tracks=[track],predictions=[],
        recent_velocity_hypotheses=dict(window_s=.8,maximum_samples_per_identity=32,tracks=[status(times)]))
    tick=dict(input=dict(time=now),expected=out)
    cold=SimpleNamespace(accepted=stamp if scene is None else scene,states={1:dict(report=dict(confirmed=supported))})
    return tick,cold


def observe(oracle,*args,**kwargs):
    tick,cold=value(*args,**kwargs);oracle.observe(tick,tick['expected']['tracks'],cold);return tick


def motion_rows():
    rows=[dict(header=dict(config=CONFIG))];history=[]
    for i,time in enumerate([0.,.1,.2,.3,.4]):
        speed=2.5 if i<3 else 0.;support=cold_report([0.,.1,.2][:i+1],i>=2,0. if i>=2 else None,last=time)
        tick=cold_tick(time,time,support,speed=speed);out=tick['expected'];t=out['tracks'][0]
        if i>=2:history.append((time,speed))
        out['recent_velocity_hypotheses']=dict(window_s=.8,maximum_samples_per_identity=32,tracks=[status([s for s,_ in history])])
        p=dict(id=1,radius=1.,dt=.2,positions=[],additional_radius_m=[])
        for k in range(41):
            elapsed=k*.2;p['positions'].append(dict(x=t['position']['x']+speed*elapsed,y=0.))
            base=.2+.3*elapsed+.05*elapsed**2
            cold=(speed+15.)*elapsed+.2 if i<2 else 0.
            old=max((abs(speed-v)*elapsed+.2 for _,v in history),default=0.)
            p['additional_radius_m'].append(max(base,cold,old))
        out['predictions']=[p];rows.append(dict(kind='tick',tick=tick))
    return rows


def audit(rows):
    with tempfile.TemporaryDirectory() as scratch:
        path=Path(scratch)/'sensors.jsonl';path.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        return check_motion_predictions(path)


class VelocityHypothesisTests(unittest.TestCase):
    def test_real_acquisitions_have_exact_clocks_and_cached_ticks_cannot_vote(self):
        oracle=VelocityHypothesesOracle(CONFIG);observe(oracle,0.,0.,[0.]);observe(oracle,.05,0.,[0.])
        observe(oracle,.1,0.,[0.],scene=.1)
        observe(oracle,.2,.2,[0.,.2]);self.assertEqual(len(oracle.history[1]),2)

    def test_unsupported_acquisitions_cannot_be_reused_after_support_appears(self):
        oracle=VelocityHypothesesOracle(CONFIG);observe(oracle,0.,0.,[],supported=False)
        observe(oracle,.05,0.,[]);observe(oracle,.1,.1,[.1])
        observe(oracle,.2,.2,[],supported=False);observe(oracle,.25,.2,[])
        observe(oracle,.3,.3,[.3])

    def test_fault_watermark_requires_strictly_new_acquisition(self):
        oracle=VelocityHypothesesOracle(CONFIG);observe(oracle,0.,0.,[0.])
        observe(oracle,.05,0.,[],failed=True);observe(oracle,.1,0.,[])
        observe(oracle,.2,.2,[.2]);self.assertEqual(oracle.history[1][0]['stamp'],.2)

    def test_acquisition_age_expires_the_window_without_a_new_scan(self):
        oracle=VelocityHypothesesOracle(CONFIG)
        for i in range(11):
            time=i*.1;observe(oracle,time,time,[j*.1 for j in range(max(0,i-8),i+1)])
        observe(oracle,1.05,1.,[j*.1 for j in range(3,11)])
        observe(oracle,1.36,1.,[])

    def test_high_rate_history_remains_bounded_and_missing_ids_release_memory(self):
        oracle=VelocityHypothesesOracle(CONFIG)
        for i in range(1000):
            time=i*.001;observe(oracle,time,time,[j*.001 for j in range(max(0,i-31),i+1)])
        self.assertEqual(oracle.maximum_samples,32)
        tick,cold=value(1.,1.,[]);tick['expected'].update(tracks=[],recent_velocity_hypotheses=dict(window_s=.8,maximum_samples_per_identity=32,tracks=[]))
        oracle.observe(tick,[],cold);self.assertEqual(oracle.history,{})

    def test_epoch_changes_cannot_inherit_an_old_velocity(self):
        oracle=VelocityHypothesesOracle(CONFIG);observe(oracle,0.,0.,[0.]);observe(oracle,.1,.1,[0.,.1])
        tick,cold=value(.2,.2,[.2]);tick['expected']['localization_epoch']=1
        oracle.observe(tick,tick['expected']['tracks'],cold);self.assertEqual(len(oracle.history[1]),1)

    def test_all_velocity_point_discs_fit_between_knots_with_consumed_age(self):
        oracle=VelocityHypothesesOracle(CONFIG);observe(oracle,0.,0.,[0.],velocity=(-1.,0.))
        tick=observe(oracle,.1,.1,[0.,.1],velocity=(1.3,0.));observe(oracle,.15,.1,[0.,.1],velocity=(1.3,0.))
        t=tick['expected']['tracks'][0];p=dict(dt=.2,positions=[dict(x=10.+1.3*(.05+i*.2),y=0.) for i in range(41)])
        radii=oracle.floor(t,p,.15)
        for k in range(161):
            time=k*.05;i=min(39,int(time/.2));f=time/.2-i
            center=[p['positions'][i][a]*(1-f)+p['positions'][i+1][a]*f for a in ['x','y']];radius=radii[i]*(1-f)+radii[i+1]*f
            for speed in [-1.,1.3]:
                for j in range(16):
                    a=math.tau*j/16;point=[10.+speed*(.05+time)+.2*math.cos(a),.2*math.sin(a)]
                    self.assertLessEqual(math.dist(center,point),radius+1e-8)

    def test_full_motion_checker_rejects_forged_padding_centers_clocks_and_support(self):
        self.assertTrue(audit(motion_rows())['verified'])
        for mutation in ['under','over','center','clock','count','support','remove']:
            rows=motion_rows();out=rows[-1]['tick']['expected'];p=out['predictions'][0]
            if mutation=='under':p['additional_radius_m'][30]-=.01
            if mutation=='over':p['additional_radius_m'][30]+=.01
            if mutation=='center':p['positions'][30]['y']+=.01
            if mutation=='clock':out['recent_velocity_hypotheses']['tracks'][0]['first_acquisition_s']=.1
            if mutation=='count':out['recent_velocity_hypotheses']['tracks'][0]['sample_count']=True
            if mutation=='support':out['cold_motion']['tracks'][0]['confirmed']=False
            if mutation=='remove':rows[0]['header']['config']=dict(CONFIG,forecast_recent_velocity_hypotheses=False)
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):audit(rows)

    def test_invalid_velocities_timestamps_and_identity_sets_are_rejected(self):
        for mutation in ['nan','speed','future','duplicate','regress']:
            oracle=VelocityHypothesesOracle(CONFIG);observe(oracle,.1,.1,[.1]);tick,cold=value(.2,.2,[.1,.2]);tracks=tick['expected']['tracks']
            if mutation=='nan':tracks[0]['velocity']['x']=math.nan
            if mutation=='speed':tracks[0]['velocity']['x']=16.
            if mutation=='future':tracks[0]['last_seen']=.3
            if mutation=='duplicate':tracks.append(copy.deepcopy(tracks[0]))
            if mutation=='regress':tick['input']['time']=.05
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):oracle.observe(tick,tracks,cold)

    def test_prerequisites_require_a_declared_boolean_and_cold_motion(self):
        for changed in [dict(forecast_recent_velocity_hypotheses=1),dict(forecast_recent_velocity_hypotheses=False),dict(forecast_cold_max_speed_m_s=None),dict(forecast_envelope=None)]:
            with self.subTest(changed=changed),self.assertRaises(ValueError):VelocityHypothesesOracle(dict(CONFIG,**changed))

    def test_all_forty_eight_inputs_retain_original_worlds_and_policies(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),48)
        for spec in specs:
            _,frozen=materialize(spec);self.assertIs(frozen.pop('forecast_recent_velocity_hypotheses'),True);self.assertEqual(frozen,original(spec))

    def test_old_budgets_sensors_deadlines_and_prior_cannot_be_tuned_to_pass(self):
        spec=specifications(['reference-2d'])[0];_,frozen=materialize(spec);verify_policy(frozen,frozen,spec)
        for key in ['new','motion','pose','cold','deadline','sensor']:
            c=copy.deepcopy(frozen);f=copy.deepcopy(frozen)
            if key=='new':c['forecast_recent_velocity_hypotheses']=1
            if key=='motion':c['forecast_envelope']['velocity_error_m_s']=.5
            if key=='pose':c['forecast_pose_sigma']=4.
            if key=='cold':c['forecast_cold_max_speed_m_s']=14.
            if key=='deadline':f['duration']+=1.
            if key=='sensor':f['heading_sensor']['variance_rad2']*=.5
            with self.subTest(key=key),self.assertRaises(ValueError):verify_policy(c,f,spec)


if __name__=='__main__':unittest.main()
