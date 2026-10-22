#!/usr/bin/env python3
"""Independent confirmation clocks, complete stop certificates and frozen policies."""
import copy
from types import SimpleNamespace
import unittest

from checked_approach_oracle import CheckedApproachOracle,blank
from check_checked_approaches import materialize,original,specifications,verify_policy
from check_road_width import verify_targets
from check_observed_detours import Decisions
from test_planning_diagnostics import fixture as planning_fixture


CONFIG=dict(replan_checked_approaches=True,forecast_recent_velocity_hypotheses=True,
    settle_obstruction_stops=True,search_obstruction_stops=True,lateral_candidate_margin_m=.35,
    vehicle=dict(radius=1.25),route=dict(points=[dict(x=0.,y=0.),dict(x=100.,y=0.)],half_width=5.5))


def value(now,scene,times,*,adopted=False,failed=False,endpoint=8.,ids=None):
    report=blank()
    if times:report.update(supporting_acquisitions=min(3,len(times)),first_acquisition_s=times[0],last_acquisition_s=times[-1],
        proposal_end_progress_m=endpoint,proposal_endpoint=dict(x=endpoint,y=0.),proposal_lateral_target_m=3.9,
        blocking_prediction_ids=[7] if ids is None else ids,adopted=adopted)
    points=[dict(position=dict(x=0.,y=0.),time=0.,speed=0.),dict(position=dict(x=0.,y=0.),time=8.,speed=0.)]
    if adopted:points=[dict(position=dict(x=x,y=0.),time=t,speed=v) for x,t,v in [(0.,0.,0.),(4.,2.,4.),(8.,4.,0.),(8.,12.,0.)]]
    out=dict(health=['AcquisitionFailed'] if failed else [],estimate=dict(pose=dict(position=dict(x=0.,y=0.),yaw=0.),speed=0.),
        checked_approach=report,trajectory=dict(points=points,lateral_target=3.9,mode='Yield'),
        predictions=[dict(id=7,dt=.2,radius=1.,positions=[dict(x=100.,y=0.)]*41,additional_radius_m=[])],
        planning_diagnostics=dict(route_progress_m=0.,selected_candidate=0,candidates=[dict(disposition='CheckedStop',profile=dict(end_progress_m=endpoint))]))
    return dict(input=dict(time=now),expected=out),SimpleNamespace(accepted=scene,states={7:dict(report=dict(confirmed=not failed))})


def observe(oracle,*args,**kwargs):
    tick,cold=value(*args,**kwargs);oracle.observe(tick,cold);return tick


class CheckedApproachTests(unittest.TestCase):
    def test_three_fresh_acquisitions_authorize_one_fully_certified_stop(self):
        o=CheckedApproachOracle(CONFIG);observe(o,0.,0.,[0.]);observe(o,.05,0.,[0.]);observe(o,.1,.1,[0.,.1]);observe(o,.2,.2,[0.,.1,.2],adopted=True)
        self.assertEqual(len(o.adoptions),1);self.assertGreater(o.bounds,0)
        observe(o,.25,.2,[])

    def test_cached_ticks_cannot_add_credit_or_authorize_a_restart(self):
        for which in ['count','adopt']:
            o=CheckedApproachOracle(CONFIG);observe(o,0.,0.,[0.]);observe(o,.1,.1,[0.,.1]);t,c=value(.2,.1,[0.,.1])
            if which=='count':t['expected']['checked_approach']['supporting_acquisitions']=3
            else:t['expected']['checked_approach']['adopted']=True
            with self.subTest(which=which),self.assertRaises(ValueError):o.observe(t,c)

    def test_fault_gap_and_epoch_withdraw_old_confirmation(self):
        o=CheckedApproachOracle(CONFIG);observe(o,0.,0.,[0.]);observe(o,.1,.1,[0.,.1]);observe(o,.15,.1,[],failed=True)
        observe(o,.2,.1,[]);observe(o,.3,.3,[.3]);observe(o,.8,.8,[.8])
        t,c=value(.9,.9,[.9]);t['expected']['localization_epoch']=1;o.observe(t,c)
        self.assertEqual(o.support['first'],.9)

    def test_endpoint_target_blocker_and_scalar_type_corruptions_are_rejected(self):
        for which in ['endpoint','target','ids','count','clock','speed']:
            o=CheckedApproachOracle(CONFIG);observe(o,0.,0.,[0.]);t,c=value(.1,.1,[0.,.1]);r=t['expected']['checked_approach']
            if which=='endpoint':r['proposal_endpoint']['x']+=1.1
            if which=='target':r['proposal_lateral_target_m']=-3.9
            if which=='ids':r['blocking_prediction_ids']=[7,7]
            if which=='count':r['supporting_acquisitions']=True
            if which=='clock':r['last_acquisition_s']=0.
            if which=='speed':t['expected']['estimate']['speed']=.3
            with self.subTest(which=which),self.assertRaises(ValueError):o.observe(t,c)

    def test_an_adopted_path_needs_the_entire_hold_and_cannot_touch_a_forecast(self):
        for which in ['hold','endpoint','contact','selected']:
            o=CheckedApproachOracle(CONFIG);observe(o,0.,0.,[0.]);observe(o,.1,.1,[0.,.1]);t,c=value(.2,.2,[0.,.1,.2],adopted=True);out=t['expected']
            if which=='hold':out['trajectory']['points'][-1]['time']=11.99
            if which=='endpoint':out['trajectory']['points'][-1]['position']['x']+=.1
            if which=='contact':out['predictions'][0]['positions']=[dict(x=4.,y=0.)]*41
            if which=='selected':out['planning_diagnostics']['candidates'][0]['disposition']='CompletePath'
            with self.subTest(which=which),self.assertRaises(ValueError):o.observe(t,c)

    def test_usable_width_keeps_the_same_declared_corridor(self):
        tick=dict(expected=dict(trajectory=dict(lateral_target=3.9,points=[dict(position=dict(x=20.,y=3.9))])))
        self.assertEqual(verify_targets(CONFIG,[tick])['side_candidate_m'],3.9)
        with self.assertRaises(ValueError):verify_targets(dict(CONFIG,replan_checked_approaches=False),[tick])
        tick['expected']['trajectory']['points'][0]['position']['y']=4.3
        with self.assertRaises(ValueError):verify_targets(CONFIG,[tick])

    def test_all_parent_inputs_and_motion_budgets_remain_frozen(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),48)
        for spec in specs:
            _,value=materialize(spec);self.assertIs(value.pop('replan_checked_approaches'),True);self.assertEqual(value,original(spec))
        spec=specs[0];_,frozen=materialize(spec);verify_policy(frozen,frozen,spec)
        for which in ['budget','width','deadline','cold','sensor','flag']:
            f=copy.deepcopy(frozen);c=copy.deepcopy(frozen)
            if which=='budget':c['forecast_envelope']['acceleration_error_m_s2']=0.
            if which=='width':c['lateral_candidate_margin_m']=.1
            if which=='deadline':f['duration']+=1.
            if which=='cold':c['forecast_cold_max_speed_m_s']=14.
            if which=='sensor':f['heading_sensor']['variance_rad2']=1e-6
            if which=='flag':c['replan_checked_approaches']=1
            with self.subTest(which=which),self.assertRaises(ValueError):verify_policy(c,f,spec)

    def test_prerequisites_reject_missing_or_mistyped_policy(self):
        for change in [dict(replan_checked_approaches=1),dict(search_obstruction_stops=False),dict(settle_obstruction_stops=False),dict(forecast_recent_velocity_hypotheses=False),dict(lateral_candidate_margin_m=None)]:
            with self.subTest(change=change),self.assertRaises(ValueError):CheckedApproachOracle(dict(CONFIG,**change))

    def test_navigation_decision_inspection_preserves_the_declared_usable_width(self):
        config,ticks=planning_fixture();config['replan_checked_approaches']=True
        for record in ticks[0]['expected']['planning_diagnostics']['candidates']:
            record['lateral_target_m']*=3.9/3.5
        active=dict(points=config['route']['points'],half_width=5.5,length_m=100.)
        decisions=Decisions(config);decisions(ticks[0],active,None,'Following')
        self.assertEqual(decisions.ticks,1)
        config['replan_checked_approaches']=False
        with self.assertRaises(ValueError):Decisions(config)(ticks[0],active,None,'Following')


if __name__=='__main__':unittest.main()
