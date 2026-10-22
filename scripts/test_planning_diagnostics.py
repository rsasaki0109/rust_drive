#!/usr/bin/env python3
"""Corruption controls for decision provenance and byte-preserving projection."""
import copy
import json
import unittest

from inspect_planning import audit,project_output


def fixture(refined=False):
    config=dict(record_planning_diagnostics=True,refine_lateral_candidates=refined,
                lateral_candidate_margin_m=.35,vehicle=dict(radius=1.25),
                route=dict(points=[dict(x=0.,y=0.),dict(x=100.,y=0.)],half_width=5.5))
    fractions=[0.,1.,-1.,.875,-.875,.75,-.75,.5,-.5] if refined else [0.,1.,-1.]
    profile=dict(point_count=2,end_progress_m=40.,end_time_s=5.,terminal_speed_m_s=8.,searched_stop=False)
    records=[dict(index=i,lateral_target_m=3.5*f,disposition='CompletePath',
                  nominal_forecast_conflict=None,profile=copy.deepcopy(profile)) for i,f in enumerate(fractions)]
    report=dict(failure=None,scene_acquisition_s=1.,route_progress_m=0.,planning_span_m=40.,
                selected_candidate=0,candidates=records,refinement_confirmation=None,
                obstruction_stop_requested=False,obstruction_stop_active=False)
    out=dict(time=1.,estimate=dict(pose=dict(position=dict(x=0.,y=0.))),health=[],command=dict(acceleration=0.),
             predictions=[dict(id=7,positions=[dict(x=20.,y=0.)]*41,radius=1.,dt=.2,additional_radius_m=[i*.02 for i in range(41)])],
             trajectory=dict(lateral_target=0.,points=[dict(position=dict(x=x,y=0.),time=t,speed=8.) for x,t in [(0.,0.),(40.,5.)]]),
             planning_diagnostics=report)
    return config,[dict(expected=out)]


class PlanningDiagnosticsTests(unittest.TestCase):
    def test_a_complete_record_is_bounded_and_matches_the_actual_selected_profile(self):
        config,ticks=fixture();result=audit(config,ticks)
        self.assertEqual(result['maximum_candidate_records'],3)
        self.assertEqual(result['selected_profiles'],dict(CompletePath=1))

    def test_enabled_capture_cannot_disappear_or_be_inserted_without_the_policy(self):
        config,ticks=fixture();ticks[0]['expected'].pop('planning_diagnostics')
        with self.assertRaises(ValueError):audit(config,ticks)
        config,ticks=fixture();config['record_planning_diagnostics']=False
        with self.assertRaises(ValueError):audit(config,ticks)
        config,ticks=fixture();config['navigation']={'active_edges':['branch']}
        with self.assertRaises(ValueError):audit(config,ticks)

    def test_a_missing_candidate_duplicate_index_or_selected_endpoint_change_is_rejected(self):
        for mutation in ['lost','index','endpoint','count','span']:
            config,ticks=fixture();report=ticks[0]['expected']['planning_diagnostics']
            if mutation=='lost':report['candidates'].pop()
            elif mutation=='index':report['candidates'][1]['index']=0
            elif mutation=='endpoint':report['candidates'][0]['profile']['end_progress_m']=39.
            elif mutation=='count':report['candidates'][0]['profile']['point_count']=99
            else:report['planning_span_m']=float('nan')
            with self.assertRaises(ValueError):audit(config,ticks)

    def test_conflict_identity_radius_time_and_policy_floor_are_independently_bound(self):
        for mutation in [None,'identity','radius','time','floor','horizon']:
            config,ticks=fixture();record=ticks[0]['expected']['planning_diagnostics']['candidates'][1]
            record['disposition']='NoCheckedStop';record['profile']=None
            conflict=dict(prediction_id=7,relative_time_s=2.,forecast_radius_m=1.2,
                          forecast_horizon_s=8.,minimum_clearance_floor_m=0.)
            record['nominal_forecast_conflict']=conflict
            if mutation=='identity':conflict['prediction_id']=8
            elif mutation=='radius':conflict['forecast_radius_m']=1.3
            elif mutation=='time':conflict['relative_time_s']=-1.
            elif mutation=='floor':conflict['minimum_clearance_floor_m']=1.
            elif mutation=='horizon':conflict['forecast_horizon_s']=9.
            if mutation is None:self.assertEqual(audit(config,ticks)['nominal_forecast_conflicts'],1)
            else:
                with self.assertRaises(ValueError):audit(config,ticks)

    def test_a_selected_refinement_requires_real_acquisition_span_and_confirmed_support(self):
        config,ticks=fixture(True);out=ticks[0]['expected'];report=out['planning_diagnostics']
        report['selected_candidate']=3;out['trajectory']['lateral_target']=3.0625
        support=dict(lateral_target_m=3.0625,acquisition_count=3,first_acquisition_s=0.,last_acquisition_s=.2,confirmed=True)
        report['refinement_confirmation']=support
        self.assertEqual(audit(config,ticks)['maximum_candidate_records'],9)
        for field,value in [('confirmed',False),('last_acquisition_s',0.),('acquisition_count',2),('lateral_target_m',2.625)]:
            changed=copy.deepcopy(ticks);changed[0]['expected']['planning_diagnostics']['refinement_confirmation'][field]=value
            with self.assertRaises(ValueError):audit(config,changed)

    def test_an_accepted_planner_proposal_cannot_hide_a_removed_fault_brake(self):
        config,ticks=fixture();out=ticks[0]['expected'];out['health']=['AcquisitionFailed']
        with self.assertRaises(ValueError):audit(config,ticks)
        out['planning_diagnostics']['scene_acquisition_s']=None;out['command']['acceleration']=-6.
        self.assertEqual(audit(config,ticks)['fault_braking_ticks'],1)

    def test_projection_preserves_every_original_float_byte_and_rejects_extra_fields(self):
        original=b'{"x":1.0000000000000002,"planning_diagnostics":{"candidates":[]}}\n'
        self.assertEqual(project_output(original),b'{"x":1.0000000000000002}\n')
        for value in [b'{"x":1.0}\n',b'{"planning_diagnostics":{},"x":1.0}\n']:
            with self.assertRaises(ValueError):project_output(value)


if __name__=='__main__':unittest.main()
