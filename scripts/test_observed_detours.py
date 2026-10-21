#!/usr/bin/env python3
"""Corruption, chronology and topology controls for observed route detours."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from check_mid_edge import reference
from check_navigation_relocalization import verify_navigation
from check_observed_detours import finish_report,remove_observed_status,verify_policy
from observed_detour_cases import materialize,specifications
from observed_detour_oracle import ObservedDetours


def config(world='detour'):
    spec=next(s for s in specifications(['reference-2d']) if s['world']==world and s['enabled'])
    _,frozen=materialize(spec)
    return dict(navigation=frozen['navigation'],vehicle=dict(radius=1.25),motion_limits=frozen['motion_limits'])


def tick(now,*,acquisition=None,speed=4.,x=20.):
    if acquisition is None:acquisition=now
    conflict=dict(prediction_id=7,relative_time_s=3.,forecast_radius_m=3.,forecast_horizon_s=8.,minimum_clearance_floor_m=0.)
    out=dict(time=now,health=[],estimate=dict(pose=dict(position=dict(x=x,y=0.),yaw=0.),speed=speed),
        tracks=[dict(id=7,last_seen=acquisition,position=dict(x=65.,y=0.),velocity=dict(x=0.,y=0.),radius=1.)],
        command=dict(acceleration=0.),
        planning_diagnostics=dict(scene_acquisition_s=acquisition,candidates=[dict(disposition='CheckedStop',nominal_forecast_conflict=copy.deepcopy(conflict)) for _ in range(3)]))
    return dict(input=dict(time=now,lidar=dict(stamp=acquisition,points=[]),lidar_failed=False,
        odometry=dict(stamp=now,speed=speed,yaw_rate=0.),navigation_update=None),expected=out)


def run(observer,ticks,*,phase='Following',revision=0,closed=None):
    active=reference(observer.nav,observer.nav.get('closed_edges',[]));result=[]
    for value in ticks:result=observer.observe(value,active,phase,revision,closed or [])
    return result


class ObservedDetourTests(unittest.TestCase):
    def test_three_fresh_acquisitions_need_a_preceding_blocked_scene(self):
        observer=ObservedDetours(config())
        for now in [0.,.1,.2]:self.assertEqual(run(observer,[tick(now)]),[])
        self.assertEqual(run(observer,[tick(.3)]),['main'])
        evidence=observer.report['retained_edges'][0]['evidence']
        self.assertEqual(evidence['acquisition_count'],3)
        self.assertEqual(evidence['first_acquisition_s'],.1)
        self.assertEqual(observer.report['last_decision'],'DetourRequested')

    def test_cached_scenes_cannot_count_or_replace_elapsed_evidence(self):
        observer=ObservedDetours(config());run(observer,[tick(0.),tick(.1)])
        for now in [.15,.2,.25]:self.assertEqual(run(observer,[tick(now,acquisition=.1)]),[])
        self.assertEqual(observer.report['candidate']['acquisition_count'],1)
        run(observer,[tick(.3,acquisition=.1)])
        self.assertIsNone(observer.report['candidate'])
        self.assertEqual(observer.report['retained_edges'],[])

    def test_fault_recovery_and_acquisition_gaps_discard_support(self):
        observer=ObservedDetours(config());run(observer,[tick(0.),tick(.1)])
        fault=tick(.2);fault['expected']['health']=['StaleGnss'];fault['expected']['planning_diagnostics']['scene_acquisition_s']=None
        run(observer,[fault,tick(.25,acquisition=.2)])
        self.assertIsNone(observer.report['candidate'])
        self.assertEqual(run(observer,[tick(.3),tick(.4)]),[])
        self.assertEqual(run(observer,[tick(.5)]),['main'])
        observer=ObservedDetours(config());run(observer,[tick(0.),tick(.1),tick(.2),tick(.7),tick(.8),tick(.9)])
        self.assertEqual(observer.report['retained_edges'],[])
        self.assertEqual(observer.report['candidate']['first_acquisition_s'],.8)

    def test_motion_or_anchor_drift_cannot_confirm_a_static_obstruction(self):
        for drift in [False,True]:
            observer=ObservedDetours(config());values=[]
            for now in [0.,.1,.2,.3,.4]:
                value=tick(now);track=value['expected']['tracks'][0]
                if drift:track['position']['x']+=3.*now
                else:track['velocity']['x']=.21
                values.append(value)
            self.assertEqual(run(observer,values),[])

    def test_an_available_candidate_or_different_blockers_veto_a_detour(self):
        for available in [False,True]:
            observer=ObservedDetours(config());values=[]
            for now in [0.,.1,.2,.3]:
                value=tick(now);record=value['expected']['planning_diagnostics']['candidates'][1]
                if available:record.update(disposition='CompletePath',nominal_forecast_conflict=None)
                else:record['nominal_forecast_conflict']['prediction_id']=8
                values.append(value)
            self.assertEqual(run(observer,values),[])

    def test_no_alternative_and_a_late_branch_leave_the_active_route_owned(self):
        for no_alternative in [False,True]:
            observer=ObservedDetours(config('no-alternative' if no_alternative else 'detour'))
            values=[tick(t,x=20. if no_alternative else 50.) for t in [0.,.1,.2,.3]]
            self.assertEqual(run(observer,values),[])
            self.assertEqual(observer.report['last_decision'],'NoAlternative' if no_alternative else 'UnreachableDivergence')

    def test_off_road_behind_or_ambiguous_edge_association_cannot_exclude_an_edge(self):
        for kind in ['off-road','behind','ambiguous']:
            observer=ObservedDetours(config());active=reference(observer.nav,[])
            if kind=='ambiguous':active['traversals'].append(copy.deepcopy(active['traversals'][1]))
            for now in [0.,.1,.2,.3]:
                value=tick(now);p=value['expected']['tracks'][0]['position']
                if kind=='off-road':p['y']=5.
                if kind=='behind':p['x']=15.
                self.assertEqual(observer.observe(value,active,'Following',0,[]),[])

    def test_external_closures_are_combined_for_search_without_becoming_local_evidence(self):
        observer=ObservedDetours(config());values=[tick(t) for t in [0.,.1,.2,.3]]
        self.assertEqual(run(observer,values,revision=5,closed=['south-branch']),['main'])
        self.assertEqual([e['evidence']['edge'] for e in observer.report['retained_edges']],['main'])
        observer=ObservedDetours(config())
        self.assertEqual(run(observer,values,revision=5,closed=['detour']),[])
        self.assertEqual(observer.report['last_decision'],'NoAlternative')

    def test_every_report_field_and_scalar_type_is_independently_checked(self):
        observer=ObservedDetours(config());run(observer,[tick(t) for t in [0.,.1,.2,.3]])
        self.assertIsNone(observer.audit_status(copy.deepcopy(observer.report)))
        for name in ['identity','clock','count','anchor','edge','decision']:
            changed=copy.deepcopy(observer.report);evidence=changed['retained_edges'][0]['evidence']
            if name=='identity':evidence['prediction_id']=8
            elif name=='clock':evidence['last_acquisition_s']+=.1
            elif name=='count':evidence['acquisition_count']=True
            elif name=='anchor':evidence['anchor_position']['x']+=.1
            elif name=='edge':evidence['edge']='detour'
            else:changed['last_decision']='NoAlternative'
            with self.subTest(name=name),self.assertRaises(ValueError):observer.audit_status(changed)

    def test_irrelevant_external_update_preserves_a_braking_detour_and_resets_votes(self):
        c=config();original=reference(c['navigation'],[]);detour=reference(c['navigation'],['main'])
        evidence=dict(edge='main',prediction_id=7,first_acquisition_s=.1,last_acquisition_s=.3,
            acquisition_count=3,anchor_position=dict(x=65.,y=0.))
        values=[]
        for now in [0.,.1,.2,.3,.4,.5,.55,.6]:
            value=tick(now,x=20. if now<=.3 else 36.6,speed=4. if now<=.3 else .2 if now==.4 else 0.)
            source=value['input'];out=value['expected'];accepted=now>=.3;switched=now==.6
            if now==.4:source['navigation_update']=dict(stamp=.4,revision=1,closed_edges=[])
            report=dict(retained_edges=[dict(evidence=copy.deepcopy(evidence),requested_at_control_s=.3)] if accepted else [],
                candidate=None,last_decision='DetourRequested' if accepted else None,last_decision_control_s=.3 if accepted else None)
            if now in [.1,.2,.3]:
                report['candidate']=dict(evidence,last_acquisition_s=now,acquisition_count={.1:1,.2:2,.3:3}[now])
            held=now in [.5,.55];out['command']['acceleration']=-.5 if held else 0.
            out['navigation']=dict(revision=int(now>=.4),phase='Following' if now<.3 or switched else 'Braking',
                active_edges=detour['edge_ids'] if switched else original['edge_ids'],
                pending_edges=detour['edge_ids'] if accepted and not switched else [],closed_edges=[],
                stop_s=36.75 if accepted and not switched else None,switches=int(switched),
                handover_hold=held,observed_obstructions=report)
            values.append(value)
        result=verify_navigation(c,values,obstruction_observer=ObservedDetours(c))
        self.assertTrue(result['passed']);self.assertEqual(result['switch_events'][0]['stamp'],.6)
        self.assertEqual(result['final_revision'],1)

    def test_all_forty_four_inputs_keep_paired_world_and_deadline_bytes(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),44)
        for spec in specs:
            if not spec['enabled']:continue
            other=dict(spec,enabled=False,mode=spec['mode'].replace('-on','-off'))
            _,on=materialize(spec);_,off=materialize(other)
            self.assertIs(on['navigation'].pop('avoid_observed_obstructions'),True)
            self.assertEqual(on,off)

    def test_policy_budgets_and_truth_scope_cannot_silently_change(self):
        spec=specifications(['reference-2d'])[0];_,frozen=materialize(spec)
        c=config();c['forecast_envelope']=frozen['forecast_envelope']
        verify_policy(c,frozen,spec)
        for change in ['truth','budget','policy']:
            altered=copy.deepcopy(c)
            if change=='truth':altered['objects']=[]
            elif change=='budget':altered['forecast_envelope']['velocity_error_m_s']=.6
            else:altered['navigation']['avoid_observed_obstructions']=False
            with self.assertRaises(ValueError):verify_policy(altered,frozen,spec)

    def test_output_projection_preserves_all_other_float_spellings(self):
        original=b'{"navigation":{"revision":0,"v":1.0000000000000002,"observed_obstructions":{"nested":[{"x":1.23}]}},"x":2.0000000000000004}\n'
        expected=b'{"navigation":{"revision":0,"v":1.0000000000000002},"x":2.0000000000000004}\n'
        self.assertEqual(remove_observed_status(original),expected)
        for line in [b'{"navigation":{}}\n',b'{"navigation":{"observed_obstructions":{},"revision":0}}\n']:
            with self.assertRaises(ValueError):remove_observed_status(line)

    def test_goal_gain_cannot_hide_new_forecast_failure_or_qualify_default_adoption(self):
        with tempfile.TemporaryDirectory() as scratch:
            root=Path(scratch);rows=[]
            for enabled in [True,False]:
                spec=dict(plant='reference-2d',world='detour',mode='detour-on' if enabled else 'detour-off',seed=7,enabled=enabled)
                _,frozen=materialize(spec);name=spec['mode']+'.json';(root/name).write_text(json.dumps(frozen))
                rows.append(dict(spec,scenario=name,passed=False,failed_gates=['forecast_coverage' if enabled else 'source_acceptance'],
                    observed_detours=dict(passed=True),summary=dict(steps=1,passed=enabled),run_exit_code=0 if enabled else 1,
                    replay_exit_code=0,replay=dict(verified=True,ticks=1)))
            report=dict(complete=True,episodes=rows);finish_report(root,report)
            self.assertTrue(report['contract_qualified'])
            self.assertEqual(report['strict_passes'],0)
            self.assertEqual(report['paired_comparisons']['pairs'][0]['new_failed_gates'],['forecast_coverage'])
            for key in ['regression_passed','passed','adoption_passed']:self.assertIs(report[key],False)


if __name__=='__main__':unittest.main()
