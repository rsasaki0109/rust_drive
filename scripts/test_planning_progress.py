#!/usr/bin/env python3
"""Independent controls for acquired dwell, blocked causes and lossless projection."""
import copy
import unittest

from check_planning_progress import baseline,materialize,parent,specifications,verify_policy
from inspect_planning_progress import audit,project_output,statuses
from test_planning_diagnostics import fixture as decision_fixture


def fixture():
    config,base=decision_fixture();config['monitor_planning_progress']=True
    ticks=[]
    for i in range(51):
        tick=copy.deepcopy(base[0]);out=tick['expected'];now=i*.05;stamp=(i//2)*.1
        tick['input']=dict(time=now,lidar=dict(stamp=stamp,points=[]),lidar_failed=False)
        out.update(time=now,emergency=False);out['estimate']['speed']=0.
        out['command']['acceleration']=-.5;out['trajectory']['mode']='Yield'
        out['trajectory']['points']=[dict(position=dict(x=0.,y=0.),time=t,speed=0.) for t in [0.,8.]]
        d=out['planning_diagnostics'];d['scene_acquisition_s']=stamp
        for record in d['candidates']:
            record['disposition']='CheckedStop'
            record['nominal_forecast_conflict']=dict(prediction_id=7,relative_time_s=2.,forecast_radius_m=1.2,
                forecast_horizon_s=8.,minimum_clearance_floor_m=0.)
            record['profile'].update(end_progress_m=0.,end_time_s=8.,terminal_speed_m_s=0.)
        out['planning_progress']=dict(state='StationaryForecastBlocked' if i>=40 else 'Observing',
            first_supporting_acquisition_s=0.,last_supporting_acquisition_s=stamp,
            supporting_acquisitions=i//2+1,supported_duration_s=stamp,blocking_prediction_ids=[7])
        ticks.append(tick)
    return config,ticks


def empty():
    return dict(state='Observing',first_supporting_acquisition_s=None,last_supporting_acquisition_s=None,
        supporting_acquisitions=0,supported_duration_s=0.,blocking_prediction_ids=[])


class PlanningProgressTests(unittest.TestCase):
    def test_two_seconds_of_real_acquisitions_and_cached_control_ticks(self):
        config,ticks=fixture();r=audit(config,ticks)
        self.assertEqual(r['blocked_control_ticks'],11)
        self.assertEqual(r['intervals'][0]['detected_at_control_s'],2.)
        self.assertEqual(ticks[41]['expected']['planning_progress']['supporting_acquisitions'],21)

    def test_every_reported_state_clock_count_duration_and_identity_is_checked(self):
        mutations=dict(state='Observing',first_supporting_acquisition_s=.1,last_supporting_acquisition_s=2.1,
            supporting_acquisitions=22,supported_duration_s=1.9,blocking_prediction_ids=[8])
        for name,value in mutations.items():
            config,ticks=fixture();ticks[40]['expected']['planning_progress'][name]=value
            with self.subTest(name=name),self.assertRaisesRegex(ValueError,'progress evidence'):audit(config,ticks)
        config,ticks=fixture();ticks[0]['expected']['planning_progress']['supporting_acquisitions']=True
        with self.assertRaisesRegex(ValueError,'mistyped'):audit(config,ticks)

    def test_fault_clears_support_and_a_cached_healthy_scan_cannot_restart_it(self):
        config,ticks=fixture();ticks=ticks[:44]
        for tick in ticks[42:]:
            tick['expected']['planning_progress']=empty()
        fault=ticks[42]['expected'];fault['health']=['StaleGnss'];fault['emergency']=True
        fault['command']['acceleration']=-6.;fault['planning_diagnostics']['scene_acquisition_s']=None
        r=audit(config,ticks)
        self.assertEqual(r['final_state'],'Observing')
        self.assertEqual(r['intervals'][0]['cleared_at_control_s'],2.1)

    def test_gap_and_cache_repetition_do_not_mature_evidence(self):
        config,ticks=fixture();ticks=[ticks[0],ticks[2],ticks[42]]
        # The first two acquired samples stay valid; the long gap starts a new prefix.
        ticks[-1]['expected']['planning_progress'].update(state='Observing',first_supporting_acquisition_s=2.1,
            last_supporting_acquisition_s=2.1,supporting_acquisitions=1,supported_duration_s=0.)
        self.assertEqual(audit(config,ticks)['blocked_control_ticks'],0)
        ticks[-1]['expected']['planning_progress']['state']='StationaryForecastBlocked'
        with self.assertRaises(ValueError):audit(config,ticks)

    def test_complete_alternative_and_goal_stops_clear_the_status(self):
        for goal in [False,True]:
            config,ticks=fixture();ticks=ticks[:43];out=ticks[-1]['expected']
            if goal:
                config['route']['points'][-1]['x']=2.
                # Keep the whole fixture at the goal and provide no supporting evidence.
                for t in ticks:t['expected']['planning_progress']=empty()
            else:
                record=out['planning_diagnostics']['candidates'][1]
                record.update(disposition='CompletePath',nominal_forecast_conflict=None)
                out['planning_progress']=empty()
            self.assertEqual(audit(config,ticks)['final_state'],'Observing')

    def test_new_blocker_and_anchor_movement_start_a_new_acquired_prefix(self):
        for change in ['identity','position','progress']:
            config,ticks=fixture();ticks=ticks[:43];out=ticks[-1]['expected'];d=out['planning_diagnostics']
            if change=='identity':
                out['predictions'][0]['id']=8
                for r in d['candidates']:r['nominal_forecast_conflict']['prediction_id']=8
            elif change=='progress':
                out['estimate']['pose']['position']['x']=.21;d['route_progress_m']=.21
                for p in out['trajectory']['points']:p['position']['x']=.21
                d['candidates'][0]['profile']['end_progress_m']=.21
            else:
                out['estimate']['pose']['position']['y']=.21
                for p in out['trajectory']['points']:p['position']['y']=.21
            report=out['planning_progress'];report.update(state='Observing',first_supporting_acquisition_s=2.1,
                supporting_acquisitions=1,supported_duration_s=0.,blocking_prediction_ids=[8] if change=='identity' else [7])
            self.assertEqual(audit(config,ticks)['final_state'],'Observing')

    def test_cached_decision_or_pose_change_clears_without_fabricating_an_acquisition(self):
        for identity in [False,True]:
            config,ticks=fixture();ticks=ticks[:42];out=ticks[-1]['expected']
            if identity:
                out['predictions'][0]['id']=8
                for r in out['planning_diagnostics']['candidates']:r['nominal_forecast_conflict']['prediction_id']=8
            else:
                out['estimate']['pose']['position']['x']=.21;out['planning_diagnostics']['route_progress_m']=.21
                for p in out['trajectory']['points']:p['position']['x']=.21
                out['planning_diagnostics']['candidates'][0]['profile']['end_progress_m']=.21
            out['planning_progress']=empty()
            self.assertEqual(audit(config,ticks)['final_state'],'Observing')

    def test_status_requires_its_policy_and_supported_inspection_context(self):
        config,ticks=fixture();config['monitor_planning_progress']=False
        with self.assertRaisesRegex(ValueError,'undeclared'):audit(config,ticks)
        config,ticks=fixture();config['monitor_planning_progress']=1
        with self.assertRaisesRegex(ValueError,'mistyped'):audit(config,ticks)
        for name in ['stop_lines','navigation','lidar3d']:
            config,ticks=fixture();config[name]=[1]
            with self.assertRaises(ValueError):statuses(config,ticks)

    def test_missing_status_or_decision_capture_is_rejected(self):
        config,ticks=fixture();ticks[0]['expected'].pop('planning_progress')
        with self.assertRaises(ValueError):audit(config,ticks)
        config,ticks=fixture();config['record_planning_diagnostics']=False
        with self.assertRaises(ValueError):audit(config,ticks)

    def test_mapped_yield_ownership_is_reconstructed_before_suppressing_support(self):
        config,ticks=fixture()
        config['yield_intersections']=[dict(stop_line=dict(id='yield',route_s_m=35.),exit_s_m=48.,
            conflict_bounds=dict(min=dict(x=42.,y=-2.1),max=dict(x=48.,y=2.1)))]
        for i,tick in enumerate(ticks):
            out=tick['expected'];stamp=(i//2)*.1;waiting=stamp<1.
            out['estimate']['pose']['yaw']=0.
            out['intersections']=dict(zones=[dict(id='yield',phase='Waiting' if waiting else 'Proceeding',
                blocking_tracks=[],clear_since=0.,committed=False)],stop_s_m=32.75 if waiting else None)
            if waiting:out['planning_progress']=empty()
            else:out['planning_progress'].update(state='Observing',first_supporting_acquisition_s=1.,
                supporting_acquisitions=i//2-9,supported_duration_s=stamp-1.)
        self.assertEqual(audit(config,ticks)['blocked_control_ticks'],0)
        ticks[0]['expected']['intersections']['stop_s_m']=33.
        with self.assertRaisesRegex(ValueError,'yield ownership'):audit(config,ticks)

    def test_projection_preserves_original_float_spelling_and_rejects_wrong_field_order(self):
        line=b'{"x":1.0000000000000002,"planning_progress":{},"planning_diagnostics":{}}\n'
        self.assertEqual(project_output(line),b'{"x":1.0000000000000002}\n')
        for value in [b'{"x":1.0}\n',b'{"planning_progress":{},"x":1.0,"planning_diagnostics":{}}\n']:
            with self.assertRaises(ValueError):project_output(value)

    def test_all_frozen_conditions_preserve_the_world_budgets_and_deadline(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),32)
        for spec in specs:
            _,value=materialize(spec);verify_policy(value,value,spec)
            original=copy.deepcopy(value);original.pop('monitor_planning_progress');original.pop('record_planning_diagnostics')
            self.assertEqual(original,parent()['frozen_inputs'][baseline(spec)['scenario']]['input'])
        _,value=materialize(specs[0]);value['duration']+=1
        with self.assertRaisesRegex(ValueError,'world, deadline or budget'):verify_policy(value,value,specs[0])


if __name__=='__main__':unittest.main()
