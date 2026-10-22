#!/usr/bin/env python3
"""Reachability, complete forecast certificates and fixed parent conditions."""
import copy
import unittest

from check_later_stops import materialize,original,specifications,verify_policy
from later_stop_oracle import reconstruct
from test_checked_approaches import CONFIG,value


def fixture(searched=True):
    tick,_=value(.2,.2,[0.,.1,.2],adopted=True)
    out=tick['expected'];out.update(command=dict(acceleration=0.),emergency=False)
    out['planning_diagnostics']['candidates'][0]['profile']['searched_stop']=searched
    return dict(CONFIG,search_later_obstruction_stops=True),[tick]


class LaterStopTests(unittest.TestCase):
    def test_certifies_accelerated_paths_and_the_entire_eight_second_hold(self):
        config,ticks=fixture();proof=reconstruct(config,ticks)
        self.assertEqual(proof['certified_checked_stops'],1)
        self.assertEqual(proof['certified_searched_stops'],1)
        self.assertGreater(proof['interval_bounds'],0)
        self.assertIs(proof['actor_truth_used'],False)
        self.assertIs(proof['counterfactual_optimality_verified'],False)

    def test_a_false_search_marker_cannot_bypass_the_path_certificate(self):
        config,ticks=fixture(False);proof=reconstruct(config,ticks)
        self.assertEqual(proof['certified_checked_stops'],1)
        self.assertEqual(proof['certified_searched_stops'],0)
        ticks[0]['expected']['predictions'][0]['positions']=[dict(x=4.,y=0.)]*41
        with self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_interior_forecast_peaks_cannot_hide_behind_clear_endpoints(self):
        config,ticks=fixture();obj=ticks[0]['expected']['predictions'][0]
        obj['additional_radius_m']=[0.]*41;obj['additional_radius_m'][30]=200.
        with self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_stop_tail_endpoint_and_mode_are_required(self):
        for which in ['short','endpoint','moving','mode']:
            config,ticks=fixture();out=ticks[0]['expected'];points=out['trajectory']['points']
            if which=='short':points[-1]['time']=11.99
            if which=='endpoint':points[-1]['position']['x']+=.1
            if which=='moving':points[-1]['speed']=.01
            if which=='mode':out['trajectory']['mode']='Cruise'
            with self.subTest(which=which),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_kinematic_integrals_authority_and_finite_values_are_checked(self):
        for which in ['arc','acceleration','deceleration','time','speed','nan']:
            config,ticks=fixture();points=ticks[0]['expected']['trajectory']['points']
            if which=='arc':points[1]['position']['x']+=.1
            if which=='acceleration':config['motion_limits']=dict(max_acceleration_m_s2=1.9)
            if which=='deceleration':config['motion_limits']=dict(max_deceleration_m_s2=1.9)
            if which=='time':points[1]['time']=0.
            if which=='speed':points[1]['speed']=-1.
            if which=='nan':points[1]['speed']=float('nan')
            with self.subTest(which=which),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_faults_and_emergencies_keep_braking_including_nonfinite_rejection(self):
        for health in [[],['AcquisitionFailed']]:
            config,ticks=fixture();out=ticks[0]['expected'];out.update(health=health,emergency=not health)
            out['command']['acceleration']=-6.;self.assertEqual(reconstruct(config,ticks)['certified_checked_stops'],0)
            for acceleration in [-5.,0.,float('nan')]:
                out['command']['acceleration']=acceleration
                with self.subTest(health=health,acceleration=acceleration),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_missing_policy_or_decision_evidence_cannot_qualify(self):
        for which in ['flag','prerequisite','diagnostics','marker','limits']:
            config,ticks=fixture();out=ticks[0]['expected']
            if which=='flag':config['search_later_obstruction_stops']=1
            if which=='prerequisite':config['replan_checked_approaches']=False
            if which=='diagnostics':out.pop('planning_diagnostics')
            if which=='marker':out['planning_diagnostics']['candidates'][0]['profile']['searched_stop']=1
            if which=='limits':config['motion_limits']=dict(max_acceleration_m_s2=True)
            with self.subTest(which=which),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_all_forty_eight_parent_inputs_retain_their_budgets_and_goals(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),48)
        for spec in specs:
            _,config=materialize(spec);verify_policy(config,config,spec)
            self.assertIs(config.pop('search_later_obstruction_stops'),True)
            self.assertEqual(config,original(spec))
        spec=specs[0];_,frozen=materialize(spec)
        for which in ['budget','width','deadline','sensor','cold','replan','flag']:
            config=copy.deepcopy(frozen);changed=copy.deepcopy(frozen)
            if which=='budget':config['forecast_envelope']['acceleration_error_m_s2']=0.
            if which=='width':config['lateral_candidate_margin_m']=.1
            if which=='deadline':changed['duration']+=1.
            if which=='sensor':changed['heading_sensor']['variance_rad2']=1e-6
            if which=='cold':config['forecast_cold_max_speed_m_s']=14.
            if which=='replan':config['replan_checked_approaches']=False
            if which=='flag':config['search_later_obstruction_stops']=1
            with self.subTest(which=which),self.assertRaises(ValueError):verify_policy(config,changed,spec)


if __name__=='__main__':unittest.main()
