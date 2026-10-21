#!/usr/bin/env python3
"""Adversarial retention, comparison and frozen-world checks for stop search."""
import copy
from pathlib import Path
import tempfile
import unittest

from check_stop_search import compare, contract, freeze, materialize, specifications
from check_absolute_heading import materialize as heading_input
from check_obstruction_stops import materialize as obstruction_input
from check_road_width import materialize as road_input
from test_obstruction_stops import config, output as original_output, sample, ticks


def output(*args, **kwargs):
    value=original_output(*args, **kwargs)
    value['command']['steering']=0.
    return value


def braking():
    value=output(path=[sample(0.,1.,0.),sample(.2,0.,.4),sample(.2,0.,8.4)],command=-2.5)
    value['estimate']['speed']=1.
    return value


class StopSearchTests(unittest.TestCase):
    def test_a_planned_stop_arms_without_an_emergency_and_cannot_restart_on_another_prefix(self):
        held = output(.1); result = contract(config(),ticks([braking(),held]))
        self.assertTrue(result['passed']); self.assertEqual(result['arms'],[0.]); self.assertEqual(result['held_ticks'],1)
        bad = output(.1,command=2.)
        self.assertFalse(contract(config(),ticks([braking(),held,bad]))['passed'])
        incomplete = output(.1); incomplete['trajectory']['points'][-1]['time']=7.9
        self.assertFalse(contract(config(),ticks([braking(),held,incomplete]))['passed'])
        self.assertEqual(contract(config(),ticks([output(command=2.)]))['arms'],[])

    def test_only_a_complete_certified_candidate_releases_the_hold(self):
        held = output(); clear = output(.1,path=[sample(0.,0.,0.),sample(40.,8.,10.)],command=2.)
        result=contract(config(),ticks([braking(),held,clear]))
        self.assertTrue(result['passed']);self.assertEqual(result['releases'],[.1])
        conflict=copy.deepcopy(clear);conflict['predictions'][0]['positions']=[dict(x=20.,y=0.)]*41
        with self.assertRaises(ValueError):contract(config(),ticks([braking(),held,conflict]))

    def test_a_health_fault_cannot_release_retention_or_emit_an_unbraked_command(self):
        held=output(); fault=output(.1,path=[sample(0.,0.,0.),sample(40.,8.,10.)],command=-6.)
        fault['health']=['StaleGnss']
        result=contract(config(),ticks([braking(),held,fault,output(.2)]))
        self.assertTrue(result['passed']);self.assertEqual(result['releases'],[]);self.assertEqual(result['held_ticks'],2)
        fault['command']['acceleration']=0.
        self.assertFalse(contract(config(),ticks([braking(),held,fault]))['passed'])
        fault['command']['acceleration']=float('nan')
        with self.assertRaises(ValueError):contract(config(),ticks([braking(),held,fault]))

    def test_comparison_retains_new_coverage_failures_and_requires_complete_replay(self):
        baseline=dict(failed_gates=['source_acceptance'],verified_emergency_steps=50)
        row=dict(world='blocked',run_exit_code=0,replay_exit_code=0,summary=dict(passed=True,steps=2),
                 replay=dict(verified=True,ticks=2),failed_gates=[],verified_emergency_steps=0)
        self.assertTrue(compare(row,baseline)['regression_passed'])
        row['failed_gates']=['forecast_coverage']
        self.assertFalse(compare(row,baseline)['regression_passed'])
        row['failed_gates']=[];row['replay']['ticks']=1
        self.assertFalse(compare(row,baseline)['regression_passed'])

    def test_mission_stop_improvement_does_not_reclassify_the_missing_goal_as_a_pass(self):
        baseline=dict(failed_gates=['source_acceptance'],verified_emergency_steps=50)
        row=dict(world='mission',run_exit_code=1,replay_exit_code=0,summary=dict(passed=False,steps=1301),
                 replay=dict(verified=True,ticks=1301),failed_gates=['source_acceptance'],verified_emergency_steps=10,
                 stop_contract=dict(passed=True),mission_hold=dict(samples=901,maximum_actual_speed_m_s=0.,positive_control_commands=0))
        self.assertTrue(compare(row,baseline)['stable_mission_stop_passed'])
        self.assertEqual(row['failed_gates'],['source_acceptance'])
        row['verified_emergency_steps']=11
        self.assertFalse(compare(row,baseline)['stable_mission_stop_passed'])
        row['verified_emergency_steps']=0;row['mission_hold']['maximum_actual_speed_m_s']=.051
        self.assertFalse(compare(row,baseline)['stable_mission_stop_passed'])

    def test_all_40_inputs_add_only_the_stop_search_policy_to_unchanged_preceding_worlds(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),40)
        self.assertEqual(len({(s['plant'],s['world'],s['mode'],s['seed']) for s in specs}),40)
        for spec in specs:
            source,value=materialize(spec);self.assertTrue(value.pop('search_obstruction_stops'))
            if spec['family']=='heading':original=heading_input(spec)
            elif spec['family']=='road':original=road_input(dict(spec,mode='fitted'))
            else:original=obstruction_input(dict(spec,mode='envelope' if spec['world']=='blocked' else 'enabled',settle=True))
            self.assertEqual((source,value),original)
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory);manifest=freeze(output,specs)
            (output/manifest['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):freeze(output,specs)


if __name__=='__main__':unittest.main()
