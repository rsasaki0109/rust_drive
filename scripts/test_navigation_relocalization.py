#!/usr/bin/env python3
import copy
import tempfile
import unittest
from pathlib import Path

from check_navigation_relocalization import freeze, materialize, specifications, verify_navigation, scenario_matches
from check_lane_resume_speeds import interval_excess
from check_mid_edge import reference


def handover_ticks():
    _,scenario=materialize(dict(plant='reference-2d',world='straight',seed=7,mode='late-closure'))
    config=dict(navigation=scenario['navigation'],vehicle=dict(radius=1.25))
    ticks=[]
    for i in range(7):
        now=i*.05;following=i>=5
        reset=dict(stamp=.1,progress_s_m=10.,lateral_m=0.,distance_to_route_m=0.,yaw_error_rad=0.,admitted=True) if i>=2 else None
        state=dict(revision=7,phase='Following' if following else 'Braking',active_edges=['head','detour' if following else 'main','tail'],pending_edges=[] if following else ['head','detour','tail'],closed_edges=['main'],stop_s=None if following else 66.75,switches=int(following))
        if reset:state['localization_reset']=reset
        inp=dict(time=now,odometry=dict(stamp=now,speed=0.,yaw_rate=0.),navigation_update=dict(stamp=0.,revision=7,closed_edges=['main']) if i==0 else None)
        out=dict(estimate=dict(pose=dict(position=dict(x=10.,y=0.),yaw=0.),speed=0.),health=['StaleGnss'] if i==2 else [],command=dict(acceleration=-6.),navigation=state)
        ticks.append(dict(input=inp,expected=out))
    return config,ticks


class NavigationRecovery(unittest.TestCase):
    def test_frozen_32_cases_retain_explicit_mission_map_and_fault_schedules(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),32)
        self.assertEqual(len(specifications(['reference-2d'],True)),12)
        for spec in specs:
            _,value=materialize(spec)
            self.assertEqual(len(value['localization_map']['points']),4320)
            self.assertEqual(value['navigation']['goal'],'goal')
            self.assertEqual([e['speed_limit_m_s'] for e in value['navigation']['network']['edges']],[6.,6.,3.,4.])
            if spec['mode']=='pre-switched':
                self.assertEqual(value['sensor_timing']['odometry_dropout_windows'],[{'from':16.,'until':19.}])
                self.assertEqual(value['gnss_dropout_windows'],[{'from':16.,'until':24.}])
            if spec['mode']=='disabled':self.assertNotIn('relocalization',value)
            else:self.assertTrue(value['relocalization']['preserve_navigation'])
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);a=freeze(p,specs);self.assertEqual(freeze(p,specs),a)
            (p/a['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):freeze(p,specs)

    def test_serde_may_omit_empty_fault_schedules_but_cannot_change_them(self):
        _,frozen=materialize(dict(plant='reference-2d',world='straight',seed=7,mode='healthy'))
        actual=copy.deepcopy(frozen);actual.pop('gnss_dropout_windows');actual['sensor_timing'].pop('odometry_dropout_windows')
        self.assertTrue(scenario_matches(actual,frozen))
        actual['gnss_dropout_windows']=[{'from':8.,'until':16.}]
        self.assertFalse(scenario_matches(actual,frozen))
        actual.pop('gnss_dropout_windows');actual['sensor_timing']['odometry_dropout_windows']=[{'from':8.,'until':11.}]
        self.assertFalse(scenario_matches(actual,frozen))

    def test_closed_form_reset_requires_three_new_votes_and_rejects_forged_ownership(self):
        config,ticks=handover_ticks();result=verify_navigation(config,ticks,reset_stamp=.1)
        self.assertEqual(result['reset_ownership']['revision'],7)
        self.assertEqual(result['switch_events'],[dict(stamp=.25,active_edges=['head','detour','tail'],revision=7,after_reset=True)])
        for corruption in ['revision','reset_geometry','admission','cached_motion','early_switch']:
            changed=copy.deepcopy(ticks)
            if corruption=='revision':changed[3]['expected']['navigation']['revision']=0
            elif corruption=='reset_geometry':changed[2]['expected']['navigation']['localization_reset']['progress_s_m']+=1.
            elif corruption=='admission':changed[2]['expected']['navigation']['localization_reset']['admitted']=False
            elif corruption=='cached_motion':changed[3]['input']['odometry']=None
            else:changed[4]['expected']['navigation']['phase']='Following'
            with self.assertRaises(ValueError):verify_navigation(config,changed,reset_stamp=.1)

    def test_normal_blocked_hold_and_fault_have_separate_original_brake_contracts(self):
        config,_=handover_ticks()
        for fault in [False,True]:
            closed=['unknown'] if fault else ['main','detour']
            tick=dict(input=dict(time=0.,odometry=dict(stamp=0.,speed=0.,yaw_rate=0.),navigation_update=dict(stamp=0.,revision=1,closed_edges=closed)),
                      expected=dict(estimate=dict(pose=dict(position=dict(x=65.,y=0.),yaw=0.),speed=0.),health=['InvalidNavigation'] if fault else [],command=dict(acceleration=-6. if fault else -.5),navigation=dict(revision=0 if fault else 1,phase='Fault' if fault else 'Blocked',active_edges=['head','main','tail'],pending_edges=[],closed_edges=[] if fault else closed,stop_s=None if fault else 66.75,switches=0)))
            self.assertTrue(verify_navigation(config,[tick])['passed'])
            tick['expected']['command']['acceleration']=-.5 if fault else -.49
            with self.assertRaisesRegex(ValueError,'failed to brake'):verify_navigation(config,[tick])

    def test_independent_speed_scoring_exposes_a_lost_active_detour_cap(self):
        _,value=materialize(dict(plant='reference-2d',world='straight',seed=7,mode='pre-switched'))
        initial=reference(value['navigation'],[]);detour=reference(value['navigation'],['main'])
        truth=dict(pose=dict(position=dict(x=90.,y=4.),yaw=0.),speed=6.)
        wrong,_=interval_excess(initial['points'],initial['zones'],truth,truth,1.25)
        correct,_=interval_excess(detour['points'],detour['zones'],truth,truth,1.25)
        self.assertEqual(wrong,0.);self.assertEqual(correct,3.)


if __name__=='__main__':unittest.main()
