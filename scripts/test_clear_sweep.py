#!/usr/bin/env python3
"""Closed-form empty-sweep resolution and lost-identity recovery corruptions."""
import copy
import math
import unittest

from check_clear_sweep_recovery import recovery_policy
from forecast_consistency_oracle import reconstruct
from planar_sweep_oracle import acquired, audit_stream, blank_recovery, calibration, validate_policy
from test_forecast_consistency import fixture as positive_fixture


def empty(stamp=1.,plant='reference-2d'):
    return dict(stamp=stamp,points=[],sweep=dict(calibration=recovery_policy(plant)['lidar'],rays=['Clear']*720))


def fixture():
    config,ticks=positive_fixture();policy=recovery_policy('reference-2d');config['clear_sweep_recovery']=policy
    clear=blank_recovery()
    for i,tick in enumerate(ticks):
        inp,out=tick['input'],tick['expected'];now=inp['time'];report=out['forecast_consistency']
        inp['odometry']=dict(stamp=now,speed=0.,yaw_rate=0.)
        if inp['lidar'] is not None:
            inp['lidar']=empty(now)
            if i<2:
                inp['lidar']['points']=[dict(x=5.,y=0.)];inp['lidar']['sweep']['rays'][360]={'Return':0}
        if i>=3:
            out.update(tracks=[],predictions=[],track_measurements=[])
            report.update(checked_measurements=0,unmonitored_measurements=0,complete_scene_check=False,
                          recovery_samples=0,recovery_first_acquisition_s=None,retained_references=0)
            gap=math.tau/720
            clear['last_evidence']=dict(acquisition_s=now,checked_center_radius_m=40.,minimum_resolved_disc_radius_m=.4,
                maximum_angular_gap_rad=gap,maximum_transverse_gap_m=40.*math.sin(gap/2.),ray_count=720)
            clear.update(confirmation_samples=i-2,first_acquisition_s=.2,last_acquisition_s=now)
            if i==5:
                clear.update(confirmation_samples=0,first_acquisition_s=None,last_acquisition_s=None,release_count=1,
                    last_release=dict(identities=[7],control_time_s=.4,first_acquisition_s=.2,last_acquisition_s=.4,acquisition_count=3))
        report['clear_sweep']=copy.deepcopy(clear)
    return config,ticks


class ClearSweepTests(unittest.TestCase):
    def test_both_actual_beam_patterns_have_a_known_circular_resolution(self):
        for plant,divisor in [('reference-2d',720),('rne-dynamic',719)]:
            p=recovery_policy(plant);proof=acquired(empty(plant=plant),p)
            self.assertAlmostEqual(proof['maximum_angular_gap_rad'],math.tau/divisor)
            self.assertAlmostEqual(proof['maximum_transverse_gap_m'],40.*math.sin(math.pi/divisor))
            self.assertGreater(.4**2-proof['maximum_transverse_gap_m']**2,0.)

    def test_an_unannotated_empty_list_is_not_a_completed_empty_acquisition(self):
        scan=empty();scan.pop('sweep')
        with self.assertRaises(ValueError):acquired(scan,recovery_policy('reference-2d'))

    def test_a_return_requires_unique_point_ownership_and_cannot_certify_emptiness(self):
        scan=empty();scan['points']=[dict(x=5.,y=0.)];scan['sweep']['rays'][360]={'Return':0}
        self.assertIsNone(acquired(scan,recovery_policy('reference-2d')))
        scan['sweep']['rays'][360]='Clear'
        with self.assertRaises(ValueError):acquired(scan)
        scan['sweep']['rays'][0]={'Return':0};scan['sweep']['rays'][360]={'Return':0}
        with self.assertRaises(ValueError):acquired(scan)

    def test_incomplete_unavailable_wrong_bearing_and_out_of_range_rays_are_rejected(self):
        for mode in ['short','unavailable','index','bearing','range','nan']:
            scan=empty()
            if mode=='short':scan['sweep']['rays'].pop()
            elif mode=='unavailable':scan['sweep']['rays'][0]='Unavailable'
            else:
                scan['sweep']['rays'][360]={'Return':0}
                if mode!='index':scan['points']=[dict(x=100. if mode=='range' else 5.,y=1. if mode=='bearing' else float('nan') if mode=='nan' else 0.)]
            with self.subTest(mode=mode),self.assertRaises(ValueError):acquired(scan)

    def test_incomplete_angular_coverage_or_unresolved_sizes_cannot_be_declared_clear(self):
        for mode in ['gap','range','size','coarse','near','infinite']:
            p=recovery_policy('reference-2d')
            if mode=='gap':p['lidar']['angular_step_rad']/=2
            elif mode=='range':p['checked_center_radius_m']=45.
            elif mode=='size':p['minimum_resolved_disc_radius_m']=.1
            elif mode=='coarse':p['lidar'].update(ray_count=64,angular_step_rad=math.tau/64)
            elif mode=='near':p['lidar']['min_range_m']=.5
            else:p['lidar']['range_error_m']=float('inf')
            with self.subTest(mode=mode),self.assertRaises(ValueError):validate_policy(p)

    def test_three_new_sweeps_have_an_independently_known_lost_identity_release(self):
        config,ticks=fixture();result=reconstruct(config,ticks)
        self.assertEqual(result['final_held_identities'],[])
        self.assertEqual(result['clear_sweep_recoveries'],[dict(identities=[7],control_time_s=.4,first_acquisition_s=.2,last_acquisition_s=.4,acquisition_count=3)])
        stream=audit_stream(config,ticks);self.assertEqual(stream['acquired_sweeps'],5)
        self.assertEqual(stream['acquired_rays'],3600);self.assertEqual(stream['complete_empty_sweeps'],3)

    def test_missing_partial_or_positive_return_evidence_cannot_release_a_missing_track(self):
        for mode in ['missing','partial','return','calibration']:
            config,ticks=fixture();scan=ticks[3]['input']['lidar']
            if mode=='missing':scan.pop('sweep')
            elif mode=='partial':scan['sweep']['rays'][0]='Unavailable'
            elif mode=='return':scan['points']=[dict(x=5.,y=0.)];scan['sweep']['rays'][360]={'Return':0}
            else:scan['sweep']['calibration']['max_range_m']=44.
            with self.subTest(mode=mode),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_moving_cached_or_delayed_sensing_cannot_fabricate_stopped_confirmation(self):
        for mode in ['moving','motion-stamp','no-motion','scan-stamp']:
            config,ticks=fixture();inp=ticks[3]['input']
            if mode=='moving':inp['odometry']['speed']=.21
            elif mode=='motion-stamp':inp['odometry']['stamp']=.1
            elif mode=='no-motion':inp['odometry']=None
            else:inp['lidar']['stamp']=.1
            with self.subTest(mode=mode),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_changed_release_ownership_witness_clocks_and_votes_are_rejected(self):
        for mode in ['owner','first','last','count','witness','early','schema']:
            config,ticks=fixture();clear=ticks[-1]['expected']['forecast_consistency']['clear_sweep']
            if mode=='owner':clear['last_release']['identities']=[8]
            elif mode=='first':clear['last_release']['first_acquisition_s']=.1
            elif mode=='last':clear['last_release']['last_acquisition_s']=.3
            elif mode=='count':clear['release_count']=0
            elif mode=='witness':clear['last_evidence']['maximum_transverse_gap_m']=0.
            elif mode=='early':ticks[3]['expected']['forecast_consistency']['state']='Observing'
            else:clear['unverified']=True
            with self.subTest(mode=mode),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_upstream_fault_and_removed_braking_cannot_hide_behind_negative_sensing(self):
        for mode in ['fault','brake','health']:
            config,ticks=fixture();out=ticks[3]['expected']
            if mode=='fault':out['health'].append('StaleGnss')
            elif mode=='brake':out['command']['acceleration']=0.
            else:out['health']=[]
            with self.subTest(mode=mode),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_capture_materialization_preserves_every_preceding_physical_input(self):
        from check_clear_sweep_recovery import baseline,materialize,packet,PARENT,specifications
        rows=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(rows),64)
        for spec in rows:
            _,value=materialize(spec);self.assertTrue(value.pop('record_planar_sweep'))
            if spec['sweep_mode']=='recover':self.assertEqual(value.pop('clear_sweep_recovery'),recovery_policy(spec['plant']))
            self.assertEqual(value,packet(str(PARENT))['frozen_inputs'][baseline(spec)['scenario']]['input'])


if __name__=='__main__':unittest.main()
