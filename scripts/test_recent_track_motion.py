#!/usr/bin/env python3
"""Check independent recent-window reconstruction and frozen-policy ownership."""
import copy
import unittest

from check_recent_track_motion import baseline,materialize,parent,specifications,verify_policy
from track_initialization_oracle import reconstruct


def fixture():
    ticks=[]
    for time,acquired,position,velocity,radius in [(0.,0.,0.,0.,1.),(.1,.1,.3,.48,1.1),
            (.2,.2,.8,4.,1.08),(.3,.3,1.2,4.,1.064),(.35,.3,1.2,4.,1.064)]:
        measurement=dict(id=1,stamp=acquired,center=dict(x=4*acquired,y=0.),radius=1.)
        ticks.append(dict(input=dict(time=time,lidar=dict(stamp=acquired,points=[])),
            expected=dict(time=time,health=[],track_measurements=[measurement],predictions=[],
                          tracks=[dict(id=1,last_seen=acquired,position=dict(x=position,y=0.),
                                       velocity=dict(x=velocity,y=0.),radius=radius)])))
    return dict(fit_recent_track_motion=True,enclose_track_measurements=True),ticks


class RecentTrackMotion(unittest.TestCase):
    def test_actual_acquisitions_fit_motion_and_cached_ticks_do_not_vote(self):
        config,ticks=fixture();r=reconstruct(config,ticks)
        self.assertEqual(r['recent_motion_fit']['fit_updates'],2)
        self.assertEqual(r['recent_motion_fit']['unsupported_updates'],1)
        self.assertEqual(r['recent_motion_fit']['maximum_samples'],3)
        self.assertEqual(r['cached_ticks'],1)

    def test_fitted_position_velocity_radius_and_clock_corruption_are_rejected(self):
        for key in ['position','velocity','radius','last_seen']:
            config,ticks=fixture();track=ticks[3]['expected']['tracks'][0]
            if key in ['position','velocity']:track[key]['x']+=.1
            else:track[key]+=.1
            with self.subTest(key=key),self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_cached_raw_measurements_cannot_be_renewed(self):
        config,ticks=fixture();ticks[-1]['expected']['track_measurements'][0]['stamp']=.35
        with self.assertRaisesRegex(ValueError,'cached raw'):reconstruct(config,ticks)

    def test_dropped_or_conflicting_fit_policy_is_rejected(self):
        for changes in [dict(fit_recent_track_motion=False),dict(confirm_track_initialization=True),
                        dict(enclose_track_measurements=False)]:
            config,ticks=fixture();config.update(changes)
            with self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_every_intervening_measurement_is_included(self):
        config,ticks=fixture();ticks[1]['expected']['track_measurements'][0]['center']['x']+=.4
        ticks[1]['expected']['tracks'][0].update(position=dict(x=.6,y=0.),velocity=dict(x=.96,y=0.),radius=1.2)
        with self.assertRaises(ValueError):reconstruct(config,ticks)

    def test_acquisition_gap_discards_old_fit_support(self):
        config,ticks=fixture();ticks=ticks[:3]
        for i in range(5,12):
            cached=copy.deepcopy(ticks[2]);cached['input']=dict(time=i*.05,lidar=None)
            cached['expected']['time']=i*.05;ticks.append(cached)
        last=copy.deepcopy(ticks[2]);last['input']=dict(time=.6,lidar=dict(stamp=.6,points=[]))
        last['expected']['time']=.6
        last['expected']['track_measurements'][0].update(stamp=.6,center=dict(x=2.,y=0.))
        last['expected']['tracks'][0].update(last_seen=.6,position=dict(x=2.1,y=0.),velocity=dict(x=3.88,y=0.),radius=1.1)
        ticks.append(last);r=reconstruct(config,ticks)
        self.assertEqual(r['recent_motion_fit']['fit_updates'],1)
        self.assertEqual(r['recent_motion_fit']['unsupported_updates'],2)

    def test_oversampling_is_bounded_and_cannot_mature_without_elapsed_time(self):
        config,base=fixture();ticks=[]
        for i in range(50):
            time=i*.0001;tick=copy.deepcopy(base[0]);tick['input']['time']=time
            tick['input']['lidar']['stamp']=time;out=tick['expected'];out['time']=time
            out['tracks'][0]['last_seen']=time;out['track_measurements'][0]['stamp']=time;ticks.append(tick)
        r=reconstruct(config,ticks)
        self.assertEqual(r['recent_motion_fit']['fit_updates'],0)
        self.assertEqual(r['recent_motion_fit']['maximum_samples'],32)

    def test_low_speed_observations_cannot_renew_stopping_fit_support(self):
        config,ticks=fixture();ticks=ticks[:3]
        for i in range(3,11):
            time=i*.1;tick=copy.deepcopy(ticks[2]);tick['input']=dict(time=time,lidar=dict(stamp=time,points=[]))
            out=tick['expected'];out['time']=time
            out['track_measurements'][0].update(stamp=time,center=dict(x=.8,y=0.))
            position=13./15. if i==3 else .8;velocity=2. if i==3 else 0.
            radius=1.+(1./15.)*.8**(i-3)
            out['tracks'][0].update(last_seen=time,position=dict(x=position,y=0.),velocity=dict(x=velocity,y=0.),radius=radius)
            ticks.append(tick)
        r=reconstruct(config,ticks)
        self.assertEqual(r['recent_motion_fit']['fit_updates'],8)
        self.assertEqual(r['recent_motion_fit']['unsupported_updates'],2)

    def test_full_frozen_cohort_changes_only_the_declared_fit_policy(self):
        specs=specifications(['reference-2d','rne-dynamic']);self.assertEqual(len(specs),32)
        for spec in specs:
            _,frozen=materialize(spec);value=copy.deepcopy(frozen);value.pop('fit_recent_track_motion')
            self.assertEqual(value,parent()['frozen_inputs'][baseline(spec)['scenario']]['input'])
            verify_policy(dict(fit_recent_track_motion=True,fit_sparse_circles=True),frozen,spec)

    def test_world_deadline_and_error_budget_changes_are_rejected(self):
        spec=specifications(['reference-2d'])[0]
        for key in ['duration','forecast_envelope']:
            _,frozen=materialize(spec)
            if key=='duration':frozen[key]+=1.
            else:frozen[key]['velocity_error_m_s']*=2.
            with self.assertRaisesRegex(ValueError,'world or error budget'):
                verify_policy(dict(fit_recent_track_motion=True,fit_sparse_circles=True),frozen,spec)

    def test_operational_recent_and_sparse_policies_are_both_required(self):
        spec=specifications(['reference-2d'])[0];_,frozen=materialize(spec)
        for config in [{},dict(fit_recent_track_motion=True),dict(fit_sparse_circles=True)]:
            with self.assertRaises(ValueError):verify_policy(config,frozen,spec)


if __name__=='__main__':unittest.main()
