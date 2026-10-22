#!/usr/bin/env python3
import copy
import json
import tempfile
import unittest
from pathlib import Path

from check_gnss_recovery import POLICY, freeze, materialize, specifications, verify_ticks
import check_gnss_handoff as handoff


def stationary(expired=False):
    config = {'initial_pose': {'position': {'x': 0., 'y': 0.}, 'yaw': 0.},
              'gnss_latency_compensation': True, 'gnss_recovery': dict(POLICY, max_hold_s=.5 if expired else 5.)}
    # Independent scalar closed form, no motion or cross terms. The two 10 m
    # outliers fail NIS. Zero-residual proposals must not contract P until the
    # third confirmation; the expired branch never contracts it again.
    p = 1.*.02/1.02
    phases = ['Ready', 'Ready', 'Holding', 'Confirming', 'Confirming', 'Expired' if expired else 'Ready']
    ticks = []
    for i in range(6):
        now = i*.2
        if i:
            p += .003
        x = 10. if i in [1, 2] else 0.
        nis = x*x/(p+.02) if i else 0.
        accept = i == 0 or (i == 5 and not expired)
        if i == 5 and accept:
            p = p*.02/(p+.02)
        stamp = now if accept else 0.
        decision = 'Accepted' if accept else ('RejectedInnovation' if i in [1, 2] else 'RecoveryHeld')
        held = max(0, i-2) if expired else min(2, max(0, i-2))
        out = {'estimate': {'pose': {'position': {'x': 0., 'y': 0.}, 'yaw': 0.}, 'speed': 0.},
               'position_variance': 2.*p, 'pose_covariance': [[p, 0., 0.], [0., p, 0.], [0., 0., .03+.0001*i]],
               'health': (['GnssInnovationHold'] if i >= 2 and not accept else [])+(['StaleGnss'] if now-stamp > .75 else []),
               'command': {'acceleration': -6. if i >= 2 and not accept else 0.},
               'localization': {'accepted_fixes': 2 if i == 5 and accept else 1,
                                'rejected_fixes': i if i < 5 or expired else 4, 'last_observed_stamp': now,
                                'last_accepted_stamp': stamp, 'last_decision': decision, 'last_nis': nis},
               'gnss_latency': {'compensated_fixes': 0, 'history_unavailable_fixes': 0},
               'gnss_recovery': {'phase': phases[i], 'hold_started_at': .4 if i >= 2 and not accept else None,
                                 'consecutive_candidates': (i-2 if i in [3, 4] else 0),
                                 'last_candidate_stamp': now if i >= 3 else None,
                                 'last_position_correction_m': 0. if i >= 3 else None,
                                 'last_yaw_correction_rad': 0. if i >= 3 else None,
                                 'held_fixes': held, 'completed_recoveries': int(i == 5 and accept)}}
        ticks.append({'input': {'time': now, 'odometry': {'stamp': now, 'speed': 0., 'yaw_rate': 0.},
                                 'gnss': {'stamp': now, 'position': {'x': x, 'y': 0.}, 'variance': .02}}, 'expected': out})
    return config, ticks


def armed_stationary(expired=False, dt=.2):
    config = dict(initial_pose=dict(position=dict(x=0.,y=0.),yaw=0.),
                  initial_pose_covariance=[[.01,0.,0.],[0.,.01,0.],[0.,0.,.0001]],
                  gnss_latency_compensation=True,gnss_recovery=dict(POLICY,max_hold_s=.5 if expired else 5.),
                  relocalization=dict(confirm_gnss_handoff=True,map_only_grace_s=5.),
                  localization_map=dict(points=[dict(x=float(i),y=float(i%3)) for i in range(80)]))
    p=.01;ticks=[]
    for i in range(6):
        now=i*dt
        if i:p+=.015*dt
        accept=i>=3 and not expired
        if i==3 and accept:p=p*.02/(p+.02)
        observed=min(i,3)*dt if i else None
        count=min(i,3 if expired else 2)
        phase='Expired' if expired and i>=3 else 'Ready' if accept else 'Confirming' if i else 'Holding'
        health=['StaleLidar']+(['StaleGnss'] if not accept else [])+(['GnssInnovationHold'] if count>=2 and not accept else [])+(['GnssHandoffExpired'] if expired and i>=3 else [])
        out=dict(estimate=dict(pose=dict(position=dict(x=0.,y=0.),yaw=0.),speed=0.),
                 position_variance=2*p,pose_covariance=[[p,0.,0.],[0.,p,0.],[0.,0.,.0001+i*.0005*dt]],
                 health=health,command=dict(acceleration=-6.),
                 localization=dict(accepted_fixes=int(accept),rejected_fixes=count,last_observed_stamp=observed,
                                   last_accepted_stamp=3*dt if accept else None,last_decision='Accepted' if accept else 'RecoveryHeld' if i else None,last_nis=0. if i else None),
                 gnss_latency=dict(compensated_fixes=0,history_unavailable_fixes=0),
                 map_localization=dict(last_observed_stamp=None,last_accepted_stamp=None,decision='Missing',matched_pose=None,gnss_outage_covered=False),
                 relocalization=dict(map_only_support=dict(reset_stamp=0.,expires_at=5.,last_accepted_map_stamp=None,status='GnssAccepted' if accept else 'Interrupted' if i else 'WaitingForMap')),
                 gnss_recovery=dict(phase=phase,hold_started_at=None if accept else 0.,consecutive_candidates=i if i in [1,2] else 0,
                                    last_candidate_stamp=observed,last_position_correction_m=0. if i else None,last_yaw_correction_rad=0. if i else None,
                                    held_fixes=count,completed_recoveries=int(accept)))
        inp=dict(time=now,odometry=dict(stamp=now,speed=0.,yaw_rate=0.),lidar=None,lidar_failed=False,
                 gnss=dict(stamp=now,position=dict(x=0.,y=0.),variance=.02) if i in [1,2,3] else None)
        ticks.append(dict(input=inp,expected=out))
    return config,ticks


class Recovery(unittest.TestCase):
    def test_raw_wheel_sample_covers_only_the_original_short_freshness_window(self):
        config,ticks=armed_stationary(dt=.05)
        for tick in ticks[1:4]:tick['input']['odometry']=None
        self.assertTrue(verify_ticks(config,ticks,measured_reset_stamp=0.)['passed'])
        for tick in ticks[4:]:tick['input']['odometry']=None
        with self.assertRaisesRegex(ValueError,'healthy measured motion'):
            verify_ticks(config,ticks,measured_reset_stamp=0.)

    def test_measured_handoff_inputs_freeze_24_cases_and_only_the_flag_changes_in_pairs(self):
        specs=handoff.specifications(['reference-2d','rne-dynamic'])
        self.assertEqual(len(specs),24)
        self.assertEqual(sum(s['mode']=='guarded' for s in specs),12)
        self.assertEqual(len(handoff.specifications(['reference-2d'],True)),8)
        for spec in (s for s in specs if s['mode']=='unguarded'):
            _,a=handoff.materialize(spec);_,b=handoff.materialize(dict(spec,mode='guarded'))
            a.pop('name');b.pop('name');self.assertTrue(b['relocalization'].pop('confirm_gnss_handoff'))
            self.assertEqual(a,b)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);manifest=handoff.freeze(root,specs)
            self.assertEqual(handoff.freeze(root,specs),manifest)
            (root/manifest['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):handoff.freeze(root,specs)

    def test_armed_epoch_closed_form_keeps_candidates_out_and_uses_the_measured_deadline(self):
        for expired in [False,True]:
            config,ticks=armed_stationary(expired)
            result=verify_ticks(config,ticks,measured_reset_stamp=0.)
            self.assertTrue(result['passed'])
            self.assertEqual(result['held_proposals'],3 if expired else 2)
            self.assertEqual(result['accepted_fixes'],0 if expired else 1)
            for key in ['clock','covariance','deadline','expiry']:
                altered=copy.deepcopy(ticks)
                if key=='clock':altered[1]['expected']['localization']['last_accepted_stamp']=.2
                elif key=='covariance':altered[1]['expected']['pose_covariance'][0][0]*=.5
                elif key=='deadline':altered[2]['expected']['gnss_recovery']['hold_started_at']=.2
                else:altered[3]['expected']['health'].append('GnssHandoffExpired') if not expired else altered[3]['expected']['health'].remove('GnssHandoffExpired')
                with self.assertRaises(ValueError):verify_ticks(config,altered,measured_reset_stamp=0.)
    def test_closed_form_pending_and_expired_cases_reject_posterior_clock_and_guard_forgeries(self):
        for expired in [False, True]:
            config, ticks = stationary(expired)
            result = verify_ticks(config, ticks)
            self.assertTrue(result['passed'])
            self.assertEqual(result['completed_recoveries'], 0 if expired else 1)
            for corruption in ['posterior', 'clock', 'proposal_evidence']:
                changed = copy.deepcopy(ticks)
                out = changed[3]['expected']
                if corruption == 'posterior':
                    out['pose_covariance'][0][0] *= .5
                elif corruption == 'clock':
                    out['localization']['last_accepted_stamp'] = .6
                else:
                    out['gnss_recovery']['completed_recoveries'] = 1
                with self.assertRaises(ValueError):
                    verify_ticks(config, changed)

    def test_all_92_cases_are_frozen_with_paired_policy_only_changes_and_separate_confirmation(self):
        specs = specifications(['reference-2d', 'rne-dynamic'])
        self.assertEqual(len(specs), 92)
        self.assertEqual({m: sum(r['mode'] == m for r in specs) for m in ['guarded', 'control', 'expiry', 'confirm']},
                         {'guarded': 56, 'control': 24, 'expiry': 4, 'confirm': 8})
        for row in specs:
            if row['mode'] == 'guarded' and 'bias' in row['world']:
                _, a = materialize(row)
                _, b = materialize(dict(row, mode='control'))
                del a['gnss_recovery']
                self.assertEqual(a, b)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frozen = freeze(root, specs)
            self.assertEqual(freeze(root, specs), frozen)
            p = root/frozen['episodes'][0]['scenario']
            altered = json.loads(p.read_text())
            altered['duration'] += 1.
            p.write_text(json.dumps(altered))
            with self.assertRaises(ValueError):
                freeze(root, specs)


if __name__ == '__main__':
    unittest.main()
