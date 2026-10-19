#!/usr/bin/env python3
import copy
import json
import tempfile
import unittest
from pathlib import Path

from check_gnss_recovery import POLICY, freeze, materialize, specifications, verify_ticks


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


class Recovery(unittest.TestCase):
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
