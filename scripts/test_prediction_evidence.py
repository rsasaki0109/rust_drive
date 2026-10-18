#!/usr/bin/env python3
"""Adversarial arithmetic fixtures for the forecast auditor, not sensor replays."""
import copy
import json
import tempfile
import unittest
from pathlib import Path

import check_hazards as auditor


def track(time, speed):
    return {'id': 11, 'last_seen': time, 'radius': .7,
            'position': {'x': 10. + .5*time, 'y': 3.},
            'velocity': {'x': speed, 'y': 0.}}


def fixture(slow):
    rows = [{'kind': 'header', 'header': {'config': {'confirm_slow_motion': slow}}}]
    position = 10.
    previous_speed = .5 if slow else 4.
    for i in range(7):
        time = i*.1
        speed = .5 if slow else (3.95 if i == 3 else 4.-.5*time)
        current = track(time, speed)
        if i:
            position += .05*(previous_speed+speed)
        current['position']['x'] = position
        previous_speed = speed
        if slow and i == 3:
            current['velocity'] = {'x': 0., 'y': .5}
        predictions = [{'id': 11, 'dt': .2, 'radius': .7,
                        'positions': [{'x': position+(0. if slow else speed*j*.2), 'y': 3.}
                                      for j in range(41)]}]
        rows.append({'kind': 'tick', 'tick': {'input': {'time': time},
                    'expected': {'tracks': [current], 'predictions': predictions}}})
    return rows


def audit(rows):
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder)/'arithmetic-only.jsonl'
        path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        return auditor.check_motion_predictions(path)


class ObservationEvidenceTests(unittest.TestCase):
    def test_intermediate_stop_turn_and_backward_step_are_not_erased_by_anchors(self):
        for contradiction in range(3):
            samples = [track(i*.1, .5) for i in range(7)]
            if contradiction == 0:
                samples[3]['velocity'] = {'x': 0., 'y': 0.}
            elif contradiction == 1:
                samples[3]['velocity'] = {'x': 0., 'y': .5}
            else:
                samples[3]['position']['x'] -= .2
            self.assertFalse(auditor.supported_slow_motion(samples, samples[-1]))

    def test_contradicted_braking_uses_velocity_and_the_old_braking_chord_is_rejected(self):
        rows = fixture(False)
        result = audit(rows)
        self.assertTrue(result['verified'])
        self.assertEqual(result['braking_forecasts'], 0)
        self.assertGreater(result['intermediate_braking_vetoes'], 0)
        changed = copy.deepcopy(rows)
        output = changed[-1]['tick']['expected']
        origin = output['tracks'][0]['position']['x']
        for i, point in enumerate(output['predictions'][0]['positions']):
            time = i*.2
            braking_time = min(time, 1.)
            point['x'] = origin+3.7*braking_time-.125*braking_time**2+3.45*(time-braking_time)
        with self.assertRaises(ValueError):
            audit(changed)

    def test_a_slow_turn_uses_the_deadband_and_the_old_confirmed_motion_is_rejected(self):
        rows = fixture(True)
        result = audit(rows)
        self.assertTrue(result['verified'])
        self.assertEqual(result['confirmed_slow_forecasts'], 0)
        changed = copy.deepcopy(rows)
        output = changed[-1]['tick']['expected']
        origin = output['tracks'][0]['position']['x']
        for i, point in enumerate(output['predictions'][0]['positions']):
            point['x'] = origin+.5*i*.2
        with self.assertRaises(ValueError):
            audit(changed)


if __name__ == '__main__':
    unittest.main()
