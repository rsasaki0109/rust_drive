#!/usr/bin/env python3
"""Independent conflict-witness regressions; these are authored arithmetic inputs."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from check_lane_occupancy import audit, first_witness


class ContainedWitnessTests(unittest.TestCase):
    def fixture(self, angle=0.):
        import math
        def point(x, y):
            return {'x': x*math.cos(angle)-y*math.sin(angle),
                    'y': x*math.sin(angle)+y*math.cos(angle)}
        config = {'vehicle': {'radius': 1.25}, 'cruise_speed': 2., 'lane_change': {
            'target_start': point(0., 3.5), 'target_end': point(80., 3.5), 'target_width_m': 3.5}}
        output = {'time': 1., 'estimate': {'pose': {'position': point(20., 0.)}, 'speed': 0.}}
        track = {'id': 7, 'position': point(7.25, 3.5), 'velocity': point(.6, 0.),
                 'last_seen': 1., 'radius': 1.}
        forecast = {'id': 7, 'radius': 1., 'dt': .2,
                    'positions': [point(7.25+.3*i*.2, 3.5) for i in range(31)],
                    'additional_radius_m': [.3*i*.2 for i in range(31)]}
        return config, output, track, forecast

    def test_contained_rear_conflict_exists_in_rotated_world_axes(self):
        for angle in [0., 1.5707963267948966, -.7]:
            witness = first_witness(*self.fixture(angle))
            self.assertIsNotNone(witness)
            self.assertAlmostEqual(witness['future_seconds'], 5.505)
            self.assertAlmostEqual(witness['rear_headway_distance_m'], 9.45)

    def test_receding_continuation_has_no_conflict_and_missing_coverage_is_rejected(self):
        config, output, track, forecast = self.fixture()
        receding = copy.deepcopy(track)
        receding['velocity']['x'] = -.6
        self.assertIsNone(first_witness(config, output, receding, forecast))
        forecast['additional_radius_m'] = [0.]*31
        with self.assertRaises(ValueError):
            first_witness(config, output, track, forecast)

    def test_a_contained_conflict_requires_a_waiting_gate_and_revoked_clear_time(self):
        config, output, track, forecast = self.fixture()
        output.update(tracks=[track], predictions=[forecast], lane_change={
            'phase': 'Ready', 'blocking_tracks': [], 'clear_since': 0.})
        rows = [{'kind': 'header', 'header': {'config': config}},
                {'kind': 'tick', 'tick': {'expected': output}}]
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)/'arithmetic-only.jsonl'
            p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            result = audit(p)
            self.assertFalse(result['verified'])
            self.assertEqual(result['missing_or_unrevoked_conflicts'], 1)
            output['lane_change'] = {'phase': 'Waiting', 'blocking_tracks': [7], 'clear_since': None}
            p.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            self.assertTrue(audit(p)['verified'])


if __name__ == '__main__':
    unittest.main()
