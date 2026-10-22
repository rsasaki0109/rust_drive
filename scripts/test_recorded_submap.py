"""Reject forged map hashes and geometry changes while recording roundoff."""
import copy
import importlib.util
import math
from pathlib import Path
import unittest

from recorded_submap import compare_submap_reports

spec = importlib.util.spec_from_file_location('submap_oracle',
                                             Path(__file__).with_name('check-recorded-submaps.py'))
oracle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oracle)


def example(translation=0., point=.1):
    policy = dict(voxel_m=.06, max_points_per_voxel=1000)
    cfg = dict(max_map_points=5000, min_pairs=1)
    pose = dict(translation_m=[translation, 0., 0.], quaternion_wxyz=[1., 0., 0., 0.])
    scan = [[point, 0., 1.]]
    voxels, rejection = oracle.fusion({}, scan, oracle.exact_pose(pose), policy, cfg)
    assert rejection is None
    row = dict(timestamp=1.e9, accepted=True, map_updated=True, root_estimate=pose)
    for when, state in [('before', {}), ('after', voxels)]:
        values = dict(map_sha256=oracle.map_digest(state),
                      map_statistics_sha256=oracle.statistics_digest(state))
        row['state_' + when] = values
        for field, value in values.items():
            row[field + '_' + when] = value
    return dict(freeze=dict(submap_policy=policy, registration_config=cfg),
                manifest_sha256='original-input', frames=[row]), [scan]


class HistoricalSubmapTests(unittest.TestCase):
    def test_roundoff_is_verified_from_actual_geometry(self):
        before, clouds = example()
        after, _ = example(math.ulp(1.))
        proof = compare_submap_reports(before, after, clouds, oracle)
        self.assertFalse(proof['exact'])
        self.assertGreaterEqual(proof['rounded_values'], 2)
        self.assertEqual(proof['map_evidence']['exact_map_states_verified'], 4)
        self.assertEqual(len(proof['map_evidence']['differing_fingerprints']), 1)

    def test_forged_fingerprints_rejected_even_when_both_reports_match(self):
        before, clouds = example()
        for field in ('map_sha256', 'map_statistics_sha256'):
            after = copy.deepcopy(before)
            after['frames'][0][field + '_after'] = '0' * 64
            after['frames'][0]['state_after'][field] = '0' * 64
            with self.subTest(field=field), self.assertRaises(ValueError):
                compare_submap_reports(after, after, clouds, oracle)

    def test_voxel_boundary_change_rejected_with_valid_hashes(self):
        before, clouds = example(point=.06)
        after, _ = example(translation=-math.ulp(.06), point=.06)
        with self.assertRaises(ValueError):
            compare_submap_reports(before, after, clouds, oracle)

    def test_large_pose_change_rejected_with_valid_hashes(self):
        before, clouds = example()
        after, _ = example(.000001)
        with self.assertRaises(ValueError):
            compare_submap_reports(before, after, clouds, oracle)

    def test_clocks_decisions_and_sources_remain_exact(self):
        before, clouds = example()
        for mutate in [lambda x: x['frames'][0].update(timestamp=math.nextafter(1.e9, math.inf)),
                       lambda x: x['frames'][0].update(accepted=False),
                       lambda x: x.update(manifest_sha256='other-input')]:
            after = copy.deepcopy(before)
            mutate(after)
            with self.assertRaises(ValueError):
                compare_submap_reports(before, after, clouds, oracle)

    def test_exact_maps_remain_exact_and_coverage_is_required(self):
        before, clouds = example()
        self.assertTrue(compare_submap_reports(before, before, clouds, oracle)['exact'])
        with self.assertRaises(ValueError):
            compare_submap_reports(before, before, [], oracle)


if __name__ == '__main__':
    unittest.main()
