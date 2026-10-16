#!/usr/bin/env python3
"""Reconstruct symmetric-control residual tails at saved poses; never fit poses."""
import argparse
import gzip
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from robust_depth_factors import correspondences,weights
from symmetric_depth_factors import residual
from projective_depth_common import ROOT,CAMERA,fresh,require,sha,write
from projective_depth_raw import pyramid


def diagnose(case):
    packet = ROOT/'assets/symmetric-keyframe-depth-v1';folder = packet/'first'/(case+'-retain')
    inventory = json.loads((packet/'files.json').read_text())
    for path in (folder/'freeze.json',folder/'sensor-only.json.gz'):
        body = path.read_bytes();expected = inventory[path.relative_to(packet).as_posix()]
        require(sha(body) == expected['sha256'] and len(body) == expected['bytes'],'archived control digest')
    freeze = json.loads((folder/'freeze.json').read_text());base = ROOT/'data'/case
    body = (base/'manifest.json').read_bytes();require(sha(body) == freeze['manifest_sha256'],'original manifest')
    frames = json.loads(body)['frames']
    with gzip.open(folder/'sensor-only.json.gz','rt') as f:ledger = json.load(f)
    require(len(frames) == len(ledger['frames']) == 180 and not ledger['ground_truth_parsed'],'complete sensor ledger')
    cache = {};rows = [];totals = dict(candidates=0,residual_over_1cm=0,residual_at_least_3cm=0,
        normal_angle_over_15deg=0,control_pairs=0,tukey_pairs=0,current_without_tukey_pair=0)
    weight_mass = squared = 0.
    for frame,row in zip(frames,ledger['frames']):
        stamp = frame['depth_timestamp'];u = row['update'];require(u['accepted'] and u['timestamp'] == stamp,'saved accepted acquisitions')
        body = (base/'raw'/frame['depth_file']).read_bytes();expected = freeze['input_sha256'][frame['depth_file']]
        require(len(body) == expected['bytes'] and sha(body) == expected['sha256'],'measured depth digest')
        current = dict(surface=pyramid(np.asarray(Image.open(io.BytesIO(body)),dtype=float)/5000,CAMERA))
        if not cache:cache = {stamp:current};continue
        ids = u['nodes'];nodes = [cache[t] for t in ids[:-1]]+[current];poses = np.asarray(u['revised_roots'])
        expected = {(e['target'],e['source']):e for e in u['final_equation']['edges']}
        seen = set();parts = [];angles = [];masses = [];current_pairs = 0;eligible = 0
        for source in range(1,len(nodes)):
            for target in range(source):
                p,n,q,m = correspondences(nodes[target],nodes[source],poses[target],poses[source],1)
                r,_ = residual(p,n,q,m)
                if len(r) < 200:
                    require((target,source) not in expected,'same control pair admission');continue
                e = expected[target,source];require(e['count'] == len(r),'same original candidate count')
                np.testing.assert_allclose(np.sqrt(r@r/len(r)),e['rms_m'],atol=1e-10,rtol=0)
                seen.add((target,source));mass = float(weights(r,1).sum());masses.append(mass)
                angle = np.arccos(np.clip(np.sum(n*m,axis=1),-1,1))
                parts.append(r);angles.append(angle);eligible += mass >= 200
                current_pairs += mass >= 200 and source == len(nodes)-1
        require(seen == set(expected),'all original final-pose edges reproduced')
        r = np.concatenate(parts);angle = np.concatenate(angles)
        stats = dict(source_index=row['source_index'],candidates=len(r),control_pairs=len(parts),tukey_pairs=eligible,
            tukey_weight_mass=float(sum(masses)),residual_over_1cm=int((abs(r) > .01).sum()),
            residual_at_least_3cm=int((abs(r) >= .03).sum()),normal_angle_over_15deg=int((angle > np.pi/12).sum()),
            abs_residual_m_p50_p95_max=np.percentile(abs(r),[50,95,100]).tolist(),
            normal_angle_rad_p50_p95_max=np.percentile(angle,[50,95,100]).tolist(),
            current_without_tukey_pair=int(current_pairs == 0))
        for key in totals:totals[key] += stats[key]
        weight_mass += sum(masses);squared += float(r@r);rows.append(stats)
        keep = u['marginalization']['kept_ids'];cache = {t:n for t,n in zip(ids,nodes) if t in keep}
    return dict(summary=dict(**totals,updates=len(rows),tukey_weight_mass=float(weight_mass),
        unweighted_rms_m=float(np.sqrt(squared/totals['candidates']))),frames=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    reports = {case:diagnose(case) for case in ('tum-fr1-room-temporal','tum-fr1-desk2-independent')}
    write(out/'diagnostic.json',dict(role='already viewed control; large residuals are not outlier truth labels',
        pose_fitting=False,numeric_ground_truth_parsed=False,scale=1,reports=reports,
        source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in
            [Path(__file__),ROOT/'scripts/robust_depth_factors.py',ROOT/'scripts/symmetric_depth_factors.py']}))


if __name__ == '__main__':main()
