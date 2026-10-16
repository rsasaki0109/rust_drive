#!/usr/bin/env python3
"""Describe archived prior/raw normal systems; no image decoding or pose fitting."""
import argparse
import gzip
import importlib.util
import json
from pathlib import Path

import numpy as np

from projective_depth_common import ROOT, fresh, require, sha, write


def describe(folder,audit):
    packet = folder.parents[1]
    manifest = json.loads((packet/'files.json').read_text())
    paths = [folder/'sensor-only.json.gz',folder/'scores.json']
    for p in paths:
        body = p.read_bytes();expected = manifest[p.relative_to(packet).as_posix()]
        require(len(body) == expected['bytes'] and sha(body) == expected['sha256'],'archived input digest')
    with gzip.open(paths[0],'rt') as f:ledger = json.load(f)
    require(not ledger['ground_truth_parsed'],'sensor-only ledger')
    prior = None;rows = []
    for row in ledger['frames']:
        u = row['update']
        require(u['prior_before_sha256'] == audit.digest(prior),'archived prior chain')
        if u['accepted'] and u['trace']:
            nodes = [dict(timestamp=t) for t in u['nodes']];poses = np.asarray(u['revised_roots'])
            hp,gp,delta = audit.independent_prior(prior,nodes,poses)
            h,g = np.asarray(u['final_equation']['h']),np.asarray(u['final_equation']['g'])
            hr,gr = h-hp,g-gp;scale = np.sqrt(np.diag(h));size = len(g)
            require(np.all(scale > 0) and np.isfinite(h).all(),'finite admitted system')
            require(np.max(abs(hp[-6:])) == 0 and np.max(abs(hp[:,-6:])) == 0
                    and np.max(abs(gp[-6:])) == 0,'no prior on a new current acquisition')
            denominator = np.linalg.norm(gr/scale)+np.linalg.norm(gp/scale)
            normalized_raw = hr/scale[:,None]/scale[None,:]
            eigen = np.linalg.eigvalsh(normalized_raw)
            d = np.asarray(delta).reshape(-1,6)
            rows.append(dict(source_index=row['source_index'],timestamp=u['timestamp'],
                scaled_prior_trace_fraction=float(np.trace(hp/scale[:,None]/scale[None,:])/size),
                scaled_gradient_cancellation=float(1-np.linalg.norm(g/scale)/denominator) if denominator else 0.,
                raw_spd_in_combined_scaling=bool(eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6),
                max_prior_anchor_translation_m=float(max(np.linalg.norm(d[:,:3],axis=1),default=0.)),
                max_prior_anchor_rotation_rad=float(max(np.linalg.norm(d[:,3:],axis=1),default=0.)),
                variable_count=size))
        if u['accepted']:prior = u['prior_commit']
        require(u['prior_after_sha256'] == audit.digest(prior),'accepted prior chain')
    # Label grouping happens after all sensor-only algebra, never feeds a fitter.
    scores = json.loads(paths[1].read_text())
    labels = {r['source_index']:r.get('accurate',False) for r in scores['frames']}
    keys = ('scaled_prior_trace_fraction','scaled_gradient_cancellation',
            'max_prior_anchor_translation_m','max_prior_anchor_rotation_rad')
    summary = {k:dict(zip(('p50','p95','max'),np.percentile([r[k] for r in rows],[50,95,100]).tolist())) for k in keys}
    summary.update(updates=len(rows),raw_spd_in_combined_scaling=sum(r['raw_spd_in_combined_scaling'] for r in rows),
                   accurate_updates=sum(labels[r['source_index']] for r in rows),new_acquisition_prior_block_exactly_zero=True)
    return dict(summary=summary,frames=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    spec = importlib.util.spec_from_file_location('prior_audit',ROOT/'scripts/check-marginalized-keyframes.py')
    audit = importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    reports = {}
    for case in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'):
        folder = ROOT/'assets/marginalized-keyframe-depth-v1/first'/(case+'-retain')
        reports[case] = describe(folder,audit)
    write(out/'diagnostic.json',dict(role='already viewed, algebra-only diagnostic; not causal attribution',
        source_sha256=sha(Path(__file__).read_bytes()),image_decoding=False,pose_fitting=False,reports=reports))


if __name__ == '__main__':main()
