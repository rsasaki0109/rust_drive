#!/usr/bin/env python3
"""Independent per-level systems at frozen rejected poses; no fitting or labels."""
import argparse
import gzip
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from projective_depth_common import ROOT, fresh, require, sha, write


def system(audit,nodes,poses,prior,scale):
    e = audit.independent_equation(nodes,poses,scale,prior)
    supported = any(x['source'] == len(nodes)-1 for x in e['edges'])
    diag = np.diag(e['h']);condition = None;translation = rotation = None
    positive = np.isfinite(e['h']).all() and np.isfinite(e['g']).all() and np.all(diag > 0)
    if positive:
        s = np.sqrt(diag);eig = np.linalg.eigvalsh(e['h']/s[:,None]/s[None,:])
        if eig[0] > 0:condition = float(eig[-1]/eig[0])
    observable = condition is not None and condition <= 1e6
    if observable:
        d,_ = audit.independent_solve(e['h'],e['g'])
        translation = float(np.linalg.norm(d[:,:3],axis=1).max())
        rotation = float(np.linalg.norm(d[:,3:],axis=1).max())
    return dict(scale=scale,current_measured_support=supported,count=e['count'],raw_rms_m=e['rms_m'],
        scaled_condition=condition,max_node_proposed_translation_m=translation,max_node_proposed_rotation_rad=rotation,
        iteration_admissible=bool(supported and observable and translation <= .1 and rotation <= .15))


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    spec = importlib.util.spec_from_file_location('patch_level_audit',ROOT/'scripts/check-patch-keyframes.py')
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);audit = module.auditor()
    packet = ROOT/'assets/patch-keyframe-depth-v1';manifest = json.loads((packet/'files.json').read_text())
    def archived(name):
        body = (packet/name).read_bytes();info = manifest[name]
        require(sha(body) == info['sha256'] and len(body) == info['bytes'],'frozen evidence '+name)
        return body
    reports = {};input_evidence = {}
    for case in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'):
        prefix = 'first/'+case+'-retain/'
        freeze = json.loads(archived(prefix+'freeze.json'))
        sensor_name = prefix+'sensor-only.json.gz';ledger = json.loads(gzip.decompress(archived(sensor_name)))
        input_evidence[case] = dict(freeze=sha(archived(prefix+'freeze.json')),sensor_ledger=sha(archived(sensor_name)))
        base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes()
        require(sha(body) == freeze['manifest_sha256'],'same selected acquisitions')
        frames = json.loads(body)['frames'];by_stamp = {f['depth_timestamp']:f for f in frames}
        surfaces = {};poses_by_stamp = {};prior = None;latest = None;rows = []
        def surface(stamp):
            if stamp not in surfaces:
                name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name]
                raw = (base/'raw'/name).read_bytes()
                require(sha(raw) == expected['sha256'] and len(raw) == expected['bytes'],'raw image '+name)
                depth = np.asarray(Image.open(io.BytesIO(raw)),dtype=float)/5000
                surfaces[stamp] = audit.pyramid(depth,freeze['camera'])
            return surfaces[stamp]
        for frame,row in zip(frames,ledger['frames'],strict=True):
            u = row['update'];stamp = frame['depth_timestamp']
            require(row['source_index'] == frame['source_index'],'same frame identity')
            if u['accepted']:
                if latest is None:poses_by_stamp = {stamp:np.eye(4)}
                else:
                    keep = u['marginalization']['kept_ids']
                    poses_by_stamp = {t:np.asarray(p) for t,p in zip(u['nodes'],u['revised_roots'],strict=True) if t in keep}
                prior = u['prior_commit'];latest = stamp
            elif not u['lost']:
                require(u['stage'] == 'fit' and u['prior_before_sha256'] == audit.digest(prior),'frozen rejected-fit input')
                ids = u['nodes'];start = np.asarray([poses_by_stamp[t] if t != stamp else poses_by_stamp[latest] for t in ids])
                nodes = [dict(timestamp=t,root=p,surface=surface(t)) for t,p in zip(ids,start,strict=True)]
                attempted = start.copy()
                for step in u['trace']:
                    np.testing.assert_allclose(step['before'],attempted,atol=1e-12,rtol=0)
                    attempted = audit.audit_step(nodes,prior,step)
                audit.verify_failed_fit(nodes,attempted,prior,u)
                entry = dict(source_index=row['source_index'],reason=u['rejection'],rejected_scale=audit.next_level(u['trace']) or 1,reported_iterations=len(u['trace']),
                    start=[system(audit,nodes,start,prior,s) for s in (4,2,1)],
                    rejected_state=[system(audit,nodes,attempted,prior,s) for s in (4,2,1)])
                rows.append(entry)
                print(case,row['source_index'],'start/failure fine admissible',entry['start'][-1]['iteration_admissible'],entry['rejected_state'][-1]['iteration_admissible'],flush=True)
        require(len(rows) == 6,'all six pre-expiry fit rejections');reports[case] = rows
    sources = [Path(__file__),*[ROOT/p for p in json.loads((packet/'source-freeze.json').read_text()) if p.startswith('scripts/')]]
    write(out/'summary.json',dict(pose_fitting=False,numeric_ground_truth_parsed=False,
        data_role='already viewed development; frozen failure diagnostic',input_evidence=input_evidence,reports=reports,
        source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources}))


if __name__ == '__main__':main()
