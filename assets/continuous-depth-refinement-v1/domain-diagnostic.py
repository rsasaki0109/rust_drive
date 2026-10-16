"""Post-hoc candidate-domain loss replay; no fitting or numeric reference labels."""
import argparse
import importlib.util
import io
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args = parser.parse_args()
    root = args.root.resolve();sys.path.insert(0,str(root/'scripts'))
    import continuous_depth_refinement as operational
    from projective_depth_common import sha,require,write
    from subpixel_depth_factors import sample
    def load(name,file):
        spec = importlib.util.spec_from_file_location(name,root/'scripts'/file)
        module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
    source_bytes = args.summary.read_bytes();source = json.loads(source_bytes)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'local label-free replay source')
    output = {};invalid = valid = 0
    for case,value in source['recorded'].items():
        checker = load('domain_check_'+case.replace('-','_'),'check-continuous-depth-refinement.py')
        audit = load('domain_raw_'+case.replace('-','_'),'check-reciprocal-keyframes.py').auditor()
        packet = root/'assets/reciprocal-keyframe-depth-v1';name = 'first/'+case+'-retain/freeze.json'
        inventory = json.loads((packet/'files.json').read_text());body = (packet/name).read_bytes()
        require(sha(body) == inventory[name]['sha256'] == value['input_freeze_sha256'],'same input freeze')
        freeze = json.loads(body);base = root/'data'/case;body = (base/'manifest.json').read_bytes()
        require(sha(body) == freeze['manifest_sha256'],'same sensor manifest')
        by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']};cache = {};rows = []
        def surfaces(stamp):
            if stamp not in cache:
                name = by_stamp[stamp]['depth_file'];body = (base/'raw'/name).read_bytes();expected = freeze['input_sha256'][name]
                require(len(body) == expected['bytes'] and sha(body) == expected['sha256'],'same depth bytes')
                image = Image.open(io.BytesIO(body));require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'same axial PNG')
                depth = np.asarray(image,dtype=float)/5000
                cache[stamp] = operational.base.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera'])
            return cache[stamp]
        for saved in value['samples']:
            report = saved['refinement'];initial = np.asarray(report['initial_poses'])
            nodes = [dict(timestamp=t,root=p.copy(),surface=surfaces(t)[0]) for t,p in zip(saved['node_timestamps'],initial,strict=True)]
            independent = [dict(n,surface=surfaces(n['timestamp'])[1]) for n in nodes]
            matches = operational.population(nodes,initial,1);other = checker.population(independent,initial,1)
            checker.check_metadata(report['frozen_population'],other)
            require(len(matches) == len(other),'same fixed factors')
            searches = [(np.asarray(s['before']),np.asarray(s['delta']),s['line_search']) for s in report['trace']]
            if report['failed_search'] is not None:
                searches.append((np.asarray(report['final_poses']),np.asarray(report['final_correction']['delta']),report['failed_search']))
            trials = []
            for search_index,(poses,direction,search) in enumerate(searches):
                for receipt in search['trials']:
                    alpha = receipt['alpha'];candidate = poses.copy()
                    for index,d in enumerate(direction,1):candidate[index] = audit.increment(poses[index],alpha*d)
                    edges = []
                    for item,wanted in zip(matches,other,strict=True):
                        a,b = item['target'],item['source']
                        valid_ids = operational.values(nodes[a],nodes[b],candidate,a,b,item['points'],item['normals'],1)[0]
                        expected_ids = checker.values(independent,candidate,a,b,wanted['points'],wanted['normals'],1)[0]
                        np.testing.assert_array_equal(valid_ids,expected_ids)
                        root_points = item['points']@candidate[b][:3,:3].T+candidate[b][:3,3]
                        q = (root_points-candidate[a][:3,3])@candidate[a][:3,:3]
                        sample_ids = sample(nodes[a]['surface'][1],q)[0]
                        relative = np.linalg.solve(candidate[a],candidate[b])
                        local = wanted['points']@relative[:3,:3].T+relative[:3,3]
                        expected_sample = checker.interpolation.independent_sample(independent[a]['surface'][1],local)[0]
                        np.testing.assert_array_equal(sample_ids,expected_sample)
                        edges.append(dict(target=a,source=b,initial_sources=len(item['source_ids']),
                            lost_interpolation_domain=len(item['source_ids'])-len(sample_ids),
                            lost_geometry_or_alignment=len(sample_ids)-len(valid_ids),
                            retained_sources=len(valid_ids)))
                    n = sum(e['initial_sources'] for e in edges)
                    interpolation_loss = sum(e['lost_interpolation_domain'] for e in edges)
                    qualification_loss = sum(e['lost_geometry_or_alignment'] for e in edges)
                    lost = interpolation_loss+qualification_loss
                    require(bool(lost) == (receipt['guard_failure'] is not None),'candidate invalidity exactly matches lost fixed sources')
                    invalid += bool(lost);valid += not lost
                    trials.append(dict(search_index=search_index,alpha=alpha,guard_failure=receipt['guard_failure'],
                        initial_source_occurrences=n,lost_source_occurrences=lost,
                        lost_interpolation_domain=interpolation_loss,lost_geometry_or_alignment=qualification_loss,
                        lost_fraction=lost/n if n else None,edges=edges))
            rows.append(dict(source_index=saved['source_index'],guard_failure=report['guard_failure'],trials=trials))
            cache = {t:cache[t] for t in saved['node_timestamps']}
        output[case] = rows
    require(invalid == 78 and valid == 1,'all archived candidate receipts replayed')
    write(args.output,dict(data_role='post-hoc label-free candidate replay, no fitting',
        input_summary_sha256=sha(source_bytes),diagnostic_source_sha256=sha(Path(__file__).read_bytes()),
        numeric_ground_truth_parsed=False,new_pose_fitting=False,invalid_candidate_trials=invalid,valid_candidate_trials=valid,
        lost_sources_are_overlapping_occurrences=True,recorded=output))
    print('Independently replayed',invalid,'invalid and',valid,'valid candidate domains',flush=True)


if __name__ == '__main__':main()
