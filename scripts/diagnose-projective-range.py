#!/usr/bin/env python3
"""Distance-stratified raw-pair residuals on viewed TUM recordings; offline probes."""
import argparse,importlib.util,io,json
from pathlib import Path
import numpy as np
from PIL import Image
from projective_depth_common import CAMERA,ROOT,fresh,require,sha,write
from projective_depth_raw import pyramid,normal_equation,compare_equation
from projective_range_raw import pairs,equation,controls

CASES=('tum-fr1-room-temporal','tum-fr1-desk2-independent','tum-fr1-rpy-heldout')
BINS=(.3,1.,2.,3.,5.)


def statistics(values):
    if len(values)==0:return dict(count=0)
    center=np.median(values)
    return dict(count=len(values),median_m=float(center),mad_scale_m=float(1.4826*np.median(abs(values-center))),
                median_abs_m=float(np.median(abs(values))),rms_m=float(np.sqrt(np.mean(values**2))),
                p95_abs_m=float(np.quantile(abs(values),.95)))


def stratify(depth,residual):
    return {str(low)+'-'+str(high):statistics(residual[(depth>=low)&(depth<high)]) for low,high in zip(BINS,BINS[1:])}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=fresh(args.output)
    sources=(Path(__file__),ROOT/'scripts/projective_range_raw.py',ROOT/'scripts/projective_depth_common.py',
             ROOT/'scripts/projective_depth_raw.py',ROOT/'scripts/check-opencv-rgbd.py')
    pins={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources}
    write(out/'freeze.json',dict(source_sha256=pins,selected_frame_offsets=list(range(1,180,12)),
        cases=CASES,depth_bins_m=BINS,scale=1,ground_truth_operational=False,
        reference_pose_role='offline local objective probe, never a runtime initialization',
        interpretation='residual dispersion includes calibration, motion-reference, correspondence and surface errors; not isolated independent sensor noise'))
    spec=importlib.util.spec_from_file_location('reference',ROOT/'scripts/check-opencv-rgbd.py')
    reference=importlib.util.module_from_spec(spec);spec.loader.exec_module(reference);reference.controls()
    summaries={}
    for case in CASES:
        base=ROOT/'data'/case;manifest_body=(base/'manifest.json').read_bytes();manifest=json.loads(manifest_body)
        inventory={x['file']:x for x in manifest['files']}
        def read(name):
            path=base/'raw'/name;require(path.is_file() and not path.is_symlink(),'regular original file')
            body=path.read_bytes();p=inventory[name];require(len(body)==p['bytes'] and sha(body)==p['sha256'],'source hash')
            return body
        labels=np.loadtxt(io.BytesIO(read('groundtruth.txt')),comments='#')
        require(labels.ndim==2 and labels.shape[1]==8 and np.isfinite(labels).all() and np.all(np.diff(labels[:,0])>0),'finite chronological offline reference')
        rows=[];ranges=[];residuals=[];normalized=[]
        for i in range(1,180,12):
            a,b=manifest['frames'][i-1:i+1]
            def depth(f):
                im=Image.open(io.BytesIO(read(f['depth_file'])))
                require(im.size==(640,480) and im.mode in ('I;16','I;16B','I'),'original measured depth')
                return np.asarray(im,dtype=float)/5000.
            old,new=pyramid(depth(a),CAMERA)[1],pyramid(depth(b),CAMERA)[1]
            truth=np.linalg.inv(reference.truth_at(labels,a['depth_timestamp']))@reference.truth_at(labels,b['depth_timestamp'])
            for direction,target,source,transform in (('forward',old,new,truth),('reverse',new,old,np.linalg.inv(truth))):
                sample=pairs(target,source,transform,1)
                raw=equation(sample,False);compare_equation(raw,normal_equation(target,source,transform,1))
                zs,zt=sample['source'][:,2],sample['target'][:,2];z=(zs+zt)/2;residual=sample['residual']
                ray=sample['source']/zs[:,None]
                incidence=abs(np.einsum('ij,ij->i',sample['normal'],ray@transform[:3,:3].T))
                # Disclose incidence-normalized values separately, never admit
                # new runtime matches or hide the original residual channel.
                usable=incidence>.5
                normalized.append((z[usable],residual[usable]/incidence[usable]))
                rows.append(dict(source_index=b['source_index'],direction=direction,
                    plane=stratify(z,residual),axial_equivalent_incidence_over_half=stratify(z[usable],residual[usable]/incidence[usable])))
                ranges.append(z);residuals.append(residual)
        all_z,all_r=np.concatenate(ranges),np.concatenate(residuals)
        nz,nr=map(np.concatenate,zip(*normalized))
        summary=dict(pair_directions=len(rows),plane=stratify(all_z,all_r),
                     axial_equivalent_incidence_over_half=stratify(nz,nr))
        write(out/(case+'.json'),dict(manifest_sha256=sha(manifest_body),summary=summary,pairs=rows,
            raw_matcher_agrees_with_original_equations=True,numeric_truth_only_for_offline_probe=True))
        summaries[case]=summary;print(case,json.dumps(summary),flush=True)
    require(all(sha((ROOT/n).read_bytes())==h for n,h in pins.items()),'frozen diagnostic sources')
    write(out/'summary.json',dict(cases=summaries,relative_weight_math_controls=controls(),
        measurement_noise_calibration_established=False,root_pose_accuracy_claim=False))


if __name__=='__main__':main()
