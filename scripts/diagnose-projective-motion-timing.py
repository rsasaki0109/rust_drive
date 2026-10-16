#!/usr/bin/env python3
"""Estimate RGB/depth pixel displacement from prior accepted poses; no truth decoding."""
import argparse,gzip,io,json
from pathlib import Path
import numpy as np
from PIL import Image
from projective_depth_common import CAMERA,ROOT,fresh,pose,require,sha,write
from projective_motion_raw import log_body,exp_body


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();out=fresh(args.output);summaries={}
    for case in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'):
        base=ROOT/'data'/case;manifest_body=(base/'manifest.json').read_bytes();manifest=json.loads(manifest_body)
        frozen=ROOT/'assets/projective-affine-v1/recorded'/case/'sensor-only.json.gz';ledger_body=frozen.read_bytes();ledger=json.loads(gzip.decompress(ledger_body));require(not ledger['ground_truth_parsed'],'pre-label estimated poses')
        binding=json.loads((frozen.parent/'freeze.json').read_text());require(binding['manifest_sha256']==sha(manifest_body),'original manifest associated with estimated poses')
        ledger_pin=json.loads((ROOT/'assets/projective-affine-v1/files.json').read_text())[str(frozen.relative_to(ROOT/'assets/projective-affine-v1'))]
        require(len(ledger_body)==ledger_pin['bytes'] and sha(ledger_body)==ledger_pin['sha256'],'immutable prior pose ledger')
        inputs={p['file']:p for p in manifest['files']};rows=[];history=[]
        for frame,saved in zip(manifest['frames'],ledger['frames'],strict=True):
            u=saved['update'];require(saved['source_index']==frame['source_index'] and u['timestamp']==frame['depth_timestamp'],'sensor acquisition identity')
            phase=frame['rgb_timestamp']-frame['depth_timestamp'];require(abs(phase)<=.02,'original association bound')
            row=dict(source_index=frame['source_index'],phase_s=phase,fresh_rgb=u['photometric']['fresh'])
            if len(history)==2 and row['fresh_rgb']:
                name=frame['depth_file'];body=(base/'raw'/name).read_bytes();pin=inputs[name];require(len(body)==pin['bytes'] and sha(body)==pin['sha256'],'qualified measured depth')
                image=Image.open(io.BytesIO(body));require(image.size==(640,480),'original camera dimensions');depth=np.asarray(image,dtype=float)/5000.
                y,x=np.mgrid[:480:16,:640:16];z=depth[::16,::16];valid=(z>.3)&(z<5);x,y,z=x[valid],y[valid],z[valid]
                points=np.column_stack([(x-CAMERA['cx'])*z/CAMERA['fx'],(y-CAMERA['cy'])*z/CAMERA['fy'],z])
                twist=log_body(np.linalg.inv(history[0][1])@history[1][1]);require(twist is not None,'unambiguous prior body motion')
                shift=exp_body(twist*(-phase/(history[1][0]-history[0][0])));q=points@shift[:3,:3].T+shift[:3,3]
                valid=q[:,2]>.3;q=q[valid];x,y=x[valid],y[valid]
                displacement=np.hypot(CAMERA['fx']*q[:,0]/q[:,2]+CAMERA['cx']-x,CAMERA['fy']*q[:,1]/q[:,2]+CAMERA['cy']-y)
                require(len(displacement)>0 and np.isfinite(displacement).all(),'finite diagnostic probes')
                row.update(history_timestamps=[history[0][0],history[1][0]],probes=len(displacement),
                    median_px=float(np.median(displacement)),p95_px=float(np.quantile(displacement,.95)),maximum_px=float(displacement.max()))
            rows.append(row)
            if u['accepted']:history.append((u['timestamp'],pose(u['first_from_current'])));history=history[-2:]
        measured=[r for r in rows if 'p95_px' in r];summary=dict(frames=len(rows),causal_probe_frames=len(measured),maximum_abs_association_s=max(abs(r['phase_s']) for r in rows),median_frame_p95_px=float(np.median([r['p95_px'] for r in measured])),maximum_frame_p95_px=max(r['p95_px'] for r in measured),frames_p95_over_half_pixel=sum(r['p95_px']>.5 for r in measured))
        write(out/(case+'.json'),dict(summary=summary,frames=rows,manifest_sha256=sha(manifest_body),estimated_pose_ledger_sha256=sha(ledger_body),numeric_truth_decoded=False));summaries[case]=summary
    write(out/'summary.json',dict(cases=summaries,numeric_truth_decoded=False,source='prior accepted exposure-aware RGB-D estimates, viewed development recordings',scope='model-predicted displacement under constant body motion/static scene; not measured timing error, not calibrated uncertainty, no compensation applied',source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in (Path(__file__),ROOT/'scripts/projective_motion_raw.py',ROOT/'scripts/projective_depth_common.py')}))
    print(json.dumps(summaries),flush=True)


if __name__=='__main__':main()
