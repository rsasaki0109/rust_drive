#!/usr/bin/env python3
"""Offline conditional uncertainty calibration check; no runtime pose inputs."""
import argparse,importlib.util,io,json
from pathlib import Path
import numpy as np
from projective_depth_common import ROOT,pose,require,sha,write
from projective_information_raw import combined,audit
from projective_motion_raw import log_body

BASE=ROOT/'artifacts/projective-information/calibration'
CHI2_6_95=12.591587243743977
SIGMA_M=.005


def reference():
    p=ROOT/'scripts/check-opencv-rgbd.py';s=importlib.util.spec_from_file_location('reference',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);m.controls();return m


def samples(rows,frames,labels):
    ref=reference();origin=np.linalg.inv(ref.truth_at(labels,frames[0]['depth_timestamp']));key=np.eye(4);out=[]
    for i,(row,frame) in enumerate(zip(rows,frames,strict=True)):
        u=row['update'];audit(u)
        if not u['accepted'] or not i:continue
        g=pose(u['first_from_current']);truth=origin@ref.truth_at(labels,frame['depth_timestamp'])
        error=np.linalg.inv(key)@g@np.linalg.inv(truth)@key;twist=log_body(error)
        require(twist is not None,'conditional error logarithm unambiguous')
        h=combined(u['final_equation'],u.get('final_photometric'));cov=SIGMA_M**2*np.linalg.inv(h)
        nees=float(twist@h@twist/SIGMA_M**2);require(np.isfinite(nees) and nees>=0,'finite conditional NEES')
        t=pose(u['iterations'][-1]['after'])[:3,3];x,y,z=t;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        j=key[:3,:3]@np.column_stack([np.eye(3),-cross]);position_cov=j@cov@j.T;rotation_cov=cov[3:,3:]
        out.append(dict(source_index=frame['source_index'],nominal_nees=nees,
            nominal_position_radius_m=float(np.sqrt(CHI2_6_95*np.linalg.eigvalsh(position_cov)[-1])),
            nominal_rotation_radius_rad=float(np.sqrt(CHI2_6_95*np.linalg.eigvalsh(rotation_cov)[-1]))))
        if u['keyframe_replaced']:key=g
    return out


def calibrate():
    require(not BASE.exists(),'new empirical calibration evidence');BASE.mkdir();factors={};pins={};reports={}
    for mode,folder in [('depth','default'),('timing','timing')]:
        values=[]
        for case in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'):
            root=ROOT/'data'/case;body=(root/'manifest.json').read_bytes();m=json.loads(body);pin=next(x for x in m['files'] if x['file']=='groundtruth.txt');truth=(root/'raw/groundtruth.txt').read_bytes()
            require(sha(truth)==pin['sha256'] and len(truth)==pin['bytes'],'original viewed reference bytes')
            path=ROOT/'artifacts/projective-information'/folder/case/'sensor-only.json';ledger=path.read_bytes();rows=json.loads(ledger)['frames']
            data=samples(rows,m['frames'],np.loadtxt(io.BytesIO(truth),comments='#'));values.extend(x['nominal_nees'] for x in data)
            pins[mode+'/'+case]=dict(manifest_sha256=sha(body),ledger_sha256=sha(ledger),reference_sha256=sha(truth))
            reports[mode+'/'+case]=data
        factors[mode]=max(1.,float(np.quantile(values,.95,method='higher')/CHI2_6_95))
    write(BASE/'freeze.json',dict(estimator_operational_changes=False,nominal_sigma_m=SIGMA_M,chi2_6_95=CHI2_6_95,
        empirical_inflation=factors,fit='higher empirical 95th percentile on both previously viewed records, separately by mode',
        caveat='strongly correlated frames, two viewed recordings, no exchangeability or calibrated probability claim',
        heldout_design_sha256=sha((ROOT/'artifacts/projective-information/heldout-rpy/design.json').read_bytes()),
        minimum_heldout_coverage=.9,maximum_projected_position_radius_m=.1,maximum_projected_rotation_radius_rad=.1,
        source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in (Path(__file__),ROOT/'scripts/projective_information_raw.py',ROOT/'scripts/projective_motion_raw.py',ROOT/'scripts/projective_depth_common.py',ROOT/'scripts/check-opencv-rgbd.py')},inputs=pins))
    write(BASE/'viewed-samples.json',reports)
    print(json.dumps(factors),flush=True)


def evaluate():
    f=json.loads((BASE/'freeze.json').read_text());require(f['nominal_sigma_m']==SIGMA_M and f['chi2_6_95']==CHI2_6_95,'fixed nominal conditional model')
    for n,h in f['source_sha256'].items():require(sha((ROOT/n).read_bytes())==h,'unchanged calibration checker '+n)
    root=ROOT/'data/tum-fr1-rpy-heldout';m=json.loads((root/'manifest.json').read_text());pin=next(x for x in m['files'] if x['file']=='groundtruth.txt');body=(root/'raw/groundtruth.txt').read_bytes();require(sha(body)==pin['sha256'],'held-out original reference')
    trial=ROOT/'artifacts/projective-information/heldout-rpy';results={}
    for mode in ('depth','timing'):
        require(json.loads((trial/mode/'raw-audit.json').read_text())['pass_raw_mathematics'],'both raw audits completed before conditional scoring')
    labels=np.loadtxt(io.BytesIO(body),comments='#')
    for mode in ('depth','timing'):
        rows=json.loads((trial/mode/'sensor-only.json').read_text())['frames'];data=samples(rows,m['frames'],labels);require(bool(data),'conditional samples available')
        factor=f['empirical_inflation'][mode];coverage=sum(x['nominal_nees']/factor<=CHI2_6_95 for x in data)/len(data)
        nominal=sum(x['nominal_nees']<=CHI2_6_95 for x in data)/len(data)
        max_position=max(x['nominal_position_radius_m']*np.sqrt(factor) for x in data);max_rotation=max(x['nominal_rotation_radius_rad']*np.sqrt(factor) for x in data)
        results[mode]=dict(accepted_samples=len(data),nominal_coverage=nominal,empirically_inflated_coverage=coverage,
            max_projected_position_radius_m=max_position,max_projected_rotation_radius_rad=max_rotation,
            empirical_gate_pass=coverage>=.9 and max_position<=.1 and max_rotation<=.1,
            calibrated_probability_claim=False,full_information_covariance_claim=False,samples=data)
    write(BASE/'heldout-results.json',dict(preregistered_freeze_sha256=sha((BASE/'freeze.json').read_bytes()),results=results,
        reference_never_operational=True,model_or_inflation_retuned_on_heldout=False,probability_calibration_established=False))
    print(json.dumps({k:{n:x for n,x in v.items() if n!='samples'} for k,v in results.items()}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('calibrate','evaluate'));a=p.parse_args();globals()[a.action]()
