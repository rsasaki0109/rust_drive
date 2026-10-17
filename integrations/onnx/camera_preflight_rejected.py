#!/usr/bin/env python3
"""Replay the fixed preflight rejection, and exercise valid old40 tickets."""
import argparse
from pathlib import Path
import camera_preflight_check as c


def rejected(rows,design,receipt):
    c.require(receipt['validator_sha256']==c.digest(c.HERE/'camera_preflight.py'),'same final validator')
    c.require(not receipt['accepted'] and receipt['row_count']==56,'complete group rejected')
    bad=[r for r in receipt['inputs'] if r['issues']]
    c.require(len(bad)==1 and bad[0]['id']=='23112889-a8ce8bc7' and bad[0]['issues'][0]['code']=='box_bounds','same original zero-width reference')
    calls=[]
    try:c.preflight.guarded_call(None,rows[0],lambda:calls.append('model'))
    except c.preflight.Rejected as error:c.require(error.issue['code']=='preflight_rejected','no ticket model guard')
    else:raise ValueError('rejected group reached callback')
    c.require(not calls,'zero callbacks')
    return dict(schema_version=1,status='rejected_before_native_or_model_execution',validator_sha256=c.digest(c.HERE/'camera_preflight.py'),checker_sha256=c.digest(c.HERE/'camera_preflight_check.py'),auditor_sha256=c.digest(__file__),protocol_sha256=c.digest(c.HERE/'profile-preflight.json'),row_count=56,rejected_inputs=bad,model_callbacks=0,model_forwards=0,native_commands=0,fresh_sample_quality=None,fresh_sample_scored=False,images_replaced=0,labels_changed=False,quality_thresholds_changed=False,adopt_candidate=False)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--controls',type=Path,required=True);p.add_argument('--data',type=Path,default=c.ROOT/'artifacts/camera-preflight-v1/data');a=p.parse_args()
    out=c.fresh(a.output);rows=c.records(a.data);design=c.load(c.HERE/'profile-preflight.json');receipt,ticket=c.preflight.inspect(rows,design)
    c.require(ticket is None and receipt==c.load(a.controls/'preflight.json'),'same first rejection, source bytes unchanged')
    c.require(not (a.controls/'inputs').exists() and not (a.controls/'summary.json').exists(),'first controls stopped before any native input')
    c.write(out/'preflight.json',receipt);c.write(out/'rows.json',[{k:str(v) if isinstance(v,Path) else v for k,v in r.items()} for r in rows]);c.write(out/'rejected-summary.json',rejected(rows,design,receipt))
    valid,ticket=c.preflight.inspect(rows[:40],design);c.require(ticket is not None,'all previous valid40 accepted')
    probes=[]
    for row in rows[:40]:c.preflight.guarded_call(ticket,row,lambda:probes.append(row['id']))
    c.require(probes==[r['id'] for r in rows[:40]],'valid tickets permit only unchanged rows')
    c.write(out/'valid40-preflight.json',valid);c.write(out/'valid40-summary.json',dict(pass_valid_controls=True,row_count=40,allowed_probe_callbacks=len(probes),native_commands=0,model_forwards=0))
    print(c.json.dumps(dict(first_rejection_reproduced=True,rejected_images=1,valid40_accepted=True,allowed_probe_callbacks=40,native_commands=0,model_forwards=0,fresh_sample_scored=False,adopt_candidate=False)),flush=True)
if __name__=='__main__':main()
