#!/usr/bin/env python3
"""Reproduce the unchanged first scoring rejection; never repair or drop labels."""
import argparse,json
from pathlib import Path
import camera_preserved_check as c


def audit(out,rows,control,annotation=None):
    annotation=annotation or (lambda r:Path(r['annotation']).read_bytes())
    try:
        c.audit_saved(out,rows,control,annotation)
    except ValueError as error:
        c.require(str(error)=='original reference bounds','same original first failure')
    else:
        raise ValueError('original invalid-reference rejection was not reproduced')
    # audit_saved validates every parent, prefix, fixture, decision and repeat
    # before it parses any numeric labels. Its unchanged failure is mandatory.
    invalid=[]
    for row in rows[40:]:
        body=annotation(row);c.require(c.hashlib.sha256(body).hexdigest()==row['annotation_sha256'],'original failed annotation bytes')
        labels=json.loads(body)
        for label in labels['frames'][0]['objects']:
            if 'box2d' not in label:continue
            box=[label['box2d'][k] for k in ('x1','y1','x2','y2')]
            if not c.linear.matcher.valid_box(box,1280,720):invalid.append(dict(image_id=row['id'],reference_id=label['id'],category=label['category'],bbox=box))
    expected=[dict(image_id='1de6d5ff-8aba6383',reference_id=22,category='traffic light',bbox=[580.262269,94.262013,591.279907,94.262013])]
    c.require(invalid==expected,'exact original zero-height annotation, no replacement')
    selected=rows[:40];policies={}
    for policy in ('coarse','baseline_supported','detail_supported','preserved'):
        projections={}
        for row in selected:
            b=c.read(c.parent(row['id'],'first','baseline'));d=c.read(c.parent(row['id'],'first','detail'))
            result=b['result']['coarse'] if policy=='coarse' else b['result'] if policy=='baseline_supported' else d['result'] if policy=='detail_supported' else c.load(out/'combined/first'/(row['id']+'.json'))['result']
            projections[row['id']]=dict(result=result)
        policies[policy]=c.linear.score(selected,projections,annotation)
    ids=lambda key:{(r['image_id'],m['reference_id']) for r in policies[key]['images'] for m in r['matches']}
    old,new=ids('baseline_supported'),ids('preserved')
    return dict(schema_version=1,status='fresh_sample_rejected_original_invalid_reference',original_checker_sha256=c.digest(c.HERE/'camera_preserved_check.py'),failure_auditor_sha256=c.digest(__file__),protocol_sha256=c.digest(c.HERE/'profile-preserved.json'),unchanged_first_rejection='original reference bounds',invalid_original_references=invalid,all56_execution_and_prefix_reconstruction_passed=True,all56_repeats_and_original_graph_reconstruction_passed=True,viewed_development=policies,viewed_reference_changes=dict(retained=len(old&new),gained=[list(x) for x in sorted(new-old)],lost=[list(x) for x in sorted(old-new)]),fresh_sample_quality=None,fresh_sample_scored=False,images_replaced=0,annotations_changed=False,quality_thresholds_changed=False,model_execution_retried=False,adopt_candidate=False,execution=dict(new_model_forwards=1056,new_native_image_runs=96,reused_parent_reports=240,reused_parent_forwards_not_counted_as_new=2640,native_fixture_runs=168,fixture_model_forwards=0,warmup_model_forwards=0),new_latency_or_availability_claim=False,control_permission=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--controls',type=Path,required=True);p.add_argument('--data',type=Path,default=c.ROOT/'artifacts/camera-preserved-v1/data');a=p.parse_args()
    target=a.output/'rejected-summary.json';c.require(not target.exists(),'new rejection receipt only')
    result=audit(a.output,c.records(a.data),c.load(a.controls/'summary.json'));c.write(target,result)
    print(json.dumps(dict(status=result['status'],execution_reconstruction_passed=True,fresh_sample_scored=False,adopt_candidate=False,model_forwards_this_audit=0)),flush=True)
if __name__=='__main__':main()
