#!/usr/bin/env python3
"""Replay preflight label validation; bind saved image receipts, never infer."""
import gzip,hashlib,json,re,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'integrations/onnx'))
import camera_preflight_check as c
import camera_preflight_rejected as rejected
def sha(b):return hashlib.sha256(b).hexdigest()
def raw(n):
    p=HERE/n;b=p.read_bytes();return gzip.decompress(b) if p.suffix=='.gz' else b
def obj(n):return json.loads(raw(n))
def annotation(row):
    for a in ('camera-preflight-v1','camera-preserved-v1','camera-detail-v1','camera-capacity-v1','camera-linear-v1'):
        p=ROOT/'assets'/a/'annotations'/(row['id']+'.json.gz')
        if p.is_file():return gzip.decompress(p.read_bytes())
    raise AssertionError('missing annotation '+row['id'])

def audit_receipt(receipt,rows,design):
    assert receipt['schema_version']==1 and receipt['algorithm']=='bounded_original_bdd_preflight_v1' and receipt['validator_sha256']==c.digest(c.HERE/'camera_preflight.py')
    assert receipt['row_count']==len(rows) and len(receipt['inputs'])==len(rows) and receipt['numeric_labels_validated'] and not receipt['labels_used_for_inference'] and not receipt['images_or_labels_changed'] and receipt['model_forwards']==0 and not receipt['control_permission']
    for row,actual in zip(rows,receipt['inputs']):
        assert all(actual[k]==row[k] for k in ('id','partition','image_sha256','annotation_sha256'))
        body=annotation(row);assert sha(body)==row['annotation_sha256']
        try:expected=c.preflight.annotation(body,row['id'],design);issues=[]
        except c.preflight.Rejected as e:expected=None;issues=[e.issue]
        assert actual['annotation']==expected and actual['issues']==issues
        image=actual['image'];assert image['sha256']==row['image_sha256'] and 0<image['bytes']<=c.preflight.LIMIT and image['format']=='JPEG' and image['width']==1280 and image['height']==720 and image['mode']=='RGB' and image['fully_decoded']
    assert receipt['accepted']==all(not r['issues'] for r in receipt['inputs'])

def main():
    index=obj('files.json');assert set(index)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p.name!='files.json'}
    for n,p in index.items():
        b=(HERE/n).read_bytes();assert len(b)==p['bytes'] and sha(b)==p['sha256'],n
    for n,p in obj('raw-files.json').items():
        b=raw(n+'.gz');assert len(b)==p['bytes'] and sha(b)==p['sha256'],n
    before=obj('before.json.gz');closure=obj('source-sha256.json')
    assert len(before['previous_sources'])==216 and len(before['prior_indexes'])==84 and len(before['prior_entries'])==8465 and len(before['core_outputs'])==245 and len(closure)==224
    archives=['camera-nano-v1','camera-latest-v1','camera-nano-repeat2x-v1','camera-supervised-v1','camera-depthwise-v1','camera-rowzip-v1','camera-linear-v1','camera-grid-v1','camera-supported-v1','camera-capacity-v1','camera-detail-v1','camera-preserved-v1','camera-preflight-v1']
    for n,h in closure.items():
        assert c.digest(ROOT/n)==h,n
        p=next(ROOT/'assets'/a/'source'/n for a in archives if (ROOT/'assets'/a/'source'/n).is_file());assert c.digest(p)==h,n
    for n,h in before['prior_indexes'].items():assert c.digest(ROOT/n)==h,n
    for n,p in before['prior_entries'].items():
        b=(ROOT/n).read_bytes();assert len(b)==p['bytes'] and sha(b)==p['sha256'],n
    for n,h in before['core_outputs'].items():assert c.digest(ROOT/n)==h,n
    assert obj('core-output-sha256.json')==before['core_outputs']
    design=c.load(c.HERE/'profile-preflight.json');assert c.digest(c.HERE/'profile-preflight.json')==before['protocol_sha256']
    previous=c.load(c.HERE/'profile-preserved.json');oldids=set(previous['all_previous_ids'])|{r['id'] for r in previous['images']}
    assert len(oldids)==104 and oldids==set(design['all_previous_ids']) and oldids<=set(design['excluded_image_stems'])
    treebody=gzip.decompress((ROOT/'assets/camera-linear-v1/tree.json.gz').read_bytes());assert sha(treebody)==design['tree_sha256'];tree=json.loads(treebody);assert tree['truncated'] is False
    paths={v['path']:v for v in tree['tree']};available=[]
    for n,r in sorted(paths.items()):
        if n.startswith('bdd/images/val/') and n.endswith('.jpg'):
            identity=Path(n).stem;label='bdd/json_labels/val/'+identity+'.json'
            if identity not in design['excluded_image_stems'] and label in paths:available.append(dict(id=identity,image=r,annotation=paths[label]))
    assert design['images']==available[:16]
    rows=obj('rejection/rows.json.gz');receipt=obj('controls/preflight.json.gz')
    assert len(rows)==len({r['id'] for r in rows})==56 and [r['id'] for r in rows[:40]]==design['viewed_ids'] and [r['id'] for r in rows[40:]]==[r['id'] for r in design['images']]
    audit_receipt(receipt,rows,design);assert receipt==obj('rejection/preflight.json.gz')
    assert rejected.rejected(rows,design,receipt)==obj('rejection/rejected-summary.json.gz')
    assert 'controls/summary.json.gz' not in index and not any(n.startswith('controls/inputs/') for n in index)
    audit_receipt(obj('rejection/valid40-preflight.json.gz'),rows[:40],design);assert obj('rejection/valid40-summary.json.gz')==dict(pass_valid_controls=True,row_count=40,allowed_probe_callbacks=40,native_commands=0,model_forwards=0)
    oldrows=json.loads(gzip.decompress((ROOT/'assets/camera-preserved-v1/validation/before-inference.json.gz').read_bytes()))['input_rows']
    assert all(all(r[k]==o[k] for k in ('id','image_sha256','annotation_sha256','original_split')) for r,o in zip(rows[:40],oldrows[:40]))
    audit_receipt(obj('negative/preflight.json.gz'),oldrows[40:],design);negative=obj('negative/summary.json.gz')
    assert negative['pass_negative_control'] and negative['model_callbacks']==negative['model_forwards']==0 and not negative['labels_changed'] and negative['images_replaced']==0 and negative['preflight_sha256']==sha(raw('negative/preflight.json.gz'))
    assert negative['validator_sha256']==c.digest(c.HERE/'camera_preflight.py') and negative['checker_sha256']==c.digest(c.HERE/'camera_preflight_check.py')
    receipts=obj('inputs.json.gz')
    for row,pin,entry in zip(rows[40:],design['images'],receipt['inputs'][40:]):
        body=annotation(row);p=pin['annotation'];assert len(body)==p['size'] and hashlib.sha1(b'blob '+str(len(body)).encode()+b'\0'+body).hexdigest()==p['sha']
        assert sha(body)==row['annotation_sha256']==receipts[row['id']+'.json']['sha256'] and row['image_sha256']==receipts[row['id']+'.jpg']['sha256'] and entry['image']['bytes']==pin['image']['size']
    correction=obj('pre-inference-correction.json.gz');initial=obj('initial-negative-preflight.json.gz')
    assert sha(raw('initial-validator.py.gz'))==correction['initial_validator_sha256']==initial['validator_sha256'] and correction['model_forwards']==0
    assert any(any(e['code']=='category_scope' for e in r['issues']) for r in initial['inputs'])
    original=c.load(c.HERE/'bdd_protocol.json');assert sha(raw('LICENSE-BDD-data.txt'))==original['license']['sha256']
    core=raw('checks/core.log.gz').decode();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed',core)))==371 and core.count('"verified": true')==49 and 'test result: FAILED' not in core
    assert 'Ran 18 tests' in raw('checks/python-tests.log.gz').decode() and 'OK' in raw('checks/python-tests.log.gz').decode()
    assert not raw('checks/fmt.log.gz') and not raw('checks/actionlint.log.gz')
    assert 'complete input preflight rejected; no native/model command started' in raw('checks/controls-first.log.gz').decode()
    assert obj('rejection/rejected-summary.json.gz')['fresh_sample_quality'] is None and not obj('rejection/rejected-summary.json.gz')['adopt_candidate']
    print(json.dumps(dict(pass_packet=True,indexed_files=len(index),previous_evidence_entries=8465,new_model_forwards=0,native_commands=0,known_bad_group_rejected=True,new_group_rejected=True,valid40_accepted=True,fresh_sample_scored=False,adopt_candidate=False,image_decode_this_audit=False)))
if __name__=='__main__':main()
