#!/usr/bin/env python3
"""Verify the preserved-measurement packet without images or inference."""
import collections,gzip,hashlib,json,re,sys,tempfile
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'integrations/onnx'))
import camera_preserved_check as c
import camera_preserved_failure as failure
def sha(b):return hashlib.sha256(b).hexdigest()
def raw(n):
    p=HERE/n;b=p.read_bytes();return gzip.decompress(b) if p.suffix=='.gz' else b
def obj(n):return json.loads(raw(n))
def annotation(row):
    for folder in ('camera-preserved-v1','camera-detail-v1','camera-capacity-v1','camera-linear-v1'):
        p=ROOT/'assets'/folder/'annotations'/(row['id']+'.json.gz')
        if p.is_file():return gzip.decompress(p.read_bytes())
    raise AssertionError('missing annotation '+row['id'])
def main():
    index=obj('files.json');assert set(index)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p.name!='files.json'}
    for n,p in index.items():
        b=(HERE/n).read_bytes();assert len(b)==p['bytes'] and sha(b)==p['sha256'],n
    for n,p in obj('raw-files.json').items():
        b=raw(n+'.gz');assert len(b)==p['bytes'] and sha(b)==p['sha256'],n
    before=obj('before.json.gz');closure=obj('source-sha256.json')
    assert len(before['previous_sources'])==207 and len(before['prior_indexes'])==83 and len(before['prior_entries'])==7958 and len(before['core_outputs'])==245 and len(closure)==216
    archives=['camera-nano-v1','camera-latest-v1','camera-nano-repeat2x-v1','camera-supervised-v1','camera-depthwise-v1','camera-rowzip-v1','camera-linear-v1','camera-grid-v1','camera-supported-v1','camera-capacity-v1','camera-detail-v1','camera-preserved-v1']
    for n,h in closure.items():
        assert c.digest(ROOT/n)==h,n
        p=next(ROOT/'assets'/a/'source'/n for a in archives if (ROOT/'assets'/a/'source'/n).is_file());assert c.digest(p)==h,n
    for n,h in before['prior_indexes'].items():assert c.digest(ROOT/n)==h,n
    for n,p in before['prior_entries'].items():
        b=(ROOT/n).read_bytes();assert len(b)==p['bytes'] and sha(b)==p['sha256'],n
    for n,h in before['core_outputs'].items():assert c.digest(ROOT/n)==h,n
    assert obj('core-output-sha256.json')==before['core_outputs']
    protocol=c.load(c.HERE/'profile-preserved.json');assert before['protocol_sha256']==c.digest(c.HERE/'profile-preserved.json')
    previous=c.load(c.HERE/'profile-detail.json');oldids=set(previous['all_previous_ids'])|{r['id'] for r in previous['images']}
    assert len(oldids)==88 and oldids==set(protocol['all_previous_ids']) and oldids<=set(protocol['excluded_image_stems'])
    treebody=gzip.decompress((ROOT/'assets/camera-linear-v1/tree.json.gz').read_bytes());assert sha(treebody)==protocol['tree_sha256'];tree=json.loads(treebody);assert tree['truncated'] is False
    paths={v['path']:v for v in tree['tree']};available=[]
    for n,r in sorted(paths.items()):
        if n.startswith('bdd/images/val/') and n.endswith('.jpg'):
            identity=Path(n).stem;label='bdd/json_labels/val/'+identity+'.json'
            if identity not in protocol['excluded_image_stems'] and label in paths:available.append(dict(id=identity,image=r,annotation=paths[label]))
    assert protocol['images']==available[:16]
    metadata=obj('validation/before-inference.json.gz');control=obj('controls/summary.json.gz');rows=metadata['input_rows']
    assert len(rows)==len({r['id'] for r in rows})==56 and [r['id'] for r in rows[:40]]==protocol['viewed_ids'] and [r['id'] for r in rows[40:]]==[r['id'] for r in protocol['images']]
    assert all(r['partition']=='viewed_development' for r in rows[:40]) and all(r['partition']=='fresh_sample' and r['original_split']=='val' for r in rows[40:])
    assert metadata['compiled_sources']==control['compiled_sources']==c.expected() and len(c.expected())==7
    assert metadata['baseline_compiled_sources']==control['baseline_compiled_sources']==c.baseline.expected() and metadata['detail_compiled_sources']==control['detail_compiled_sources']==c.detail.expected()
    assert metadata['checker_sha256']==control['checker_sha256']==c.digest(c.HERE/'camera_preserved_check.py') and metadata['fetcher_sha256']==c.digest(c.HERE/'camera_preserved_fetch.py') and metadata['protocol_sha256']==c.digest(c.HERE/'profile-preserved.json')
    assert metadata['control_summary_sha256']==sha(raw('controls/summary.json.gz'))
    for k in ('executable_sha256','baseline_executable_sha256','detail_executable_sha256'):assert metadata[k]==control[k]
    assert metadata['warmup_model_forwards']==metadata['prior_new_model_forwards']==0 and not metadata['numeric_annotations_parsed_by_this_process']
    assert control['valid_fixtures']==6 and len(control['invalid_fixtures'])==11 and control['pass_controls'] and control['existing_output_preserved'] and control['model_forwards']==0
    assert set(control['complete_input_pixels'])==set(control['baseline_input_pixels'])=={r['id'] for r in rows}
    pins={str(c.parent(r['id'],m,k).relative_to(ROOT)):c.digest(c.parent(r['id'],m,k)) for r in rows[:40] for m in c.MODES for k in ('baseline','detail')}
    assert len(pins)==240 and metadata['frozen_parent_sha256']==pins
    counts=collections.Counter();key=lambda d:json.dumps(c.prior.nano.result_bits(d),sort_keys=True)
    for row in rows[:40]:
        b=c.read(c.parent(row['id'],'first','baseline'));d=c.read(c.parent(row['id'],'first','detail'));small={key(x) for x in d['result']['detections']}
        for x in b['result']['detections']:
            if key(x) not in small:counts[str(x['class_index'])]+=1
    diagnosed=obj('diagnosis.json.gz');assert diagnosed['by_class']==dict(counts) and diagnosed['baseline_measurements_not_retained_by_detail']==sum(counts.values())==46 and diagnosed['new_model_forwards']==0 and not diagnosed['new_numeric_labels_parsed']
    receipts=obj('inputs.json.gz')
    for row,pin in zip(rows[40:],protocol['images']):
        b=annotation(row);p=pin['annotation'];assert len(b)==p['size'] and hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()==p['sha']
        assert sha(b)==row['annotation_sha256']==receipts[row['id']+'.json']['sha256'] and row['image_sha256']==receipts[row['id']+'.jpg']['sha256']
    old=json.loads(gzip.decompress((ROOT/'assets/camera-detail-v1/validation/before-inference.json.gz').read_bytes()))['input_rows']
    assert all(all(r[k]==o[k] for k in ('id','image_sha256','annotation_sha256','original_split')) for r,o in zip(rows[:40],old))
    original=c.load(c.HERE/'bdd_protocol.json');assert sha(raw('LICENSE-BDD-data.txt'))==original['license']['sha256']
    with tempfile.TemporaryDirectory(prefix='camera-preserved-audit-') as temp:
        out=Path(temp)
        for n in index:
            if n.startswith('validation/') and n.endswith('.json.gz'):
                p=out/n.removeprefix('validation/').removesuffix('.gz');p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(raw(n))
        hashes=c.load(out/'before-label-scoring.json');assert not hashes['numeric_annotations_parsed_by_this_process'] and len(hashes['output_sha256'])==432
        for n,h in hashes['output_sha256'].items():assert c.digest(out/n)==h,n
        summary=failure.audit(out,rows,control,annotation);assert summary==obj('validation/rejected-summary.json.gz')
        assert 'validation/summary.json.gz' not in index and summary['fresh_sample_quality'] is None and not summary['fresh_sample_scored'] and not summary['adopt_candidate']
        assert not summary['annotations_changed'] and not summary['quality_thresholds_changed'] and summary['images_replaced']==0 and not summary['model_execution_retried']
        for i in range(6):
            p=out/f'control-{i}.json';p.write_bytes(raw(f'controls/inputs/{i}.json.gz'));c.check(obj(f'controls/{i}-actual.json.gz'),p)
    ex=summary['execution'];assert ex['new_model_forwards']==1056 and ex['new_native_image_runs']==96 and ex['reused_parent_reports']==240 and ex['reused_parent_forwards_not_counted_as_new']==2640 and ex['native_fixture_runs']==168 and ex['fixture_model_forwards']==ex['warmup_model_forwards']==0
    assert not summary['new_latency_or_availability_claim'] and not summary['control_permission']
    core=raw('checks/core.log.gz').decode();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed',core)))==371 and core.count('"verified": true')==49 and 'test result: FAILED' not in core
    assert 'test result: ok. 6 passed' in raw('checks/native-tests.log.gz').decode() and 'Ran 12 tests' in raw('checks/python-tests.log.gz').decode()
    assert not raw('checks/fmt.log.gz') and not raw('checks/actionlint.log.gz')
    assert 'ValueError: original reference bounds' in raw('checks/validation.log.gz').decode()
    print(json.dumps(dict(pass_packet=True,indexed_files=len(index),previous_evidence_entries=7958,new_model_forwards=1056,reused_forwards_not_counted_as_new=2640,native_fixture_runs=168,fresh_sample_scored=False,adopt_candidate=False,model_execution_this_check=False)))
if __name__=='__main__':main()
