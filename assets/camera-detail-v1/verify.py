#!/usr/bin/env python3
"""Reconstruct frozen detail-view results and provenance without inference."""
import collections
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'integrations/onnx'))
import camera_detail_check as c


def sha(body):return hashlib.sha256(body).hexdigest()
def raw(name):
    p=HERE/name;body=p.read_bytes()
    return gzip.decompress(body) if p.suffix=='.gz' else body
def obj(name):return json.loads(raw(name))
def annotation(row):
    for folder in (HERE,ROOT/'assets/camera-capacity-v1',ROOT/'assets/camera-linear-v1'):
        path=folder/'annotations'/(row['id']+'.json.gz')
        if path.is_file():return gzip.decompress(path.read_bytes())
    raise AssertionError('missing original annotation '+row['id'])


def diagnosis():
    saved=obj('selection-diagnosis.json.gz');body=gzip.decompress((ROOT/'assets/camera-capacity-v1/validation/summary.json.gz').read_bytes())
    assert sha(body)==saved['source_summary_sha256'];previous=json.loads(body)
    before=json.loads(gzip.decompress((ROOT/'assets/camera-capacity-v1/validation/before-inference.json.gz').read_bytes()));counts=collections.defaultdict(collections.Counter)
    mapping=c.load(c.HERE/'profile-capacity.json')['class_mapping']
    for row in before['input_rows'][8:]:
        report=c.read(c.parent(row['id'],'first'));objects={v['id']:v for v in json.loads(annotation(row))['frames'][0]['objects']}
        scored=next(v for v in previous['groups']['fresh_sample']['coarse']['images'] if v['image_id']==row['id']);local=[]
        for view in c.make_fixture(report['source_analysis'])['views']:
            rect=view['rectangle']
            for d in view['detections']:
                q=dict(d);q['bbox']=[float(c.np.float32(d['bbox'][i])+c.np.float32(rect['x' if i%2==0 else 'y'])) for i in range(4)];local.append(q)
        for miss in scored['false_negatives']:
            ref=objects[miss.get('reference_id',miss.get('id'))];box=[ref['box2d'][k] for k in ('x1','y1','x2','y2')];kind=ref['category'];side=max(box[2]-box[0],box[3]-box[1])
            key='max_side_under32px' if side<32 else 'max_side_32to64px' if side<64 else 'max_side_at_least64px'
            counts[kind][key]+=1;counts[kind]['coarse_false_negatives']+=1;counts[kind]['same_class_in_any_view_at_iou50']+=max((float(c.grid.iou32(box,d['bbox'])) for d in local if d['class_index']==mapping[kind]),default=0)>=.5
    assert saved['misses_by_category']=={k:dict(v) for k,v in counts.items()}
    assert saved['new_model_forwards']==0 and not saved['new_data_used'] and saved['diagnostic_size_buckets_only']


def main():
    index=obj('files.json');assert set(index)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p.name!='files.json'}
    for name,pin in index.items():
        body=(HERE/name).read_bytes();assert len(body)==pin['bytes'] and sha(body)==pin['sha256'],name
    for name,pin in obj('raw-files.json').items():
        body=raw(name+'.gz');assert len(body)==pin['bytes'] and sha(body)==pin['sha256'],name
    before=obj('before.json.gz');closure=obj('source-sha256.json')
    assert len(before['previous_sources'])==198 and len(before['prior_indexes'])==82 and len(before['prior_entries'])==7705 and len(before['core_outputs'])==245 and len(closure)==207
    archives=['camera-nano-v1','camera-latest-v1','camera-nano-repeat2x-v1','camera-supervised-v1','camera-depthwise-v1','camera-rowzip-v1','camera-linear-v1','camera-grid-v1','camera-supported-v1','camera-capacity-v1','camera-detail-v1']
    for name,pin in closure.items():
        assert sha((ROOT/name).read_bytes())==pin,name
        archived=next(ROOT/'assets'/f/'source'/name for f in archives if (ROOT/'assets'/f/'source'/name).is_file());assert sha(archived.read_bytes())==pin,name
    for name,pin in before['prior_indexes'].items():assert sha((ROOT/name).read_bytes())==pin,name
    for name,pin in before['prior_entries'].items():
        body=(ROOT/name).read_bytes();assert len(body)==pin['bytes'] and sha(body)==pin['sha256'],name
    for name,pin in before['core_outputs'].items():assert sha((ROOT/name).read_bytes())==pin,name
    assert obj('core-output-sha256.json')==before['core_outputs']
    protocol=c.load(c.HERE/'profile-detail.json');assert c.digest(c.HERE/'profile-detail.json')==before['protocol_sha256']
    assert sha(raw('selection-diagnosis.json.gz'))==protocol['diagnosis_sha256'];diagnosis()
    cap=c.load(c.HERE/'profile-capacity.json');support=c.load(c.HERE/'profile-supported.json')
    prior_ids=set(support['viewed_ids'])|{v['id'] for v in support['images']}|set(cap['viewed_ids'])|{v['id'] for v in cap['images']}
    assert len(prior_ids)==72 and set(protocol['all_previous_ids'])==prior_ids and prior_ids<=set(protocol['excluded_image_stems'])
    tree_body=gzip.decompress((ROOT/'assets/camera-linear-v1/tree.json.gz').read_bytes());assert sha(tree_body)==protocol['tree_sha256'];tree=json.loads(tree_body);assert tree['truncated'] is False
    paths={r['path']:r for r in tree['tree']};available=[]
    for name,row in sorted(paths.items()):
        if name.startswith('bdd/images/val/') and name.endswith('.jpg'):
            identity=Path(name).stem;label='bdd/json_labels/val/'+identity+'.json'
            if identity not in protocol['excluded_image_stems'] and label in paths:available.append(dict(id=identity,image=row,annotation=paths[label]))
    assert protocol['images']==available[:16]
    metadata=obj('validation/before-inference.json.gz');control=obj('controls/summary.json.gz');rows=metadata['input_rows']
    assert len(rows)==len({r['id'] for r in rows})==40 and [r['id'] for r in rows[:24]]==protocol['viewed_ids'] and [r['id'] for r in rows[24:]]==[v['id'] for v in protocol['images']]
    assert all(r['partition']=='viewed_development' for r in rows[:24]) and all(r['partition']=='fresh_sample' and r['original_split']=='val' for r in rows[24:])
    assert metadata['compiled_sources']==control['compiled_sources']==c.expected() and len(c.expected())==87
    assert metadata['baseline_compiled_sources']==control['baseline_compiled_sources']==c.baseline.expected()
    assert metadata['checker_sha256']==control['checker_sha256']==c.digest(c.HERE/'camera_detail_check.py') and metadata['fetcher_sha256']==c.digest(c.HERE/'camera_detail_fetch.py') and metadata['protocol_sha256']==c.digest(c.HERE/'profile-detail.json')
    assert metadata['control_summary_sha256']==sha(raw('controls/summary.json.gz'))
    assert metadata['executable_sha256']==control['executable_sha256'] and metadata['baseline_executable_sha256']==control['baseline_executable_sha256']
    assert metadata['warmup_model_forwards']==metadata['prior_new_model_forwards']==0 and not metadata['numeric_annotations_parsed_by_this_process']
    assert control['valid_fixtures']==11 and len(control['invalid_fixtures'])==9 and control['existing_output_preserved'] and control['pass_controls'] and control['model_forwards']==0
    assert set(control['complete_input_pixels'])==set(control['baseline_input_pixels'])=={r['id'] for r in rows}
    assert len(metadata['frozen_parent_sha256'])==72
    for name,pin in metadata['frozen_parent_sha256'].items():assert c.digest(ROOT/name)==pin,name
    receipts=obj('inputs.json.gz')
    for row,pin in zip(rows[24:],protocol['images']):
        body=annotation(row);label=pin['annotation'];assert len(body)==label['size'] and hashlib.sha1(b'blob '+str(len(body)).encode()+b'\0'+body).hexdigest()==label['sha']
        assert sha(body)==row['annotation_sha256']==receipts[row['id']+'.json']['sha256'] and row['image_sha256']==receipts[row['id']+'.jpg']['sha256']
    old=json.loads(gzip.decompress((ROOT/'assets/camera-capacity-v1/validation/before-inference.json.gz').read_bytes()))['input_rows']
    assert all(all(row[k]==prior[k] for k in ('id','image_sha256','annotation_sha256','original_split')) for row,prior in zip(rows[:24],old))
    original=c.load(c.HERE/'bdd_protocol.json');assert sha(raw('LICENSE-BDD-data.txt'))==original['license']['sha256']
    with tempfile.TemporaryDirectory(prefix='camera-detail-audit-') as temp:
        out=Path(temp)
        for name in index:
            if name.startswith('validation/') and name.endswith('.json.gz'):
                path=out/name.removeprefix('validation/').removesuffix('.gz');path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw(name))
        hashes=c.load(out/'before-label-scoring.json');assert not hashes['numeric_annotations_parsed_by_this_process'] and len(hashes['output_sha256'])==168
        for name,pin in hashes['output_sha256'].items():assert c.digest(out/name)==pin,name
        summary=c.audit_saved(out,rows,control,annotation);assert summary==obj('validation/summary.json.gz')
        for i in range(11):
            path=out/f'control-{i}.json';path.write_bytes(raw(f'controls/inputs/{i}.json.gz'));c.check_fixture(obj(f'controls/{i}-actual.json.gz'),path)
    ex=summary['execution'];assert ex['new_model_forwards']==1920 and ex['new_detail_model_forwards']==1440 and ex['new_baseline_model_forwards']==480 and ex['reused_baseline_reports']==72 and ex['reused_baseline_forwards_not_counted_as_new']==720 and ex['warmup_model_forwards']==0
    assert not summary['new_latency_or_availability_claim'] and not summary['control_permission']
    core=raw('checks/core.log.gz').decode();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed',core)))==371 and core.count('"verified": true')==49 and 'test result: FAILED' not in core
    assert 'test result: ok. 40 passed' in raw('checks/native-tests.log.gz').decode() and 'Ran 16 tests' in raw('checks/python-tests.log.gz').decode()
    assert not raw('checks/fmt.log.gz') and not raw('checks/actionlint.log.gz')
    assert 'with_detail' in raw('checks/initial-native-compile.log.gz').decode() and obj('pre-inference-correction.json.gz')['model_forwards']==0
    print(json.dumps(dict(pass_packet=True,indexed_files=len(index),previous_evidence_entries=7705,new_model_forwards=1920,reused_forwards_not_counted_as_new=720,fresh_images=16,adopt_candidate=summary['adopt_candidate'],model_execution_this_check=False)))


if __name__=='__main__':main()
