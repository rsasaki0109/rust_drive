#!/usr/bin/env python3
"""Verify frozen sources, original labels and every saved result without NN."""
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
import camera_capacity_check as c


def sha(body):return hashlib.sha256(body).hexdigest()
def raw(name):
    path=HERE/name
    return gzip.decompress(path.read_bytes()) if path.suffix=='.gz' else path.read_bytes()
def obj(name):return json.loads(raw(name))
def check_files():
    index=obj('files.json')
    assert set(index)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p.name!='files.json'}
    for name,pin in index.items():
        body=raw(name) if not name.endswith('.gz') else (HERE/name).read_bytes()
        assert len(body)==pin['bytes'] and sha(body)==pin['sha256'],name
    for name,pin in obj('raw-files.json').items():
        body=raw(name+'.gz');assert len(body)==pin['bytes'] and sha(body)==pin['sha256'],name
    return index


def main():
    index=check_files();before=obj('before.json.gz');closure=obj('source-sha256.json')
    assert len(before['previous_sources'])==187 and len(before['prior_indexes'])==81 and len(before['prior_entries'])==7497 and len(before['core_outputs'])==245
    assert len(closure)==198
    archives=['camera-nano-v1','camera-latest-v1','camera-nano-repeat2x-v1','camera-supervised-v1','camera-depthwise-v1','camera-rowzip-v1','camera-linear-v1','camera-grid-v1','camera-supported-v1','camera-capacity-v1']
    for name,pin in closure.items():
        assert sha((ROOT/name).read_bytes())==pin,name
        candidates=[ROOT/'assets'/p/'source'/name for p in archives]
        archived=next(p for p in candidates if p.is_file());assert sha(archived.read_bytes())==pin,name
    for name,pin in before['prior_indexes'].items():assert sha((ROOT/name).read_bytes())==pin,name
    for name,pin in before['prior_entries'].items():
        body=(ROOT/name).read_bytes();assert len(body)==pin['bytes'] and sha(body)==pin['sha256'],name
    for name,pin in before['core_outputs'].items():assert sha((ROOT/name).read_bytes())==pin,name
    assert obj('core-output-sha256.json')==before['core_outputs']
    protocol=c.load(c.HERE/'profile-capacity.json');initial=obj('selection-protocol.json.gz');completion=obj('protocol-completion.json.gz')
    assert sha(raw('selection-protocol.json.gz'))==before['protocol_sha256']==completion['selection_protocol_sha256']
    assert c.digest(c.HERE/'profile-capacity.json')==completion['complete_protocol_sha256']
    assert completion['selection_unchanged'] and not completion['numeric_annotations_parsed'] and not completion['image_decode_or_model_execution']
    assert all(initial[k]==protocol[k] for k in initial if k!='evaluation')
    assert protocol['evaluation'].startswith(initial['evaluation'])
    previous=c.load(ROOT/'integrations/onnx/profile-supported.json')
    previous_ids=set(previous['viewed_ids'])|{r['id'] for r in previous['images']}
    assert len(previous_ids)==56 and previous_ids<=set(protocol['excluded_image_stems'])
    tree_body=gzip.decompress((ROOT/'assets/camera-linear-v1/tree.json.gz').read_bytes());assert sha(tree_body)==protocol['tree_sha256']
    tree=json.loads(tree_body);assert tree['truncated'] is False
    paths={r['path']:r for r in tree['tree']};available=[]
    for name,row in sorted(paths.items()):
        if name.startswith('bdd/images/val/') and name.endswith('.jpg'):
            identity=Path(name).stem;label='bdd/json_labels/val/'+identity+'.json'
            if identity not in protocol['excluded_image_stems'] and label in paths:available.append(dict(id=identity,image=row,annotation=paths[label]))
    assert protocol['images']==available[:16]
    metadata=obj('validation/before-inference.json.gz');control=obj('controls/summary.json.gz');rows=metadata['input_rows']
    assert len(rows)==len({r['id'] for r in rows})==24
    assert [r['id'] for r in rows[:8]]==protocol['viewed_ids']
    assert [r['id'] for r in rows[8:]]==[r['id'] for r in protocol['images']]
    assert all(r['partition']=='viewed_development' for r in rows[:8]) and all(r['partition']=='fresh_sample' and r['original_split']=='val' for r in rows[8:])
    assert metadata['compiled_sources']==control['compiled_sources']==c.expected() and len(c.expected())==83
    assert metadata['nano_compiled_sources']==control['nano_compiled_sources']==c.prior.expected()
    assert metadata['checker_sha256']==control['checker_sha256']==c.digest(c.HERE/'camera_capacity_check.py')
    assert metadata['fetcher_sha256']==c.digest(c.HERE/'camera_capacity_fetch.py') and metadata['protocol_sha256']==c.digest(c.HERE/'profile-capacity.json')
    assert metadata['control_summary_sha256']==sha(raw('controls/summary.json.gz'))
    assert metadata['executable_sha256']==control['executable_sha256'] and metadata['nano_executable_sha256']==control['nano_executable_sha256']
    assert metadata['prior_new_model_forwards']==metadata['warmup_model_forwards']==0 and not metadata['numeric_annotations_parsed_by_this_process']
    assert control['valid_fixtures']==11 and len(control['invalid_fixtures'])==9 and control['model_forwards']==0 and control['pass_controls'] and control['existing_output_preserved']
    assert set(control['complete_input_pixels'])=={r['id'] for r in rows}
    def annotation(row):
        path=HERE/'annotations'/(row['id']+'.json.gz')
        if not path.exists():path=ROOT/'assets/camera-linear-v1/annotations'/(row['id']+'.json.gz')
        return gzip.decompress(path.read_bytes())
    receipts=obj('inputs.json.gz')
    for row,pin in zip(rows[8:],protocol['images']):
        body=annotation(row);label=pin['annotation']
        assert len(body)==label['size'] and hashlib.sha1(b'blob '+str(len(body)).encode()+b'\0'+body).hexdigest()==label['sha']
        assert sha(body)==row['annotation_sha256']==receipts[row['id']+'.json']['sha256']
        assert row['image_sha256']==receipts[row['id']+'.jpg']['sha256']
    original=c.load(c.HERE/'bdd_protocol.json')
    for row,pin in zip(rows[:8],original['images']):
        assert row['image_sha256']==pin['image']['sha256'] and row['annotation_sha256']==pin['annotation']['sha256']
    assert sha(raw('LICENSE-BDD-data.txt'))==original['license']['sha256']
    with tempfile.TemporaryDirectory(prefix='camera-capacity-audit-') as temp:
        out=Path(temp)
        for name in index:
            if name.startswith('validation/') and name.endswith('.json.gz'):
                path=out/name.removeprefix('validation/').removesuffix('.gz');path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw(name))
        hashes=c.load(out/'before-label-scoring.json')
        assert not hashes['numeric_annotations_parsed_by_this_process'] and len(hashes['output_sha256'])==120
        for name,pin in hashes['output_sha256'].items():assert c.digest(out/name)==pin,name
        summary=c.audit_saved(out,rows,control,annotation);assert summary==obj('validation/summary.json.gz')
        for i in range(11):
            path=out/f'control-{i}.json';path.write_bytes(raw(f'controls/inputs/{i}.json.gz'))
            c.check_fixture(obj(f'controls/{i}-actual.json.gz'),path)
    execution=summary['execution'];assert execution['new_model_forwards']==912 and execution['new_s_model_forwards']==720 and execution['new_nano_model_forwards']==192 and execution['warmup_model_forwards']==0 and execution['reused_model_forwards_counted_as_new']==0
    assert summary['adopt_candidate'] is False
    assert all(not v['precision_no_coarse_regression'] for v in summary['relative_gates'].values())
    assert all(not group['supported']['quality_gates']['passed'] for group in summary['groups'].values())
    assert not summary['new_latency_or_availability_claim'] and not summary['control_permission']
    core=raw('checks/core.log.gz').decode();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed',core)))==371 and core.count('"verified": true')==49 and 'test result: FAILED' not in core
    assert 'test result: ok. 40 passed' in raw('checks/native-tests.log.gz').decode()
    assert 'Ran 16 tests' in raw('checks/python-tests.log.gz').decode()
    assert not raw('checks/actionlint.log.gz') and not raw('checks/fmt.log.gz')
    assert '33 passed; 7 failed' in raw('checks/initial-native-tests.log.gz').decode()
    assert obj('pre-inference-corrections.json.gz')['model_forwards']==0
    print(json.dumps(dict(pass_packet=True,indexed_files=len(index),previous_evidence_entries=7497,new_model_forwards=912,fresh_images=16,adopt_candidate=summary['adopt_candidate'],model_execution_this_check=False)))


if __name__=='__main__':main()
