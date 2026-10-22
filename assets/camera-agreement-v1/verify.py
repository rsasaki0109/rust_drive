#!/usr/bin/env python3
"""Replay the viewed-only trial without images, weights, network or inference."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import re
import runpy
import sys
import tempfile

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'integrations/onnx'))
import camera_agreement_check as c
def sha(b):return hashlib.sha256(b).hexdigest()
def raw(n):
    p=HERE/n;b=p.read_bytes();return gzip.decompress(b) if p.suffix=='.gz' else b
def obj(n):return json.loads(raw(n))
def annotation(row):
    for name in ('camera-detail-v1','camera-capacity-v1','camera-linear-v1'):
        p=ROOT/'assets'/name/'annotations'/(row['id']+'.json.gz')
        if p.is_file():return gzip.decompress(p.read_bytes())
    raise AssertionError('missing original annotation '+row['id'])
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',type=Path);args=p.parse_args()
    index=obj('files.json');assert set(index)=={str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p.name!='files.json'}
    for n,pin in index.items():
        b=(HERE/n).read_bytes();assert len(b)==pin['bytes'] and sha(b)==pin['sha256'],n
    for n,pin in obj('raw-files.json').items():
        b=raw(n+'.gz');assert len(b)==pin['bytes'] and sha(b)==pin['sha256'],n
    before=obj('before.json.gz');sources=obj('source-sha256.json')
    assert len(before['previous_sources'])==224 and len(before['prior_indexes'])==85 and len(before['prior_entries'])==8520 and len(before['core_outputs'])==245 and len(sources)==231
    archives=['camera-nano-v1','camera-latest-v1','camera-nano-repeat2x-v1','camera-supervised-v1','camera-depthwise-v1','camera-rowzip-v1','camera-linear-v1','camera-grid-v1','camera-supported-v1','camera-capacity-v1','camera-detail-v1','camera-preserved-v1','camera-preflight-v1','camera-agreement-v1']
    for n,h in sources.items():
        assert c.digest(ROOT/n)==h,n
        saved=next(ROOT/'assets'/a/'source'/n for a in archives if (ROOT/'assets'/a/'source'/n).is_file());assert c.digest(saved)==h,n
    for n,h in before['prior_indexes'].items():assert c.digest(ROOT/n)==h,n
    for n,pin in before['prior_entries'].items():
        b=(ROOT/n).read_bytes();assert len(b)==pin['bytes'] and sha(b)==pin['sha256'],n
    for n,h in before['core_outputs'].items():assert c.digest(ROOT/n)==h,n
    assert obj('core-output-sha256.json')==before['core_outputs']
    assert c.digest(c.HERE/'profile-agreement.json')==before['protocol_sha256']
    design=c.load(c.HERE/'profile-agreement.json');frozen=obj('validation/before-processing.json.gz');rows=frozen['input_rows']
    assert rows==c.old.read(ROOT/'assets/camera-preflight-v1/rejection/rows.json.gz')[:40] and [r['id'] for r in rows]==design['viewed_ids'] and len({r['id'] for r in rows})==40
    assert frozen['compiled_sources']==c.expected() and frozen['protocol_sha256']==before['protocol_sha256'] and frozen['checker_sha256']==c.digest(c.HERE/'camera_agreement_check.py') and frozen['numeric_labels_already_viewed'] and frozen['numeric_labels_used_for_integrity'] and not frozen['labels_used_for_inference'] and frozen['new_model_forwards']==0
    assert len(frozen['parent_sha256'])==240
    for n,h in frozen['parent_sha256'].items():assert c.digest(ROOT/n)==h,n
    receipt=obj('validation/preflight.json.gz');assert receipt==c.old.read(ROOT/'assets/camera-preflight-v1/rejection/valid40-preflight.json.gz')
    previous=runpy.run_path(str(ROOT/'assets/camera-preflight-v1/verify.py'));previous['audit_receipt'](receipt,rows,design)
    control=obj('controls/summary.json.gz');assert control['pass_controls'] and control['valid_fixtures']==8 and len(control['invalid_fixtures'])==14 and control['existing_output_preserved'] and control['compiled_sources']==c.expected() and control['checker_sha256']==c.digest(c.HERE/'camera_agreement_check.py') and control['executable_sha256']==frozen['executable_sha256'] and control['model_forwards']==0
    assert all(v['returncode']==2 for v in control['invalid_fixtures'])
    correction=obj('initial-control-correction.json.gz')
    assert sha(raw('initial-checker.py.gz'))==correction['initial_checker_sha256'] and sha(raw('initial-controls.log.gz'))==correction['initial_traceback_sha256'] and correction['model_forwards']==0 and not correction['development_processing_started']
    initial=obj('initial-bad-0-accepted.json.gz');assert initial['model_forwards']==0 and initial['result']['stats']['added']==1 and 'ValueError: native exit:' in raw('initial-controls.log.gz').decode()
    pinned=obj('validation/before-scoring.json.gz');assert len(pinned['outputs'])==120 and pinned['numeric_labels_already_viewed'] and pinned['new_model_forwards']==0
    with tempfile.TemporaryDirectory(prefix='rustdriving-agreement-audit-') as temporary:
        out=Path(temporary)
        for n in index:
            if n.startswith('validation/fixtures/') or n.startswith('validation/combined/') or n.startswith('controls/inputs/') and not n.endswith('oversize.json.gz') or re.fullmatch(r'controls/\d+-actual.json.gz',n):
                path=out/n.removesuffix('.gz');path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw(n))
        for i in range(8):c.check(c.load(out/f'controls/{i}-actual.json'),out/f'controls/inputs/{i}.json')
        for n,h in pinned['outputs'].items():assert sha((out/'validation'/n).read_bytes())==h,n
        result=c.audit(out/'validation',rows,annotation);assert result==obj('validation/summary.json.gz')
        if args.binary:
            current=out/'current';current.mkdir()
            for mode in c.MODES:
                for row in rows:
                    fixture=out/'validation/fixtures'/mode/(row['id']+'.json');path=current/(mode+'-'+row['id']+'.json')
                    actual=c.check(c.old.native(args.binary,['--fixture',fixture],path),fixture)
                    saved=c.load(out/'validation/combined'/mode/(row['id']+'.json'))['result'];c.old.prior.compare(saved,actual,'current model-free executable')
    assert not result['independent_sample_validated'] and result['independent_sample_quality'] is None and not result['fresh_sample_scored'] and not result['adopt_candidate']
    assert result['execution']==dict(new_model_forwards=0,new_native_image_runs=0,reused_parent_reports=240,reused_parent_forwards_not_counted_as_new=2640,new_development_fixture_runs=120,fixture_model_forwards=0)
    core=raw('checks/core.log.gz').decode();assert sum(map(int,re.findall(r'test result: ok\. (\d+) passed',core)))==371 and core.count('"verified": true')==49 and 'test result: FAILED' not in core
    assert 'Ran 16 tests' in raw('checks/python-tests.log.gz').decode() and 'OK' in raw('checks/python-tests.log.gz').decode() and 'test result: ok. 8 passed' in raw('checks/rust-tests.log.gz').decode()
    assert not raw('checks/fmt.log.gz') and not raw('checks/actionlint.log.gz')
    print(json.dumps(dict(pass_packet=True,indexed_files=len(index),previous_evidence_entries=8520,viewed_images=40,new_model_forwards=0,independent_sample_validated=False,adopt_candidate=False,current_fixture_replays=120 if args.binary else 0,image_decode_this_audit=False)))
if __name__=='__main__':main()
