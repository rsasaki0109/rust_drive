#!/usr/bin/env python3
"""Check fixed detector equivalence and report production release timings."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys

REPO=Path(__file__).resolve().parents[1]
REFERENCE=REPO/'crates/perception/tests/fixtures/legacy_cluster_detector.rs'
GATES=dict(reference_commit='2da76ef2de409c829a13499d7a62593057bcd1fc',
    reference_detector_sha256='7a84897a23a3230404f25ba9b684802e116dad31a91f1ec86f70d6b05aa47869',
    workloads={'small-64':64,'circle-720':720,'circle-4096':4096,'compact-8192':8192,
        'separated-8192':8192,'circles-512':512,'sparse-arcs-512':512},
    repetitions=9,bitwise_output_equivalence=True,wall_clock_is_ci_gate=False,
    scope='Synthetic body-XY point clouds; full production release detector versus frozen reference. '
          'Medians are local diagnostics, not real-time or measured-road-data qualification.')


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    suffix='.exe' if sys.platform=='win32' else ''
    parser.add_argument('--binary',type=Path,default=REPO/('target/release/examples/cluster_index'+suffix))
    args=parser.parse_args();output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    (output/'frozen-protocol.json').write_text(json.dumps(GATES,indent=2)+'\n')
    assert sha(REFERENCE)==GATES['reference_detector_sha256'],'frozen detector changed'
    binary=args.binary.resolve();digest=sha(binary)
    with (output/'stdout.jsonl').open('w') as stdout,(output/'stderr').open('w') as stderr:
        code=subprocess.run([str(binary)],stdout=stdout,stderr=stderr).returncode
    assert code==0,(output/'stderr').read_text()
    rows=[json.loads(line) for line in (output/'stdout.jsonl').read_text().splitlines()]
    assert len(rows)==len(GATES['workloads']) and {r['case'] for r in rows}==set(GATES['workloads'])
    for row in rows:
        assert row['points']==GATES['workloads'][row['case']] and row['repetitions']==GATES['repetitions']
        assert row['bitwise_equal'] is True and row['sparse_circles']==(row['case']=='sparse-arcs-512')
        for name in ['legacy_ns','indexed_ns']:
            assert len(row[name])==GATES['repetitions'] and all(type(v) is int and v>0 for v in row[name])
            row[name.replace('_ns','_median_ns')]=statistics.median(row[name])
        row['median_speedup']=row['legacy_median_ns']/row['indexed_median_ns']
    assert sha(binary)==digest,'benchmark executable changed'
    report=dict(complete=True,passed=True,gates=GATES,binary_sha256=digest,
        frozen_reference_sha256=sha(REFERENCE),cases=rows,
        note='Point order, adaptive directed connectivity, shape fitting, filtering and transforms are compared bit for bit. Timing is outside the equality checks; each pair alternates execution order.')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for row in rows:print(row['case'],f"{row['median_speedup']:.2f}x",'identical bits')


if __name__=='__main__':main()
