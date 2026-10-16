#!/usr/bin/env python3
"""Failure and concurrent-publication controls for exclusive JSON evidence writes."""
import argparse,concurrent.futures,json
from pathlib import Path
from unittest.mock import patch
import numpy as np
import projective_depth_common as common


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=common.fresh(a.output);checks=[]
    bad=out/'invalid.json'
    try:common.write(bad,dict(samples=[1,2,3],gate=np.bool_(True)))
    except TypeError:pass
    else:raise ValueError('unsupported NumPy scalar unexpectedly serializes')
    common.require(not bad.exists() and not list(out.glob('.*.prepare-*')),'serialization failure publishes no partial report');checks.append('late serialization failure')
    target=out/'existing.json';target.write_bytes(b'original evidence\n')
    try:common.write(target,dict(changed=True))
    except FileExistsError:pass
    else:raise ValueError('existing evidence overwritten')
    common.require(target.read_bytes()==b'original evidence\n','original file survives');checks.append('existing evidence')
    for label,method in [('flush failure','fsync'),('publication failure','link')]:
        target=out/(method+'.json')
        with patch.object(common.os,method,side_effect=OSError('injected IO failure')):
            try:common.write(target,dict(valid=True))
            except OSError:pass
            else:raise ValueError('injected write fault ignored')
        common.require(not target.exists() and not list(out.glob('.*.prepare-*')),'IO failure leaves no final or temporary report');checks.append(label)
    target=out/'race.json'
    def writer(index):
        try:common.write(target,dict(writer=index,samples=list(range(1000))));return True
        except FileExistsError:return False
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:passed=list(pool.map(writer,(1,2)))
    body=json.loads(target.read_text());common.require(sum(passed)==1 and body['writer'] in (1,2) and body['samples']==list(range(1000)),'one complete concurrent winner')
    common.require(not list(out.glob('.*.prepare-*')),'no orphaned temporary reports');checks.append('concurrent publication')
    expected='''{\n  "count": 179,\n  "passed": false,\n  "value": 0.005\n}\n''';target=out/'canonical.json';common.write(target,dict(count=179,passed=False,value=.005))
    common.require(target.read_text()==expected,'preserve canonical evidence JSON bytes');checks.append('canonical bytes')
    common.write(out/'summary.json',dict(pass_atomic_report_controls=True,checks=checks,partial_reports_published=0))


if __name__=='__main__':main()
