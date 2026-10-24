#!/usr/bin/env python3
"""Independently audit real CADC projections and score declared geometric proxies."""
import argparse
from collections import Counter,defaultdict
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
from cadc_recorded import digest,require,project,reference_detect,proxy_iou,maximum_matches

ROOT=Path(__file__).resolve().parent.parent
SOURCES=['Cargo.toml','Cargo.lock','rust-toolchain.toml','crates/sim/Cargo.toml',
 'crates/sim/examples/cadc_planar_study.rs','crates/perception/src/lib.rs',
 'crates/perception/src/clustering.rs','crates/core/src/lib.rs',
 'integrations/cadc/protocol-v1.json','scripts/cadc_recorded.py',
 'scripts/check-cadc-recorded.py','scripts/test_cadc_recorded.py','scripts/requirements-cadc.txt']
def sha(path):return digest(Path(path).read_bytes())
def eligible(box,profile):
    pos=box['position'];dim=box['dimensions']
    require(all(math.isfinite(v) for v in [*pos.values(),*dim.values(),box['yaw']]),'nonfinite annotation')
    require(all(dim[k]>0 for k in ('x','y','z')),'nonpositive cuboid size')
    r=math.hypot(pos['x'],pos['y'])
    return profile['min_range']<=r<=profile['max_range'] and pos['z']+dim['z']/2>=profile['z_min'] and pos['z']-dim['z']/2<=profile['z_max']

def analyze(data,report,protocol,output):
    receipt=json.loads((data/'receipt.json').read_text());sensor_bytes=(data/'sensor.json').read_bytes();sensor=json.loads(sensor_bytes)
    require(receipt['protocol_sha256']==sha(protocol),'protocol receipt differs')
    require(receipt['sensor_sha256']==digest(sensor_bytes),'sensor receipt differs')
    config=json.loads(protocol.read_text());profile=config['profile']
    require(report['profile']==profile and report['schema_version']==1,'native report profile differs')
    identities={r['id']:r for r in receipt['frames']}
    require(len(identities)==len(receipt['frames'])==len(sensor['frames'])==len(report['frames']),'frame count differs')
    require(set(identities)=={f['id'] for f in sensor['frames']}=={f['id'] for f in report['frames']},'frame identity differs')
    by_id={r['id']:r for r in report['frames']};rows=[];inputs=[]
    for frame in sensor['frames']:
        item=identities[frame['id']];native=by_id[frame['id']]
        point_bytes=(data/item['point_path']).read_bytes()
        require(digest(point_bytes)==item['point_sha256'],'point bytes changed')
        labels=(data/item['annotation_path']).read_bytes()
        require(digest(labels)==item['annotation_sha256'],'annotation bytes changed')
        cloud=project(point_bytes,profile)
        require(native['stamp']==frame['stamp'] and native['sweep_present'] is False,'recorded timestamp/sweep changed')
        for k in ('total_rows','invalid_xyz_rows','slab_rows'):
            require(native[k]==cloud[k],'native '+k+' differs')
        require(native['selected_rows']==cloud['rows'],'native angular nearest-return selection differs')
        reference=reference_detect(cloud['points']);detections=native['detections']
        require(len(reference)==len(detections),'independent clustering count differs')
        for actual,expected in zip(detections,reference):
            error=math.hypot(actual['center']['x']-expected['center']['x'],actual['center']['y']-expected['center']['y'])
            require(error<1e-6 and abs(actual['radius']-expected['radius'])<1e-6,'independent detector geometry differs')
        boxes=json.loads(labels)[item['frame']]['cuboids'];scored=[b for b in boxes if eligible(b,profile)]
        overlaps=[[proxy_iou(d,b,config['association']['polygon_vertices']) for d in detections] for b in scored]
        adjacency=[sorted([j for j,v in enumerate(row) if v>=config['association']['iou_threshold']],key=lambda j:(-row[j],j)) for row in overlaps]
        matches=maximum_matches(adjacency);matched={i for i,_ in matches}
        bins=[]
        for lo,hi in zip(config['association']['range_edges'],config['association']['range_edges'][1:]):
            indices=[i for i,b in enumerate(scored) if lo<=math.hypot(b['position']['x'],b['position']['y'])<hi or
                     hi==profile['max_range'] and math.hypot(b['position']['x'],b['position']['y'])==hi]
            bins.append({'near':lo,'far':hi,'eligible':len(indices),'matched':sum(i in matched for i in indices)})
        row={'id':frame['id'],'pair':item['pair'],'domain':item['domain'],'frame':item['frame'],
            'totalXYZIRows':native['total_rows'],'invalidXYZRows':native['invalid_xyz_rows'],
            'slabRows':native['slab_rows'],'projectedReturns':len(cloud['points']),
            'nativeProxies':len(detections),'eligibleCuboids':len(scored),'matchedCuboids':len(matches),
            'unassociatedProxies':len(detections)-len(matches),'rangeBins':bins,
            'eligibleLabels':dict(Counter(b['label'] for b in scored)),
            'associationAmbiguities':sum(len(a)>1 for a in adjacency),
            'inputPointSha256':item['point_sha256'],
            'nativeDetectionSha256':digest(json.dumps(detections,separators=(',',':')).encode())}
        rows.append(row)
        inputs.append({k:v for k,v in item.items() if k not in ('point_path','annotation_path')})
    source=os.environ.get('RUSTDRIVING_SOURCE_SHA','uncommitted')
    return {'schema_version':1,'sourceCommit':source,'protocolSha256':sha(protocol),'profile':profile,
        'scope':config['scope'],'frames':rows,'inputs':inputs,
        'nativeReportSha256':digest(json.dumps(report,separators=(',',':')).encode()),
        'audits':{'binaryAndProjection':True,'independentExhaustiveDetector':True,
                  'noCalibratedSweep':True,'inputAndAnnotationHashes':True},
        'sourceSha256':{p:sha(ROOT/p) for p in SOURCES}}

def aggregate(rows):
    result={}
    for domain in ('clear','winter'):
        selected=[r for r in rows if r['domain']==domain]
        eligible_count=sum(r['eligibleCuboids'] for r in selected);matched=sum(r['matchedCuboids'] for r in selected)
        result[domain]={'frames':len(selected),'rawRows':sum(r['totalXYZIRows'] for r in selected),
            'projectedReturns':sum(r['projectedReturns'] for r in selected),
            'eligibleCuboids':eligible_count,'matchedCuboids':matched,
            'proxyMatchFraction':matched/eligible_count if eligible_count else None,
            'unassociatedProxiesPerFrame':sum(r['unassociatedProxies'] for r in selected)/len(selected),
            'rangeBins':[{'near':lo,'far':hi,
              'eligible':sum(b['eligible'] for r in selected for b in r['rangeBins'] if b['near']==lo),
              'matched':sum(b['matched'] for r in selected for b in r['rangeBins'] if b['near']==lo)}
              for lo,hi in [(2,15),(15,30),(30,45)]]}
    return result
def plot(summary,target):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    domains=['clear','winter'];colors=['#39a39a','#6d86b4']
    fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
    for j,domain in enumerate(domains):
        rows=summary['aggregate'][domain]['rangeBins']
        values=[b['matched']/b['eligible'] if b['eligible'] else float('nan') for b in rows]
        axes[0].bar([i+j*.36 for i in range(3)],values,width=.36,color=colors[j],label=domain)
        for i,b in enumerate(rows):
            if not b['eligible']:axes[0].text(i+j*.36,.02,'n/a',ha='center',rotation=90)
    axes[0].set_xticks([.18,1.18,2.18],['2–15','15–30','30–45']);axes[0].set_ylim(0,1)
    axes[0].set_xlabel('Cuboid centre horizontal range (m)');axes[0].set_ylabel('Matched / eligible cuboids')
    axes[0].set_title('Fixed planar proxy overlap ≥ 0.1');axes[0].legend()
    for ax,key,title in [(axes[1],'unassociatedProxiesPerFrame','Unassociated outputs / frame'),
                         (axes[2],'projectedReturns','Selected observed returns (total)')]:
        ax.bar(domains,[summary['aggregate'][d][key] for d in domains],color=colors)
        ax.set_title(title)
    fig.suptitle('CADC+ recorded planar diagnostic · 2 route pairs · no weather coefficient fit')
    fig.savefig(target,dpi=150);plt.close(fig)
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path);p.add_argument('--native',type=Path);p.add_argument('--binary',type=Path);p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--merge',type=Path,nargs='+')
    args=p.parse_args();require(not args.output.exists(),'output destination must be new')
    args.output.mkdir(parents=True)
    if args.merge:
        reports=[json.loads(path.read_text()) for path in args.merge]
        source=os.environ['RUSTDRIVING_SOURCE_SHA']
        require(all(r['sourceCommit']==source for r in reports),'source identities differ')
        require(all(r['protocolSha256']==sha(args.protocol) for r in reports),'protocol identities differ')
        require(all(r['sourceSha256']==reports[0]['sourceSha256'] for r in reports),'source hashes differ')
        rows=[f for r in reports for f in r['frames']]
        expected={domain+'-'+str(pair)+'-'+str(frame).zfill(10) for pair in (0,1) for domain in ('clear','winter') for frame in (0,10,20,30,40,50,60,70)}
        require(len(rows)==32 and {r['id'] for r in rows}==expected,'frozen 32 frames incomplete')
        require(all(r['nativeExecutableSha256']==reports[0]['nativeExecutableSha256'] for r in reports),'native executable differs')
        for name,h in reports[0]['sourceSha256'].items():require(sha(name)==h,'tested source changed')
        summary={'sourceCommit':source,'protocolSha256':sha(args.protocol),'frames':32,'routePairs':2,
            'aggregate':aggregate(rows),
            'interpretation':'Geometric circle proxies on a fixed LiDAR-local slab, not semantic 3D AP or a causal snowfall comparison. Unassociated proxies include walls, ground and unlabelled objects; they are not proven false objects. Different days contain different traffic, occlusions and surface conditions. No fitted weather coefficients or driving result.'}
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
        raw={'sourceCommit':source,'frames':rows,'inputs':[i for r in reports for i in r['inputs']],
             'sourceSha256':reports[0]['sourceSha256'],'audits':[r['audits'] for r in reports]}
        with (args.output/'report.json.gz').open('wb') as stream:
            with gzip.GzipFile(fileobj=stream,filename='',mode='wb',mtime=0) as writer:
                writer.write(json.dumps(raw,separators=(',',':'),allow_nan=False).encode())
        plot(summary,args.output/'overview.png')
        provenance={'sourceCommit':source,'protocolSha256':sha(args.protocol),'sourceSha256':reports[0]['sourceSha256'],
            'dataLicense':'Original CADC: CC BY-NC 4.0. Raw point clouds, annotations and spatial detector outputs are not redistributed. See dataset source terms for inputs.',
            'dataset':'https://uwaterloo.ca/waterloo-intelligent-systems-engineering-lab/cadc-plus',
            'nativeExecutableSha256':reports[0]['nativeExecutableSha256'],
            'nativeReports':[r['nativeReportSha256'] for r in reports],
            'files':{p.name:sha(p) for p in args.output.iterdir() if p.is_file()}}
        (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
        print('CADC VERIFIED SUMMARY',json.dumps(summary),flush=True)
    else:
        require(args.data is not None and args.native is not None and args.binary is not None,'data, native report and binary required')
        report=analyze(args.data,json.loads(args.native.read_text()),args.protocol,args.output)
        report['nativeReportSha256']=sha(args.native)
        report['nativeExecutableSha256']=sha(args.binary)
        (args.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('CADC AUDITED',json.dumps({'frames':len(report['frames']),'aggregate':aggregate(report['frames'])}),flush=True)
if __name__=='__main__':main()
