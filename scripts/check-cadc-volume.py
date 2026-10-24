#!/usr/bin/env python3
"""Audit frozen CADC volume inputs, geometry and paired diagnostic outcomes."""
import argparse
from collections import Counter
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
from cadc_recorded import digest,require,proxy_iou,maximum_matches
from cadc_volume_reference import crop,ground_reference,xyz_reference

ROOT=Path(__file__).resolve().parent.parent
spec=importlib.util.spec_from_file_location('cadc_baseline_audit',ROOT/'scripts/check-cadc-recorded.py')
BASE=importlib.util.module_from_spec(spec);spec.loader.exec_module(BASE)
EXTRA=['crates/perception/src/ground3d.rs','crates/perception/src/objects3d.rs',
 'crates/sim/examples/cadc_volume_study.rs','integrations/cadc/protocol-volume-v1.json',
 'scripts/cadc_volume_acquire.py','scripts/cadc_volume_reference.py','scripts/check-cadc-volume.py',
 'scripts/test_cadc_volume.py','scripts/requirements-cadc-volume.txt','.github/workflows/cadc-volume.yml',
 'assets/cadc-planar-v1/summary.json','assets/cadc-planar-v1/provenance.json','assets/cadc-planar-v1/report.json.gz']
SOURCES=BASE.SOURCES+EXTRA
METHODS=['planar_v1','ground_dense_xy','ground_xyz']
def sha(path):return digest(Path(path).read_bytes())
def json_file(path):return json.loads(Path(path).read_text())
def frozen_previous(protocol):
    manifest=json_file(ROOT/'assets/cadc-planar-v1/provenance.json')
    require(manifest['sourceCommit']==protocol['baseline_source'],'previous source identity differs')
    require(manifest['protocolSha256']==protocol['baseline_protocol_sha256'],'previous protocol differs')
    path=ROOT/'assets/cadc-planar-v1/report.json.gz'
    require(sha(path)==manifest['files']['report.json.gz'],'previous report changed')
    with gzip.open(path,'rb') as stream:raw=stream.read(8*1024*1024+1)
    require(len(raw)<=8*1024*1024,'previous report byte bound')
    report=json.loads(raw);require(len(report['frames'])==32,'previous sample incomplete')
    return {r['id']:r for r in report['frames']}
def check_detections(actual,expected):
    require(len(actual)==len(expected),'independent detection count differs')
    for a,b in zip(actual,expected):
        require(math.hypot(a['center']['x']-b['center']['x'],a['center']['y']-b['center']['y'])<1e-6 and
                abs(a['radius']-b['radius'])<1e-6,'independent detector geometry differs')
def score(detections,boxes,protocol):
    config=protocol['association']
    overlaps=[[proxy_iou(d,b,config['polygon_vertices']) for d in detections] for b in boxes]
    graph=[sorted([j for j,v in enumerate(row) if v>=config['iou_threshold']],key=lambda j:(-row[j],j)) for row in overlaps]
    matches=maximum_matches(graph);matched={i for i,_ in matches}
    ranges=[]
    for lo,hi in zip(config['range_edges'],config['range_edges'][1:]):
        indices=[i for i,b in enumerate(boxes) if lo<=math.hypot(b['position']['x'],b['position']['y'])<hi or
                 hi==45 and math.hypot(b['position']['x'],b['position']['y'])==45]
        ranges.append({'near':lo,'far':hi,'eligible':len(indices),'matched':sum(i in matched for i in indices)})
    return {'outputs':len(detections),'eligible':len(boxes),'matched':len(matches),
            'unassociated':len(detections)-len(matches),'rangeBins':ranges}
def baseline_score(row):
    return {'outputs':row['nativeProxies'],'eligible':row['eligibleCuboids'],'matched':row['matchedCuboids'],
            'unassociated':row['unassociatedProxies'],'rangeBins':row['rangeBins']}
def analyze(args):
    import numpy as np
    protocol=json_file(args.protocol);report=json_file(args.native);baseline=json_file(args.baseline)
    previous=frozen_previous(protocol)
    frozen_manifest=json_file(ROOT/'assets/cadc-planar-v1/provenance.json')
    require(baseline['sourceCommit']==protocol['baseline_source'] and baseline['sourceSha256']==frozen_manifest['sourceSha256'],'baseline was not executed from the pinned historical source')
    receipt=json_file(args.data/'receipt.json');sensor=json_file(args.data/'sensor.json')
    require(receipt['protocol_sha256']==sha(args.protocol)==baseline['protocolSha256'],'protocol receipt differs')
    require(receipt['sensor_sha256']==sha(args.data/'sensor.json'),'sensor receipt differs')
    require(report['schema_version']==1 and sensor['profile']==protocol['profile'],'sensor/native schema differs')
    expected_settings={k:protocol[k] for k in ('volume','terrain','clustering')}
    expected_settings['proxy']={k:protocol['proxy'][k] for k in ('padding_m','min_radius_m','max_radius_m')}
    require(report['settings']==expected_settings,'native volume settings differ from frozen protocol')
    by_id={r['id']:r for r in report['frames']};base={r['id']:r for r in baseline['frames']}
    inputs={r['id']:r for r in receipt['frames']}
    require(len(by_id)==len(report['frames'])==len(base)==len(inputs)==16,'route pair must contain 16 frames')
    require(set(by_id)==set(base)==set(inputs)=={r['id'] for r in sensor['frames']},'native frame identities differ')
    records=[];verified_previous=0
    for frame in sensor['frames']:
        identity=frame['id'];item=inputs[identity];native=by_id[identity];b=base[identity]
        payload=(args.data/item['point_path']).read_bytes();labels=(args.data/item['annotation_path']).read_bytes()
        require(digest(payload)==item['point_sha256']==b['inputPointSha256'],'input point identity differs')
        require(digest(labels)==item['annotation_sha256'],'annotation identity differs')
        sequence=next(s for s in protocol['sequences'] if s['domain']==item['domain'] and s['pair']==item['pair'])
        require(item['sequence']==sequence['sequence'] and item['annotation_sha256']==sequence['annotation_sha256'],'frozen sequence differs')
        require(item['archive_url']==sequence['url'] and item['archive_etag']==sequence['etag'] and item['archive_bytes']==sequence['size'],'archive version differs')
        if item['domain']=='clear' and item['sequence'] in protocol['archive_sha256']:
            require(item['archive_sha256']==protocol['archive_sha256'][item['sequence']],'full archive identity differs')
        if item['pair'] in protocol['partitions']['previous']:
            require(identity in previous and b==previous[identity],'previous planar result changed')
            verified_previous+=1;partition='previous'
        else:
            require(item['pair'] in protocol['partitions']['additional'],'unregistered additional sequence')
            partition='additional'
        cloud=crop(payload,protocol['volume']);points=cloud['points']
        require(native['stamp']==frame['stamp'] and native['sweep_present'] is False,'timestamp/sweep differs')
        for key in ('total_rows','invalid_xyz_rows'):require(native[key]==cloud[key],'volume row accounting differs')
        require(native['selected_rows']==cloud['rows'],'independent XYZ crop differs')
        ground,diagnostics=ground_reference(points,protocol['terrain'])
        require(native['ground_indices']==ground and native['ground']==diagnostics,'independent ground morphology/support differs')
        ground_set=set(ground)
        retained=[i for i in range(len(points)) if i not in ground_set] if diagnostics['confident'] else list(range(len(points)))
        require(native['retained_indices']==retained,'unknown ground support removed points')
        flattened=points.copy();flattened[:,2]=0
        xy_objects,xy_noise=xyz_reference(flattened,retained,protocol['clustering'],protocol['proxy'])
        require(native['xy_noise_indices']==xy_noise and len(native['xy_objects'])==len(xy_objects),'independent XY partition differs')
        for actual,expected in zip(native['xy_objects'],xy_objects):
            require(actual['indices']==expected['indices'] and actual['point_count']==expected['point_count'],'independent XY component differs')
            for key in ('min','max'):
                require(all(abs(actual[key][k]-expected[key][k])<1e-9 for k in ('x','y','z')),'measured flattened AABB differs')
            check_detections([actual['proxy']],[expected['proxy']])
        check_detections(native['dense_xy'],[o['proxy'] for o in xy_objects])
        xy_voxels=len(np.unique(np.floor(flattened[retained]/protocol['clustering']['voxel_size_m']).astype('int64'),axis=0)) if retained else 0
        require(native['xy_occupied_voxels']==xy_voxels and 0<=native['xy_object_work']<=protocol['clustering']['max_candidate_work'],'XY work/voxel accounting differs')
        objects,noise=xyz_reference(points,retained,protocol['clustering'],protocol['proxy'])
        require(native['noise_indices']==noise and len(native['objects'])==len(objects),'independent XYZ partition differs')
        for actual,expected in zip(native['objects'],objects):
            require(actual['indices']==expected['indices'] and actual['point_count']==expected['point_count'],'independent XYZ component differs')
            for key in ('min','max'):
                require(all(abs(actual[key][k]-expected[key][k])<1e-9 for k in ('x','y','z')),'measured XYZ AABB differs')
            check_detections([actual['proxy']],[expected['proxy']])
        voxels=len(np.unique(np.floor(points[retained]/protocol['clustering']['voxel_size_m']).astype('int64'),axis=0)) if retained else 0
        require(native['occupied_voxels']==voxels and 0<=native['object_work']<=protocol['clustering']['max_candidate_work'],'native work/voxel accounting differs')
        # Annotation geometry enters only after every native sensing output has passed independent checks.
        cuboids=json.loads(labels)[item['frame']]['cuboids'];eligible=[box for box in cuboids if BASE.eligible(box,protocol['profile'])]
        methods={'planar_v1':baseline_score(b),
            'ground_dense_xy':score(native['dense_xy'],eligible,protocol),
            'ground_xyz':score([o['proxy'] for o in native['objects']],eligible,protocol)}
        require(all(m['eligible']==len(eligible) for m in methods.values()),'paired denominator differs')
        records.append({'id':identity,'domain':item['domain'],'pair':item['pair'],'frame':item['frame'],'partition':partition,
            'rawRows':native['total_rows'],'croppedReturns':len(points),'groundReturns':len(ground),
            'retainedReturns':len(retained),'groundSupported':diagnostics['confident'],
            'groundSupportedFraction':diagnostics['supported_fraction'],'xyzNoiseReturns':len(noise),
            'eligibleLabels':dict(Counter(box['label'] for box in eligible)),'methods':methods,
            'inputPointSha256':item['point_sha256'],'nativeFrameSha256':digest(json.dumps(native,separators=(',',':')).encode())})
    pairs={i['pair'] for i in inputs.values()}
    require(len(pairs)==1,'route shard mixes pair ordinals')
    pair=next(iter(pairs));expected={d+'-'+str(pair)+'-'+str(f).zfill(10) for d in ('clear','winter') for f in protocol['frames']}
    require(set(inputs)==expected,'route shard frame protocol differs')
    require(verified_previous==(16 if pair in protocol['partitions']['previous'] else 0),'previous route-pair baseline incomplete')
    source=os.environ['RUSTDRIVING_SOURCE_SHA']
    result={'schema_version':1,'sourceCommit':source,'protocolSha256':sha(args.protocol),
        'sourceSha256':{p:sha(ROOT/p) for p in SOURCES},'frames':records,'inputs':baseline['inputs'],
        'nativeExecutableSha256':sha(args.binary),'baselineExecutableSha256':baseline['nativeExecutableSha256'],
        'nativeReportSha256':sha(args.native),'baselineReportSha256':baseline['nativeReportSha256'],'baselineSourceCommit':baseline['sourceCommit'],
        'audits':{'independentCrop':True,'independentArrayMorphology':True,'independentFlattenedConnectivity':True,
            'independentUndirectedConnectivity':True,'unknownSupportKeepsPoints':True,'previousPlanarFramesVerified':verified_previous,
            'pairedEligibility':True,'noSweepCertificate':True}}
    return result

def aggregate(frames):
    result={}
    for partition in ('previous','additional'):
        result[partition]={}
        for domain in ('clear','winter'):
            rows=[r for r in frames if r['partition']==partition and r['domain']==domain]
            if not rows:continue
            methods={}
            for method in METHODS:
                total=[r['methods'][method] for r in rows];eligible=sum(r['eligible'] for r in total);matched=sum(r['matched'] for r in total)
                methods[method]={'frames':len(rows),'eligible':eligible,'matched':matched,
                    'matchFraction':matched/eligible if eligible else None,
                    'outputsPerFrame':sum(r['outputs'] for r in total)/len(rows),
                    'unassociatedPerFrame':sum(r['unassociated'] for r in total)/len(rows),
                    'rangeBins':[{'near':lo,'far':hi,
                        'eligible':sum(b['eligible'] for r in total for b in r['rangeBins'] if b['near']==lo),
                        'matched':sum(b['matched'] for r in total for b in r['rangeBins'] if b['near']==lo)}
                        for lo,hi in ((2,15),(15,30),(30,45))],
                    'vsPlanar':{'matchedBetterFrames':sum(r['methods'][method]['matched']>r['methods']['planar_v1']['matched'] for r in rows),
                        'matchedWorseFrames':sum(r['methods'][method]['matched']<r['methods']['planar_v1']['matched'] for r in rows),
                        'unassociatedWorseFrames':sum(r['methods'][method]['unassociated']>r['methods']['planar_v1']['unassociated'] for r in rows)}}
            result[partition][domain]={'frames':len(rows),'supportedGroundFrames':sum(r['groundSupported'] for r in rows),
                'rawRows':sum(r['rawRows'] for r in rows),'retainedReturns':sum(r['retainedReturns'] for r in rows),'methods':methods}
    return result
def plot(summary,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors=['#379b94','#d2953e','#6c82b8'];labels=['Planar v1','Ground + dense XY','Ground + XYZ']
    figure,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    for row,domain in enumerate(('clear','winter')):
        results=summary['aggregate']['additional'][domain]['methods']
        for j,method in enumerate(METHODS):
            bins=results[method]['rangeBins']
            values=[b['matched']/b['eligible'] if b['eligible'] else float('nan') for b in bins]
            axes[row,0].bar([i+j*.25 for i in range(3)],values,width=.25,color=colors[j],label=labels[j])
            for i,b in enumerate(bins):
                if not b['eligible']:axes[row,0].text(i+j*.25,.02,'n/a',rotation=90,ha='center')
        axes[row,0].set_xticks([.25,1.25,2.25],['2–15','15–30','30–45']);axes[row,0].set_ylim(0,1)
        axes[row,0].set_title(domain+' · matched / eligible cuboids');axes[row,0].set_xlabel('Horizontal centre range (m)')
        axes[row,0].legend(fontsize=8)
        axes[row,1].bar(labels,[results[m]['unassociatedPerFrame'] for m in METHODS],color=colors)
        axes[row,1].set_title(domain+' · unassociated geometric outputs / frame')
    figure.suptitle('CADC+ additional sequences · 32 new scans · fixed proxy overlap ≥ 0.1\nSame city and winter dates; not semantic AP or causal snow attribution',fontsize=12)
    figure.savefig(path,dpi=150);plt.close(figure)
def merge(args):
    reports=[json_file(path) for path in args.merge];source=os.environ['RUSTDRIVING_SOURCE_SHA']
    require(len(reports)==4 and all(r['sourceCommit']==source and r['protocolSha256']==sha(args.protocol) for r in reports),'source/protocol differs')
    for r in reports:
        require(r['sourceSha256']==reports[0]['sourceSha256'],'source content identities differ')
        for key in ('nativeExecutableSha256','baselineExecutableSha256'):require(r[key]==reports[0][key],'native build differs')
    for p,h in reports[0]['sourceSha256'].items():require(sha(ROOT/p)==h,'tested source changed')
    frames=[r for report in reports for r in report['frames']]
    expected={d+'-'+str(p)+'-'+str(f).zfill(10) for d in ('clear','winter') for p in (0,1,2,3) for f in range(0,80,10)}
    require(len(frames)==64 and {r['id'] for r in frames}==expected,'frozen 64 scans incomplete')
    require(sum(r['audits']['previousPlanarFramesVerified'] for r in reports)==32,'frozen previous baseline incomplete')
    summary={'schema_version':1,'sourceCommit':source,'protocolSha256':sha(args.protocol),'frames':64,
        'previousFrames':32,'additionalFrames':32,'aggregate':aggregate(frames),
        'interpretation':'Fixed geometric proxy comparison. Additional sequences are metadata-selected from the same city/winter dates, not an official held-out validation split. Different dates confound weather attribution. Eligibility remains the original slab-intersecting cuboids; no semantic AP, hidden-shape, calibrated sweep, weather fit or driving claim. More geometric outputs can increase both matching and unassociated outputs.'}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    raw={'sourceCommit':source,'frames':frames,'inputs':[i for r in reports for i in r['inputs']],
         'audits':[r['audits'] for r in reports],'sourceSha256':reports[0]['sourceSha256']}
    with (args.output/'report.json.gz').open('wb') as stream:
        with gzip.GzipFile(fileobj=stream,filename='',mode='wb',mtime=0) as writer:
            writer.write(json.dumps(raw,separators=(',',':'),allow_nan=False).encode())
    plot(summary,args.output/'overview.png')
    provenance={'sourceCommit':source,'sourceSha256':reports[0]['sourceSha256'],'protocolSha256':sha(args.protocol),
        'nativeExecutableSha256':reports[0]['nativeExecutableSha256'],'baselineExecutableSha256':reports[0]['baselineExecutableSha256'],'baselineSourceCommit':reports[0]['baselineSourceCommit'],
        'nativeReports':[r['nativeReportSha256'] for r in reports],'baselineReports':[r['baselineReportSha256'] for r in reports],
        'dataLicense':'Original CADC CC BY-NC 4.0; consult CADC+ source terms. No raw point clouds, annotation files or spatial detector outputs redistributed.',
        'files':{p.name:sha(p) for p in args.output.iterdir() if p.is_file()}}
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print('CADC VOLUME VERIFIED',json.dumps(summary),flush=True)
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path);p.add_argument('--native',type=Path);p.add_argument('--binary',type=Path)
    p.add_argument('--baseline',type=Path);p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--merge',type=Path,nargs='+')
    args=p.parse_args();require(not args.output.exists(),'output must be a new directory');args.output.mkdir(parents=True)
    if args.merge:merge(args)
    else:
        require(all(x is not None for x in (args.data,args.native,args.binary,args.baseline)),'native, baseline and input paths required')
        result=analyze(args);(args.output/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print('CADC VOLUME AUDITED',json.dumps({'frames':len(result['frames']),'aggregate':aggregate(result['frames'])}),flush=True)
if __name__=='__main__':main()
