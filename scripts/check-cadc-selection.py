#!/usr/bin/env python3
"""Freeze and audit measured-component policy selection before new CADC sequences."""
import argparse
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
from cadc_recorded import digest,require,proxy_iou,box_polygon,maximum_matches
from cadc_selection_reference import proxy,select_reference,aggregate,choose_policy,retention_constraints

ROOT=Path(__file__).resolve().parent.parent
spec=importlib.util.spec_from_file_location('legacy_cadc_metrics',ROOT/'scripts/check-cadc-recorded.py')
BASE=importlib.util.module_from_spec(spec);spec.loader.exec_module(BASE)
SOURCES=['Cargo.toml','Cargo.lock','rust-toolchain.toml','crates/core/src/lib.rs',
 'crates/perception/Cargo.toml','crates/perception/src/lib.rs','crates/perception/src/objects3d.rs',
 'crates/perception/src/component_selection.rs','crates/sim/Cargo.toml',
 'crates/sim/examples/cadc_component_selection.rs','scripts/cadc_recorded.py','scripts/check-cadc-recorded.py',
 'scripts/cadc_selection_acquire.py','scripts/cadc_selection_reference.py','scripts/check-cadc-selection.py',
 'scripts/test_cadc_selection.py','scripts/requirements-cadc-selection.txt',
 'integrations/cadc/protocol-selection-v1.json','.github/workflows/cadc-component-selection.yml',
 'assets/cadc-volume-v1/summary.json','assets/cadc-volume-v1/report.json.gz','assets/cadc-volume-v1/provenance.json']
def sha(path):return digest(Path(path).read_bytes())
def read(path):return json.loads(Path(path).read_text())
def previous_frames(protocol):
    manifest=read(ROOT/'assets/cadc-volume-v1/provenance.json')
    require(manifest['sourceCommit']==protocol['raw_source'],'raw reference source differs')
    path=ROOT/'assets/cadc-volume-v1/report.json.gz'
    require(sha(path)==manifest['files']['report.json.gz'],'previous report identity differs')
    with gzip.open(path,'rb') as stream:data=stream.read(16*1024*1024+1)
    require(len(data)<=16*1024*1024,'previous report byte limit')
    records=json.loads(data)['frames'];require(len(records)==64,'previous 64 frames incomplete')
    return {r['id']:r for r in records},manifest
def validate_selection(selection,protocol,source):
    require(selection['sourceCommit']==source and selection['protocolSha256']==sha(ROOT/'integrations/cadc/protocol-selection-v1.json'),'selected policy source/protocol differs')
    recomputed=choose_policy(selection['developmentSummary'],protocol)
    require(selection['selected']==recomputed,'selected policy differs from development objective')
    require(len(selection['developmentInputs'])==64,'selection development inputs incomplete')
    require({r['pair'] for r in selection['developmentInputs']}==set(protocol['study_partitions']['development']),'validation data entered policy choice')
    return recomputed['policy']
def prepare(args):
    protocol=read(args.protocol);raw=read(args.raw);audit=read(args.raw_audit)
    _,manifest=previous_frames(protocol)
    require(audit['sourceCommit']==protocol['raw_source'] and audit['sourceSha256']==manifest['sourceSha256'],'raw baseline is not the pinned historical implementation')
    require(audit['nativeReportSha256']==sha(args.raw),'raw native report identity differs')
    require(audit['protocolSha256']==sha(args.protocol),'raw protocol identity differs')
    pairs={r['pair'] for r in audit['frames']};require(len(pairs)==1,'shard mixes sequence pairs')
    pair=next(iter(pairs));require(pair in protocol['study_partitions'][args.phase],'wrong study partition')
    if args.phase=='validation':
        require(args.selection is not None and args.freeze is not None,'validation requires an already frozen choice')
        selection=read(args.selection);frozen=read(args.freeze)
        require(frozen['selectionSha256']==sha(args.selection) and frozen['sourceCommit']==os.environ['RUSTDRIVING_SOURCE_SHA'],'freeze identity differs')
        policies=[validate_selection(selection,protocol,os.environ['RUSTDRIVING_SOURCE_SHA'])]
    else:policies=protocol['policies']
    common=protocol['selection_common']
    require(raw['settings']['proxy']=={k:protocol['proxy'][k] for k in ('padding_m','min_radius_m','max_radius_m')},'raw proxy settings differ')
    require([common['radius_padding_m'],common['min_proxy_radius_m'],common['max_proxy_radius_m']]==
            [protocol['proxy'][k] for k in ('padding_m','min_radius_m','max_radius_m')],'selection changed proxy geometry')
    frames=[]
    for frame in raw['frames']:
        frames.append({'id':frame['id'],'data_point_count':len(frame['selected_rows']),
            'candidates':[{k:component[k] for k in ('min','max','point_count','indices')} for component in frame['objects']]})
    result={'schema_version':1,'common':common,'policies':policies,'frames':frames}
    return result

def overlap_graph(components,cuboids,protocol):
    # Conservative axis-aligned bounds reject disjoint geometry before polygon clipping.
    boxes=[]
    for box in cuboids:
        rectangle=box_polygon(box)
        boxes.append((min(p[0] for p in rectangle),max(p[0] for p in rectangle),
                      min(p[1] for p in rectangle),max(p[1] for p in rectangle)))
    graph=[]
    for box,bounds in zip(cuboids,boxes):
        overlaps=[]
        for i,component in enumerate(components):
            detection=component['proxy'];centre=detection['center'];radius=detection['radius']
            if centre['x']+radius<bounds[0] or centre['x']-radius>bounds[1] or centre['y']+radius<bounds[2] or centre['y']-radius>bounds[3]:continue
            value=proxy_iou(detection,box,protocol['association']['polygon_vertices'])
            if value>=protocol['association']['iou_threshold']:overlaps.append((i,value))
        graph.append([i for i,value in sorted(overlaps,key=lambda item:(-item[1],item[0]))])
    return graph
def score_subset(selected,graph,cuboids,eligible_ids):
    chosen=set(selected);eligible_graph=[[i for i in graph[b] if i in chosen] for b in eligible_ids]
    matches=maximum_matches(eligible_graph);matched={b for b,_ in matches}
    eligible_overlap={i for row in eligible_graph for i in row}
    all_overlap={i for row in graph for i in row if i in chosen}
    ranges=[]
    for lo,hi in ((2,15),(15,30),(30,45)):
        indices=[i for i,b in enumerate(eligible_ids) if lo<=math.hypot(cuboids[b]['position']['x'],cuboids[b]['position']['y'])<hi or
                 hi==45 and math.hypot(cuboids[b]['position']['x'],cuboids[b]['position']['y'])==hi]
        ranges.append({'near':lo,'far':hi,'eligible':len(indices),'matched':sum(i in matched for i in indices)})
    context={'matched':len(matches),'unassignedEligibleOverlap':len(eligible_overlap)-len(matches),
        'outsideEligibilityOnly':len(all_overlap)-len(eligible_overlap),'noAnnotatedOverlap':len(chosen)-len(all_overlap)}
    require(sum(context.values())==len(chosen),'overlap-context partition differs')
    return {'outputs':len(chosen),'eligible':len(eligible_ids),'matched':len(matches),
            'unassociated':len(chosen)-len(matches),'rangeBins':ranges,'context':context}
def audit(args):
    protocol=read(args.protocol);raw=read(args.raw);raw_audit=read(args.raw_audit)
    selector_input=read(args.input);native=read(args.native)
    expected=prepare(args);require(selector_input==expected,'selection input changed geometry or policy')
    require(native['schema_version']==1 and native['common']==expected['common'] and native['policies']==expected['policies'],'native selection settings differ')
    ids={r['id']:r for r in native['frames']};raw_ids={r['id']:r for r in raw['frames']}
    source_ids={r['id']:r for r in raw_audit['frames']}
    require(len(ids)==len(native['frames'])==len(raw_ids)==len(source_ids)==16 and set(ids)==set(raw_ids)==set(source_ids),'route-pair frame identities differ')
    receipt=read(args.data/'receipt.json');inputs={r['id']:r for r in receipt['frames']}
    require(set(inputs)==set(ids) and receipt['protocol_sha256']==sha(args.protocol),'input receipt differs')
    old,_=previous_frames(protocol);records=[];verified=0
    for frame in expected['frames']:
        identity=frame['id'];source=source_ids[identity];raw_frame=raw_ids[identity];native_frame=ids[identity];item=inputs[identity]
        if args.phase=='development':
            require(identity in old and source==old[identity],'previous raw XYZ result changed');verified+=1
        decisions={d['id']:d for d in native_frame['decisions']}
        require(len(decisions)==len(native_frame['decisions'])==len(expected['policies']) and set(decisions)=={p['id'] for p in expected['policies']},'policy decisions incomplete')
        components=frame['candidates'];raw_components=raw_frame['objects']
        for c,r in zip(components,raw_components):
            centre,radius=proxy(c,protocol['selection_common'])
            require(math.hypot(centre['x']-r['proxy']['center']['x'],centre['y']-r['proxy']['center']['y'])<1e-9 and abs(radius-r['proxy']['radius'])<1e-9,'selection proxy changed measured hypothesis')
        labels=(args.data/item['annotation_path']).read_bytes();points=(args.data/item['point_path']).read_bytes()
        require(digest(labels)==item['annotation_sha256'] and digest(points)==item['point_sha256']==source['inputPointSha256'],'recorded inputs changed')
        cuboids=json.loads(labels)[item['frame']]['cuboids'];eligible=[i for i,b in enumerate(cuboids) if BASE.eligible(b,protocol['profile'])]
        graph=overlap_graph(raw_components,cuboids,protocol);raw_score=score_subset(range(len(components)),graph,cuboids,eligible)
        require({k:v for k,v in raw_score.items() if k!='context'}==source['methods']['ground_xyz'],'raw score differs from pinned audit')
        methods={'raw':raw_score};details={}
        for policy in expected['policies']:
            actual=decisions[policy['id']];reference=select_reference(components,protocol['selection_common'],policy)
            for key,value in reference.items():require(actual[key]==value,'independent selection differs: '+identity+' '+policy['id']+' '+key)
            limit=min(protocol['selection_common']['max_candidate_work'],len(components)*(len(components)-1)//2)
            require(0<=actual['candidate_work']<=limit,'selection comparison accounting differs')
            assigned=actual['kept_indices']+actual['low_support_indices']+[s[0] for s in actual['suppressed']]
            require(sorted(assigned)==list(range(len(components))) and all(s[1] in actual['kept_indices'] for s in actual['suppressed']),'selection failed to partition raw components')
            value=score_subset(actual['kept_indices'],graph,cuboids,eligible)
            require(value['matched']<=raw_score['matched'] and value['outputs']<=raw_score['outputs'],'subset created new hypotheses or matches')
            methods[policy['id']]=value
            details[policy['id']]={'kept':actual['kept_indices'],'lowSupport':actual['low_support_indices'],
                'suppressed':actual['suppressed'],'candidateWork':actual['candidate_work'],
                'withheldReturnMembership':sum(components[i]['point_count'] for i in set(range(len(components)))-set(actual['kept_indices']))}
        histogram={str(lo)+'-'+str(hi):sum(lo<=c['point_count']<=hi for c in components) for lo,hi in ((3,4),(5,7),(8,11),(12,15),(16,31),(32,500000))}
        records.append({'id':identity,'pair':item['pair'],'domain':item['domain'],'frame':item['frame'],'phase':args.phase,
            'methods':methods,'selection':details,'componentSupportHistogram':histogram,
            'rawFrameSha256':source['nativeFrameSha256'],'inputPointSha256':item['point_sha256'],'annotationSha256':item['annotation_sha256'],
            'rawReturnsUnchanged':True})
    require(verified==(16 if args.phase=='development' else 0),'development replay incomplete')
    result={'schema_version':1,'sourceCommit':os.environ['RUSTDRIVING_SOURCE_SHA'],'protocolSha256':sha(args.protocol),
        'sourceSha256':{p:sha(ROOT/p) for p in SOURCES},'phase':args.phase,'policyIds':[p['id'] for p in expected['policies']],
        'frames':records,'inputs':raw_audit['inputs'],'rawSourceCommit':protocol['raw_source'],
        'nativeExecutableSha256':sha(args.binary),'rawExecutableSha256':raw_audit['nativeExecutableSha256'],
        'baselineExecutableSha256':raw_audit['baselineExecutableSha256'],'rawReportSha256':sha(args.raw),
        'selectionInputSha256':sha(args.input),'selectionReportSha256':sha(args.native),
        'selectionSha256':sha(args.selection) if args.selection else None,'freeze':read(args.freeze) if args.freeze else None,
        'audits':{'independentNumericalLens':True,'independentAllPairsSelection':True,'unchangedRawGeometry':True,
            'sourceReturnPartition':True,'previousFramesReproduced':verified,'noValidationPolicySweep':args.phase!='validation' or len(expected['policies'])==1}}
    return result

def choose(args):
    protocol=read(args.protocol);reports=[read(p) for p in args.reports];source=os.environ['RUSTDRIVING_SOURCE_SHA']
    require(len(reports)==4 and all(r['phase']=='development' and r['sourceCommit']==source and r['protocolSha256']==sha(args.protocol) for r in reports),'development reports differ')
    frames=[f for r in reports for f in r['frames']]
    expected={d+'-'+str(p)+'-'+str(f).zfill(10) for p in range(4) for d in ('clear','winter') for f in protocol['frames']}
    require(len(frames)==64 and {f['id'] for f in frames}==expected,'development 64 frames incomplete')
    require(all(r['sourceSha256']==reports[0]['sourceSha256'] for r in reports),'development code identities differ')
    summary=aggregate(frames,[p['id'] for p in protocol['policies']]);selected=choose_policy(summary,protocol)
    result={'schema_version':1,'sourceCommit':source,'protocolSha256':sha(args.protocol),'selected':selected,
        'criteria':protocol['choice'],'developmentSummary':summary,
        'developmentInputs':[{'id':r['id'],'pair':r['pair'],'pointSha256':r['inputPointSha256']} for r in frames],
        'developmentReports':[sha(p) for p in args.reports],
        'interpretation':'Chosen using only the previous 64 scans. Validation point clouds must be acquired after freezing this selection. Counts are geometric proxies, not semantic precision or calibrated detection probability.'}
    return result

def plot(summary,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colours=['#4c6683','#3b9a8f'];context_colours=['#3b9a8f','#dcaa4c','#7f9fca','#c4c8cd']
    context_keys=['matched','unassignedEligibleOverlap','outsideEligibilityOnly','noAnnotatedOverlap']
    context_labels=['Matched eligible','Unassigned eligible overlap','Ineligible annotation overlap','No annotation overlap']
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    selected=summary['selectedPolicy']['id'];handles=[];legend=[]
    for row,domain in enumerate(('clear','winter')):
        data=summary['validation'][domain]
        for j,method in enumerate(('raw',selected)):
            bins=data[method]['rangeBins'];values=[b['matched']/b['eligible'] if b['eligible'] else float('nan') for b in bins]
            bars=axes[row,0].bar([i+j*.34 for i in range(3)],values,width=.34,color=colours[j])
            if row==0:handles.append(bars);legend.append('Raw hypotheses' if j==0 else 'Selected hypotheses')
        axes[row,0].set_xticks([.17,1.17,2.17],['2–15','15–30','30–45']);axes[row,0].set_ylim(0,1)
        axes[row,0].set_title(domain+' · matched / eligible cuboids');axes[row,0].set_xlabel('Horizontal centre range (m)')
        bottom=[0.,0.]
        for key,colour,label in zip(context_keys,context_colours,context_labels):
            values=[data[m]['context'][key]/data[m]['frames'] for m in ('raw',selected)]
            bars=axes[row,1].bar(['Raw','Selected'],values,bottom=bottom,color=colour)
            bottom=[b+v for b,v in zip(bottom,values)]
            if row==0:handles.append(bars);legend.append(label)
        axes[row,1].set_title(domain+' · geometric hypotheses / frame')
    fig.suptitle('CADC component selection · 32 new validation scans · '+selected+'\nFrozen after 64 development scans; planar overlap context is not semantic precision',fontsize=12)
    fig.legend(handles,legend,loc='outside lower center',ncol=3,fontsize=9)
    fig.savefig(path,dpi=150);plt.close(fig)

def merge(args):
    protocol=read(args.protocol);reports=[read(p) for p in args.reports];selection=read(args.selection);freeze=read(args.freeze)
    source=os.environ['RUSTDRIVING_SOURCE_SHA'];policy=validate_selection(selection,protocol,source);identity=policy['id']
    require(freeze['sourceCommit']==source and freeze['selectionSha256']==sha(args.selection),'frozen selection identity differs')
    require(len(reports)==6 and all(r['sourceCommit']==source and r['protocolSha256']==sha(args.protocol) for r in reports),'six-shard source/protocol differs')
    for r in reports:
        require(r['sourceSha256']==reports[0]['sourceSha256'],'source contents differ')
        for key in ('nativeExecutableSha256','rawExecutableSha256','baselineExecutableSha256'):require(r[key]==reports[0][key],'executable identities differ')
        if r['phase']=='validation':require(r['selectionSha256']==sha(args.selection) and r['freeze']==freeze and r['policyIds']==[identity],'validation policy was not frozen')
    for path,h in reports[0]['sourceSha256'].items():require(sha(ROOT/path)==h,'tested source changed: '+path)
    frames=[f for r in reports for f in r['frames']]
    expected={d+'-'+str(p)+'-'+str(f).zfill(10) for p in range(6) for d in ('clear','winter') for f in protocol['frames']}
    require(len(frames)==96 and {f['id'] for f in frames}==expected,'96 registered scans incomplete')
    development=[f for f in frames if f['phase']=='development'];validation=[f for f in frames if f['phase']=='validation']
    require(len(development)==64 and len(validation)==32 and sum(r['audits']['previousFramesReproduced'] for r in reports)==64,'phase partition or previous replay differs')
    dev=aggregate(development,[p['id'] for p in protocol['policies']]);require(dev==selection['developmentSummary'],'frozen development summary differs')
    actual_inputs=sorted((f['id'],f['pair'],f['inputPointSha256']) for f in development)
    frozen_inputs=sorted((f['id'],f['pair'],f['pointSha256']) for f in selection['developmentInputs'])
    require(actual_inputs==frozen_inputs,'frozen development input identities differ')
    valid=aggregate(validation,[identity]);constraints=retention_constraints(valid,identity,protocol['qualification']['minimum_validation_retention'])
    ratios={d:valid[d][identity]['outputs']/valid[d]['raw']['outputs'] if valid[d]['raw']['outputs'] else None for d in ('clear','winter')}
    qualified=all(c['passes'] for c in constraints) and all(v is not None and v<=protocol['qualification']['maximum_validation_output_ratio'] for v in ratios.values())
    summary={'schema_version':1,'sourceCommit':source,'rawSourceCommit':protocol['raw_source'],'protocolSha256':sha(args.protocol),
        'frames':96,'developmentFrames':64,'validationFrames':32,'selectedPolicy':policy,'freeze':freeze,
        'development':dev,'validation':valid,'validationRetention':constraints,'validationOutputRatio':ratios,'qualifiedExperimental':qualified,
        'interpretation':'Hypothesis count/association retention experiment, not semantic precision, calibrated uncertainty, weather causality or driving qualification. Source returns and raw components are retained. Missing annotation overlap is not proof of a false object. Validation uses additional sequences from the same city/dates, not an official held-out split.'}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    raw={'sourceCommit':source,'frames':frames,'inputs':[i for r in reports for i in r['inputs']],'audits':[r['audits'] for r in reports]}
    with (args.output/'report.json.gz').open('wb') as stream:
        with gzip.GzipFile(fileobj=stream,filename='',mode='wb',mtime=0) as writer:writer.write(json.dumps(raw,separators=(',',':'),allow_nan=False).encode())
    (args.output/'selection.json').write_bytes(args.selection.read_bytes());plot(summary,args.output/'overview.png')
    provenance={'sourceCommit':source,'sourceSha256':reports[0]['sourceSha256'],'rawSourceCommit':protocol['raw_source'],
        'protocolSha256':sha(args.protocol),'freeze':freeze,
        'nativeExecutableSha256':reports[0]['nativeExecutableSha256'],'rawExecutableSha256':reports[0]['rawExecutableSha256'],
        'baselineExecutableSha256':reports[0]['baselineExecutableSha256'],'selectionInputReports':[r['selectionInputSha256'] for r in reports],
        'selectionOutputReports':[r['selectionReportSha256'] for r in reports],
        'dataLicense':'Original CADC CC BY-NC 4.0; consult CADC+ source terms. No raw clouds, annotations or spatial detector outputs redistributed.',
        'files':{p.name:sha(p) for p in args.output.iterdir() if p.is_file()}}
    (args.output/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print('SELECTION VALIDATION',json.dumps({'selectedPolicy':policy,'qualifiedExperimental':qualified,'validationRetention':constraints,'validationOutputRatio':ratios,'validation':valid}),flush=True)
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('prepare','audit','choose','merge'))
    for name in ('protocol','data','raw','raw-audit','input','native','binary','selection','freeze','output'):
        parser.add_argument('--'+name,type=Path,required=name in ('protocol','output'))
    parser.add_argument('--phase',choices=('development','validation'));parser.add_argument('--reports',type=Path,nargs='+')
    args=parser.parse_args();require(not args.output.exists(),'destination must be new')
    if args.command=='prepare':
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(prepare(args),separators=(',',':'))+'\n')
    elif args.command=='choose':
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(choose(args),indent=2,allow_nan=False)+'\n')
        selected=read(args.output)['selected'];print('SELECTION CHOSEN',json.dumps(selected),flush=True)
    else:
        args.output.mkdir(parents=True)
        if args.command=='audit':
            result=audit(args);(args.output/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
            print('SELECTION AUDITED',json.dumps({'phase':args.phase,'frames':len(result['frames']),'summary':aggregate(result['frames'],result['policyIds'])}),flush=True)
        else:merge(args)
if __name__=='__main__':main()
