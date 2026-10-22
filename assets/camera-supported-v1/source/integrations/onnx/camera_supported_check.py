#!/usr/bin/env python3
"""Independent coarse-preserving support and fixed fresh-image evaluation."""
import argparse
import copy
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image
import camera_grid_check as grid
linear=grid.linear
prior=grid.prior
HERE,ROOT=grid.HERE,grid.ROOT
require,digest,load,write,fresh=grid.require,grid.digest,grid.load,grid.write,grid.fresh
native=grid.native
GRID=grid.GRID
NEW=['integrations/onnx/src/camera_supported.rs','integrations/onnx/src/camera_supported_coarse.rs',
 'integrations/onnx/src/camera_supported_coarse_context.rs','integrations/onnx/src/bin/rustdriving-camera-supported.rs',
 'integrations/onnx/profile-supported.json']
MODES=('first','repeat','reference')

def expected():return dict(grid.expected(),**{n:digest(ROOT/n) for n in NEW})

def read_raw(path):
    path=Path(path);body=path.read_bytes()
    return gzip.decompress(body) if path.suffix=='.gz' else body

def make_fixture(parent):
    b=parent['base_prediction']
    return dict(width=b['width'],height=b['height'],views=[dict(rectangle=v['rectangle'],detections=v['prediction']['detections']) for v in b['views']])

def original_parent(identity,mode):
    folder={'first':'pass-0/grid','repeat':'pass-1/grid','reference':'original-graph'}[mode]
    return ROOT/'assets/camera-grid-v1/validation'/folder/(identity+'.json.gz')

def records(data):
    rows=grid.records(ROOT/'artifacts/camera-grid-v1/data')
    for r in rows:r['partition']='viewed_development'
    p=load(HERE/'profile-supported.json');old=load(HERE/'profile-grid.json')
    require(all(p[k]==old[k] for k in ('class_mapping','unscored_category_reason','quality_gates','confidence_threshold','nms_iou_threshold','match_iou_threshold','model_sha256')),'unchanged scoring/model contract')
    pins=load(data/'inputs.json')
    for image in p['images']:
        names=[Path(image[k]['path']).name for k in ('image','annotation')]
        for k,n in zip(('image','annotation'),names):
            body=(data/n).read_bytes();pin=image[k]
            require(len(body)==pin['size'] and hashlib.sha1(b'blob '+str(len(body)).encode()+b'\0'+body).hexdigest()==pin['sha'] and digest(data/n)==pins[n]['sha256'],'original fixed Git inputs')
        rows.append(dict(id=image['id'],partition='fresh_sample',original_split='val',path=data/names[0],annotation=data/names[1],image_sha256=pins[names[0]]['sha256'],annotation_sha256=pins[names[1]]['sha256']))
    require(len(rows)==len({r['id'] for r in rows})==56,'complete forty viewed plus sixteen new')
    return rows

def supported_oracle(f):
    naive=grid.oracle(f)
    prefix=len(prior.nano.tiles.rectangles(f['width'],f['height']))
    coarse_f=dict(f,views=f['views'][:prefix]);coarse=prior.nano.consistency.oracle(coarse_f,prior.nano.tiles)
    proposals=[];stats=dict(supplemental_input_boxes=0,interior_edge_rejected=0,proposals=0,added=0,iou_comparisons=0)
    for view,row in enumerate(f['views'][prefix:],prefix):
        rect=row['rectangle']
        for rank,d in enumerate(row['detections']):
            stats['supplemental_input_boxes']+=1
            l,t,r,b=np.asarray(d['bbox'],dtype=np.float32)
            if ((rect['x']>0 and l<=1) or (rect['y']>0 and t<=1)
                or (rect['x']+rect['width']<f['width'] and r>=rect['width']-1)
                or (rect['y']+rect['height']<f['height'] and b>=rect['height']-1)):
                stats['interior_edge_rejected']+=1;continue
            bbox=(np.asarray([l,t,r,b],dtype=np.float32)+np.asarray([rect['x'],rect['y'],rect['x'],rect['y']],dtype=np.float32)).astype(float).tolist()
            proposals.append(dict(detection=dict(class_index=d['class_index'],confidence=float(np.float32(d['confidence'])),bbox=bbox),origin=dict(view=view,rank=rank)))
    proposals.sort(key=lambda p:(-p['detection']['confidence'],p['detection']['class_index'],p['detection']['bbox'][0],p['origin']['view'],p['origin']['rank']))
    stats['proposals']=len(proposals)
    def overlap(a,b):
        stats['iou_comparisons']+=1;require(stats['iou_comparisons']<=27000000,'support work bound')
        return grid.iou32(a['bbox'],b['bbox'])>np.float32(.45)
    additions=[];origins=[];decisions=[]
    for c in proposals:
        d=c['detection'];entry=dict(origin=c['origin'],witnesses=[],conflicting_vehicle_label=False,coarse_rank=None,added_rank=None,reason='insufficient_support')
        for rank,b in enumerate(coarse['detections']):
            if overlap(d,b):entry['coarse_rank']=rank;break
        if entry['coarse_rank'] is not None:
            entry['reason']='coarse_overlap';decisions.append(entry);continue
        best={}
        for o in proposals:
            other=o['detection'];same=d['class_index']==other['class_index']
            vehicles=d['class_index'] in (2,5,7) and other['class_index'] in (2,5,7)
            if not (same or vehicles) or not overlap(d,other):continue
            if not same:entry['conflicting_vehicle_label']=True;continue
            view=o['origin']['view'];old=best.get(view)
            if old is None or (-other['confidence'],o['origin']['rank'])<(-old['detection']['confidence'],old['origin']['rank']):best[view]=o
        entry['witnesses']=[best[v]['origin'] for v in sorted(best)]
        if entry['conflicting_vehicle_label']:entry['reason']='vehicle_ambiguity'
        elif len(entry['witnesses'])>=2:
            suppressed=False
            for b in additions:
                if b['class_index']==d['class_index'] and overlap(d,b):suppressed=True;break
            if suppressed:entry['reason']='suppressed_addition'
            elif len(coarse['detections'])+len(additions)>=300:entry['reason']='output_cap'
            else:
                entry.update(reason='added',added_rank=len(additions));additions.append(d);origins.append(c['origin'])
        decisions.append(entry)
    stats['added']=len(additions)
    return dict(naive=naive,coarse=coarse,detections=coarse['detections']+additions,detection_origins=coarse['detection_origins']+origins,decisions=decisions,stats=stats)

def audit_source(actual,row,pixels,pins,reference=False):
    grid=True
    sources=pins
    algorithm='yolox_nano_grid_image_analysis_v1' if grid else 'yolox_nano_depthwise_parallel_image_analysis_v1'
    require(actual['algorithm']==algorithm and actual['schema_version']==1 and actual['compiled_sources']==sources,'own image producer')
    require(actual['control_permission'] is False and actual['source_image_sha256']==row['image_sha256'],'image identity and no control')
    if grid:require(actual['original_graph_reference'] is reference and actual['profile_sha256']==digest(HERE/'profile-grid.json'),'grid reference/profile identity')
    else:require(actual['graph_optimization']=='validated_resize2x_and_rowwise_depthwise3x3','frozen baseline')
    base=actual['base_prediction'];n=12 if grid else 4;chosen=pixels['views'][:n]
    require((base['width'],base['height'])==(1280,720) and len(base['views'])==n,'complete view inventory')
    require(base['model_sha256']==prior.MODEL_SHA and base['source_image_sha256']==row['image_sha256'] and base['compiled_sources']==sources,'base source/model receipt')
    require(base['algorithm']==('pinned_yolox_nano_grid_fixed_views_v1' if grid else 'pinned_yolox_nano_depthwise_parallel_fixed_views_v1'),'base variant identity')
    require(prior.nano.exact_f32(base['score_threshold'],.3) and prior.nano.exact_f32(base['nms_iou_threshold'],.45),'original merge thresholds')
    for view,oracle_view in zip(base['views'],chosen):
        require(view['rectangle']==oracle_view['rectangle'] and view['rgb_sha256']==oracle_view['rgb_sha256'],'every independently cropped pixel')
        p=view['prediction'];r=view['rectangle']
        alg=('pinned_nano_grid_original_plan_triangle_v1' if reference else 'pinned_nano_grid_depthwise_triangle_v1') if grid else 'pinned_yolox_nano_depthwise_cpu_image_triangle_argmax_classwise'
        profile='yolox-nano-grid-v1' if grid else 'yolox-nano-depthwise-v1'
        require(p['algorithm']==alg and p['model_profile']==profile and p['compiled_sources']==sources,'individual inference producer')
        require(p['model_profile_sha256']==digest(HERE/('profile-grid.json' if grid else 'profile-depthwise.json')),'detector profile')
        require(p['model_sha256']==prior.MODEL_SHA and p['input_shape']==[1,3,416,416] and p['output_shape']==[1,3549,85],'fixed model tensors')
        require((p['width'],p['height'])==(r['width'],r['height']) and p['source_image_sha256'] is None,'local detection coordinates')
        require(prior.nano.exact_f32(p['score_threshold'],.3) and prior.nano.exact_f32(p['nms_iou_threshold'],.45),'unchanged detector thresholds')
        require(all(type(p[k]) in (float,int) and math.isfinite(p[k]) and p[k]>0 for k in ('model_load_seconds','inference_seconds')),'finite actual observations')
    f=dict(width=base['width'],height=base['height'],views=[dict(rectangle=v['rectangle'],detections=v['prediction']['detections']) for v in base['views']])
    support=GRID if grid else prior.nano.tiles
    prior.nano.tiles.same(support.merge(f),{k:base[k] for k in ('detections','detection_origins','merge')})
    prior.nano.tiles.same(prior.nano.consistency.oracle(f,support),actual['result'])
    ex=actual['execution'];require({k:v for k,v in ex.items() if k!='analysis_wall_s'}==dict(model_read=True,images_read=1,model_forwards=n,inference_run=True,view_workers=1 if reference else 4,shared_model_plans=1),'complete inference count and worker budget')
    require(math.isfinite(ex['analysis_wall_s']) and ex['analysis_wall_s']>=max(v['prediction']['inference_seconds'] for v in base['views']),'whole analysis interval')

def check_result(actual,f):
    expected_result=supported_oracle(f);prior.compare(expected_result,actual,'complete support/coarse/naive reconstruction')
    prefix=len(actual['coarse']['detections'])
    prior.compare(actual['coarse']['detections'],actual['detections'][:prefix],'coarse measurements are immutable prefix')
    prior.compare(actual['coarse']['detection_origins'],actual['detection_origins'][:prefix],'coarse lineage is immutable prefix')
    return expected_result

def check_fixture_report(report,path):
    require(report['schema_version']==1 and report['algorithm']=='supported_grid_fixture_v1' and report['compiled_sources']==expected(),'fixture producer/source closure')
    require(report['source_fixture_sha256']==digest(path) and report['model_read'] is False and report['images_read'] is False and report['model_forwards']==0 and report['control_permission'] is False,'fixture provenance without inference')
    return check_result(report['result'],load(path))

def check_image(report,row,pixels,reference=False):
    require(report['schema_version']==1 and report['algorithm']=='nano_supported_grid_image_v1' and report['compiled_sources']==expected(),'supported image producer')
    require(report['profile_sha256']==digest(HERE/'profile-supported.json') and report['source_image_sha256']==row['image_sha256'] and report['control_permission'] is False,'supported profile/image and no control')
    source=report['source_analysis'];audit_source(source,row,pixels,expected(),reference)
    f=make_fixture(source);expected_result=check_result(report['result'],f)
    prior.compare(expected_result['naive'],source['result'],'same internally measured naive result')
    return source

def controls(args):
    rows=records(args.data);out=fresh(args.output);inputs=fresh(out/'inputs');cases=[]
    for i in range(8):cases.append(json.loads(read_raw(ROOT/f'assets/camera-grid-v1/controls/inputs/{i}.json.gz')))
    def add(f,view,kind,score,box):
        r=f['views'][view]['rectangle'];f['views'][view]['detections'].append(dict(class_index=kind,confidence=score,bbox=[box[0]-r['x'],box[1]-r['y'],box[2]-r['x'],box[3]-r['y']]))
    for count in (1,2):
        f=grid.fixture();add(f,4,9,.8,[300.,100.,350.,150.]);add(f,4,9,.7,[300.,100.,350.,150.])
        if count==2:add(f,5,9,.6,[300.,100.,350.,150.])
        cases.append(f)
    f=grid.fixture()
    for view,kind,score in ((4,2,.9),(5,7,.8),(8,2,.7)):add(f,view,kind,score,[300.,330.,350.,380.])
    cases.append(f)
    for i,f in enumerate(cases):
        path=inputs/f'{i}.json';write(path,f);report=native(args.binary,['--fixture',path],out/f'{i}-actual.json');check_fixture_report(report,path)
    invalid=[]
    for source in sorted((ROOT/'assets/camera-grid-v1/controls/inputs').glob('*.json.gz')):
        if source.name.split('.')[0].isdigit():continue
        name=source.name.removesuffix('.gz');path=inputs/name;path.write_bytes(read_raw(source))
        receipt=native(args.binary,['--fixture',path],out/(name+'-unexpected.json'),False);invalid.append(dict(name=name,**receipt))
    preserve=out/'preserve.txt';preserve.write_text('preserve\n')
    result=subprocess.run([str(args.binary),'--fixture',str(inputs/'0.json'),'--output',str(preserve)],text=True,capture_output=True)
    require(result.returncode==2 and preserve.read_text()=='preserve\n','existing complete output preserved')
    pixels=copy.deepcopy(json.loads(read_raw(ROOT/'assets/camera-grid-v1/controls/summary.json.gz'))['complete_input_pixels'])
    for row in rows[40:]:
        path=out/(row['id']+'-local-pixel-export.json')
        r=native(args.pixels_binary,['--preprocess',row['path'],'--input-size',416,'--include-pixels'],path)
        require(r['compiled_sources']==linear.expected() and r['source_image_sha256']==row['image_sha256'] and r['model_read'] is False and r['inference_run'] is False,'frozen model-free pixel reader')
        rgb=np.asarray(Image.open(io.BytesIO(bytes.fromhex(r['decoded_rgb_png_hex']))).convert('RGB'))
        require(hashlib.sha256(rgb.tobytes()).hexdigest()==r['rgb_sha256'],'original decoded pixel hash')
        views=[]
        for rect in grid.rectangles(rgb.shape[1],rgb.shape[0]):
            x,y,w,h=[rect[k] for k in ('x','y','width','height')];raw=np.ascontiguousarray(rgb[y:y+h,x:x+w])
            views.append(dict(rectangle=rect,rgb_sha256=hashlib.sha256(raw.tobytes()).hexdigest()))
        pixels[row['id']]=dict(source_image_sha256=row['image_sha256'],views=views,local_pixel_export_sha256=digest(path))
    summary=dict(schema_version=1,checker_sha256=digest(__file__),executable_sha256=digest(args.binary),compiled_sources=expected(),pixel_executable_sha256=digest(args.pixels_binary),valid_fixtures=len(cases),invalid_fixtures=invalid,existing_output_preserved=True,complete_input_pixels=pixels,model_forwards=0,pass_controls=True)
    write(out/'summary.json',summary);print(json.dumps(dict(pass_controls=True,valid_fixtures=len(cases),invalid_fixtures=len(invalid),pixels=56,model_forwards=0)),flush=True)

def audit_saved(out,rows,control,annotation=None):
    measured={};supported={}
    for mode in MODES:
        measured[mode]={};supported[mode]={}
        for row in rows:
            if row['partition']=='viewed_development':
                parent=json.loads(read_raw(original_parent(row['id'],mode)));grid.audit_image(parent,row,control['complete_input_pixels'][row['id']],reference=mode=='reference')
                path=out/'fixtures'/mode/(row['id']+'.json');require(load(path)==make_fixture(parent),'fixture made from exact frozen native parent')
                r=load(out/'postprocessed'/mode/(row['id']+'.json'));check_fixture_report(r,path);result=r['result'];source=parent
            else:
                r=load(out/'fresh'/mode/(row['id']+'.json'));source=check_image(r,row,control['complete_input_pixels'][row['id']],mode=='reference');result=r['result']
            measured[mode][row['id']]=source;supported[mode][row['id']]=result
    for row in rows:
        for mode in ('repeat','reference'):
            grid.same_views(measured['first'][row['id']],measured[mode][row['id']])
            prior.compare(supported['first'][row['id']],supported[mode][row['id']],'every repeated/current-reference supported decision and measurement')
    groups={};relative={};changes={}
    for group in ('viewed_development','fresh_sample'):
        selected=[r for r in rows if r['partition']==group];policies={}
        for policy in ('coarse','naive','supported'):
            projections={r['id']:dict(result=supported['first'][r['id']] if policy=='supported' else supported['first'][r['id']][policy]) for r in selected}
            policies[policy]=linear.score(selected,projections,annotation)
        groups[group]=policies;a,b,n=[policies[k] for k in ('coarse','supported','naive')]
        relative[group]=dict(precision_no_coarse_regression=b['overall']['precision']>=a['overall']['precision'],precision_better_than_naive=b['overall']['precision']>n['overall']['precision'],recall_above_coarse=b['overall']['recall']>a['overall']['recall'],per_present_class_recall_no_regression=all(v['recall'] is None or b['per_class'][k]['recall']>=v['recall'] for k,v in a['per_class'].items()))
        ids=lambda p:{(r['image_id'],m['reference_id']) for r in policies[p]['images'] for m in r['matches']}
        old,new=ids('coarse'),ids('supported');changes[group]=dict(retained=len(old&new),gained=[list(x) for x in sorted(new-old)],lost=[list(x) for x in sorted(old-new)])
    return dict(schema_version=1,protocol_sha256=digest(HERE/'profile-supported.json'),groups=groups,relative_gates=relative,reference_changes=changes,
        adopt_candidate=all(all(v.values()) for v in relative.values()) and groups['fresh_sample']['supported']['quality_gates']['passed'],
        execution=dict(new_model_forwards=576,new_native_image_runs=48,reused_native_parent_reports=120,reused_parent_forwards_not_counted_as_new=1440,native_fixture_postprocessing_runs=120,postprocessing_model_forwards=0),new_latency_or_availability_claim=False,control_permission=False)

def run(args):
    control=load(args.controls/'summary.json');require(control['checker_sha256']==digest(__file__) and control['executable_sha256']==digest(args.binary) and control['compiled_sources']==expected() and control['pass_controls'],'matching complete controls')
    out=fresh(args.output);rows=records(args.data)
    parents={str(original_parent(r['id'],m).relative_to(ROOT)):digest(original_parent(r['id'],m)) for r in rows[:40] for m in MODES}
    before=dict(checker_sha256=digest(__file__),fetcher_sha256=digest(HERE/'camera_supported_fetch.py'),executable_sha256=digest(args.binary),compiled_sources=expected(),protocol_sha256=digest(HERE/'profile-supported.json'),control_summary_sha256=digest(args.controls/'summary.json'),input_rows=[{k:str(v) if isinstance(v,Path) else v for k,v in r.items()} for r in rows],frozen_parent_sha256=parents,prior_new_model_forwards=0,warmup_model_forwards=0,numeric_annotations_parsed_by_this_process=False)
    write(out/'before-inference.json',before)
    for mode in MODES:
        fresh(out/'fixtures'/mode);fresh(out/'postprocessed'/mode);fresh(out/'fresh'/mode)
    for mode in MODES:
        for row in rows[:40]:
            parent=json.loads(read_raw(original_parent(row['id'],mode)));grid.audit_image(parent,row,control['complete_input_pixels'][row['id']],reference=mode=='reference')
            path=out/'fixtures'/mode/(row['id']+'.json');write(path,make_fixture(parent));native(args.binary,['--fixture',path],out/'postprocessed'/mode/(row['id']+'.json'))
    for mode in MODES:
        for row in rows[40:]:
            native(args.binary,['--model',args.model,'--reference-image' if mode=='reference' else '--image',row['path']],out/'fresh'/mode/(row['id']+'.json'))
            print(f"fresh {mode} {row['id']}: all12 views saved",flush=True)
    write(out/'before-label-scoring.json',dict(numeric_annotations_parsed_by_this_process=False,output_sha256={str(p.relative_to(out)):digest(p) for p in sorted(out.rglob('*.json')) if p.name!='before-inference.json'}))
    result=audit_saved(out,rows,control)
    require(before['checker_sha256']==digest(__file__) and before['executable_sha256']==digest(args.binary) and before['compiled_sources']==expected() and before['fetcher_sha256']==digest(HERE/'camera_supported_fetch.py'),'source/executable unchanged throughout evaluation')
    require(all(digest(ROOT/p)==h for p,h in parents.items()),'frozen parents unchanged');records(args.data)
    write(out/'summary.json',result)
    print(json.dumps({g:{p:v['overall'] for p,v in group.items()} for g,group in result['groups'].items()}),flush=True)
    print(json.dumps(dict(adopt_candidate=result['adopt_candidate'],new_model_forwards=576)),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=('controls','run','audit'));p.add_argument('--output',type=Path,required=True);p.add_argument('--controls',type=Path)
    p.add_argument('--binary',type=Path,default=ROOT/'integrations/onnx/target/release/rustdriving-camera-supported');p.add_argument('--pixels-binary',type=Path,default=ROOT/'integrations/onnx/target/release/rustdriving-camera-linear');p.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model/yolox_nano.onnx');p.add_argument('--data',type=Path,default=ROOT/'artifacts/camera-supported-v1/data')
    a=p.parse_args();a.binary=a.binary.resolve();a.pixels_binary=a.pixels_binary.resolve()
    if a.command=='controls':controls(a)
    elif a.command=='run':require(a.controls is not None,'controls required');run(a)
    else:require(audit_saved(a.output,records(a.data),load(a.controls/'summary.json'))==load(a.output/'summary.json'),'saved audit');print(json.dumps(dict(pass_audit=True,model_execution_this_check=False)))

if __name__=='__main__':main()
