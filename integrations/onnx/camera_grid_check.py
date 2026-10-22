#!/usr/bin/env python3
"""Bounded view inventory, independent grid merging and fixed BDD quality."""
import argparse
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import re
import subprocess
from types import SimpleNamespace
import numpy as np
from PIL import Image
import camera_linear_check as linear
prior=linear.prior
HERE,ROOT=linear.HERE,linear.ROOT
require,digest,load,write,fresh=linear.require,linear.digest,linear.load,linear.write,linear.fresh
native=linear.native
NEW=['integrations/onnx/src/camera_grid.rs','integrations/onnx/src/camera_grid_parallel.rs',
 'integrations/onnx/src/camera_grid_detector.rs','integrations/onnx/src/bin/rustdriving-camera-grid.rs',
 'integrations/onnx/profile-grid.json']
iou32=prior.nano.tiles.iou32

def expected():return dict(prior.expected(),**{n:digest(ROOT/n) for n in NEW})

def rectangles(width,height):
    require(type(width) is int and type(height) is int and width>0 and height>0 and width*height<=16000000,'bounded grid source')
    result=prior.nano.tiles.rectangles(width,height)
    side=min(width,height,416)
    def positions(dimension):
        gap=dimension-side;steps=math.ceil(gap/max(1,math.floor(side*.75)))
        require(steps+1<=16,'bounded grid axis')
        return [i*gap//steps for i in range(steps+1)] if steps else [0]
    xs,ys=positions(width),positions(height)
    require(len(xs)*len(ys)<=16,'bounded complete grid')
    for y in ys:
        for x in xs:
            r=dict(x=x,y=y,width=side,height=side)
            if r not in result:result.append(r)
    require(len(result)<=16,'bounded combined inventory')
    return result

def merge(fixture):
    width, height = fixture['width'], fixture['height']
    require([v['rectangle'] for v in fixture['views']] == rectangles(width, height), 'view inventory')
    candidates = []
    stats = dict(input_boxes=0, interior_edge_rejected=0, suppressed=0,
                 comparisons=0, output_cap_reached=False)
    for view, row in enumerate(fixture['views']):
        rect = row['rectangle']
        for rank, detection in enumerate(row['detections']):
            stats['input_boxes'] += 1
            l, t, r, b = np.asarray(detection['bbox'], dtype=np.float32)
            if ((rect['x'] > 0 and l <= 1) or (rect['y'] > 0 and t <= 1)
                    or (rect['x']+rect['width'] < width and r >= rect['width']-1)
                    or (rect['y']+rect['height'] < height and b >= rect['height']-1)):
                stats['interior_edge_rejected'] += 1
                continue
            box = np.asarray([l, t, r, b], dtype=np.float32)+np.asarray(
                [rect['x'], rect['y'], rect['x'], rect['y']], dtype=np.float32)
            candidates.append((dict(class_index=detection['class_index'],
                                    confidence=float(np.float32(detection['confidence'])),
                                    bbox=box.astype(float).tolist()), dict(view=view, rank=rank)))
    candidates.sort(key=lambda row: (-row[0]['confidence'], row[0]['class_index'],
                                    row[0]['bbox'][0], row[1]['view'], row[1]['rank']))
    kept, origins = [], []
    for candidate, origin in candidates:
        suppressed = False
        for other in kept:
            stats['comparisons'] += 1
            if (candidate['class_index'] == other['class_index']
                    and iou32(candidate['bbox'], other['bbox']) > np.float32(.45)):
                suppressed = True
                break
        if suppressed:
            stats['suppressed'] += 1
        else:
            kept.append(candidate)
            origins.append(origin)
            if len(kept) == 300:
                stats['output_cap_reached'] = True
                break
    require(stats['comparisons'] <= 1440000, 'comparison bound')
    return dict(detections=kept, detection_origins=origins, merge=stats)

GRID=SimpleNamespace(merge=merge,iou32=iou32,np=np)

def oracle(fixture):return prior.nano.consistency.oracle(fixture,GRID)

def records(data):
    p=load(HERE/'profile-grid.json');old=load(HERE/'profile-linear.json')
    require(all(p[k]==old[k] for k in ('class_mapping','unscored_category_reason','quality_gates','confidence_threshold','nms_iou_threshold','match_iou_threshold')),'unchanged scoring scope and gates')
    rows=linear.records(ROOT/'artifacts/camera-linear-v1/data')
    for row in rows:row['partition']='viewed_development'
    pins=load(data/'inputs.json')
    for item in p['images']:
        names=[Path(item[k]['path']).name for k in ('image','annotation')]
        for kind,name in zip(('image','annotation'),names):
            body=(data/name).read_bytes();pin=item[kind]
            require(len(body)==pin['size'] and hashlib.sha1(b'blob '+str(len(body)).encode()+b'\0'+body).hexdigest()==pin['sha'],'original grid-sample Git blob')
            require(digest(data/name)==pins[name]['sha256'],'input receipt')
        rows.append(dict(id=item['id'],partition='fresh_sample',original_split='val',path=data/names[0],annotation=data/names[1],image_sha256=pins[names[0]]['sha256'],annotation_sha256=pins[names[1]]['sha256']))
    require(len(rows)==len({r['id'] for r in rows})==40,'all forty fixed inputs')
    return rows

def fixture(width=1280,height=720):
    return dict(width=width,height=height,views=[dict(rectangle=r,detections=[]) for r in rectangles(width,height)])

def controls(args):
    rows=records(args.data);out=fresh(args.output);cases=[]
    inputs=fresh(out/'inputs')
    for width,height in ((1280,720),(720,1280),(641,640),(320,240),(416,416),(900,900)):
        f=fixture(width,height);cases.append((f'{width}x{height}-empty',f))
    f=fixture()
    for view,row in enumerate(f['views']):
        r=row['rectangle'];width,height=r['width'],r['height']
        row['detections']=[dict(class_index=2,confidence=.8,bbox=[40.,50.,100.,150.]),dict(class_index=0,confidence=.7,bbox=[1.,20.,35.,80.])]
    cases.append(('interior-edges-and-global-coordinates',f))
    f=fixture()
    for view,kind,score in ((0,2,.7),(1,2,.8),(4,5,.9)):
        r=f['views'][view]['rectangle'];f['views'][view]['detections']=[dict(class_index=kind,confidence=score,bbox=[100.-r['x'],100.-r['y'],200.-r['x'],200.-r['y']])]
    cases.append(('class-ambiguity-preserved-or-full-image-resolved',f))
    for index,(name,f) in enumerate(cases):
        path=inputs/f'{index}.json';write(path,f)
        actual=native(args.binary,['--merge-fixture',path],out/f'{index}-merge.json');prior.nano.tiles.same(merge(f),actual)
        actual=native(args.binary,['--reconcile-fixture',path],out/f'{index}-consistency.json');prior.nano.tiles.same(oracle(f),actual)
    bad=[]
    complete=fixture()
    for name,change in (
        ('missing-view',lambda f:f['views'].pop()),
        ('wrong-order',lambda f:f['views'].reverse()),
        ('extra-view',lambda f:f['views'].append(copy.deepcopy(f['views'][0]))),
        ('bad-box',lambda f:f['views'][0]['detections'].append(dict(class_index=2,confidence=.9,bbox=[0.,0.,2000.,2000.]))),
        ('low-confidence',lambda f:f['views'][0]['detections'].append(dict(class_index=2,confidence=.2,bbox=[1.,1.,10.,10.]))),
        ('invalid-class',lambda f:f['views'][0]['detections'].append(dict(class_index=80,confidence=.9,bbox=[1.,1.,10.,10.]))),
        ('pixel-budget',lambda f:f.update(width=5000,height=5000)),
        ('view-budget',lambda f:f.update(width=1920,height=1080)),
        ('unknown-field',lambda f:f.update(unknown=True))):
        f=copy.deepcopy(complete);change(f);path=inputs/(name+'.json');write(path,f)
        receipt=native(args.binary,['--merge-fixture',path],out/(name+'-unexpected.json'),False);bad.append(dict(name=name,**receipt))
    keep=out/'preserve.txt';keep.write_text('preserve\n')
    r=subprocess.run([str(args.binary),'--merge-fixture',str(inputs/'0.json'),'--output',str(keep)],text=True,capture_output=True)
    require(r.returncode==2 and keep.read_text()=='preserve\n','preserve complete output')
    pixels={}
    for row in rows:
        path=out/(row['id']+'-local-pixel-export.json')
        report=native(args.pixels_binary,['--preprocess',row['path'],'--input-size',416,'--include-pixels'],path)
        require(report['compiled_sources']==linear.expected() and report['source_image_sha256']==row['image_sha256'] and report['model_read'] is False and report['inference_run'] is False,'frozen model-free pixel reader')
        rgb=np.asarray(Image.open(io.BytesIO(bytes.fromhex(report['decoded_rgb_png_hex']))).convert('RGB'))
        require(hashlib.sha256(rgb.tobytes()).hexdigest()==report['rgb_sha256'],'decoded original RGB')
        inventory=native(args.binary,['--inspect-image',row['path']],out/(row['id']+'-inventory.json'))
        require(inventory['compiled_sources']==expected() and inventory['views']==rectangles(rgb.shape[1],rgb.shape[0]) and inventory['source_image_sha256']==row['image_sha256'],'independent complete grid')
        require(inventory['model_read'] is False and inventory['inference_run'] is False and inventory['control_permission'] is False,'inventory without inference')
        crop=[]
        for rect in inventory['views']:
            x,y,w,h=[rect[k] for k in ('x','y','width','height')]
            values=np.ascontiguousarray(rgb[y:y+h,x:x+w])
            crop.append(dict(rectangle=rect,rgb_sha256=hashlib.sha256(values.tobytes()).hexdigest()))
        pixels[row['id']]=dict(source_image_sha256=row['image_sha256'],views=crop,local_pixel_export_sha256=digest(path))
    summary=dict(schema_version=1,checker_sha256=digest(__file__),executable_sha256=digest(args.binary),compiled_sources=expected(),
        baseline_executable_sha256=digest(args.baseline),pixel_executable_sha256=digest(args.pixels_binary),valid_fixtures=len(cases),invalid_fixtures=bad,
        complete_input_pixels=pixels,existing_output_preserved=True,model_forwards=0,pass_controls=True)
    write(out/'summary.json',summary);print(json.dumps(dict(pass_controls=True,valid_fixtures=len(cases),invalid_fixtures=len(bad),pixel_images=len(pixels),model_forwards=0)),flush=True)

def audit_image(actual,row,pixels,grid=True,reference=False):
    sources=expected() if grid else prior.expected()
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

def same_views(a,b,prefix=False):
    av,bv=a['base_prediction']['views'],b['base_prediction']['views']
    if prefix:av=av[:4];bv=bv[:4]
    require(len(av)==len(bv),'compared view inventory')
    for x,y in zip(av,bv):
        require(x['rectangle']==y['rectangle'] and x['rgb_sha256']==y['rgb_sha256'],'current exact original crop')
        prior.compare(x['prediction']['detections'],y['prediction']['detections'],'every class/box/confidence f32 bit')
    if not prefix:
        for key in ('detections','detection_origins','merge'):prior.compare(a['base_prediction'][key],b['base_prediction'][key],'current global merge')
        prior.compare(a['result'],b['result'],'current consistency')

def audit_saved(out,rows,control,annotation=None):
    reports={}
    for repeat in (0,1):
        for mode in ('baseline','grid'):
            reports[repeat,mode]={}
            for row in rows:
                report=load(out/f'pass-{repeat}'/mode/(row['id']+'.json'));audit_image(report,row,control['complete_input_pixels'][row['id']],grid=mode=='grid');reports[repeat,mode][row['id']]=report
    for row in rows:
        for mode in ('baseline','grid'):same_views(reports[0,mode][row['id']],reports[1,mode][row['id']])
        for repeat in (0,1):same_views(reports[repeat,'grid'][row['id']],reports[repeat,'baseline'][row['id']],True)
        ref=load(out/'original-graph'/(row['id']+'.json'));audit_image(ref,row,control['complete_input_pixels'][row['id']],reference=True);same_views(reports[0,'grid'][row['id']],ref)
    groups={};changes={};gates={}
    for group in ('viewed_development','fresh_sample'):
        selected=[r for r in rows if r['partition']==group]
        scores={mode:linear.score(selected,reports[0,mode],annotation) for mode in ('baseline','grid')};groups[group]=scores
        a,b=[scores[m] for m in ('baseline','grid')]
        gates[group]=dict(precision_no_regression=b['overall']['precision']>=a['overall']['precision'],recall_strictly_improved=b['overall']['recall']>a['overall']['recall'],per_present_class_recall_no_regression=all(v['recall'] is None or b['per_class'][k]['recall']>=v['recall'] for k,v in a['per_class'].items()))
        identities=lambda m:{(r['image_id'],x['reference_id']) for r in scores[m]['images'] for x in r['matches']}
        a_ids,b_ids=identities('baseline'),identities('grid');changes[group]=dict(retained=len(a_ids&b_ids),gained=[list(x) for x in sorted(b_ids-a_ids)],lost=[list(x) for x in sorted(a_ids-b_ids)])
    return dict(schema_version=1,protocol_sha256=digest(HERE/'profile-grid.json'),groups=groups,relative_gates=gates,reference_changes=changes,
       adopt_candidate=all(all(x.values()) for x in gates.values()) and groups['fresh_sample']['grid']['quality_gates']['passed'],
       execution=dict(native_image_runs=200,model_forwards=1760,baseline_forwards=320,grid_forwards=960,current_original_graph_forwards=480,repeat_per_view_pairs=640,original_graph_per_view_pairs=480,original_prefix_per_view_pairs=320),
       new_latency_or_availability_claim=False,control_permission=False)

def run(args):
    c=load(args.controls/'summary.json');require(c['checker_sha256']==digest(__file__) and c['executable_sha256']==digest(args.binary) and c['compiled_sources']==expected() and c['pass_controls'],'matching complete controls')
    require(c['baseline_executable_sha256']==digest(args.baseline),'same current baseline binary')
    rows=records(args.data);out=fresh(args.output)
    before=dict(checker_sha256=digest(__file__),fetcher_sha256=digest(HERE/'camera_grid_fetch.py'),compiled_sources=expected(),baseline_sources=prior.expected(),
        executable_sha256=digest(args.binary),baseline_executable_sha256=digest(args.baseline),control_summary_sha256=digest(args.controls/'summary.json'),
        input_rows=[{k:str(v) if isinstance(v,Path) else v for k,v in r.items()} for r in rows],protocol_sha256=digest(HERE/'profile-grid.json'),
        prior_model_forwards_this_experiment=0,warmup_model_forwards=0,numeric_annotations_parsed_by_this_process=False)
    write(out/'before-inference.json',before)
    for repeat in (0,1):
        for mode in ('baseline','grid'):fresh(out/f'pass-{repeat}'/mode)
        for i,row in enumerate(rows):
            order=('grid','baseline') if (i+repeat)%2==0 else ('baseline','grid')
            for mode in order:native(args.binary if mode=='grid' else args.baseline,['--model',args.model,'--image',row['path']],out/f'pass-{repeat}'/mode/(row['id']+'.json'))
            print(f"pass {repeat} {row['partition']} {row['id']}: both inventories saved",flush=True)
    fresh(out/'original-graph')
    for row in rows:native(args.binary,['--model',args.model,'--reference-image',row['path']],out/'original-graph'/(row['id']+'.json'))
    write(out/'before-label-scoring.json',dict(numeric_annotations_parsed_by_this_process=False,native_report_sha256={str(p.relative_to(out)):digest(p) for p in sorted(out.rglob('*.json')) if p.name!='before-inference.json'}))
    summary=audit_saved(out,rows,c)
    require(before['checker_sha256']==digest(__file__) and before['compiled_sources']==expected() and before['executable_sha256']==digest(args.binary) and before['baseline_executable_sha256']==digest(args.baseline) and before['fetcher_sha256']==digest(HERE/'camera_grid_fetch.py'),'source/executables unchanged through evaluation')
    records(args.data);write(out/'summary.json',summary)
    print(json.dumps({g:{m:r['overall'] for m,r in v.items()} for g,v in summary['groups'].items()}),flush=True)
    print(json.dumps(dict(adopt_candidate=summary['adopt_candidate'],model_forwards=1760)),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=('controls','run','audit'))
    p.add_argument('--binary',type=Path,default=ROOT/'integrations/onnx/target/release/rustdriving-camera-grid');p.add_argument('--baseline',type=Path,default=ROOT/'integrations/onnx/target/release/rustdriving-camera-depthwise')
    p.add_argument('--pixels-binary',type=Path,default=ROOT/'integrations/onnx/target/release/rustdriving-camera-linear');p.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model/yolox_nano.onnx')
    p.add_argument('--data',type=Path,default=ROOT/'artifacts/camera-grid-v1/data');p.add_argument('--controls',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.binary=a.binary.resolve();a.baseline=a.baseline.resolve();a.pixels_binary=a.pixels_binary.resolve()
    if a.command=='controls':controls(a)
    elif a.command=='run':require(a.controls is not None,'controls required');run(a)
    else:require(audit_saved(a.output,records(a.data),load(a.controls/'summary.json'))==load(a.output/'summary.json'),'saved audit');print(json.dumps(dict(pass_audit=True,model_execution_this_check=False)))

if __name__=='__main__':main()
