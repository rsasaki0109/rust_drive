#!/usr/bin/env python3
"""Independent S640 capacity and supported-view comparison on fixed inputs."""
import argparse,copy,gzip,hashlib,io,json,math,re,subprocess
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from PIL import Image
import camera_supported_check as previous
grid,linear,prior=previous.grid,previous.linear,previous.prior
HERE,ROOT=previous.HERE,previous.ROOT
require,digest,load,write,fresh=previous.require,previous.digest,previous.load,previous.write,previous.fresh
native=previous.native
iou32=grid.iou32
NEW=['integrations/onnx/src/camera_repeat_detector.rs','integrations/onnx/profile-repeat2x.json',
 'integrations/onnx/src/camera_capacity_grid.rs','integrations/onnx/src/camera_capacity_parallel.rs',
 'integrations/onnx/src/camera_capacity_detector.rs','integrations/onnx/src/camera_capacity_supported.rs',
 'integrations/onnx/src/bin/rustdriving-camera-capacity.rs','integrations/onnx/profile-capacity.json']
MODEL_SHA='c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063'
MODES=('first','repeat','reference')
def expected():return dict(previous.expected(),**{n:digest(ROOT/n) for n in NEW})

def rectangles(width,height):
    require(type(width) is int and type(height) is int and width>0 and height>0 and width*height<=16000000,'bounded grid source')
    result=prior.nano.tiles.rectangles(width,height)
    side=min(width,height,640)
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
def oracle(f):return prior.nano.consistency.oracle(f,GRID)

def supported_oracle(f):
    naive=oracle(f)
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
        return iou32(a['bbox'],b['bbox'])>np.float32(.45)
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

def fixture(width=1280,height=720):
    return dict(width=width,height=height,views=[dict(rectangle=r,detections=[]) for r in rectangles(width,height)])

def records(data):
    design=load(HERE/'profile-capacity.json');old=load(HERE/'bdd_protocol.json');baseline=load(HERE/'profile-linear.json')
    require(all(design[k]==baseline[k] for k in ('class_mapping','unscored_category_reason','quality_gates','confidence_threshold','nms_iou_threshold','match_iou_threshold')),'unchanged complete scoring scope')
    require(design['model_sha256']==MODEL_SHA and design['model_bytes']==35858002,'pinned S model')
    require(digest(ROOT/'artifacts/bdd-camera'/old['license']['file'])==old['license']['sha256'],'original research-data license')
    rows=[]
    for image in old['images']:
        rows.append(dict(id=image['id'],partition='viewed_development',original_split=image['original_split'],path=ROOT/'artifacts/bdd-camera'/image['image']['file'],annotation=ROOT/'artifacts/bdd-camera'/image['annotation']['file'],image_sha256=image['image']['sha256'],annotation_sha256=image['annotation']['sha256']))
    pins=load(data/'inputs.json')
    for image in design['images']:
        names=[Path(image[k]['path']).name for k in ('image','annotation')]
        for k,n in zip(('image','annotation'),names):
            body=(data/n).read_bytes();pin=image[k]
            require(len(body)==pin['size'] and hashlib.sha1(b'blob '+str(len(body)).encode()+b'\0'+body).hexdigest()==pin['sha'] and digest(data/n)==pins[n]['sha256'],'original fixed Git inputs')
        rows.append(dict(id=image['id'],partition='fresh_sample',original_split='val',path=data/names[0],annotation=data/names[1],image_sha256=pins[names[0]]['sha256'],annotation_sha256=pins[names[1]]['sha256']))
    require(len(rows)==len({r['id'] for r in rows})==24,'complete viewed8 plus fresh16')
    for r in rows:require(digest(r['path'])==r['image_sha256'] and digest(r['annotation'])==r['annotation_sha256'],'unchanged source bytes')
    return rows

def make_fixture(source):return previous.make_fixture(source)

def check_result(actual,f):
    answer=supported_oracle(f);prior.compare(answer,actual,'all coarse/naive/supported boxes, lineage, decisions and counters')
    n=len(answer['coarse']['detections']);prior.compare(answer['coarse']['detections'],actual['detections'][:n],'immutable coarse prefix')
    return answer

def check_source(actual,row,pixels,reference=False):
    pins=expected();profile=digest(HERE/'profile-capacity.json')
    require(actual['schema_version']==1 and actual['algorithm']=='yolox_s_capacity_grid_image_analysis_v1' and actual['compiled_sources']==pins,'S grid producer/source closure')
    require(actual['control_permission'] is False and actual['source_image_sha256']==row['image_sha256'] and actual['profile_sha256']==profile and actual['original_graph_reference'] is reference,'S identity and graph/reference profile')
    base=actual['base_prediction'];views=base['views']
    require(base['schema_version']==1 and base['algorithm']=='pinned_yolox_s_capacity_grid_fixed_views_v1' and base['compiled_sources']==pins,'S base producer')
    require((base['width'],base['height'])==(1280,720) and len(views)==10 and len(pixels['views'])==10,'complete ten view inventory')
    require(base['model_sha256']==MODEL_SHA and base['source_image_sha256']==row['image_sha256'],'S model and original image')
    require(prior.nano.exact_f32(base['score_threshold'],.3) and prior.nano.exact_f32(base['nms_iou_threshold'],.45),'merge thresholds')
    for view,pixel in zip(views,pixels['views']):
        r=view['rectangle'];p=view['prediction']
        require(r==pixel['rectangle'] and view['rgb_sha256']==pixel['rgb_sha256'],'every exact decoded crop pixel')
        require(p['schema_version']==1 and p['algorithm']==('pinned_s_capacity_original_plan_triangle_v1' if reference else 'pinned_s_capacity_repeat2x_triangle_v1') and p['model_profile']=='yolox-s-capacity-v1' and p['compiled_sources']==pins and p['model_profile_sha256']==profile,'internally executed S detector')
        require(p['model_sha256']==MODEL_SHA and p['input_shape']==[1,3,640,640] and p['output_shape']==[1,8400,85],'S fixed tensor contract')
        require((p['width'],p['height'])==(r['width'],r['height']) and p['source_image_sha256'] is None,'local source-pixel coordinates')
        require(prior.nano.exact_f32(p['score_threshold'],.3) and prior.nano.exact_f32(p['nms_iou_threshold'],.45),'original inference thresholds')
        require(all(type(p[k]) in (float,int) and math.isfinite(p[k]) and p[k]>0 for k in ('model_load_seconds','inference_seconds')),'finite real timing observations')
        require(len(p['detections'])<=300,'bounded local predictions')
        for d in p['detections']:
            require(type(d['class_index']) is int and 0<=d['class_index']<80 and type(d['confidence']) in (float,int) and math.isfinite(d['confidence']) and float(np.float32(.3))<=d['confidence']<=1 and linear.matcher.valid_box(d['bbox'],r['width'],r['height']),'valid local measurement')
    f=make_fixture(actual)
    prior.compare(merge(f),{k:base[k] for k in ('detections','detection_origins','merge')},'independent grid merge')
    prior.compare(oracle(f),actual['result'],'independent unchanged consistency')
    ex=actual['execution'];require({k:v for k,v in ex.items() if k!='analysis_wall_s'}==dict(model_read=True,images_read=1,model_forwards=10,inference_run=True,view_workers=1 if reference else 4,shared_model_plans=1),'complete ten forwards and bounded workers')
    require(type(ex['analysis_wall_s']) in (float,int) and math.isfinite(ex['analysis_wall_s']) and ex['analysis_wall_s']>=max(v['prediction']['inference_seconds'] for v in views),'whole measured analysis interval')

def check_image(r,row,pixels,reference=False):
    require(r['schema_version']==1 and r['algorithm']=='s_capacity_grid_image_v1' and r['compiled_sources']==expected(),'own S supported producer')
    require(r['profile_sha256']==digest(HERE/'profile-capacity.json') and r['source_image_sha256']==row['image_sha256'] and r['control_permission'] is False,'S profile/image and no control')
    source=r['source_analysis'];check_source(source,row,pixels,reference)
    answer=check_result(r['result'],make_fixture(source));prior.compare(answer['naive'],source['result'],'same measured naive analysis')
    return source

def check_fixture(r,path):
    require(r['schema_version']==1 and r['algorithm']=='s_capacity_grid_fixture_v1' and r['compiled_sources']==expected(),'fixture producer')
    require(r['source_fixture_sha256']==digest(path) and r['model_read'] is False and r['images_read'] is False and r['model_forwards']==0 and r['control_permission'] is False,'fixture is explicit model-free diagnostics')
    return check_result(r['result'],load(path))

def add(f,view,kind,score,box):
    r=f['views'][view]['rectangle'];f['views'][view]['detections'].append(dict(class_index=kind,confidence=score,bbox=[box[0]-r['x'],box[1]-r['y'],box[2]-r['x'],box[3]-r['y']]))

def controls(a):
    rows=records(a.data);out=fresh(a.output);inputs=fresh(out/'inputs')
    cases=[fixture(w,h) for w,h in ((1280,720),(720,1281),(641,640),(320,240),(640,640),(900,900))]
    for count in (1,2):
        f=fixture();add(f,4,9,.8,[400.,100.,450.,150.]);add(f,4,9,.7,[400.,100.,450.,150.])
        if count==2:add(f,5,9,.6,[400.,100.,450.,150.])
        cases.append(f)
    f=fixture()
    for view,kind,score in ((4,2,.9),(5,7,.8),(8,2,.7)):add(f,view,kind,score,[400.,330.,450.,380.])
    cases.append(f)
    f=fixture();add(f,0,2,.6,[400.,100.,450.,150.]);add(f,4,9,.9,[400.,100.,450.,150.]);add(f,5,9,.8,[400.,100.,450.,150.]);cases.append(f)
    f=fixture()
    for view in (4,5):
        add(f,view,0,.9,[400.,100.,450.,150.]);add(f,view,1,.8,[400.,100.,450.,150.])
    cases.append(f)
    for i,f in enumerate(cases):
        path=inputs/f'{i}.json';write(path,f);check_fixture(native(a.binary,['--fixture',path],out/f'{i}-actual.json'),path)
    bad=[]
    f=fixture();f['views'].pop();bad.append(f)
    f=fixture();f['views'][4],f['views'][5]=f['views'][5],f['views'][4];bad.append(f)
    for class_index,confidence,box in ((80,.8,[10.,10.,20.,20.]),(0,.2,[10.,10.,20.,20.]),(0,.8,[-1.,10.,20.,20.]),(0,.8,[10.,10.,10.,20.])):
        f=fixture();f['views'][0]['detections']=[dict(class_index=class_index,confidence=confidence,bbox=box)];bad.append(f)
    f=fixture();f['unexpected']=True;bad.append(f)
    bad.extend([dict(width=0,height=720,views=[]),dict(width=2560,height=1440,views=[])])
    failures=[]
    for i,f in enumerate(bad):
        path=inputs/f'invalid-{i}.json';write(path,f);failures.append(native(a.binary,['--fixture',path],out/f'invalid-{i}-unexpected.json',False))
    preserve=out/'preserve.txt';preserve.write_text('preserve\n');r=subprocess.run([str(a.binary),'--fixture',str(inputs/'0.json'),'--output',str(preserve)],text=True,capture_output=True)
    require(r.returncode==2 and preserve.read_text()=='preserve\n','no clobber of previous output')
    pixels={}
    for row in rows:
        path=out/(row['id']+'-local-pixel-export.json')
        r=native(a.pixels_binary,['--preprocess',row['path'],'--input-size',640,'--include-pixels'],path)
        require(r['compiled_sources']==linear.expected() and r['source_image_sha256']==row['image_sha256'] and r['model_read'] is False and r['inference_run'] is False,'frozen model-free original pixel reader')
        rgb=np.asarray(Image.open(io.BytesIO(bytes.fromhex(r['decoded_rgb_png_hex']))).convert('RGB'))
        require(hashlib.sha256(rgb.tobytes()).hexdigest()==r['rgb_sha256'],'original RGB hash')
        views=[]
        for rect in rectangles(rgb.shape[1],rgb.shape[0]):
            x,y,w,h=[rect[k] for k in ('x','y','width','height')]
            views.append(dict(rectangle=rect,rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb[y:y+h,x:x+w]).tobytes()).hexdigest()))
        pixels[row['id']]=dict(source_image_sha256=row['image_sha256'],views=views,local_pixel_export_sha256=digest(path))
    summary=dict(schema_version=1,checker_sha256=digest(__file__),executable_sha256=digest(a.binary),nano_executable_sha256=digest(a.nano_binary),pixel_executable_sha256=digest(a.pixels_binary),compiled_sources=expected(),nano_compiled_sources=prior.expected(),complete_input_pixels=pixels,valid_fixtures=len(cases),invalid_fixtures=failures,existing_output_preserved=True,model_forwards=0,pass_controls=True)
    write(out/'summary.json',summary);print(json.dumps(dict(pass_controls=True,valid_fixtures=len(cases),invalid_fixtures=len(bad),pixel_images=len(pixels),model_forwards=0)),flush=True)

def audit_saved(out,rows,control,annotation=None):
    supported={};sources={};nano={}
    for mode in MODES:
        supported[mode]={};sources[mode]={}
        for row in rows:
            r=load(out/'s640'/mode/(row['id']+'.json'));sources[mode][row['id']]=check_image(r,row,control['complete_input_pixels'][row['id']],mode=='reference');supported[mode][row['id']]=r['result']
    for mode in ('first','repeat'):
        nano[mode]={}
        for row in rows:
            r=load(out/'nano'/mode/(row['id']+'.json'));grid.audit_image(r,row,control['complete_input_pixels'][row['id']],grid=False);nano[mode][row['id']]=r
    for row in rows:
        for mode in ('repeat','reference'):
            grid.same_views(sources['first'][row['id']],sources[mode][row['id']]);prior.compare(supported['first'][row['id']],supported[mode][row['id']],'every repeated/current original-graph decision')
        grid.same_views(nano['first'][row['id']],nano['repeat'][row['id']])
    groups={};relative={};changes={};capacity={}
    for group in ('viewed_development','fresh_sample'):
        selected=[r for r in rows if r['partition']==group];policies={}
        for policy in ('nano_coarse','coarse','naive','supported'):
            projections={r['id']:dict(result=nano['first'][r['id']]['result'] if policy=='nano_coarse' else supported['first'][r['id']] if policy=='supported' else supported['first'][r['id']][policy]) for r in selected}
            policies[policy]=linear.score(selected,projections,annotation)
        groups[group]=policies;a,b,n=[policies[k] for k in ('coarse','supported','naive')]
        relative[group]=dict(precision_no_coarse_regression=b['overall']['precision']>=a['overall']['precision'],precision_better_than_naive=b['overall']['precision']>n['overall']['precision'],recall_above_coarse=b['overall']['recall']>a['overall']['recall'],per_present_class_recall_no_regression=all(v['recall'] is None or b['per_class'][k]['recall']>=v['recall'] for k,v in a['per_class'].items()))
        ids=lambda p:{(r['image_id'],m['reference_id']) for r in policies[p]['images'] for m in r['matches']}
        old,new=ids('coarse'),ids('supported');changes[group]=dict(retained=len(old&new),gained=[list(x) for x in sorted(new-old)],lost=[list(x) for x in sorted(old-new)])
        small=policies['nano_coarse']['overall'];large=a['overall'];capacity[group]=dict(nano_coarse=small,s_coarse=large,precision_difference=large['precision']-small['precision'],recall_difference=large['recall']-small['recall'])
    return dict(schema_version=1,protocol_sha256=digest(HERE/'profile-capacity.json'),groups=groups,relative_gates=relative,reference_changes=changes,capacity_comparison=capacity,adopt_candidate=all(all(v.values()) for v in relative.values()) and groups['fresh_sample']['supported']['quality_gates']['passed'],execution=dict(new_s_model_forwards=720,new_nano_model_forwards=192,new_model_forwards=912,new_s_native_image_runs=72,new_nano_native_image_runs=48,warmup_model_forwards=0,reused_model_forwards_counted_as_new=0),new_latency_or_availability_claim=False,control_permission=False)

def run(a):
    control=load(a.controls/'summary.json');require(control['checker_sha256']==digest(__file__) and control['executable_sha256']==digest(a.binary) and control['nano_executable_sha256']==digest(a.nano_binary) and control['compiled_sources']==expected() and control['nano_compiled_sources']==prior.expected() and control['pass_controls'],'complete matching controls')
    require(digest(a.model)==MODEL_SHA and digest(a.nano_model)==prior.MODEL_SHA,'both pinned model files')
    out=fresh(a.output);rows=records(a.data)
    before=dict(checker_sha256=digest(__file__),fetcher_sha256=digest(HERE/'camera_capacity_fetch.py'),executable_sha256=digest(a.binary),nano_executable_sha256=digest(a.nano_binary),compiled_sources=expected(),nano_compiled_sources=prior.expected(),protocol_sha256=digest(HERE/'profile-capacity.json'),control_summary_sha256=digest(a.controls/'summary.json'),input_rows=[{k:str(v) if isinstance(v,Path) else v for k,v in r.items()} for r in rows],warmup_model_forwards=0,prior_new_model_forwards=0,numeric_annotations_parsed_by_this_process=False)
    write(out/'before-inference.json',before)
    for mode in MODES:
        fresh(out/'s640'/mode)
        for row in rows:
            native(a.binary,['--model',a.model,'--reference-image' if mode=='reference' else '--image',row['path']],out/'s640'/mode/(row['id']+'.json'))
            print(f"S640 {mode} {row['id']}: all10 views saved",flush=True)
    for mode in ('first','repeat'):
        fresh(out/'nano'/mode)
        for row in rows:
            native(a.nano_binary,['--model',a.nano_model,'--image',row['path']],out/'nano'/mode/(row['id']+'.json'))
            print(f"Nano4 {mode} {row['id']}: all4 views saved",flush=True)
    write(out/'before-label-scoring.json',dict(numeric_annotations_parsed_by_this_process=False,output_sha256={str(p.relative_to(out)):digest(p) for p in sorted(out.rglob('*.json')) if p.name!='before-inference.json'}))
    result=audit_saved(out,rows,control)
    require(before['checker_sha256']==digest(__file__) and before['executable_sha256']==digest(a.binary) and before['nano_executable_sha256']==digest(a.nano_binary) and before['compiled_sources']==expected() and before['fetcher_sha256']==digest(HERE/'camera_capacity_fetch.py'),'source/executable unchanged throughout evaluation');records(a.data)
    write(out/'summary.json',result)
    print(json.dumps({g:{p:v['overall'] for p,v in group.items()} for g,group in result['groups'].items()}),flush=True)
    print(json.dumps(dict(adopt_candidate=result['adopt_candidate'],new_model_forwards=912)),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=('controls','run','audit'));p.add_argument('--output',type=Path,required=True);p.add_argument('--controls',type=Path)
    for name,default in [('binary','integrations/onnx/target/release/rustdriving-camera-capacity'),('nano-binary','integrations/onnx/target/release/rustdriving-camera-depthwise'),('pixels-binary','integrations/onnx/target/release/rustdriving-camera-linear'),('model','artifacts/camera-model-s/yolox_s.onnx'),('nano-model','artifacts/camera-model/yolox_nano.onnx'),('data','artifacts/camera-capacity-v1/data')]:p.add_argument('--'+name,type=Path,default=ROOT/default)
    a=p.parse_args()
    for k in ('binary','nano_binary','pixels_binary'):setattr(a,k,getattr(a,k).resolve())
    if a.command=='controls':controls(a)
    elif a.command=='run':require(a.controls is not None,'controls required');run(a)
    else:require(audit_saved(a.output,records(a.data),load(a.controls/'summary.json'))==load(a.output/'summary.json'),'saved audit');print(json.dumps(dict(pass_audit=True,model_execution_this_check=False)))
if __name__=='__main__':main()
