#!/usr/bin/env python3
"""Independent fixed S640/416px detail comparison, with preserved baseline."""
import argparse,copy,gzip,hashlib,io,json,math,subprocess
from pathlib import Path
import numpy as np
from PIL import Image
import camera_capacity_check as baseline
grid,linear,prior=baseline.grid,baseline.linear,baseline.prior
HERE,ROOT=baseline.HERE,baseline.ROOT
require,digest,load,write,fresh=baseline.require,baseline.digest,baseline.load,baseline.write,baseline.fresh
native=baseline.native
MODEL_SHA=baseline.MODEL_SHA
MODES=('first','repeat','reference')
NEW=['integrations/onnx/src/camera_detail_detector.rs','integrations/onnx/src/camera_detail_parallel.rs',
 'integrations/onnx/src/bin/rustdriving-camera-detail.rs','integrations/onnx/profile-detail.json']
def expected():return dict(baseline.expected(),**{n:digest(ROOT/n) for n in NEW})
rectangles,merge,oracle,fixture=grid.rectangles,grid.merge,grid.oracle,grid.fixture
supported_oracle=baseline.previous.supported_oracle
make_fixture=baseline.make_fixture
add=baseline.add

def parent(identity,mode):
    return ROOT/'assets/camera-capacity-v1/validation/s640'/mode/(identity+'.json.gz')
def read(path):return json.loads(baseline.previous.read_raw(path))

def records(data):
    rows=baseline.records(ROOT/'artifacts/camera-capacity-v1/data')
    for r in rows:r['partition']='viewed_development'
    design=load(HERE/'profile-detail.json');old=load(HERE/'profile-capacity.json')
    require(all(design[k]==old[k] for k in ('class_mapping','unscored_category_reason','quality_gates','confidence_threshold','nms_iou_threshold','match_iou_threshold','policy','model_sha256','model_bytes','input_size','output_shape')),'unchanged model, support and all-label scoring')
    pins=load(data/'inputs.json')
    for image in design['images']:
        names=[Path(image[k]['path']).name for k in ('image','annotation')]
        for k,n in zip(('image','annotation'),names):
            body=(data/n).read_bytes();pin=image[k]
            require(len(body)==pin['size'] and hashlib.sha1(b'blob '+str(len(body)).encode()+b'\0'+body).hexdigest()==pin['sha'] and digest(data/n)==pins[n]['sha256'],'original fixed Git inputs')
        rows.append(dict(id=image['id'],partition='fresh_sample',original_split='val',path=data/names[0],annotation=data/names[1],image_sha256=pins[names[0]]['sha256'],annotation_sha256=pins[names[1]]['sha256']))
    require(len(rows)==len({r['id'] for r in rows})==40,'complete viewed24 and fresh16 inputs')
    return rows

def check_result(actual,f):
    answer=supported_oracle(f);prior.compare(answer,actual,'all coarse/naive/supported boxes, lineage, decisions and counters')
    n=len(answer['coarse']['detections']);prior.compare(answer['coarse']['detections'],actual['detections'][:n],'immutable coarse prefix')
    return answer

def check_source(actual,row,pixels,reference=False):
    pins=expected();profile=digest(HERE/'profile-detail.json')
    require(actual['schema_version']==1 and actual['algorithm']=='yolox_s_detail_grid_image_analysis_v1' and actual['compiled_sources']==pins,'S grid producer/source closure')
    require(actual['control_permission'] is False and actual['source_image_sha256']==row['image_sha256'] and actual['profile_sha256']==profile and actual['original_graph_reference'] is reference,'S identity and graph/reference profile')
    base=actual['base_prediction'];views=base['views']
    require(base['schema_version']==1 and base['algorithm']=='pinned_yolox_s_detail_grid_fixed_views_v1' and base['compiled_sources']==pins,'S base producer')
    require((base['width'],base['height'])==(1280,720) and len(views)==12 and len(pixels['views'])==12,'complete twelve view inventory')
    require(base['model_sha256']==MODEL_SHA and base['source_image_sha256']==row['image_sha256'],'S model and original image')
    require(prior.nano.exact_f32(base['score_threshold'],.3) and prior.nano.exact_f32(base['nms_iou_threshold'],.45),'merge thresholds')
    for view,pixel in zip(views,pixels['views']):
        r=view['rectangle'];p=view['prediction']
        require(r==pixel['rectangle'] and view['rgb_sha256']==pixel['rgb_sha256'],'every exact decoded crop pixel')
        require(p['schema_version']==1 and p['algorithm']==('pinned_s_detail_original_plan_triangle_v1' if reference else 'pinned_s_detail_repeat2x_triangle_v1') and p['model_profile']=='yolox-s-detail-v1' and p['compiled_sources']==pins and p['model_profile_sha256']==profile,'internally executed S detector')
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
    ex=actual['execution'];require({k:v for k,v in ex.items() if k!='analysis_wall_s'}==dict(model_read=True,images_read=1,model_forwards=12,inference_run=True,view_workers=1 if reference else 4,shared_model_plans=1),'complete twelve forwards and bounded workers')
    require(type(ex['analysis_wall_s']) in (float,int) and math.isfinite(ex['analysis_wall_s']) and ex['analysis_wall_s']>=max(v['prediction']['inference_seconds'] for v in views),'whole measured analysis interval')

def check_image(r,row,pixels,reference=False):
    require(r['schema_version']==1 and r['algorithm']=='s_detail_grid_image_v1' and r['compiled_sources']==expected(),'own S supported producer')
    require(r['profile_sha256']==digest(HERE/'profile-detail.json') and r['source_image_sha256']==row['image_sha256'] and r['control_permission'] is False,'S profile/image and no control')
    source=r['source_analysis'];check_source(source,row,pixels,reference)
    answer=check_result(r['result'],make_fixture(source));prior.compare(answer['naive'],source['result'],'same measured naive analysis')
    return source

def check_fixture(r,path):
    require(r['schema_version']==1 and r['algorithm']=='s_detail_grid_fixture_v1' and r['compiled_sources']==expected(),'fixture producer')
    require(r['source_fixture_sha256']==digest(path) and r['model_read'] is False and r['images_read'] is False and r['model_forwards']==0 and r['control_permission'] is False,'fixture is explicit model-free diagnostics')
    return check_result(r['result'],load(path))

def controls(a):
    rows=records(a.data);out=fresh(a.output);inputs=fresh(out/'inputs')
    cases=[fixture(w,h) for w,h in ((1280,720),(720,1281),(641,640),(320,240),(640,640),(900,900))]
    for count in (1,2):
        f=fixture();add(f,4,9,.8,[300.,100.,350.,150.]);add(f,4,9,.7,[300.,100.,350.,150.])
        if count==2:add(f,5,9,.6,[300.,100.,350.,150.])
        cases.append(f)
    f=fixture()
    for view,kind,score in ((4,2,.9),(5,7,.8),(8,2,.7)):add(f,view,kind,score,[300.,330.,350.,380.])
    cases.append(f)
    f=fixture();add(f,0,2,.6,[300.,100.,350.,150.]);add(f,4,9,.9,[300.,100.,350.,150.]);add(f,5,9,.8,[300.,100.,350.,150.]);cases.append(f)
    f=fixture()
    for view in (4,5):
        add(f,view,0,.9,[300.,100.,350.,150.]);add(f,view,1,.8,[300.,100.,350.,150.])
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
    pixels={};baseline_pixels={}
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
        base_views=[]
        for rect in baseline.rectangles(rgb.shape[1],rgb.shape[0]):
            x,y,w,h=[rect[k] for k in ('x','y','width','height')]
            base_views.append(dict(rectangle=rect,rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb[y:y+h,x:x+w]).tobytes()).hexdigest()))
        baseline_pixels[row['id']]=dict(source_image_sha256=row['image_sha256'],views=base_views,local_pixel_export_sha256=digest(path))

    summary=dict(schema_version=1,checker_sha256=digest(__file__),executable_sha256=digest(a.binary),baseline_executable_sha256=digest(a.baseline_binary),pixel_executable_sha256=digest(a.pixels_binary),compiled_sources=expected(),baseline_compiled_sources=baseline.expected(),complete_input_pixels=pixels,baseline_input_pixels=baseline_pixels,valid_fixtures=len(cases),invalid_fixtures=failures,existing_output_preserved=True,model_forwards=0,pass_controls=True)
    write(out/'summary.json',summary);print(json.dumps(dict(pass_controls=True,valid_fixtures=len(cases),invalid_fixtures=len(bad),pixel_images=len(pixels),model_forwards=0)),flush=True)

def audit_saved(out,rows,control,annotation=None):
    detail={};sources={};base={}
    for mode in MODES:
        detail[mode]={};sources[mode]={};base[mode]={}
        for row in rows:
            r=load(out/'detail'/mode/(row['id']+'.json'));sources[mode][row['id']]=check_image(r,row,control['complete_input_pixels'][row['id']],mode=='reference');detail[mode][row['id']]=r['result']
            b=read(parent(row['id'],mode)) if row['partition']=='viewed_development' else load(out/'baseline'/mode/(row['id']+'.json'))
            baseline.check_image(b,row,control['baseline_input_pixels'][row['id']],mode=='reference');base[mode][row['id']]=b
            grid.same_views(sources[mode][row['id']],b['source_analysis'],prefix=True)
            prior.compare(r['result']['coarse'],b['result']['coarse'],'original4 complete consistency is unchanged')
    for row in rows:
        for mode in ('repeat','reference'):
            grid.same_views(sources['first'][row['id']],sources[mode][row['id']]);prior.compare(detail['first'][row['id']],detail[mode][row['id']],'every current detail output bit and support decision')
            grid.same_views(base['first'][row['id']]['source_analysis'],base[mode][row['id']]['source_analysis']);prior.compare(base['first'][row['id']]['result'],base[mode][row['id']]['result'],'every baseline output bit and decision')
    groups={};relative={};changes={}
    for group in ('viewed_development','fresh_sample'):
        selected=[r for r in rows if r['partition']==group];policies={}
        for policy in ('coarse','baseline_supported','detail_naive','detail_supported'):
            def projected(row):
                d=detail['first'][row['id']]
                return base['first'][row['id']]['result'] if policy=='baseline_supported' else d['coarse'] if policy=='coarse' else d['naive'] if policy=='detail_naive' else d
            policies[policy]=linear.score(selected,{r['id']:dict(result=projected(r)) for r in selected},annotation)
        groups[group]=policies;c,b,n,d=[policies[k] for k in ('coarse','baseline_supported','detail_naive','detail_supported')]
        relative[group]=dict(precision_no_coarse_regression=d['overall']['precision']>=c['overall']['precision'],precision_no_baseline_regression=d['overall']['precision']>=b['overall']['precision'],precision_above_detail_naive=d['overall']['precision']>n['overall']['precision'],recall_above_coarse=d['overall']['recall']>c['overall']['recall'],recall_above_baseline=d['overall']['recall']>b['overall']['recall'],per_present_class_recall_no_baseline_regression=all(v['recall'] is None or d['per_class'][k]['recall']>=v['recall'] for k,v in b['per_class'].items()))
        ids=lambda p:{(r['image_id'],m['reference_id']) for r in policies[p]['images'] for m in r['matches']}
        old,new=ids('baseline_supported'),ids('detail_supported');changes[group]=dict(retained=len(old&new),gained=[list(x) for x in sorted(new-old)],lost=[list(x) for x in sorted(old-new)])
    return dict(schema_version=1,protocol_sha256=digest(HERE/'profile-detail.json'),groups=groups,relative_gates=relative,reference_changes=changes,adopt_candidate=all(all(v.values()) for v in relative.values()) and groups['fresh_sample']['detail_supported']['quality_gates']['passed'],execution=dict(new_detail_model_forwards=1440,new_baseline_model_forwards=480,new_model_forwards=1920,new_detail_image_runs=120,new_baseline_image_runs=48,reused_baseline_reports=72,reused_baseline_forwards_not_counted_as_new=720,warmup_model_forwards=0),new_latency_or_availability_claim=False,control_permission=False)

def run(a):
    control=load(a.controls/'summary.json');require(control['checker_sha256']==digest(__file__) and control['executable_sha256']==digest(a.binary) and control['baseline_executable_sha256']==digest(a.baseline_binary) and control['compiled_sources']==expected() and control['baseline_compiled_sources']==baseline.expected() and control['pass_controls'],'matching complete controls')
    require(digest(a.model)==MODEL_SHA,'same pinned S model')
    out=fresh(a.output);rows=records(a.data)
    parents={str(parent(r['id'],m).relative_to(ROOT)):digest(parent(r['id'],m)) for r in rows[:24] for m in MODES}
    before=dict(checker_sha256=digest(__file__),fetcher_sha256=digest(HERE/'camera_detail_fetch.py'),executable_sha256=digest(a.binary),baseline_executable_sha256=digest(a.baseline_binary),compiled_sources=expected(),baseline_compiled_sources=baseline.expected(),protocol_sha256=digest(HERE/'profile-detail.json'),control_summary_sha256=digest(a.controls/'summary.json'),input_rows=[{k:str(v) if isinstance(v,Path) else v for k,v in r.items()} for r in rows],frozen_parent_sha256=parents,warmup_model_forwards=0,prior_new_model_forwards=0,numeric_annotations_parsed_by_this_process=False)
    write(out/'before-inference.json',before)
    for mode in MODES:
        fresh(out/'detail'/mode);fresh(out/'baseline'/mode)
        for row in rows:
            native(a.binary,['--model',a.model,'--reference-image' if mode=='reference' else '--image',row['path']],out/'detail'/mode/(row['id']+'.json'))
            print(f"detail {mode} {row['id']}: all12 views saved",flush=True)
        for row in rows[24:]:
            native(a.baseline_binary,['--model',a.model,'--reference-image' if mode=='reference' else '--image',row['path']],out/'baseline'/mode/(row['id']+'.json'))
            print(f"baseline {mode} {row['id']}: all10 views saved",flush=True)
    write(out/'before-label-scoring.json',dict(numeric_annotations_parsed_by_this_process=False,output_sha256={str(p.relative_to(out)):digest(p) for p in sorted(out.rglob('*.json')) if p.name!='before-inference.json'}))
    result=audit_saved(out,rows,control)
    require(before['checker_sha256']==digest(__file__) and before['executable_sha256']==digest(a.binary) and before['baseline_executable_sha256']==digest(a.baseline_binary) and before['compiled_sources']==expected() and before['fetcher_sha256']==digest(HERE/'camera_detail_fetch.py'),'unchanged source and executables')
    require(all(digest(ROOT/n)==h for n,h in parents.items()),'frozen parents unchanged');records(a.data)
    write(out/'summary.json',result)
    print(json.dumps({g:{p:v['overall'] for p,v in policies.items()} for g,policies in result['groups'].items()}),flush=True)
    print(json.dumps(dict(adopt_candidate=result['adopt_candidate'],new_model_forwards=1920)),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=('controls','run','audit'));p.add_argument('--output',type=Path,required=True);p.add_argument('--controls',type=Path)
    for name,default in [('binary','integrations/onnx/target/release/rustdriving-camera-detail'),('baseline-binary','integrations/onnx/target/release/rustdriving-camera-capacity'),('pixels-binary','integrations/onnx/target/release/rustdriving-camera-linear'),('model','artifacts/camera-model-s/yolox_s.onnx'),('data','artifacts/camera-detail-v1/data')]:p.add_argument('--'+name,type=Path,default=ROOT/default)
    a=p.parse_args()
    for k in ('binary','baseline_binary','pixels_binary'):setattr(a,k,getattr(a,k).resolve())
    if a.command=='controls':controls(a)
    elif a.command=='run':require(a.controls is not None,'controls required');run(a)
    else:require(audit_saved(a.output,records(a.data),load(a.controls/'summary.json'))==load(a.output/'summary.json'),'saved audit');print(json.dumps(dict(pass_audit=True,model_execution_this_check=False)))
if __name__=='__main__':main()
