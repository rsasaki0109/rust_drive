#!/usr/bin/env python3
"""Independently verify anchors, additions and the two frozen native parents."""
import argparse,copy,gzip,hashlib,io,json,math,struct,subprocess
from pathlib import Path
import numpy as np
from PIL import Image
import camera_detail_check as detail
baseline=detail.baseline
grid,linear,prior=detail.grid,detail.linear,detail.prior
HERE,ROOT=detail.HERE,detail.ROOT
require,digest,load,write,fresh=detail.require,detail.digest,detail.load,detail.write,detail.fresh
native=detail.native
MODES=('first','repeat','reference')
SOURCES=['integrations/onnx/src/camera_preserved.rs','integrations/onnx/src/bin/rustdriving-camera-preserved.rs','integrations/onnx/src/lib.rs','integrations/onnx/Cargo.toml','integrations/onnx/Cargo.lock','integrations/onnx/profile-preserved.json','rust-toolchain.toml']
def expected():return {n:digest(ROOT/n) for n in SOURCES}
def read(p):return detail.read(p)

def parent(identity,mode,kind):
    p=load(HERE/'profile-detail.json')
    if kind=='baseline' and identity in p['viewed_ids']:return detail.parent(identity,mode)
    return ROOT/'assets/camera-detail-v1/validation'/kind/mode/(identity+'.json.gz')

def records(data):
    rows=detail.records(ROOT/'artifacts/camera-detail-v1/data')
    for r in rows:r['partition']='viewed_development'
    design=load(HERE/'profile-preserved.json');old=load(HERE/'profile-detail.json')
    require(all(design[k]==old[k] for k in ('class_mapping','unscored_category_reason','quality_gates','confidence_threshold','nms_iou_threshold','match_iou_threshold','model_sha256','model_bytes','input_size','output_shape')),'unchanged model and scoring')
    pins=load(data/'inputs.json')
    for image in design['images']:
        names=[Path(image[k]['path']).name for k in ('image','annotation')]
        for k,n in zip(('image','annotation'),names):
            b=(data/n).read_bytes();pin=image[k]
            require(len(b)==pin['size'] and hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()==pin['sha'] and digest(data/n)==pins[n]['sha256'],'original pinned Git input')
        rows.append(dict(id=image['id'],partition='fresh_sample',original_split='val',path=data/names[0],annotation=data/names[1],image_sha256=pins[names[0]]['sha256'],annotation_sha256=pins[names[1]]['sha256']))
    require(len(rows)==len({r['id'] for r in rows})==56,'complete viewed40 plus fresh16')
    return rows

def make_input(base,small):
    source=base['source_analysis']['base_prediction'];a,b=base['result'],small['result']
    return dict(width=source['width'],height=source['height'],anchor=a['detections'],anchor_origins=a['detection_origins'],proposals=b['detections'],proposal_origins=b['detection_origins'])

def total_key(v):
    bits=struct.unpack('<I',struct.pack('<f',v))[0]
    return (~bits)&0xffffffff if bits&0x80000000 else bits^0x80000000

def oracle(f):
    a=copy.deepcopy(f['anchor']);p=copy.deepcopy(f['proposals'])
    for d in a+p:
        d['confidence']=float(np.float32(d['confidence']));d['bbox']=np.asarray(d['bbox'],dtype=np.float32).astype(float).tolist()
    stats=dict(anchor_boxes=len(a),proposal_boxes=len(p),added=0,anchor_overlap=0,suppressed_additions=0,output_cap=0,iou_comparisons=0)
    def overlaps(x,y):
        stats['iou_comparisons']+=1;require(stats['iou_comparisons']<=180000,'work bound')
        return grid.iou32(x['bbox'],y['bbox'])>np.float32(.45)
    added=[];ranks=[];decisions=[]
    order=sorted(range(len(p)),key=lambda i:(-p[i]['confidence'],p[i]['class_index'],total_key(p[i]['bbox'][0]),i))
    for rank in order:
        d=p[rank];entry=dict(proposal_rank=rank,anchor_rank=None,addition_rank=None,reason='output_cap')
        if len(a)+len(added)>=300:stats['output_cap']+=1;decisions.append(entry);continue
        for i,anchor in enumerate(a):
            if overlaps(d,anchor):entry['anchor_rank']=i;break
        if entry['anchor_rank'] is not None:entry['reason']='anchor_overlap';stats['anchor_overlap']+=1;decisions.append(entry);continue
        for i,other in enumerate(added):
            if d['class_index']==other['class_index'] and overlaps(d,other):entry['addition_rank']=i;break
        if entry['addition_rank'] is not None:entry['reason']='suppressed_addition';stats['suppressed_additions']+=1
        else:entry.update(reason='added',addition_rank=len(added));added.append(d);ranks.append(rank)
        decisions.append(entry)
    stats['added']=len(added)
    lineage=[dict(stream='anchor',source_rank=i,origin=o) for i,o in enumerate(f['anchor_origins'])]
    lineage += [dict(stream='detail',source_rank=i,origin=f['proposal_origins'][i]) for i in ranks]
    return dict(detections=a+added,lineage=lineage,decisions=decisions,stats=stats)

def check(report,path):
    require(report['schema_version']==1 and report['algorithm']=='baseline_preserving_camera_fixture_v1' and report['compiled_sources']==expected(),'own fixture producer')
    require(report['profile_sha256']==digest(HERE/'profile-preserved.json') and report['source_fixture_sha256']==digest(path),'profile and exact fixture bytes')
    require(report['model_read'] is False and report['images_read'] is False and report['model_forwards']==0 and report['control_permission'] is False,'explicit zero-forward diagnostic')
    f=load(path);answer=oracle(f);prior.compare(answer,report['result'],'all box bits, lineage, decisions and work')
    n=len(f['anchor']);prior.compare(f['anchor'],report['result']['detections'][:n],'every anchor measurement retained in order')
    require(report['result']['lineage'][:n]==[dict(stream='anchor',source_rank=i,origin=o) for i,o in enumerate(f['anchor_origins'])],'complete anchor lineage prefix')
    return answer

def fixture(anchor=None,proposals=None):
    anchor=anchor or [];proposals=proposals or []
    return dict(width=1280,height=720,anchor=anchor,anchor_origins=[dict(view=0,rank=i) for i in range(len(anchor))],proposals=proposals,proposal_origins=[dict(view=4,rank=i) for i in range(len(proposals))])
def box(kind=2,score=.8,b=(100.,100.,150.,150.)):return dict(class_index=kind,confidence=score,bbox=list(b))

def controls(a):
    out=fresh(a.output);inputs=fresh(out/'inputs');rows=records(a.data)
    cases=[fixture(),fixture([box(2,.4)],[box(7,.99)]),fixture([box(2,.3,(-0.,10.,30.,40.)),box(0,.8)],[box(9,.9,(500.,100.,510.,120.))]),fixture([], [box(9,.8),box(9,.8),box(0,.7)]),fixture([], [box(9,.8,(0.,10.,30.,40.)),box(9,.8,(-0.,10.,30.,40.))]),fixture([box(9,.5,(float(i%20*5),float(i//20*5),float(i%20*5+2),float(i//20*5+2))) for i in range(300)],[box()])]
    for i,f in enumerate(cases):
        path=inputs/f'{i}.json';write(path,f);check(native(a.binary,['--fixture',path],out/f'{i}-actual.json'),path)
    bad=[]
    for kind in range(10):
        f=fixture([box(),box(b=(300.,100.,350.,150.))])
        if kind==0:f['anchor_origins'].pop()
        elif kind==1:f['anchor_origins'][0]['view']=16
        elif kind==2:f['anchor_origins'][0]['rank']=300
        elif kind==3:f['anchor']=f['anchor']*151
        elif kind==4:f['anchor'][0]['bbox'][0]=-1.
        elif kind==5:f['anchor'][0]['confidence']=.2
        elif kind==6:f['anchor'][0]['class_index']=80
        elif kind==7:f['anchor_origins'][1]=f['anchor_origins'][0]
        elif kind==8:f['unexpected']=True
        else:f['width']=0
        bad.append(f)
    failures=[]
    for i,f in enumerate(bad):
        path=inputs/f'invalid-{i}.json';write(path,f);failures.append(native(a.binary,['--fixture',path],out/f'invalid-{i}-unexpected.json',False))
    path=inputs/'oversize.json';path.write_bytes(b' '*(4*1024*1024+1));failures.append(native(a.binary,['--fixture',path],out/'oversize-unexpected.json',False))
    preserve=out/'preserve.txt';preserve.write_text('preserve\n');r=subprocess.run([str(a.binary),'--fixture',str(inputs/'0.json'),'--output',str(preserve)],text=True,capture_output=True)
    require(r.returncode==2 and preserve.read_text()=='preserve\n','complete output cannot be overwritten')
    old=read(ROOT/'assets/camera-detail-v1/controls/summary.json.gz');pixels=copy.deepcopy(old['complete_input_pixels']);base_pixels=copy.deepcopy(old['baseline_input_pixels'])
    for row in rows[40:]:
        path=out/(row['id']+'-local-pixel-export.json');r=native(a.pixels_binary,['--preprocess',row['path'],'--input-size',640,'--include-pixels'],path)
        require(r['compiled_sources']==linear.expected() and r['source_image_sha256']==row['image_sha256'] and r['model_read'] is False and r['inference_run'] is False,'frozen model-free pixel reader')
        rgb=np.asarray(Image.open(io.BytesIO(bytes.fromhex(r['decoded_rgb_png_hex']))).convert('RGB'));require(hashlib.sha256(rgb.tobytes()).hexdigest()==r['rgb_sha256'],'decoded RGB')
        for inventory,rects in ((pixels,detail.rectangles(1280,720)),(base_pixels,baseline.rectangles(1280,720))):
            views=[]
            for rect in rects:
                x,y,w,h=[rect[k] for k in ('x','y','width','height')];views.append(dict(rectangle=rect,rgb_sha256=hashlib.sha256(np.ascontiguousarray(rgb[y:y+h,x:x+w]).tobytes()).hexdigest()))
            inventory[row['id']]=dict(source_image_sha256=row['image_sha256'],views=views,local_pixel_export_sha256=digest(path))
    summary=dict(schema_version=1,checker_sha256=digest(__file__),executable_sha256=digest(a.binary),baseline_executable_sha256=digest(a.baseline_binary),detail_executable_sha256=digest(a.detail_binary),compiled_sources=expected(),baseline_compiled_sources=baseline.expected(),detail_compiled_sources=detail.expected(),complete_input_pixels=pixels,baseline_input_pixels=base_pixels,valid_fixtures=len(cases),invalid_fixtures=failures,existing_output_preserved=True,model_forwards=0,pass_controls=True)
    write(out/'summary.json',summary);print(json.dumps(dict(pass_controls=True,valid_fixtures=len(cases),invalid_fixtures=len(failures),new_pixel_exports=16,model_forwards=0)),flush=True)

def parents(out,row,mode,control):
    b=read(parent(row['id'],mode,'baseline')) if row['partition']=='viewed_development' else load(out/'baseline'/mode/(row['id']+'.json'))
    d=read(parent(row['id'],mode,'detail')) if row['partition']=='viewed_development' else load(out/'detail'/mode/(row['id']+'.json'))
    baseline.check_image(b,row,control['baseline_input_pixels'][row['id']],mode=='reference');detail.check_image(d,row,control['complete_input_pixels'][row['id']],mode=='reference')
    grid.same_views(b['source_analysis'],d['source_analysis'],prefix=True);prior.compare(b['result']['coarse'],d['result']['coarse'],'same original4 consistency')
    return b,d

def audit_saved(out,rows,control,annotation=None):
    measured={};combined={}
    for mode in MODES:
        measured[mode]={};combined[mode]={}
        for row in rows:
            b,d=parents(out,row,mode,control);measured[mode][row['id']]=(b,d)
            path=out/'fixtures'/mode/(row['id']+'.json');require(load(path)==make_input(b,d),'fixture derives from exact original parents')
            combined[mode][row['id']]=check(load(out/'combined'/mode/(row['id']+'.json')),path)
    for row in rows:
        for mode in ('repeat','reference'):
            for i in (0,1):grid.same_views(measured['first'][row['id']][i]['source_analysis'],measured[mode][row['id']][i]['source_analysis'])
            prior.compare(combined['first'][row['id']],combined[mode][row['id']],'every repeated/current-reference combined result')
    groups={};relative={};changes={}
    for group in ('viewed_development','fresh_sample'):
        selected=[r for r in rows if r['partition']==group];policies={}
        for policy in ('coarse','baseline_supported','detail_supported','preserved'):
            def projected(row):
                b,d=measured['first'][row['id']]
                return b['result']['coarse'] if policy=='coarse' else b['result'] if policy=='baseline_supported' else d['result'] if policy=='detail_supported' else combined['first'][row['id']]
            policies[policy]=linear.score(selected,{r['id']:dict(result=projected(r)) for r in selected},annotation)
        groups[group]=policies;c,b,d,p=[policies[k] for k in ('coarse','baseline_supported','detail_supported','preserved')]
        ids=lambda key:{(r['image_id'],m['reference_id']) for r in policies[key]['images'] for m in r['matches']}
        old,new=ids('baseline_supported'),ids('preserved');changes[group]=dict(retained=len(old&new),gained=[list(x) for x in sorted(new-old)],lost=[list(x) for x in sorted(old-new)])
        relative[group]=dict(precision_no_coarse_regression=p['overall']['precision']>=c['overall']['precision'],precision_no_baseline_regression=p['overall']['precision']>=b['overall']['precision'],recall_above_coarse=p['overall']['recall']>c['overall']['recall'],recall_above_baseline=p['overall']['recall']>b['overall']['recall'],per_present_class_recall_no_regression=all(v['recall'] is None or p['per_class'][k]['recall']>=v['recall'] for source in (b,d) for k,v in source['per_class'].items()),all_baseline_matched_references_retained=old<=new)
    return dict(schema_version=1,protocol_sha256=digest(HERE/'profile-preserved.json'),groups=groups,relative_gates=relative,reference_changes=changes,adopt_candidate=all(all(v.values()) for v in relative.values()) and groups['fresh_sample']['preserved']['quality_gates']['passed'],execution=dict(new_model_forwards=1056,new_native_image_runs=96,reused_parent_reports=240,reused_parent_forwards_not_counted_as_new=2640,native_fixture_runs=168,fixture_model_forwards=0,warmup_model_forwards=0),new_latency_or_availability_claim=False,control_permission=False)

def run(a):
    control=load(a.controls/'summary.json');require(control['checker_sha256']==digest(__file__) and control['executable_sha256']==digest(a.binary) and control['baseline_executable_sha256']==digest(a.baseline_binary) and control['detail_executable_sha256']==digest(a.detail_binary) and control['compiled_sources']==expected() and control['pass_controls'],'matching complete controls')
    require(digest(a.model)==baseline.MODEL_SHA,'same pinned S model');out=fresh(a.output);rows=records(a.data)
    pins={str(parent(r['id'],m,k).relative_to(ROOT)):digest(parent(r['id'],m,k)) for r in rows[:40] for m in MODES for k in ('baseline','detail')}
    before=dict(checker_sha256=digest(__file__),fetcher_sha256=digest(HERE/'camera_preserved_fetch.py'),executable_sha256=digest(a.binary),baseline_executable_sha256=digest(a.baseline_binary),detail_executable_sha256=digest(a.detail_binary),compiled_sources=expected(),baseline_compiled_sources=baseline.expected(),detail_compiled_sources=detail.expected(),protocol_sha256=digest(HERE/'profile-preserved.json'),control_summary_sha256=digest(a.controls/'summary.json'),input_rows=[{k:str(v) if isinstance(v,Path) else v for k,v in r.items()} for r in rows],frozen_parent_sha256=pins,prior_new_model_forwards=0,warmup_model_forwards=0,numeric_annotations_parsed_by_this_process=False)
    write(out/'before-inference.json',before)
    for mode in MODES:
        for kind in ('baseline','detail'):fresh(out/kind/mode)
        for row in rows[40:]:
            for kind,binary in (('baseline',a.baseline_binary),('detail',a.detail_binary)):
                native(binary,['--model',a.model,'--reference-image' if mode=='reference' else '--image',row['path']],out/kind/mode/(row['id']+'.json'))
                print(f"fresh {mode} {kind} {row['id']}: complete report saved",flush=True)
    for mode in MODES:
        fresh(out/'fixtures'/mode);fresh(out/'combined'/mode)
        for row in rows:
            b,d=parents(out,row,mode,control);path=out/'fixtures'/mode/(row['id']+'.json');write(path,make_input(b,d));native(a.binary,['--fixture',path],out/'combined'/mode/(row['id']+'.json'))
    write(out/'before-label-scoring.json',dict(numeric_annotations_parsed_by_this_process=False,output_sha256={str(p.relative_to(out)):digest(p) for p in sorted(out.rglob('*.json')) if p.name!='before-inference.json'}))
    result=audit_saved(out,rows,control)
    require(before['checker_sha256']==digest(__file__) and before['executable_sha256']==digest(a.binary) and before['baseline_executable_sha256']==digest(a.baseline_binary) and before['detail_executable_sha256']==digest(a.detail_binary) and before['compiled_sources']==expected() and before['fetcher_sha256']==digest(HERE/'camera_preserved_fetch.py'),'unchanged sources and executables');require(all(digest(ROOT/n)==h for n,h in pins.items()),'frozen parents unchanged');records(a.data)
    write(out/'summary.json',result);print(json.dumps({g:{p:v['overall'] for p,v in group.items()} for g,group in result['groups'].items()}),flush=True);print(json.dumps(dict(adopt_candidate=result['adopt_candidate'],new_model_forwards=1056)),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=('controls','run','audit'));p.add_argument('--output',type=Path,required=True);p.add_argument('--controls',type=Path)
    for name,value in [('binary','integrations/onnx/target/release/rustdriving-camera-preserved'),('baseline-binary','integrations/onnx/target/release/rustdriving-camera-capacity'),('detail-binary','integrations/onnx/target/release/rustdriving-camera-detail'),('pixels-binary','integrations/onnx/target/release/rustdriving-camera-linear'),('model','artifacts/camera-model-s/yolox_s.onnx'),('data','artifacts/camera-preserved-v1/data')]:p.add_argument('--'+name,type=Path,default=ROOT/value)
    a=p.parse_args()
    for key in ('binary','baseline_binary','detail_binary','pixels_binary'):setattr(a,key,getattr(a,key).resolve())
    if a.command=='controls':controls(a)
    elif a.command=='run':require(a.controls is not None,'controls required');run(a)
    else:require(audit_saved(a.output,records(a.data),load(a.controls/'summary.json'))==load(a.output/'summary.json'),'saved audit');print(json.dumps(dict(pass_audit=True,model_execution_this_check=False)))
if __name__=='__main__':main()
