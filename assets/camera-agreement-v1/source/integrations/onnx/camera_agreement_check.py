#!/usr/bin/env python3
"""Independent dense-matrix reconstruction of a viewed-only agreement trial."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import numpy as np
import camera_preserved_check as old
import camera_preflight as preflight

HERE,ROOT=old.HERE,old.ROOT
require,digest,load,write,fresh=old.require,old.digest,old.load,old.write,old.fresh
SOURCES=['integrations/onnx/src/camera_agreement.rs','integrations/onnx/src/bin/rustdriving-camera-agreement.rs','integrations/onnx/src/lib.rs','integrations/onnx/Cargo.toml','integrations/onnx/Cargo.lock','integrations/onnx/profile-agreement.json','rust-toolchain.toml']
MODES=old.MODES
def expected():return {n:digest(ROOT/n) for n in SOURCES}
def records():
    rows=old.read(ROOT/'assets/camera-preflight-v1/rejection/rows.json.gz')[:40]
    design=load(HERE/'profile-agreement.json');previous=load(HERE/'profile-preserved.json')
    require([r['id'] for r in rows]==design['viewed_ids'] and len({r['id'] for r in rows})==40,'complete fixed viewed40')
    require(all(r['partition']=='viewed_development' for r in rows),'no independent sample claim')
    for k in ('class_mapping','unscored_category_reason','quality_gates','confidence_threshold','nms_iou_threshold','match_iou_threshold','model_sha256','model_bytes','input_size','output_shape'):
        require(design[k]==previous[k],'unchanged model and scoring: '+k)
    return rows
def make_input(base,detail):
    a,b=base['result'],detail['result'];coarse=a['coarse'];source=base['source_analysis']['base_prediction']
    return dict(width=source['width'],height=source['height'],coarse=coarse['detections'],coarse_origins=coarse['detection_origins'],wide=a['detections'],wide_origins=a['detection_origins'],detail=b['detections'],detail_origins=b['detection_origins'])
def validate(f):
    require(set(f)=={'width','height','coarse','coarse_origins','wide','wide_origins','detail','detail_origins'},'complete fixture fields')
    require(all(type(f[k]) is int and f[k]>0 for k in ('width','height')) and f['width']*f['height']<=16000000,'bounded dimensions')
    for stream in ('coarse','wide','detail'):
        boxes,origins=f[stream],f[stream+'_origins'];require(len(boxes)==len(origins)<=300,'bounded complete stream')
        require(len({(o['view'],o['rank']) for o in origins})==len(origins),'unique origins')
        for d,o in zip(boxes,origins):
            require(set(d)=={'class_index','confidence','bbox'} and set(o)=={'view','rank'},'typed measurement and origin')
            require(type(o['view']) is int and 0<=o['view']<16 and type(o['rank']) is int and 0<=o['rank']<300,'bounded origin')
            require(type(d['class_index']) is int and 0<=d['class_index']<80 and type(d['confidence']) in (int,float) and np.isfinite(d['confidence']) and np.float32(.3)<=np.float32(d['confidence'])<=1,'bounded confidence and class')
            require(old.linear.matcher.valid_box(d['bbox'],f['width'],f['height']),'source-pixel box')
    n=len(f['coarse'])
    for stream in ('wide','detail'):
        require(len(f[stream])>=n,'complete parent prefix')
        old.prior.compare(f['coarse'],f[stream][:n],'exact coarse box bits')
        require(f[stream+'_origins'][:n]==f['coarse_origins'],'exact coarse origins')
def oracle(f):
    validate(f);f=copy.deepcopy(f)
    for stream in ('coarse','wide','detail'):
        for d in f[stream]:
            d['confidence']=float(np.float32(d['confidence']));d['bbox']=np.asarray(d['bbox'],dtype=np.float32).astype(float).tolist()
    n=len(f['coarse']);left,right=f['wide'][n:],f['detail'][n:]
    scores=np.zeros((len(left),len(right)),dtype=np.float32)
    stats=dict(coarse_boxes=n,wide_additions=len(left),detail_additions=len(right),added=0,agreement_comparisons=0,anchor_comparisons=0,nms_comparisons=0,iou_comparisons=0)
    def iou(a,b,kind):
        stats[kind]+=1;stats['iou_comparisons']+=1;require(stats['iou_comparisons']<=270000,'bounded comparison work')
        return old.grid.iou32(a['bbox'],b['bbox'])
    for i,a in enumerate(left):
        for j,b in enumerate(right):
            if a['class_index']==b['class_index']:scores[i,j]=iou(a,b,'agreement_comparisons')
    best_left=np.argmax(scores,axis=1) if len(right) and len(left) else []
    best_right=np.argmax(scores,axis=0) if len(right) and len(left) else []
    emitted=copy.deepcopy(f['coarse']);lineage=[dict(stream='coarse',source_rank=i,origin=o) for i,o in enumerate(f['coarse_origins'])];decisions=[]
    for i,a in enumerate(left):
        d=dict(wide_rank=n+i,detail_rank=None,agreement_iou_bits=None,reason='no_agreement')
        if len(right) and scores[i,best_left[i]]>np.float32(.45):
            j=int(best_left[i]);d.update(detail_rank=n+j,agreement_iou_bits=struct.unpack('<I',struct.pack('<f',scores[i,j]))[0],reason='non_mutual')
            if int(best_right[j])==i:
                d['reason']='output_cap'
                if len(emitted)<300:
                    d['reason']='anchor_overlap'
                    veto=any(iou(a,b,'anchor_comparisons')>np.float32(.45) for b in f['coarse'])
                    if not veto:
                        d['reason']='suppressed_addition'
                        veto=any(iou(a,b,'nms_comparisons')>np.float32(.45) for b in emitted[n:] if a['class_index']==b['class_index'])
                        if not veto:
                            emitted.append(a);lineage.append(dict(stream='wide',source_rank=n+i,origin=f['wide_origins'][n+i]));stats['added']+=1;d['reason']='added'
        decisions.append(d)
    return dict(detections=emitted,lineage=lineage,decisions=decisions,stats=stats)
def check(report,path):
    require(report['schema_version']==1 and report['algorithm']=='mutual_scale_agreement_fixture_v1' and report['compiled_sources']==expected(),'own complete fixture producer')
    require(report['profile_sha256']==digest(HERE/'profile-agreement.json') and report['source_fixture_sha256']==digest(path),'exact protocol and fixture bytes')
    require(report['model_read'] is False and report['images_read'] is False and report['model_forwards']==0 and report['control_permission'] is False,'no inference or control claim')
    answer=oracle(load(path));old.prior.compare(answer,report['result'],'every measurement bit, witness, decision and counter')
    return answer
def fixture(coarse=None,wide=None,detail=None):
    coarse=coarse or [];wide=coarse+(wide or []);detail=coarse+(detail or [])
    origins=lambda n,v:[dict(view=v,rank=i) for i in range(n)]
    prefix=origins(len(coarse),0)
    return dict(width=1280,height=720,coarse=copy.deepcopy(coarse),coarse_origins=copy.deepcopy(prefix),wide=copy.deepcopy(wide),wide_origins=copy.deepcopy(prefix)+origins(len(wide)-len(coarse),4),detail=copy.deepcopy(detail),detail_origins=copy.deepcopy(prefix)+origins(len(detail)-len(coarse),4))
box=old.box
def controls(a):
    out=fresh(a.output);inputs=fresh(out/'inputs')
    cases=[fixture(),fixture([box(0,.4,(-0.,10.,20.,30.))],[box(9,.8,(100.,100.,120.,120.))],[box(9,.7,(101.,100.,121.,120.))]),fixture([], [box(),box()],[box()]),fixture([], [box(2)],[box(7)]),fixture([box(7)],[box(2)],[box(2)]),fixture([], [box(2,.8,(0.,0.,20.,20.))],[box(2,.8,(0.,0.,9.,20.))]),fixture([], [box(2,.8,(100.,100.,120.,120.)),box(2,.8,(102.,100.,122.,120.))],[box(2,.8,(102.,100.,122.,120.))]),fixture([box(9,.5,(float(i%20*5),float(i//20*5),float(i%20*5+2),float(i//20*5+2))) for i in range(300)])]
    for i,f in enumerate(cases):
        path=inputs/f'{i}.json';write(path,f);check(old.native(a.binary,['--fixture',path],out/f'{i}-actual.json'),path)
    failures=[]
    for kind in range(12):
        f=fixture([box()], [box(9,.8,(500.,100.,510.,120.))],[box(9,.7,(500.,100.,510.,120.))])
        if kind==0:f['detail'][0]['confidence']=.9
        elif kind==1:f['detail_origins'][0]['view']=1
        elif kind==2:f['wide_origins'].pop()
        elif kind==3:f['detail_origins'][1]=f['detail_origins'][0]
        elif kind==4:f['wide_origins'][1]['view']=16
        elif kind==5:f['wide_origins'][1]['rank']=300
        elif kind==6:f['wide'][1]['bbox'][0]=-1.
        elif kind==7:f['wide'][1]['confidence']=.2
        elif kind==8:f['wide'][1]['class_index']=80
        elif kind==9:f['unexpected']=True
        elif kind==10:f['width']=0
        else:f['detail']*=151;f['detail_origins']*=151
        path=inputs/f'bad-{kind}.json';write(path,f);failures.append(old.native(a.binary,['--fixture',path],out/f'bad-{kind}.json',False))
    path=inputs/'symlink.json';path.symlink_to((inputs/'0.json').resolve());failures.append(old.native(a.binary,['--fixture',path],out/'bad-symlink.json',False))
    path=inputs/'oversize.json';path.write_bytes(b' '*(4*1024*1024+1));failures.append(old.native(a.binary,['--fixture',path],out/'bad-oversize.json',False))
    path=out/'preserve.txt';path.write_text('existing output\n')
    r=subprocess.run([str(a.binary),'--fixture',str(inputs/'0.json'),'--output',str(path)],capture_output=True,text=True,timeout=90)
    require(r.returncode==2 and path.read_text()=='existing output\n','complete publication does not overwrite')
    summary=dict(pass_controls=True,valid_fixtures=len(cases),invalid_fixtures=failures,existing_output_preserved=True,compiled_sources=expected(),checker_sha256=digest(__file__),executable_sha256=digest(a.binary),model_forwards=0)
    write(out/'summary.json',summary);print(json.dumps(dict(pass_controls=True,valid_fixtures=len(cases),invalid_fixtures=len(failures),model_forwards=0)),flush=True)
def audit(out,rows,annotation=None):
    parent_control=old.read(ROOT/'assets/camera-preserved-v1/controls/summary.json.gz');measured={};combined={}
    for mode in MODES:
        measured[mode]={};combined[mode]={}
        for row in rows:
            b,d=old.parents(out,row,mode,parent_control);measured[mode][row['id']]=(b,d)
            path=out/'fixtures'/mode/(row['id']+'.json');require(load(path)==make_input(b,d),'fixture derives from both complete frozen producers')
            combined[mode][row['id']]=check(load(out/'combined'/mode/(row['id']+'.json')),path)
    for row in rows:
        for mode in ('repeat','reference'):
            for i in (0,1):old.grid.same_views(measured['first'][row['id']][i]['source_analysis'],measured[mode][row['id']][i]['source_analysis'])
            old.prior.compare(combined['first'][row['id']],combined[mode][row['id']],'full repeat/reference result')
    policies={}
    for policy in ('coarse','baseline_supported','detail_supported','preserved','agreement'):
        projected={}
        for row in rows:
            b,d=measured['first'][row['id']]
            r=b['result']['coarse'] if policy=='coarse' else b['result'] if policy=='baseline_supported' else d['result'] if policy=='detail_supported' else old.oracle(old.make_input(b,d)) if policy=='preserved' else combined['first'][row['id']]
            projected[row['id']]=dict(result=r)
        policies[policy]=old.linear.score(rows,projected,annotation)
    ids=lambda key:{(r['image_id'],m['reference_id']) for r in policies[key]['images'] for m in r['matches']}
    old_ids,new_ids=ids('coarse'),ids('agreement');coarse,candidate=policies['coarse'],policies['agreement']
    gates=dict(precision_no_coarse_regression=candidate['overall']['precision']>=coarse['overall']['precision'],recall_above_coarse=candidate['overall']['recall']>coarse['overall']['recall'],per_present_class_recall_no_coarse_regression=all(v['recall'] is None or candidate['per_class'][k]['recall']>=v['recall'] for k,v in coarse['per_class'].items()),all_coarse_matched_references_retained=old_ids<=new_ids)
    return dict(schema_version=1,protocol_sha256=digest(HERE/'profile-agreement.json'),checker_sha256=digest(__file__),viewed_development=policies,development_gates=gates,reference_changes=dict(retained=len(old_ids&new_ids),gained=[list(v) for v in sorted(new_ids-old_ids)],lost=[list(v) for v in sorted(old_ids-new_ids)]),independent_sample_quality=None,independent_sample_validated=False,fresh_sample_scored=False,adopt_candidate=False,execution=dict(new_model_forwards=0,new_native_image_runs=0,reused_parent_reports=240,reused_parent_forwards_not_counted_as_new=2640,new_development_fixture_runs=120,fixture_model_forwards=0),new_latency_or_availability_claim=False,control_permission=False)
def run(a):
    out=fresh(a.output);rows=records();design=load(HERE/'profile-agreement.json');receipt,ticket=preflight.inspect(rows,design);write(out/'preflight.json',receipt)
    require(ticket is not None and receipt==old.read(ROOT/'assets/camera-preflight-v1/rejection/valid40-preflight.json.gz'),'complete unchanged valid40 before native commands')
    control=load(a.controls/'summary.json');require(control['pass_controls'] and control['compiled_sources']==expected() and control['executable_sha256']==digest(a.binary),'same validated native executable')
    parent_control=old.read(ROOT/'assets/camera-preserved-v1/controls/summary.json.gz');pins={}
    for row in rows:
        for mode in MODES:
            for kind in ('baseline','detail'):
                path=old.parent(row['id'],mode,kind);pins[str(path.relative_to(ROOT))]=digest(path)
    write(out/'before-processing.json',dict(protocol_sha256=digest(HERE/'profile-agreement.json'),checker_sha256=digest(__file__),input_rows=rows,compiled_sources=expected(),executable_sha256=digest(a.binary),parent_sha256=pins,numeric_labels_already_viewed=True,numeric_labels_used_for_integrity=True,labels_used_for_inference=False,new_model_forwards=0))
    for mode in MODES:
        fresh(out/'fixtures'/mode);fresh(out/'combined'/mode)
        for row in rows:
            b,d=old.parents(out,row,mode,parent_control);path=out/'fixtures'/mode/(row['id']+'.json');write(path,make_input(b,d))
            check(preflight.guarded_call(ticket,row,old.native,a.binary,['--fixture',path],out/'combined'/mode/(row['id']+'.json')),path)
        print(mode+': 40 model-free fixtures saved',flush=True)
    ticket.verify_all(rows)
    write(out/'before-scoring.json',dict(protocol_sha256=digest(HERE/'profile-agreement.json'),outputs={str(p.relative_to(out)):digest(p) for p in sorted((out/'combined').rglob('*.json'))},numeric_labels_already_viewed=True,new_model_forwards=0))
    result=audit(out,rows);write(out/'summary.json',result)
    print(json.dumps(dict(viewed_development={k:v['overall'] for k,v in result['viewed_development'].items()},development_gates=result['development_gates'],independent_sample_validated=False,adopt_candidate=False,new_model_forwards=0)),flush=True)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=('controls','run'));p.add_argument('--output',type=Path,required=True);p.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-agreement');p.add_argument('--controls',type=Path)
    a=p.parse_args();require(a.command!='run' or a.controls is not None,'run requires --controls');(controls if a.command=='controls' else run)(a)
if __name__=='__main__':main()
