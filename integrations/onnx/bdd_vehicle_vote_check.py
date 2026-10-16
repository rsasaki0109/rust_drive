#!/usr/bin/env python3
"""Independent vehicle-label reconciliation and fixed viewed BDD evaluation."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
DESIGN_SHA='f09cd83c9d0f7e3b8c4484ba49df0aac606c3b322acac405185c6073f6c9078e'
TILE_CHECK_SHA='0753d2cdf819f08f837ac7f88a3a1d9cc5ae940b4eba32ec2c11cbaf15e9b110'
CLASSES=(2,5,7)


def require(condition,message):
    if not condition:raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def load(path):
    require(path.stat().st_size<=4*1024*1024,'bounded JSON')
    return json.loads(path.read_bytes())


def write(path,data):
    with path.open('x') as stream:json.dump(data,stream,indent=2,allow_nan=False);stream.write('\n')


def fresh(path):
    path=path.absolute()
    require(not path.exists() and not path.is_symlink()
            and not any(p.is_symlink() for p in path.parents),'fresh real directory')
    path.mkdir(parents=True)
    return path


def context():
    require(digest(HERE/'bdd_tiles_check.py')==TILE_CHECK_SHA,'pinned independent tile geometry')
    import bdd_tiles_check as tiles
    import bdd_s_check as old
    require(digest(HERE/'bdd_s_check.py')==tiles.MATCHER_SHA,'pinned original BDD matcher')
    design,pins,matcher=old.context()
    return tiles,old,design,matcher


def native(binary,args,output,expected=0):
    result=subprocess.run([str(binary),*map(str,args),'--output',str(output)],
                          capture_output=True,text=True,timeout=180)
    require(result.returncode==expected,'native outcome: '+result.stderr[-2000:])
    return load(output) if expected==0 else dict(exit=result.returncode,stderr=result.stderr)


def oracle(fixture,tiles):
    base=tiles.merge(fixture);restored=[];work=base['merge']['comparisons']
    for view in range(len(fixture['views'])):
        single=copy.deepcopy(fixture)
        for index,row in enumerate(single['views']):
            if index!=view:row['detections']=[]
        result=tiles.merge(single);restored.append(result);work+=result['merge']['comparisons']
    ds=base['detections'];origins=base['detection_origins'];parent=list(range(len(ds)));edges=set()
    def root(index):
        while parent[index]!=index:index=parent[index]
        return index
    for i,a in enumerate(ds):
        for j,b in enumerate(ds[:i]):
            if (a['class_index'] in CLASSES and b['class_index'] in CLASSES
                    and a['class_index']!=b['class_index'] and origins[i]['view']!=origins[j]['view']):
                work+=1
                if tiles.iou32(a['bbox'],b['bbox'])>np.float32(.45):
                    edges.add((j,i));parent[root(i)]=root(j)
    components={}
    for i in range(len(ds)):components.setdefault(root(i),[]).append(i)
    removed=set();resolutions=[];ambiguities=[]
    for members in sorted(components.values(),key=lambda c:c[0]):
        if len(members)<2:continue
        if len(members)>3 or any((j,i) not in edges for k,i in enumerate(members) for j in members[:k]):
            ambiguities.append(dict(base_ranks=members,reason='not a clique of distinct classes and views'));continue
        votes=[];kinds={ds[i]['class_index'] for i in members}
        for view,proposals in enumerate(restored):
            for rank,candidate in enumerate(proposals['detections']):
                if candidate['class_index'] not in kinds:continue
                eligible=True
                for i in members:
                    work+=1
                    if tiles.iou32(candidate['bbox'],ds[i]['bbox'])<=np.float32(.45):eligible=False;break
                if eligible:
                    votes.append(dict(view=view,source_rank=proposals['detection_origins'][rank]['rank'],
                                      class_index=candidate['class_index'],confidence=candidate['confidence']));break
        if not votes:
            ambiguities.append(dict(base_ranks=members,reason='no eligible per-view vote'));continue
        counts={kind:sum(v['class_index']==kind for v in votes) for kind in CLASSES}
        tied={kind for kind,count in counts.items() if count==max(counts.values())}
        full=next((v for v in votes if v['view']==0 and v['class_index'] in tied),None)
        winner=full if full is not None else min((v for v in votes if v['class_index'] in tied),
                                               key=lambda v:(-v['confidence'],v['class_index']))
        selected=next(i for i in members if ds[i]['class_index']==winner['class_index'])
        removed.update(i for i in members if i!=selected)
        resolutions.append(dict(base_ranks=members,selected_base_rank=selected,class_index=winner['class_index'],
                                votes=votes,full_image_tie_break=len(tied)>1 and full is not None))
    kept=[i for i in range(len(ds)) if i not in removed]
    require(work<=1200000,'bounded comparison work')
    return dict(base_merged=base,detections=[ds[i] for i in kept],detection_origins=[origins[i] for i in kept],
                retained_base_ranks=kept,resolutions=resolutions,ambiguities=ambiguities,comparison_work=work)


def controls(args):
    tiles,old,design,matcher=context();output=fresh(args.output)
    def fixture():return dict(width=1280,height=720,views=[dict(rectangle=r,detections=[]) for r in tiles.rectangles(1280,720)])
    def add(f,view,kind,score,x,width=100):
        offset=f['views'][view]['rectangle']['x']
        f['views'][view]['detections'].append(dict(class_index=kind,confidence=score,bbox=[x-offset,100.,x+width-offset,200.]))
    fixtures=[]
    for name,entries in [
        ('full-context-tie',[(0,2,.6,400),(2,5,.9,400)]),
        ('distinct-view-majority',[(0,2,.6,400),(1,2,.7,400),(2,5,.9,400)]),
        ('no-full-image-tie',[(1,2,.7,400),(2,7,.9,400)]),
        ('same-view-remains',[(0,2,.6,400),(0,5,.9,400)]),
        ('person-overlap-remains',[(0,0,.8,400),(2,2,.9,400)]),
        ('non-clique-chain',[(0,2,.9,400),(1,7,.8,430),(2,5,.7,460)]),
    ]:
        f=fixture()
        for entry in entries:add(f,*entry)
        fixtures.append((name,f))
    f=fixture();add(f,0,2,.9,400);add(f,1,2,.8,460);add(f,2,7,.7,410,140)
    fixtures.append(('repeated-class-ambiguity',f))
    rng=np.random.default_rng(20261016)
    for case in range(3):
        f=fixture()
        for row in f['views']:
            for index in range(80):
                x,y=rng.uniform([2,2],[row['rectangle']['width']-60,650]);w,h=rng.uniform([5,5],[60,60])
                row['detections'].append(dict(class_index=int(rng.choice(CLASSES)),confidence=float(rng.uniform(.3,1)),bbox=[x,y,x+w,y+h]))
        fixtures.append((f'random-{case}',f))
    fixtures.append(('max1200',load(ROOT/'assets/yolox-s-tiles-v1/final-controls/merge-5-input.json')))
    rows=[];mutants=[]
    for index,(name,f) in enumerate(fixtures):
        path=output/f'{index}-input.json';write(path,f)
        actual=native(args.binary,['--reconcile-fixture',path],output/f'{index}-actual.json')
        expected=oracle(f,tiles);tiles.same(expected,actual)
        write(output/f'{index}-oracle.json',expected)
        if index==0:
            require(len(actual['detections'])==1 and actual['detections'][0]['class_index']==2
                    and actual['resolutions'][0]['full_image_tie_break'],'full-context tie control')
            changes={
                'vote':lambda x:x['resolutions'][0]['votes'][0].__setitem__('class_index',7),
                'lineage':lambda x:x['retained_base_ranks'].__setitem__(0,0),
                'confidence':lambda x:x['detections'][0].__setitem__('confidence',.99),
                'work':lambda x:x.__setitem__('comparison_work',x['comparison_work']+1),
            }
            for key,change in changes.items():
                mutant=copy.deepcopy(actual);change(mutant)
                try:tiles.same(expected,mutant)
                except ValueError:mutants.append(key)
                else:raise ValueError('undetected control corruption: '+key)
        rows.append(dict(name=name,base_count=len(actual['base_merged']['detections']),retained=len(actual['detections']),
                         resolutions=len(actual['resolutions']),ambiguities=len(actual['ambiguities']),comparison_work=actual['comparison_work']))
    negatives=[]
    for name in ('missing-view','wrong-rectangle','bad-class','bad-confidence','bad-geometry','over-cap','unknown-field','oversized'):
        f=copy.deepcopy(fixtures[0][1])
        if name=='missing-view':f['views'].pop()
        elif name=='wrong-rectangle':f['views'][1]['rectangle']['width']+=1
        elif name=='bad-class':f['views'][0]['detections'][0]['class_index']=80
        elif name=='bad-confidence':f['views'][0]['detections'][0]['confidence']=.29
        elif name=='bad-geometry':f['views'][0]['detections'][0]['bbox'][2]=1281
        elif name=='over-cap':f['views'][0]['detections']*=301
        elif name=='unknown-field':f['simulator_truth']=True
        path=output/(name+'-input.json')
        if name=='oversized':path.write_bytes(b' '*(4*1024*1024+1))
        else:write(path,f)
        negative=native(args.binary,['--reconcile-fixture',path],output/(name+'-unexpected.json'),2)
        require(not (output/(name+'-unexpected.json')).exists(),'invalid fixture published output')
        negatives.append(dict(name=name,**negative))
    existing=output/'existing.txt';existing.write_text('preserve\n')
    result=native(args.binary,['--model','missing','--image','missing'],existing,2)
    require('fresh output required' in result['stderr'] and existing.read_text()=='preserve\n','existing output admitted input reads')
    receipt=native(args.binary,['--executable-receipt'],output/'executable.json')
    require(receipt['executable_sha256']==digest(args.binary) and not receipt['model_read']
            and not receipt['images_read'] and not receipt['inference_run'],'receipt without inference')
    summary=dict(schema_version=1,checker_sha256=digest(__file__),executable_sha256=digest(args.binary),
                 cases=rows,rejected_corruptions=mutants,rejected_inputs=negatives,existing_output_preserved=True,pass_controls=True)
    write(output/'summary.json',summary);print(json.dumps(summary))


def run(args):
    tiles,old,design,matcher=context()
    require(digest(args.design)==DESIGN_SHA,'fixed pre-label design')
    control=load(args.controls/'summary.json')
    require(control['pass_controls'] and control['checker_sha256']==digest(__file__)
            and control['executable_sha256']==digest(args.binary),'frozen controls')
    output=fresh(args.output)
    receipt=native(args.binary,['--executable-receipt'],output/'executable.json')
    for path,pin in receipt['compiled_sources'].items():require(digest(ROOT/path)==pin,'compiled source identity')
    require(digest(args.model)==receipt['model_sha256'] and args.model.stat().st_size==receipt['model_bytes'],'pinned model')
    inputs={}
    from bdd_fetch import protocol
    for image in design['images']:
        for key in ('image','annotation'):
            pin=image[key];require(digest(args.data/pin['file'])==pin['sha256'],'original input pin')
            inputs[pin['file']]=pin['sha256']
    licence=protocol()['license'];require(digest(args.data/licence['file'])==licence['sha256'],'data licence')
    inputs[licence['file']]=licence['sha256']
    freeze=dict(schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),design_sha256=DESIGN_SHA,
                checker_sha256=digest(__file__),compiled_sources=receipt['compiled_sources'],
                executable_sha256=digest(args.binary),model_sha256=digest(args.model),input_sha256=inputs,
                numeric_reference_labels_parsed=False,controls_sha256=digest(args.controls/'summary.json'),
                data_role='all eight previously viewed BDD images')
    write(output/'before-inference.json',freeze)
    passes=[]
    for mode in ('first','repeat'):
        directory=fresh(output/mode);reports={}
        for image in design['images']:
            reports[image['id']]=native(args.binary,['--model',args.model,'--image',args.data/image['image']['file']],
                                          directory/(image['id']+'.json'))
        passes.append(reports)
    write(output/'before-reference-parsing.json',dict(numeric_reference_labels_parsed=False,
          native_report_sha256={str(p.relative_to(output)):digest(p) for p in sorted(output.glob('*/*.json'))}))
    tiles.same(tiles.without_timing(passes[0]),tiles.without_timing(passes[1]))
    projected={};selected_reports={};events=[];base_compiled=None
    for image in design['images']:
        report=passes[0][image['id']];parent=report['base_prediction']
        require(report['schema_version']==1 and report['algorithm']=='yolox_s_vehicle_view_vote_v1'
                and report['compiled_sources']==receipt['compiled_sources'],'new native report identity')
        historical=load(ROOT/'assets/yolox-s-tiles-v1/final-regression/first'/(image['id']+'.json'))
        tiles.historical_same(tiles.without_timing(historical),tiles.without_timing(parent))
        require(parent['source_image_sha256']==image['image']['sha256']
                and parent['model_sha256']==receipt['model_sha256'],'original image and base model')
        tiles.thresholds(copy.deepcopy(parent))
        fixture=dict(width=parent['width'],height=parent['height'],views=[])
        for view in parent['views']:
            inner=copy.deepcopy(view['prediction']);tiles.thresholds(inner)
            inner['source_image_sha256']=image['image']['sha256'];base_compiled=inner['compiled_sources']
            old.validate_prediction(inner,dict(image,width=view['rectangle']['width'],height=view['rectangle']['height']),design,base_compiled)
            fixture['views'].append(dict(rectangle=view['rectangle'],detections=inner['detections']))
        expected=oracle(fixture,tiles);tiles.same(expected,report['result'])
        tiles.same(expected['base_merged'],{k:parent[k] for k in expected['base_merged']})
        projection=copy.deepcopy(parent['views'][0]['prediction']);tiles.thresholds(projection)
        projection['source_image_sha256']=image['image']['sha256'];projection['detections']=expected['detections']
        projected[image['id']]=projection;selected_reports[image['id']]=dict(detections=expected['detections'])
        events.append(dict(image_id=image['id'],resolutions=expected['resolutions'],ambiguities=expected['ambiguities'],
                           retained_base_ranks=expected['retained_base_ranks'],comparison_work=expected['comparison_work']))
    # The unchanged original numeric reference/matching policy starts only here.
    scored=old.score(args.data,projected,design,base_compiled,matcher)
    scored['quality_gates']=dict(design['evaluation']['quality_gates'],**{'pass':scored['viewed_quality_gates_passed']})
    previous=load(ROOT/'assets/yolox-s-tiles-v1/results.json')
    def ids(doc):return {(r['image_id'],m['reference_id']) for r in doc['images'] for m in r['matches']}
    before,after=ids(previous),ids(scored)
    preserved=before<=after and all(scored['per_class'][k]['true_positives']>=v['true_positives'] for k,v in previous['per_class'].items())
    relative=preserved and scored['overall']['false_positives']<33
    scored.update(schema_version=1,algorithm='yolox_s_vehicle_view_vote_v1',data_role='previously viewed regression',
                  previous_tiled_matches=dict(retained=len(before&after),gained=sorted(after-before),lost=sorted(before-after)),
                  relative_adoption_passed=relative,calibration=tiles.calibration(scored,selected_reports,design['class_mapping']),
                  reconciliation_events=events,integrity=dict(native_repeated_f32=True,independent_reconciliation=True,
                  historical_base_unchanged_within_absolute_tolerance=1e-5,native_image_processes=16,view_forward_passes=64))
    require(digest(args.binary)==freeze['executable_sha256'] and digest(__file__)==freeze['checker_sha256']
            and all(digest(ROOT/p)==pin for p,pin in receipt['compiled_sources'].items()),'frozen executable/sources unchanged')
    write(output/'results.json',scored)
    print(json.dumps(dict(overall=scored['overall'],relative_adoption_passed=relative,previous_tiled_matches=scored['previous_tiled_matches'],
                          original_quality_passed=scored['quality_gates']['pass'])))
    return 0 if scored['quality_gates']['pass'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('controls','run'))
    parser.add_argument('--binary',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--design',type=Path,default=ROOT/'assets/yolox-vehicle-vote-v1/design.json')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--data',type=Path,default=ROOT/'artifacts/bdd-camera')
    args=parser.parse_args();args.binary=args.binary.absolute()
    if args.command=='controls':controls(args);return 0
    require(args.controls is not None,'controls required before inference');return run(args)


if __name__=='__main__':sys.exit(main())
