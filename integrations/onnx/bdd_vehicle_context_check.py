#!/usr/bin/env python3
"""Source-bound full-context vehicle reconciliation, preserving failed voting."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
SPEC=importlib.util.spec_from_file_location('vehicle_vote_support',HERE/'bdd_vehicle_vote_check.py')
support=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(support)
require,load,write,fresh,digest,native=(support.require,support.load,support.write,support.fresh,support.digest,support.native)
VOTE_SHA='69c1708d1f441567fb09564a56003f30305f8bd2ca2e364221f269743a0f8803'
DESIGN_SHA='5188597210feb0d7b6ac68a6d08f2d567cea0668abfca81d75ad7e0195e3d591'


def context():
    require(digest(HERE/'bdd_vehicle_vote_check.py')==VOTE_SHA,'unchanged failed vote checker')
    return support.context()


def oracle(fixture,tiles):
    voting=support.oracle(fixture,tiles);base=voting['base_merged'];removed=set();selections=[];ambiguous=[]
    for component in voting['resolutions']:
        full=next((v for v in component['votes'] if v['view']==0),None)
        if full is None:ambiguous.append(component['base_ranks']);continue
        selected=next(i for i in component['base_ranks'] if base['detections'][i]['class_index']==full['class_index'])
        removed.update(i for i in component['base_ranks'] if i!=selected)
        selections.append(dict(base_ranks=component['base_ranks'],selected_base_rank=selected,
                               class_index=full['class_index'],full_image_source_rank=full['source_rank']))
    kept=[i for i in range(len(base['detections'])) if i not in removed]
    return dict(voting_result=voting,detections=[base['detections'][i] for i in kept],
                detection_origins=[base['detection_origins'][i] for i in kept],retained_base_ranks=kept,
                selections=selections,ambiguous_without_full_context=ambiguous)


def controls(args):
    tiles,old,design,matcher=context();output=fresh(args.output)
    original=ROOT/'assets/yolox-vehicle-vote-v1/controls'
    cases=[(p.stem,load(p)) for p in sorted(original.glob('[0-9]*-input.json'),key=lambda p:int(p.name.split('-')[0]))]
    require(len(cases)==11,'all original voting controls')
    f=copy.deepcopy(cases[0][1])
    for row in f['views']:row['detections']=[]
    for view,kind,score in [(0,2,.6),(1,5,.8),(2,5,.9),(3,5,.85)]:
        x=f['views'][view]['rectangle']['x']
        f['views'][view]['detections'].append(dict(class_index=kind,confidence=score,bbox=[600-x,100.,700-x,200.]))
    cases.append(('full-anchor-versus-three-correlated-crops',f))
    rows=[];corruptions=[]
    for index,(name,fixture) in enumerate(cases):
        path=output/f'{index}-input.json';write(path,fixture)
        actual=native(args.binary,['--reconcile-fixture',path],output/f'{index}-actual.json')
        expected=oracle(fixture,tiles);tiles.same(expected,actual);write(output/f'{index}-oracle.json',expected)
        if index==0:
            changes={
                'full-source-rank':lambda r:r['selections'][0].__setitem__('full_image_source_rank',99),
                'confidence':lambda r:r['detections'][0].__setitem__('confidence',.99),
                'retained-lineage':lambda r:r['retained_base_ranks'].__setitem__(0,0),
                'preserved-vote':lambda r:r['voting_result']['resolutions'][0]['votes'][0].__setitem__('class_index',7),
            }
            for key,change in changes.items():
                mutant=copy.deepcopy(actual);change(mutant)
                try:tiles.same(expected,mutant)
                except ValueError:corruptions.append(key)
                else:raise ValueError('undetected corruption: '+key)
        if index==2:require(len(actual['detections'])==2 and len(actual['ambiguous_without_full_context'])==1,'no full context keeps both hypotheses')
        if index==11:require(actual['voting_result']['detections'][0]['class_index']==5
                              and actual['detections'][0]['class_index']==2
                              and len(actual['detections'])==1,'full image overrides correlated crop majority')
        rows.append(dict(name=name,base=len(actual['voting_result']['base_merged']['detections']),retained=len(actual['detections']),
                         selections=len(actual['selections']),without_full_context=len(actual['ambiguous_without_full_context']),
                         comparison_work=actual['voting_result']['comparison_work']))
    rejected=[]
    for name in ('missing-view','wrong-rectangle','bad-class','bad-confidence','bad-geometry','over-cap','unknown-field','oversized'):
        if name=='oversized':
            path=output/'oversized-input.json';path.write_bytes(b' '*(4*1024*1024+1))
        else:path=original/(name+'-input.json')
        result=native(args.binary,['--reconcile-fixture',path],output/(name+'-unexpected.json'),2)
        require(not (output/(name+'-unexpected.json')).exists(),'invalid fixture produced report')
        rejected.append(dict(name=name,**result))
    path=output/'existing.txt';path.write_text('preserve\n')
    result=native(args.binary,['--model','missing','--image','missing'],path,2)
    require('fresh output required' in result['stderr'] and path.read_text()=='preserve\n','existing output read inputs')
    receipt=native(args.binary,['--executable-receipt'],output/'executable.json')
    require(receipt['executable_sha256']==digest(args.binary) and not receipt['model_read'] and not receipt['inference_run'],'binary receipt')
    summary=dict(schema_version=1,checker_sha256=digest(__file__),vote_checker_sha256=VOTE_SHA,
                 executable_sha256=digest(args.binary),cases=rows,rejected_corruptions=corruptions,
                 rejected_inputs=rejected,existing_output_preserved=True,pass_controls=True)
    write(output/'summary.json',summary);print(json.dumps(summary))


def run(args):
    tiles,old,design,matcher=context()
    require(digest(args.design)==DESIGN_SHA,'fixed full-context design')
    control=load(args.controls/'summary.json')
    require(control['pass_controls'] and control['checker_sha256']==digest(__file__)
            and control['executable_sha256']==digest(args.binary),'matching pretrial controls')
    output=fresh(args.output);receipt=native(args.binary,['--executable-receipt'],output/'executable.json')
    for path,pin in receipt['compiled_sources'].items():require(digest(ROOT/path)==pin,'compiled source identity')
    require(digest(args.model)==receipt['model_sha256'] and args.model.stat().st_size==receipt['model_bytes'],'model identity')
    from bdd_fetch import protocol
    inputs={}
    for image in design['images']:
        for key in ('image','annotation'):
            pin=image[key];require(digest(args.data/pin['file'])==pin['sha256'],'source image/label identity')
            inputs[pin['file']]=pin['sha256']
    licence=protocol()['license'];require(digest(args.data/licence['file'])==licence['sha256'],'original data licence')
    inputs[licence['file']]=licence['sha256']
    freeze=dict(schema_version=1,design_sha256=DESIGN_SHA,checker_sha256=digest(__file__),
                vote_checker_sha256=VOTE_SHA,compiled_sources=receipt['compiled_sources'],executable_sha256=digest(args.binary),
                model_sha256=digest(args.model),input_sha256=inputs,controls_sha256=digest(args.controls/'summary.json'),
                numeric_reference_labels_parsed=False,data_role='viewed development after failed majority; no fresh holdout')
    write(output/'before-inference.json',freeze);passes=[]
    for mode in ('first','repeat'):
        directory=fresh(output/mode);reports={}
        for image in design['images']:
            reports[image['id']]=native(args.binary,['--model',args.model,'--image',args.data/image['image']['file']],
                                          directory/(image['id']+'.json'))
        passes.append(reports)
    write(output/'before-reference-parsing.json',dict(numeric_reference_labels_parsed=False,
          native_report_sha256={str(p.relative_to(output)):digest(p) for p in sorted(output.glob('*/*.json'))}))
    tiles.same(tiles.without_timing(passes[0]),tiles.without_timing(passes[1]))
    projected={};vote_projected={};selected_reports={};events=[];compiled=None
    for image in design['images']:
        report=passes[0][image['id']];parent=report['base_prediction']
        require(report['schema_version']==1 and report['algorithm']=='yolox_s_vehicle_full_context_v1'
                and report['compiled_sources']==receipt['compiled_sources'],'full-context native identity')
        historical=load(ROOT/'assets/yolox-s-tiles-v1/final-regression/first'/(image['id']+'.json'))
        tiles.historical_same(tiles.without_timing(historical),tiles.without_timing(parent))
        require(parent['source_image_sha256']==image['image']['sha256']
                and parent['model_sha256']==receipt['model_sha256'],'base image/model')
        fixture=dict(width=parent['width'],height=parent['height'],views=[])
        tiles.thresholds(copy.deepcopy(parent))
        for view in parent['views']:
            inner=copy.deepcopy(view['prediction']);tiles.thresholds(inner);inner['source_image_sha256']=image['image']['sha256']
            compiled=inner['compiled_sources']
            old.validate_prediction(inner,dict(image,width=view['rectangle']['width'],height=view['rectangle']['height']),design,compiled)
            fixture['views'].append(dict(rectangle=view['rectangle'],detections=inner['detections']))
        expected=oracle(fixture,tiles);tiles.same(expected,report['result'])
        tiles.same(expected['voting_result']['base_merged'],{k:parent[k] for k in expected['voting_result']['base_merged']})
        old_vote=load(ROOT/'assets/yolox-vehicle-vote-v1/first/first'/(image['id']+'.json'))
        tiles.historical_same(old_vote['result'],expected['voting_result'])
        projection=copy.deepcopy(parent['views'][0]['prediction']);tiles.thresholds(projection)
        projection['source_image_sha256']=image['image']['sha256'];projection['detections']=expected['detections']
        projected[image['id']]=projection;selected_reports[image['id']]=dict(detections=expected['detections'])
        vote_projection=copy.deepcopy(projection);vote_projection['detections']=expected['voting_result']['detections'];vote_projected[image['id']]=vote_projection
        events.append(dict(image_id=image['id'],selections=expected['selections'],ambiguous_without_full_context=expected['ambiguous_without_full_context'],
                           original_vote_resolutions=expected['voting_result']['resolutions'],retained_base_ranks=expected['retained_base_ranks']))
    scored=old.score(args.data,projected,design,compiled,matcher)
    failed_vote=old.score(args.data,vote_projected,design,compiled,matcher)
    tiles.historical_same(load(ROOT/'assets/yolox-vehicle-vote-v1/results.json')['overall'],failed_vote['overall'])
    previous=load(ROOT/'assets/yolox-s-tiles-v1/results.json')
    def ids(doc):return {(r['image_id'],m['reference_id']) for r in doc['images'] for m in r['matches']}
    before,after=ids(previous),ids(scored)
    preserved=before<=after and all(scored['per_class'][k]['true_positives']>=v['true_positives'] for k,v in previous['per_class'].items())
    relative=preserved and scored['overall']['false_positives']<33
    scored.update(schema_version=1,algorithm='yolox_s_vehicle_full_context_v1',data_role='post-failure viewed development',
                  quality_gates=dict(design['evaluation']['quality_gates'],**{'pass':scored['viewed_quality_gates_passed']}),
                  previous_tiled_matches=dict(retained=len(before&after),gained=sorted(after-before),lost=sorted(before-after)),
                  relative_adoption_passed=relative,calibration=tiles.calibration(scored,selected_reports,design['class_mapping']),
                  failed_vote_result=failed_vote,reconciliation_events=events,
                  integrity=dict(native_repeated_f32=True,independent_reconciliation=True,failed_majority_unchanged=True,
                  historical_base_unchanged_within_absolute_tolerance=1e-5,native_image_processes=16,view_forward_passes=64))
    require(digest(args.binary)==freeze['executable_sha256'] and digest(__file__)==freeze['checker_sha256']
            and all(digest(ROOT/p)==pin for p,pin in receipt['compiled_sources'].items()),'sources/binary unchanged during inference')
    write(output/'results.json',scored)
    print(json.dumps(dict(overall=scored['overall'],relative_adoption_passed=relative,previous_tiled_matches=scored['previous_tiled_matches'],
                          original_quality_passed=scored['quality_gates']['pass'])))
    return 0 if scored['quality_gates']['pass'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('controls','run'))
    parser.add_argument('--binary',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--design',type=Path,default=ROOT/'assets/yolox-vehicle-context-v1/design.json')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--data',type=Path,default=ROOT/'artifacts/bdd-camera')
    args=parser.parse_args();args.binary=args.binary.absolute()
    if args.command=='controls':controls(args);return 0
    require(args.controls is not None,'pretrial controls required');return run(args)


if __name__=='__main__':raise SystemExit(main())
