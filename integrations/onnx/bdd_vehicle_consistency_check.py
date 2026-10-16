#!/usr/bin/env python3
"""Independent ambiguity-preserving Rust postprocessing on recorded native views."""
import argparse
import copy
import importlib.util
from pathlib import Path

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
SPEC=importlib.util.spec_from_file_location('vehicle_context_support',HERE/'bdd_vehicle_context_check.py')
context_support=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(context_support)
require,load,write,fresh,digest,native=(context_support.require,context_support.load,context_support.write,
                                    context_support.fresh,context_support.digest,context_support.native)
CONTEXT_SHA='c7af90491ed3d526e6adb71dc61955213b4ea13001e9eca9c8627281ac75eff3'
DESIGN_SHA='d6ba48d8852b7aa92eb717a0cdd596f90a8eded4d8f343f10989b24eb28e797f'


def context():
    require(digest(HERE/'bdd_vehicle_context_check.py')==CONTEXT_SHA,'unchanged full-context checker')
    return context_support.context()


def parent_report(path):
    body=load(path)
    return body['base_prediction'] if 'base_prediction' in body else body


def oracle(fixture,tiles):
    prior=context_support.oracle(fixture,tiles);voting=prior['voting_result'];base=voting['base_merged']
    full=copy.deepcopy(fixture)
    for row in full['views'][1:]:row['detections']=[]
    full=tiles.merge(full);work=voting['comparison_work']+full['merge']['comparisons']
    removed=set();decisions=[]
    for component in voting['resolutions']:
        kinds={base['detections'][i]['class_index'] for i in component['base_ranks']};support=[]
        for rank,candidate in enumerate(full['detections']):
            if candidate['class_index'] not in kinds:continue
            eligible=True
            for index in component['base_ranks']:
                work+=1
                if tiles.iou32(candidate['bbox'],base['detections'][index]['bbox'])<=tiles.np.float32(.45):eligible=False;break
            if eligible:support.append(dict(source_rank=full['detection_origins'][rank]['rank'],
                                            class_index=candidate['class_index'],confidence=candidate['confidence']))
        unique={v['class_index'] for v in support};selected=None
        if len(unique)==1:
            kind=next(iter(unique));selected=next(i for i in component['base_ranks'] if base['detections'][i]['class_index']==kind)
            removed.update(i for i in component['base_ranks'] if i!=selected)
        decisions.append(dict(base_ranks=component['base_ranks'],full_image_support=support,selected_base_rank=selected))
    kept=[i for i in range(len(base['detections'])) if i not in removed];require(work<=1400000,'comparison work bound')
    return dict(context_result=prior,detections=[base['detections'][i] for i in kept],
                detection_origins=[base['detection_origins'][i] for i in kept],retained_base_ranks=kept,
                decisions=decisions,comparison_work=work)


def controls(args):
    tiles,old,design,matcher=context();out=fresh(args.output)
    source=ROOT/'assets/yolox-vehicle-context-v1/final-controls'
    fixtures=[(p.stem,load(p)) for p in sorted(source.glob('[0-9]*-input.json'),key=lambda p:int(p.name.split('-')[0]))]
    require(len(fixtures)==12,'all prior full-context controls')
    f=copy.deepcopy(fixtures[0][1])
    for row in f['views']:row['detections']=[]
    for view,kind,score in [(0,2,.6),(0,5,.85),(2,5,.9)]:
        offset=f['views'][view]['rectangle']['x']
        f['views'][view]['detections'].append(dict(class_index=kind,confidence=score,bbox=[400-offset,100.,500-offset,200.]))
    fixtures.append(('full-image-is-itself-ambiguous',f));rows=[]
    for i,(name,f) in enumerate(fixtures):
        path=out/f'{i}-input.json';write(path,f)
        actual=native(args.binary,['--reconcile-fixture',path],out/f'{i}-actual.json');expected=oracle(f,tiles)
        tiles.same(expected,actual);write(out/f'{i}-oracle.json',expected)
        if i==12:require(len(actual['detections'])==2 and actual['decisions'][0]['selected_base_rank'] is None,'ambiguous full image must preserve both labels')
        rows.append(dict(name=name,retained=len(actual['detections']),decisions=len(actual['decisions']),work=actual['comparison_work']))
    # Validate actual producer fields and its complete recomputed global merge.
    parent=parent_report(args.parents/'first'/('00a395fe-d60c0b47.json'))
    operations={
        'model':lambda p:p.__setitem__('model_sha256','0'*64),
        'producer-sources':lambda p:p.__setitem__('compiled_sources',{}),
        'threshold':lambda p:p.__setitem__('score_threshold',.2),
        'profile':lambda p:p['views'][0]['prediction'].__setitem__('model_profile_sha256','0'*64),
        'tensor':lambda p:p['views'][0]['prediction'].__setitem__('input_shape',[1,3,320,320]),
        'pixel-hash':lambda p:p['views'][0].__setitem__('rgb_sha256','invalid'),
        'box':lambda p:p['detections'][0]['bbox'].__setitem__(0,p['detections'][0]['bbox'][0]+1),
        'lineage':lambda p:p['detection_origins'][0].__setitem__('rank',999),
        'merge-work':lambda p:p['merge'].__setitem__('comparisons',p['merge']['comparisons']+1),
    };rejected=[]
    for name,change in operations.items():
        mutant=copy.deepcopy(parent);change(mutant);path=out/(name+'-input.json');write(path,mutant)
        receipt=native(args.binary,['--input',path],out/(name+'-unexpected.json'),2)
        require(not (out/(name+'-unexpected.json')).exists(),'invalid producer published output');rejected.append(dict(name=name,**receipt))
    existing=out/'existing.txt';existing.write_text('preserve\n')
    native(args.binary,['--input','missing'],existing,2);require(existing.read_text()=='preserve\n','existing file overwritten')
    receipt=native(args.binary,['--executable-receipt'],out/'executable.json')
    require(receipt['executable_sha256']==digest(args.binary) and not receipt['inference_run'],'postprocessor receipt')
    summary=dict(schema_version=1,checker_sha256=digest(__file__),executable_sha256=digest(args.binary),
                 cases=rows,rejected_producer_corruptions=rejected,existing_output_preserved=True,pass_controls=True)
    write(out/'summary.json',summary)


def run(args):
    tiles,old,design,matcher=context();require(digest(args.design)==DESIGN_SHA,'fixed consistency design')
    control=load(args.controls/'summary.json')
    require(control['pass_controls'] and control['checker_sha256']==digest(__file__)
            and control['executable_sha256']==digest(args.binary),'matching controls')
    out=fresh(args.output);receipt=native(args.binary,['--executable-receipt'],out/'executable.json')
    for path,pin in receipt['compiled_sources'].items():require(digest(ROOT/path)==pin,'compiled sources')
    parent_pins={str(p.relative_to(args.parents)):digest(p) for p in sorted(args.parents.glob('*/*.json'))}
    require(len(parent_pins)==16,'all first/repeat native image reports')
    write(out/'before-postprocessing.json',dict(design_sha256=DESIGN_SHA,checker_sha256=digest(__file__),
          executable_sha256=digest(args.binary),compiled_sources=receipt['compiled_sources'],native_parent_sha256=parent_pins,
          numeric_references_parsed_in_this_process=False,model_inference_run_in_this_process=False,
          data_role='viewed development; source detector already executed on all8 images twice'))
    passes=[]
    for mode in ('first','repeat'):
        directory=fresh(out/mode);inputs=fresh(out/('input-'+mode));reports={}
        for image in design['images']:
            parent=parent_report(args.parents/mode/(image['id']+'.json'));path=inputs/(image['id']+'.json');write(path,parent)
            report=native(args.binary,['--input',path],directory/(image['id']+'.json'))
            require(report['source_report_sha256']==digest(path) and report['source_image_sha256']==image['image']['sha256']
                    and report['compiled_sources']==receipt['compiled_sources'] and not report['model_read']
                    and not report['images_read'] and not report['inference_run'],'postprocessing provenance')
            reports[image['id']]=report
        passes.append(reports)
    write(out/'before-reference-parsing.json',dict(numeric_references_parsed_in_this_process=False,
          report_sha256={str(p.relative_to(out)):digest(p) for p in sorted(out.glob('*/*.json'))}))
    projected={};selected={};events=[];compiled=None
    for image in design['images']:
        first=passes[0][image['id']];repeat=passes[1][image['id']]
        require(first['schema_version']==1 and first['algorithm']=='yolox_s_vehicle_consistency_v1','report identity')
        tiles.same(first['result'],repeat['result'])
        parent=parent_report(args.parents/'first'/(image['id']+'.json'))
        historical=load(ROOT/'assets/yolox-s-tiles-v1/final-regression/first'/(image['id']+'.json'))
        tiles.historical_same(tiles.without_timing(historical),tiles.without_timing(parent))
        fixture=dict(width=parent['width'],height=parent['height'],views=[dict(rectangle=v['rectangle'],detections=v['prediction']['detections']) for v in parent['views']])
        expected=oracle(fixture,tiles);tiles.same(expected,first['result'])
        inner=copy.deepcopy(parent['views'][0]['prediction']);tiles.thresholds(inner)
        compiled=inner['compiled_sources'];inner['source_image_sha256']=image['image']['sha256'];inner['detections']=expected['detections']
        projected[image['id']]=inner;selected[image['id']]=dict(detections=expected['detections'])
        events.append(dict(image_id=image['id'],decisions=expected['decisions'],retained_base_ranks=expected['retained_base_ranks'],comparison_work=expected['comparison_work']))
    scored=old.score(args.data,projected,design,compiled,matcher);previous=load(ROOT/'assets/yolox-s-tiles-v1/results.json')
    def ids(doc):return {(r['image_id'],m['reference_id']) for r in doc['images'] for m in r['matches']}
    before,after=ids(previous),ids(scored)
    relative=before<=after and scored['overall']['false_positives']<33 and all(scored['per_class'][k]['true_positives']>=v['true_positives'] for k,v in previous['per_class'].items())
    scored.update(schema_version=1,algorithm='yolox_s_vehicle_consistency_v1',data_role='third viewed development design; no fresh holdout',
        quality_gates=dict(design['evaluation']['quality_gates'],**{'pass':scored['viewed_quality_gates_passed']}),
        previous_tiled_matches=dict(retained=len(before&after),gained=sorted(after-before),lost=sorted(before-after)),
        relative_adoption_passed=relative,calibration=tiles.calibration(scored,selected,design['class_mapping']),decisions=events,
        integrity=dict(independent_full_support_reconstruction=True,postprocessing_runs=16,postprocessing_repeat_identical=True,
                       new_model_forward_passes=0,source_native_image_reports=16))
    require(all(digest(args.parents/name)==pin for name,pin in parent_pins.items())
            and digest(__file__)==control['checker_sha256'] and digest(args.binary)==receipt['executable_sha256'],'sources changed during postprocessing')
    write(out/'results.json',scored)
    print({'overall':scored['overall'],'retained':len(before&after),'relative_passed':relative,'original_quality_passed':scored['quality_gates']['pass']})
    return 0 if scored['quality_gates']['pass'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('command',choices=('controls','run'))
    parser.add_argument('--binary',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--controls',type=Path)
    parser.add_argument('--parents',type=Path,default=ROOT/'assets/yolox-vehicle-context-v1/first')
    parser.add_argument('--data',type=Path,default=ROOT/'artifacts/bdd-camera')
    parser.add_argument('--design',type=Path,default=ROOT/'assets/yolox-vehicle-consistency-v1/design.json')
    args=parser.parse_args();args.binary=args.binary.absolute()
    if args.command=='controls':controls(args);return 0
    require(args.controls is not None,'controls required');return run(args)


if __name__=='__main__':raise SystemExit(main())
