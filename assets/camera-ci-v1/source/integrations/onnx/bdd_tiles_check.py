#!/usr/bin/env python3
"""Independent crop/merge controls and frozen, viewed BDD multi-view regression.

The historical matcher, categories, references, and quality gates remain pinned.
No label parsing occurs until both complete native inference passes are saved.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DESIGN_SHA = 'eea7f87cb2dba4f9cdda9cc9ea2d139e62565a199b517d128077290b7f4a0859'
MATCHER_SHA = '35053e7a0e329df354cf694936832c05a7f502e4af7b97337cacf4f9807a25d9'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path):
    require(Path(path).stat().st_size <= 64*1024*1024, 'bounded JSON')
    return json.loads(Path(path).read_bytes())


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def fresh(path):
    path = Path(path).absolute()
    require(not path.exists() and not path.is_symlink(), 'fresh output required')
    require(not any(p.is_symlink() for p in path.parents), 'symlink output parent')
    path.mkdir(parents=True)
    return path


def native(binary, args, output, success=True):
    result = subprocess.run([str(binary), *map(str, args), '--output', str(output)],
                            capture_output=True, text=True, timeout=180)
    require((result.returncode == 0) == success, 'native outcome: '+result.stderr[-2000:])
    if success:
        return load(output)
    return dict(exit=result.returncode, stderr=result.stderr)


def rectangles(width, height):
    result = [dict(x=0, y=0, width=width, height=height)]
    side, gap = min(width, height), abs(width-height)
    for offset in (0, gap//2, gap):
        rectangle = dict(x=offset if width >= height else 0,
                         y=offset if width < height else 0, width=side, height=side)
        if rectangle not in result:
            result.append(rectangle)
    return result


def iou32(first, second):
    # Independent array formulation; arithmetic is deliberately f32 like the ABI.
    a, b = np.asarray(first, dtype=np.float32), np.asarray(second, dtype=np.float32)
    overlap = np.maximum(np.float32(0), np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2]))
    intersection = overlap[0]*overlap[1]
    sa, sb = np.maximum(np.float32(0), a[2:]-a[:2]), np.maximum(np.float32(0), b[2:]-b[:2])
    union = sa[0]*sa[1]+sb[0]*sb[1]-intersection
    return intersection/union if union > 0 else np.float32(0)


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
    require(stats['comparisons'] <= 360000, 'comparison bound')
    return dict(detections=kept, detection_origins=origins, merge=stats)


def same(expected, actual, path='$'):
    if isinstance(expected, dict):
        require(isinstance(actual, dict) and expected.keys() == actual.keys(), 'keys: '+path)
        for key in expected:
            same(expected[key], actual[key], path+'.'+str(key))
    elif isinstance(expected, list):
        require(isinstance(actual, list) and len(expected) == len(actual), 'length: '+path)
        for index, (a, b) in enumerate(zip(expected, actual)):
            same(a, b, path+f'[{index}]')
    elif type(expected) is float:
        require(type(actual) in (int, float) and np.isfinite(actual)
                and np.float32(expected) == np.float32(actual), 'f32 measurement: '+path)
    else:
        require(type(expected) is type(actual) and expected == actual, 'identity: '+path)


def historical_same(expected, actual, path='$'):
    # Keep the historical S replay's existing cross-host tolerance. Local
    # repeat inference and coordinate/NMS controls above still compare f32.
    if type(expected) is float:
        require(type(actual) in (int,float) and np.isfinite(actual)
                and abs(float(np.float32(expected))-float(np.float32(actual)))<=1e-5,
                'historical measurement: '+path)
    elif isinstance(expected,dict):
        require(isinstance(actual,dict) and expected.keys()==actual.keys(),'historical keys: '+path)
        for key in expected:historical_same(expected[key],actual[key],path+'.'+str(key))
    elif isinstance(expected,list):
        require(isinstance(actual,list) and len(expected)==len(actual),'historical length: '+path)
        for index,(a,b) in enumerate(zip(expected,actual)):historical_same(a,b,path+f'[{index}]')
    else:
        require(type(expected) is type(actual) and expected==actual,'historical identity: '+path)


def without_timing(value):
    if isinstance(value, dict):
        return {k:without_timing(v) for k,v in value.items()
                if k not in ('inference_seconds', 'model_load_seconds')}
    if isinstance(value, list):
        return list(map(without_timing, value))
    return value


def thresholds(report):
    # serde_json::Value preserves f32 as its exact promoted f64 value. Compare
    # the f32 ABI, then adapt only metadata to the immutable matcher's JSON form.
    require(type(report['score_threshold']) is float and type(report['nms_iou_threshold']) is float
            and report['score_threshold'] in (.3, float(np.float32(.3)))
            and report['nms_iou_threshold'] in (.45, float(np.float32(.45))), 'fixed f32 thresholds')
    report['score_threshold'] = .3
    report['nms_iou_threshold'] = .45


def controls(binary, output):
    out = fresh(output)
    crop_rows, merge_rows, rejected = [], [], []
    for width, height in [(1280,720),(720,1281),(37,37),(38,37),(37,39),(1,1)]:
        y, x = np.indices((height, width), dtype=np.uint32)
        rgb = np.stack([(x*7+y)%256, (y*11+x)%256, (x+y*3)%256], axis=-1).astype(np.uint8)
        image = out/f'pixels-{width}-{height}.png'
        Image.fromarray(rgb).save(image)
        report = native(binary, ['--inspect-views','--image',image], out/f'crops-{width}-{height}.json')
        expected = []
        for rect in rectangles(width,height):
            crop = rgb[rect['y']:rect['y']+rect['height'], rect['x']:rect['x']+rect['width']]
            expected.append(dict(rectangle=rect, rgb_sha256=hashlib.sha256(crop.tobytes()).hexdigest()))
        same(expected, report['views'])
        require(report['source_image_sha256'] == digest(image) and not report['model_read']
                and not report['inference_run'], 'model-free crop control')
        crop_rows.append(dict(width=width,height=height,views=len(expected),pass_check=True))
    rng = np.random.default_rng(462701)
    fixtures = []
    for width,height in [(1280,720),(720,1281),(80,80),(81,80)]:
        fixture = dict(width=width,height=height,views=[])
        for rect in rectangles(width,height):
            detections = []
            for _ in range(80):
                a = rng.uniform([0,0],[rect['width']-2,rect['height']-2])
                b = rng.uniform(a+1,[rect['width'],rect['height']])
                detections.append(dict(class_index=int(rng.integers(0,8)),
                                       confidence=float(rng.uniform(.3,1)),bbox=[*a,*b]))
            fixture['views'].append(dict(rectangle=rect,detections=detections))
        fixtures.append(fixture)
    hand = dict(width=1280,height=720,views=[dict(rectangle=r,detections=[]) for r in rectangles(1280,720)])
    for view, kind, confidence, box in [(0,2,.8,[300,100,400,200]),(2,2,.8,[20,100,120,200]),
                                      (2,0,.7,[20,100,120,200]),(2,7,.9,[1,10,30,30]),
                                      (1,9,.95,[0,0,5,5]),(3,9,.94,[715,0,720,5])]:
        hand['views'][view]['detections'].append(dict(class_index=kind,confidence=confidence,bbox=box))
    fixtures.append(hand)
    cap = dict(width=1280,height=720,views=[dict(rectangle=r,detections=[]) for r in rectangles(1280,720)])
    for row in cap['views']:
        row['detections'] = [dict(class_index=k%80, confidence=.9,
                                  bbox=[float(2+(k//80)*15),float(2+(k%80)*7),
                                        float(5+(k//80)*15),float(5+(k%80)*7)]) for k in range(300)]
    fixtures.append(cap)
    for index, fixture in enumerate(fixtures):
        path = out/f'merge-{index}-input.json'
        write(path,fixture)
        actual = native(binary,['--merge-fixture',path],out/f'merge-{index}-actual.json')
        expected = merge(fixture)
        same(expected,actual)
        write(out/f'merge-{index}-oracle.json',expected)
        merge_rows.append(dict(index=index,**actual['merge'],kept=len(actual['detections']),pass_check=True))
    invalid = []
    for change in ('missing_view','wrong_rectangle','over_cap','bad_class','bad_confidence','bad_geometry','unknown_field','oversize_dimensions'):
        fixture = copy.deepcopy(hand)
        if change == 'missing_view': fixture['views'].pop()
        elif change == 'wrong_rectangle': fixture['views'][2]['rectangle']['x'] += 1
        elif change == 'over_cap': fixture['views'][0]['detections'] *= 301
        elif change == 'bad_class': fixture['views'][0]['detections'][0]['class_index'] = 80
        elif change == 'bad_confidence': fixture['views'][0]['detections'][0]['confidence'] = .29
        elif change == 'bad_geometry': fixture['views'][0]['detections'][0]['bbox'][2] = 1281
        elif change == 'unknown_field': fixture['simulator_truth'] = 1
        elif change == 'oversize_dimensions': fixture['width'] = 16000001
        invalid.append((change,fixture))
    for name, fixture in invalid:
        path = out/(name+'-input.json')
        write(path,fixture)
        result = native(binary,['--merge-fixture',path],out/(name+'-unexpected.json'),False)
        require(not (out/(name+'-unexpected.json')).exists(),'failed merge published report')
        rejected.append(dict(name=name,**result))
    existing = out/'existing.json'
    existing.write_text('preserve this exact output\n')
    result = native(binary,['--model','missing-model','--image','missing-image'],existing,False)
    require('fresh report required' in result['stderr'] and existing.read_text() == 'preserve this exact output\n',
            'existing output must reject before model read')
    receipt = native(binary,['--executable-receipt'],out/'executable.json')
    threshold_controls = []
    for score,nms,accepted in [(.3,.45,True),(float(np.float32(.3)),float(np.float32(.45)),True),
                               (.299,.45,False),(.3,.451,False),(True,.45,False),
                               (float('nan'),.45,False)]:
        try:
            thresholds(dict(score_threshold=score,nms_iou_threshold=nms))
            passed = True
        except ValueError:
            passed = False
        require(passed == accepted,'threshold representation control')
        threshold_controls.append(dict(score=repr(score),nms=repr(nms),expected_accept=accepted))
    summary = dict(schema_version=1,kind='independent_yolox_tiles_controls',
                   executable_sha256=digest(binary),checker_sha256=digest(__file__),
                   crop_controls=crop_rows,merge_controls=merge_rows,rejections=rejected,
                   threshold_representation_controls=threshold_controls,
                   existing_output_preserved=True,pass_controls=True)
    require(receipt['executable_sha256'] == summary['executable_sha256'],'binary receipt')
    write(out/'summary.json',summary)
    print(json.dumps(summary))


def calibration(scored, reports, mapping):
    observations = []
    for row in scored['images']:
        selected = [d for d in reports[row['image_id']]['detections'] if d['class_index'] in mapping.values()]
        correct = {m['prediction_index'] for m in row['matches']}
        observations.extend((d['confidence'], int(k in correct)) for k,d in enumerate(selected))
    bins = []
    for lower,upper in zip(np.linspace(0,1,6)[:-1],np.linspace(0,1,6)[1:]):
        rows = [(s,y) for s,y in observations if lower <= s and (s < upper or upper == 1)]
        bins.append(dict(lower=float(lower),upper=float(upper),count=len(rows),
                         mean_confidence=sum(s for s,_ in rows)/len(rows) if rows else None,
                         matched_fraction=sum(y for _,y in rows)/len(rows) if rows else None))
    n = len(observations)
    return dict(kind='descriptive viewed selected-detection reliability; no fitted calibration',
                scoped_predictions=n,bins=bins,
                brier=sum((s-y)**2 for s,y in observations)/n if n else None,
                expected_calibration_error=sum(b['count']*abs(b['mean_confidence']-b['matched_fraction'])
                                               for b in bins if b['count'])/n if n else None,
                limitations='Conditional on score >=0.3 and emitted boxes; excludes undetected objects; viewed data; not a calibrated probability certificate.')


def run(args):
    require(digest(args.design) == DESIGN_SHA,'fixed tile design')
    require(digest(HERE/'bdd_s_check.py') == MATCHER_SHA,'historical checker pin')
    import bdd_s_check as old
    design, pins, oracle = old.context()
    controls_report = load(args.controls/'summary.json')
    require(controls_report['pass_controls'] and controls_report['executable_sha256'] == digest(args.binary)
            and controls_report['checker_sha256'] == digest(__file__),'controls/source/binary freeze')
    out = fresh(args.output)
    receipt = native(args.binary,['--executable-receipt'],out/'executable.json')
    compiled = receipt['compiled_sources']
    for path, sha in compiled.items():
        require(digest(ROOT/path) == sha,'compiled source identity: '+path)
    base_compiled = {path:sha for path,sha in compiled.items()
                     if path not in ('integrations/onnx/src/yolox_tiles.rs',
                                     'integrations/onnx/src/bin/rustdriving-camera-detect-tiled-s.rs',
                                     'integrations/onnx/profile-tiles-s.json')}
    require(digest(args.model) == receipt['model_sha256'] and args.model.stat().st_size == receipt['model_bytes'], 'model pin')
    data_pins = {}
    for image in design['images']:
        for key in ('image','annotation'):
            pin = image[key]
            require(digest(args.data/pin['file']) == pin['sha256'],'original input identity')
            data_pins[pin['file']] = pin['sha256']
    from bdd_fetch import protocol
    licence = protocol()['license']
    require(digest(args.data/licence['file']) == licence['sha256'],'original data licence')
    data_pins[licence['file']] = licence['sha256']
    # Hashing annotation bytes above does not parse numeric reference boxes.
    freeze = dict(schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),
                  design_sha256=digest(args.design),checker_sha256=digest(__file__),
                  executable_sha256=digest(args.binary),compiled_sources=compiled,
                  controls_sha256=digest(args.controls/'summary.json'),input_sha256=data_pins,
                  model_sha256=digest(args.model),labels_parsed=False,
                  data_role='already viewed regression; not an independent holdout')
    write(out/'before-inference.json',freeze)
    passes = []
    for name in ('first','repeat'):
        directory = fresh(out/name)
        reports = {}
        for image in design['images']:
            reports[image['id']] = native(args.binary,['--model',args.model,'--image',args.data/image['image']['file']],
                                           directory/(image['id']+'.json'))
        passes.append(reports)
    write(out/'before-label-parse.json',dict(labels_parsed=False,
          prediction_sha256={str(p.relative_to(out)):digest(p) for p in sorted(out.glob('*/*.json'))}))
    return audit_native(args,out,passes,freeze,design,oracle,compiled,base_compiled,receipt,old)


def audit_native(args,out,passes,freeze,design,oracle,compiled,base_compiled,receipt,old):
    same(without_timing(passes[0]),without_timing(passes[1]))
    projected, timings, historical = {}, [], []
    for image in design['images']:
        report = passes[0][image['id']]
        require(report['schema_version'] == 1
                and report['algorithm'] == 'pinned_yolox_s_full_plus_three_squares_v1', 'fixed tiled algorithm')
        require(report['compiled_sources'] == compiled and report['model_sha256'] == receipt['model_sha256']
                and report['source_image_sha256'] == image['image']['sha256'],'native tiled identity')
        require((report['width'],report['height']) == (image['width'],image['height']), 'fixed native dimensions')
        thresholds(copy.deepcopy(report))
        fixture = dict(width=report['width'],height=report['height'],views=[])
        for view in report['views']:
            rect = view['rectangle']
            inner = copy.deepcopy(view['prediction'])
            thresholds(inner)
            inner['source_image_sha256'] = image['image']['sha256']
            old.validate_prediction(inner,dict(image,width=rect['width'],height=rect['height']),design,base_compiled)
            require(len(view['rgb_sha256']) == 64 and int(view['rgb_sha256'],16) >= 0,'RGB identity')
            fixture['views'].append(dict(rectangle=rect,detections=inner['detections']))
        expected = merge(fixture)
        same(expected,{k:report[k] for k in expected})
        single = load(ROOT/'assets/yolox-s-bdd-v1/first'/(image['id']+'.json'))
        old.validate_prediction(single,image,design,base_compiled)
        # One-view control is the identical native S algorithm, preprocessing and decoder.
        a,b = single['detections'],report['views'][0]['prediction']['detections']
        historical_same(a,b)
        historical.append(dict(image_id=image['id'],full_view_unchanged=True,absolute_tolerance=1e-5))
        projection = copy.deepcopy(report['views'][0]['prediction'])
        thresholds(projection)
        projection['source_image_sha256'] = image['image']['sha256']
        projection['detections'] = expected['detections']
        projected[image['id']] = projection
        timings.append(dict(image_id=image['id'],views=len(report['views']),
                            summed_native_inference_seconds=sum(v['prediction']['inference_seconds'] for v in report['views'])))
    scored = old.score(args.data,projected,design,base_compiled,oracle)
    scored['quality_gates'] = dict(design['evaluation']['quality_gates'],
                                   **{'pass':scored['viewed_quality_gates_passed']})
    scored['evaluation'] = 'Fixed multi-view YOLOX-S on all eight viewed BDD frames; unchanged original references/matcher/gates; not BDD mAP'
    scored['algorithm'] = 'pinned_yolox_s_full_plus_three_squares_v1'
    scored['calibration'] = calibration(scored,passes[0],design['class_mapping'])
    scored['native_inference_times'] = timings
    scored['integrity'] = dict(repeated_native_inference_identical=True,independent_merge=True,
                               historical_single_view_controls=historical,labels_parsed_after_both_native_passes=True)
    previous = load(ROOT/'assets/yolox-s-bdd-v1/first-results.json')
    def ids(result):
        return {(r['image_id'],m['reference_id']) for r in result['images'] for m in r['matches']}
    before,after = ids(previous),ids(scored)
    scored['previous_s_reference_matches'] = dict(retained=len(before&after),gained=sorted(after-before),lost=sorted(before-after))
    write(out/'results.json',scored)
    write(out/'after-inference.json',dict(executable_unchanged=digest(args.binary)==freeze['executable_sha256'],
          source_unchanged=all(digest(ROOT/p)==sha for p,sha in compiled.items()),
          prediction_sha256={str(p.relative_to(out)):digest(p) for p in sorted(out.glob('*/*.json'))},
          auditor_sha256=digest(__file__)))
    print(json.dumps({k:scored[k] for k in ('overall','per_class','quality_gates')}))
    return scored


def audit(args):
    require(args.predictions is not None, 'saved native prediction directory required')
    require(digest(args.design) == DESIGN_SHA and digest(HERE/'bdd_s_check.py') == MATCHER_SHA,
            'fixed design and original matcher')
    import bdd_s_check as old
    design,pins,oracle = old.context()
    source = args.predictions.absolute()
    freeze, receipt = load(source/'before-inference.json'), load(source/'executable.json')
    journal = load(source/'before-label-parse.json')
    require(freeze['labels_parsed'] is False and journal['labels_parsed'] is False,'original pre-label journals')
    require(freeze['executable_sha256'] == digest(args.binary) == receipt['executable_sha256'], 'unchanged native binary')
    require(freeze['model_sha256'] == digest(args.model) == receipt['model_sha256'],'unchanged native model')
    compiled = freeze['compiled_sources']
    require(compiled == receipt['compiled_sources'] and all(digest(ROOT/p)==sha for p,sha in compiled.items()),
            'unchanged compiled sources')
    archived = load(source.parent/'candidate-source-sha256.json')
    controls_receipt = load(args.controls/'summary.json')
    require(freeze['checker_sha256'] == archived['integrations/onnx/bdd_tiles_check.py']
            == controls_receipt['checker_sha256'], 'preserved first auditor identity')
    require(controls_receipt['pass_controls'] and controls_receipt['executable_sha256'] == digest(args.binary), 'first native controls')
    require(digest(args.controls/'summary.json') == freeze['controls_sha256'], 'original controls unchanged')
    for name,sha in journal['prediction_sha256'].items():
        require(digest(source/name) == sha,'first/repeat raw prediction changed')
    require(len(journal['prediction_sha256']) == 16,'all eight original images twice')
    for name,sha in freeze['input_sha256'].items():
        require(digest(args.data/name) == sha,'original image/label identity')
    base_compiled = {p:sha for p,sha in compiled.items() if p not in (
        'integrations/onnx/src/yolox_tiles.rs','integrations/onnx/src/bin/rustdriving-camera-detect-tiled-s.rs',
        'integrations/onnx/profile-tiles-s.json')}
    passes = [{image['id']:load(source/pass_name/(image['id']+'.json')) for image in design['images']}
              for pass_name in ('first','repeat')]
    out = fresh(args.output)
    write(out/'auditor-repair.json',dict(kind='auditor-only f32 JSON metadata representation repair',
        prior_checker_sha256=freeze['checker_sha256'],current_checker_sha256=digest(__file__),
        native_binary_and_all_16_predictions_unchanged=True,source_prediction_sha256=journal['prediction_sha256'],
        numeric_threshold_change=False,quality_gate_change=False,reference_change=False))
    return audit_native(args,out,passes,freeze,design,oracle,compiled,base_compiled,receipt,old)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['controls','run','audit'])
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--predictions',type=Path)
    parser.add_argument('--data',type=Path,default=ROOT/'artifacts/bdd-camera')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--design',type=Path,default=ROOT/'assets/yolox-s-tiles-v1/design.json')
    args = parser.parse_args()
    args.binary = args.binary.absolute()
    if args.command == 'controls':
        controls(args.binary,args.output)
    else:
        require(args.controls is not None,'controls receipt required')
        result = audit(args) if args.command == 'audit' else run(args)
        # Integrity success and physical quality failure are separate outcomes.
        return 0 if result['quality_gates']['pass'] else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
