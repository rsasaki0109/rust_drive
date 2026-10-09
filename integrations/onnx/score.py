#!/usr/bin/env python3
"""Independent single-image pixel-box check; not COCO AP or driving validation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from fetch import checked


def iou(a, b):
    left, top = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, right - left) * max(0, bottom - top)
    union = ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection)
    return intersection / union if union else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path('artifacts/camera-model'))
    parser.add_argument('--detections', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    checked('astronaut.jpg', (args.data/'astronaut.jpg').read_bytes())
    labels = json.loads(checked('instances.json', (args.data/'instances.json').read_bytes()))
    report = json.loads(args.detections.read_bytes())
    if report['source_image_sha256'] != '874dba0332a5a9a6a9268e732745f57c7ba21bc733867463daaae4a766f0a03f' or report['score_threshold'] != 0.3 or report['nms_iou_threshold'] != 0.45 or report['model_sha256'] != 'c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d' or (report['width'],report['height']) != (512,512):
        raise ValueError('wrong model or image dimensions')
    refs = [r for r in labels['annotations'] if r['image_id'] == 1 and r['category_id'] == 1 and r['iscrowd'] == 0]
    if len(refs) != 1:
        raise ValueError('reference changed')
    x,y,w,h=refs[0]['bbox']
    reference=[x,y,x+w,y+h]
    detections=report['detections']
    for detection in detections:
        bbox = detection['bbox']
        if len(bbox) != 4 or not all(math.isfinite(v) for v in bbox) or not 0 <= bbox[0] < bbox[2] <= 512 or not 0 <= bbox[1] < bbox[3] <= 512 or not 0 <= detection['class_index'] < 80 or not math.isfinite(detection['confidence']) or not 0.3 <= detection['confidence'] <= 1:
            raise ValueError('invalid detection')
    matches = [d for d in detections if d['class_index'] == 0 and iou(d['bbox'],reference) >= 0.5]
    summary={
        'schema_version':1, 'evaluation':'one measured public-domain astronaut portrait, independently supplied torchvision person box',
        'image_count':1, 'reference_object_count':1, 'detection_count':len(detections),
        'true_positive_count':min(1,len(matches)), 'false_negative_count':int(not matches),
        'unmatched_prediction_count':len(detections)-min(1,len(matches)),
        'best_person_iou':max([iou(d['bbox'],reference) for d in detections if d['class_index']==0],default=0),
        'confidence_threshold':0.3, 'match_iou_threshold':0.5,
        'limitations':['Portrait is outside driving domain; one image establishes actual inference only.', 'No COCO AP, road-camera accuracy, metric depth, or vehicle-control validation.'],
        'detection_sha256':hashlib.sha256(args.detections.read_bytes()).hexdigest(),
    }
    args.output.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__ == '__main__':
    main()
