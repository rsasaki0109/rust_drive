#!/usr/bin/env python3
"""Run actual CPU model, independent box scoring and fail-closed asset probes."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import struct
import subprocess
import sys
import zlib

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent

def png_blank(path):
    def chunk(kind, body):
        return struct.pack('>I', len(body)) + kind + body + struct.pack('>I', zlib.crc32(kind+body))
    body = b''.join(b'\0' + bytes([114,114,114])*512 for _ in range(512))
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR',struct.pack('>IIBBBBB',512,512,8,2,0,0,0)) + chunk(b'IDAT',zlib.compress(body)) + chunk(b'IEND',b''))

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--data',type=Path,default=ROOT/'artifacts/camera-model')
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/camera-model/check')
    parser.add_argument('--summary',type=Path)
    args=parser.parse_args()
    binary=args.binary.resolve(); data=args.data.resolve(); out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=True)
    model=data/'yolox_nano.onnx'
    subprocess.run([sys.executable,str(HERE/'fetch.py'),'--output',str(data)],check=True)
    def infer(image,target,model_file=model,expected=0):
        result=subprocess.run([str(binary),'--model',str(model_file),'--image',str(image),'--output',str(target)],capture_output=True,text=True)
        if result.returncode!=expected:
            raise RuntimeError(f'inference code {result.returncode}, expected {expected}: {result.stderr}')
        return result
    for name in ['first','repeat']:
        infer(data/'astronaut.jpg',out/f'{name}.json')
    first=json.loads((out/'first.json').read_text());repeat=json.loads((out/'repeat.json').read_text())
    if first['detections'] != repeat['detections']:
        raise ValueError('repeated actual CPU inference box mismatch')
    subprocess.run([sys.executable,str(HERE/'score.py'),'--data',str(data),'--detections',str(out/'first.json'),'--output',str(out/'score.json')],check=True)
    score=json.loads((out/'score.json').read_text())
    # Baseline acceptance for this exact independently labelled portrait, not general AP.
    if score['true_positive_count']!=1 or score['unmatched_prediction_count']!=0 or score['best_person_iou']<0.9:
        raise ValueError('pinned portrait execution/box baseline changed')
    blank=out/'synthetic-blank.png';png_blank(blank)
    infer(blank,out/'blank.json')
    if json.loads((out/'blank.json').read_text())['detections']:
        raise ValueError('synthetic blank produced detections at fixed threshold')
    bad=out/'bad.jpg';bad.write_bytes(b'not a measured JPEG')
    infer(bad,out/'bad.json',expected=2)
    if (out/'bad.json').exists():
        raise ValueError('malformed image produced output')
    corrupt=out/'corrupt.onnx'
    with corrupt.open('wb') as f:
        f.truncate(3659407)
    result=infer(data/'astronaut.jpg',out/'corrupt.json',corrupt,2)
    if 'SHA-256' not in result.stderr:
        raise ValueError('same-length corrupt model did not fail SHA verification')
    # Independent checker must reject altered image identity, thresholds and box geometry.
    mutations=[('source_image_sha256','0'*64),('score_threshold',0.01),('bbox',[0,0,float('nan'),512])]
    for key,value in mutations:
        changed=json.loads(json.dumps(first))
        if key=='bbox':changed['detections'][0]['bbox']=value
        else:changed[key]=value
        file=out/f'mutation-{key}.json';file.write_text(json.dumps(changed))
        result=subprocess.run([sys.executable,str(HERE/'score.py'),'--data',str(data),'--detections',str(file),'--output',str(out/'mutation-score.json')],capture_output=True,text=True)
        if result.returncode==0:
            raise ValueError(f'independent checker accepted {key} mutation')
    source_files=sorted(p for p in HERE.rglob('*') if p.is_file() and p.suffix in ['.rs','.toml','.lock','.py'] and '__pycache__' not in str(p))
    summary={
        'schema_version':1,'recorded_utc':datetime.now(timezone.utc).isoformat(),
        'host':{'os':platform.system(),'architecture':platform.machine(),'cpu_quota_cores':float(Path('/sys/fs/cgroup/cpu.max').read_text().split()[0])/float(Path('/sys/fs/cgroup/cpu.max').read_text().split()[1]) if Path('/sys/fs/cgroup/cpu.max').exists() and Path('/sys/fs/cgroup/cpu.max').read_text().split()[0] != 'max' else None,'gpu_used':False},
        'model_sha256':digest(model),'image_sha256':digest(data/'astronaut.jpg'),'annotation_sha256':digest(data/'instances.json'),
        'binary_sha256':digest(binary),'source_sha256':{str(p.relative_to(ROOT)):digest(p) for p in source_files},
        'actual_inference_runs':3,'repeated_detection_identity':True,'synthetic_blank_detection_count':0,
        'malformed_image_rejected':True,'same_length_corrupt_model_rejected':True,'independent_checker_mutations_rejected':3,
        'model_load_seconds':first['model_load_seconds'],'inference_seconds':[first['inference_seconds'],repeat['inference_seconds']],
        'measured_image_score':score,
        'limitations':['One NASA portrait is not road-camera validation or a COCO AP benchmark.','Model release is published under an Apache-2.0 repository; no separate weight license is published.','Offline image-coordinate boxes are not connected to vehicle control.'],
    }
    target=args.summary or out/'results.json';target.write_text(json.dumps(summary,indent=2)+'\n')
    print(f'actual CPU inference, independent pixel-box acceptance and five rejection checks passed; {target}')

if __name__=='__main__':main()
