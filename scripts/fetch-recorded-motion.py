#!/usr/bin/env python3
"""Acquire untouched TUM indices200..211; transfer hashes precede any depth decoding."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
ROOT=Path(__file__).resolve().parent.parent
NAME='tum-fr1-xyz-motion'
REPO='MarcelBruckner-TUMProjects/3D-Scanning-Motion-Capture'
REV='f367047ee71f5304c6d7deaec55c4874bb8b035e'
PREFIX='Exercise_1/data/rgbd_dataset_freiburg1_xyz/'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--dataset',choices=(NAME,NAME+'-v2'),default=NAME+'-v2');parser.add_argument('--prepare-manifest',action='store_true');parser.add_argument('--verify-only',action='store_true');args=parser.parse_args()
    name_dataset=args.dataset;begin=260 if name_dataset.endswith('-v2') else 200
    destination=ROOT/'data'/name_dataset;raw=destination/'raw';raw.mkdir(parents=True,exist_ok=True)
    manifest_path=destination/'manifest.json'
    if args.prepare_manifest:
        if manifest_path.exists():raise ValueError('refuse to replace preregistered manifest')
        baseline=json.loads((ROOT/'data/tum-fr1-xyz-tight/manifest.json').read_text());manifest=dict(baseline);manifest['dataset']=name_dataset
        index=(ROOT/'data/tum-fr1-xyz-tight/raw/depth.txt').read_bytes()
        rows=[line.split() for line in index.decode().splitlines() if line and not line.startswith('#')]
        manifest['selection_policy']=f'Untouched original source indices{begin}..{begin+11} inclusive, declared before any depth decoding or sensor fitting. All eleven pairs and first-cloud local-map fits retained. Earlier original0..110/10, fast120..131, tight140..151 and failed motion200..211 are viewed regression only. New260..271 selected via timestamp-bracket availability only after200..211 failed frozen0.02s mocap gap. No reference transform or sensor fit/error used for selection; identical numerical registration/preprocessing/uncertainty gates. Same indoor room/sequence; no new-environment generalization.'
        manifest['frames']=[{'file':Path(rows[i][1]).name,'timestamp':float(rows[i][0]),'source_index':i,'split':'calibration' if i<begin+3 else 'held_out'} for i in range(begin,begin+12)]
        manifest['files']=baseline['files'][:2]
        for f in manifest['files']:shutil.copyfile(ROOT/'data/tum-fr1-xyz-tight/raw'/f['file'],raw/f['file'])
        for frame in manifest['frames']:
            name=frame['file'];source=PREFIX+'depth/'+name;target=raw/name
            subprocess.run(['curl','--fail','--location','--silent','--show-error','--connect-timeout','20','--max-time','60','--max-filesize','4000000',f'https://raw.githubusercontent.com/{REPO}/{REV}/{source}','--output',str(target)],check=True)
            b=target.read_bytes();manifest['files'].append({'file':name,'source_path':source,'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest(),'role':'depth_frame','split':frame['split'],'timestamp':frame['timestamp'],'encoding':'PNG uint16 grayscale 640x480'})
        manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    manifest=json.loads(manifest_path.read_text())
    if manifest['dataset']!=name_dataset or manifest['revision']!=REV or manifest['repository']!=REPO or [f['source_index'] for f in manifest['frames']]!=list(range(begin,begin+12)):raise ValueError('changed fixed source selection')
    if sum(f['bytes'] for f in manifest['files'])>4_000_000:raise ValueError('bounded subset too large')
    for f in manifest['files']:
        name=f['file']
        if Path(name).name!=name or '/' in name or '\\' in name or not 0<f['bytes']<=4_000_000:raise ValueError('unsafe manifest input')
        target=raw/name
        valid=lambda:target.is_file() and target.stat().st_size==f['bytes'] and hashlib.sha256(target.read_bytes()).hexdigest()==f['sha256']
        if not valid():
            if args.verify_only:raise ValueError('missing or changed pinned raw '+name)
            partial=target.with_suffix('.download')
            subprocess.run(['curl','--fail','--location','--silent','--show-error','--connect-timeout','20','--max-time','60','--max-filesize',str(f['bytes']),f'https://raw.githubusercontent.com/{REPO}/{REV}/{f["source_path"]}','--output',str(partial)],check=True)
            if partial.stat().st_size!=f['bytes'] or hashlib.sha256(partial.read_bytes()).hexdigest()!=f['sha256']:raise ValueError('pinned transfer SHA mismatch')
            partial.replace(target)
        if not valid():raise ValueError('source SHA mismatch')
    print(f'{name_dataset}: 14 source files verified; no PNG decoding or mocap parsing performed')
if __name__=='__main__':main()
