#!/usr/bin/env python3
"""Preregistered original TUM FR1 trials: opaque acquisition, run, raw audit, score."""
import argparse,gzip,importlib.util,io,json,tarfile
from pathlib import Path
import numpy as np
from PIL import Image
from projective_depth_common import ROOT,CAMERA,Client,audit_sequence,pose,require,sha,write
from projective_information_raw import audit as information_audit
from projective_depth_raw import reconstruct_update
from projective_timing_raw import TimingHistory

BASE=ROOT/'artifacts/projective-information/heldout'
DATA=ROOT/'data/tum-fr1-plant-heldout'
BINARY=ROOT/'integrations/projective-depth/target/release/rustdriving-projective-depth'
ARCHIVE_ROOT='rgbd_dataset_freiburg1_plant/'
URL='https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_plant.tgz'
LIMITS=dict(compressed=832*1024*1024,decompressed=2*1024**3,members=12000,member=4*1024**2,metadata=4*1024**2,selected=128*1024**2)
RECORDING='plant'
EXPECTED_BYTES=695620135
MODES={'depth':{},'timing':dict(photometric='keyframe',motion_model='constant_velocity',timing='previous_motion')}


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def frozen():
    f=json.loads((BASE/'design.json').read_text())
    require(f['binary_sha256']==sha(BINARY.read_bytes()),'unchanged preregistered binary')
    for n,h in f['source_sha256'].items():require(sha((ROOT/n).read_bytes())==h,'unchanged held-out source '+n)
    return f


def inputs():
    f=frozen();body=(DATA/'manifest.json').read_bytes();manifest=json.loads(body)
    require(manifest['preregistration_sha256']==sha((BASE/'design.json').read_bytes()),'preregistered manifest')
    inventory={x['file']:x for x in manifest['files']}
    require(len(inventory)==len(manifest['files']) and len(manifest['frames'])==180,'complete unique input inventory')
    def read(name):
        require(Path(name).name==name and name in inventory,'plain pinned input path')
        p=DATA/'raw'/name;pin=inventory[name]
        require(not p.is_symlink() and p.stat().st_size==pin['bytes']<=4*1024**2,'bounded raw input')
        body=p.read_bytes();require(sha(body)==pin['sha256'],'raw input digest');return body
    for name in inventory:read(name)
    return f,manifest,read


def prepare():
    require(not BASE.exists() and not DATA.exists(),'fresh trial and data paths')
    BASE.mkdir(parents=True)
    pins=json.loads((ROOT/'artifacts/projective-information/candidate-source-sha256.json').read_text())
    sources=set(pins)|{'scripts/projective-heldout.py','scripts/fetch-independent-dataset.py','scripts/qualify-rgbd-independent.py','scripts/check-opencv-rgbd.py'}
    sources={n:sha((ROOT/n).read_bytes()) for n in sorted(sources)}
    write(BASE/'design.json',dict(schema_version=1,dataset='tum-fr1-'+RECORDING+'-heldout',source_url=URL,archive_root=ARCHIVE_ROOT,
        expected_archive_bytes_from_http_head=EXPECTED_BYTES,limits=LIMITS,first_depth_index=100,last_depth_index=279,frames=180,updates=179,
        modes=MODES,camera=CAMERA,depth_units_per_metre=5000,source_sha256=sources,binary_sha256=sha(BINARY.read_bytes()),
        reference_role='original independent TUM mocap; numeric poses deferred until both complete sensor ledgers and independent raw audits',
        gates=dict(root_translation_m=.1,root_rotation_rad=.1,reference_bracket_s=.02,rgb_association_s=.02,resets=0),
        provenance='TUM RGB-D benchmark, Sturm et al. IROS 2012; general official CC BY 4.0 terms documented in docs/tum-source-provenance.md; raw not redistributed',
        status='new '+RECORDING+' recording and fixed interval, never used for algorithm development in this chat; same FR1 camera family, not automotive',
        no_select_on_accuracy=True,source_frozen_before_archive_payload=True))
    with tarfile.open(BASE/'source.tar.gz','w:gz') as t:
        for n in sources:t.add(ROOT/n,arcname=n)


def acquire():
    f=frozen();archive=BASE/'archive.tgz'
    require(archive.is_file() and archive.stat().st_size==f['expected_archive_bytes_from_http_head'],'complete bounded official transfer')
    source=module('bounded_source',ROOT/'scripts/fetch-independent-dataset.py');source.ARCHIVE_ROOT=ARCHIVE_ROOT
    receipt=source.archive_identity(archive,LIMITS);scanned=source.scan_archive(archive,limits=LIMITS)
    qualifier=module('metadata_only',ROOT/'scripts/qualify-rgbd-independent.py');selected=qualifier.select_metadata(scanned['metadata'])
    wanted={kind+'/'+x[kind+'_file'].removeprefix(kind+'-') for x in selected['frames'] for kind in ('depth','rgb')}
    require(not(wanted-set(scanned['inventory'])),'all fixed-window images present; no fallback window')
    DATA.mkdir();raw=DATA/'raw';raw.mkdir()
    for n,body in scanned['metadata'].items():
        with (raw/n).open('xb') as stream:stream.write(body)
    copied=source.scan_archive(archive,wanted,raw,limits=LIMITS)
    require(scanned['metadata']==copied['metadata'] and source.archive_identity(archive,LIMITS)==receipt,'archive unchanged through selection')
    files=[]
    for n in sorted(wanted|set(scanned['metadata'])):
        pin=copied['selected_files'].get(n)
        if pin is None:body=scanned['metadata'][n];pin=dict(bytes=len(body),sha256=sha(body))
        files.append(dict(file=source.local_filename(n),source_path=n,bytes=pin['bytes'],sha256=pin['sha256']))
    write(DATA/'manifest.json',dict(dataset='tum-fr1-'+RECORDING+'-heldout',preregistration_sha256=sha((BASE/'design.json').read_bytes()),
        official_archive=dict(url=URL,root=ARCHIVE_ROOT,published_checksum=None,**receipt),camera=CAMERA,frames=selected['frames'],files=files))
    write(BASE/'acquisition.json',dict(archive=receipt,metadata_only_qualification=selected['timestamps'],image_headers_read=False,pixels_decoded=False,
        numeric_reference_poses_decoded=False,selected_images=len(wanted),members=scanned['members'],decompressed_bytes=scanned['decompressed_bytes']))
    frozen()


def image(read,frame,color=False):
    im=Image.open(io.BytesIO(read(frame['rgb_file' if color else 'depth_file'])))
    require(im.size==(640,480) and im.mode in (('RGB',) if color else ('I;16','I;16B','I')),'original sensor image')
    if color:
        a=np.asarray(im,dtype=np.uint32);return ((77*a[:,:,0]+150*a[:,:,1]+29*a[:,:,2])>>8).astype(np.uint8)
    return np.asarray(im,dtype=float)/5000.


def run():
    f,manifest,read=inputs()
    for mode,config in MODES.items():
        out=BASE/mode;out.mkdir();write(out/'freeze.json',dict(binary_sha256=f['binary_sha256'],configuration=config,
            manifest_sha256=sha((DATA/'manifest.json').read_bytes()),preregistration_sha256=sha((BASE/'design.json').read_bytes()),
            sensor_only=True,ground_truth_operational=False,information_guard='physical_spectrum_v1'))
        client=Client(BINARY,CAMERA,configuration=config);rows=[]
        try:
            for frame in manifest['frames']:
                rgb=None if mode=='depth' else dict(timestamp=frame['rgb_timestamp'],gray=image(read,frame,True).ravel().tolist())
                u=client.frame(frame['depth_timestamp'],image(read,frame),image=rgb);rows.append(dict(source_index=frame['source_index'],update=u));information_audit(u)
            client.finish()
        except BaseException:client.abort();raise
        write(out/'sensor-only.json',dict(frames=rows,ground_truth_parsed=False));frozen()


def raw_audit():
    _,manifest,read=inputs()
    for mode in MODES:
        out=BASE/mode;rows=json.loads((out/'sensor-only.json').read_text())['frames'];require(len(rows)==180,'all selected acquisitions')
        audit_sequence(rows,[x['depth_timestamp'] for x in manifest['frames']]);history=TimingHistory()
        model=weights=key_gray=key_rgb=last_rgb=None;counts=[]
        for frame,row in zip(manifest['frames'],rows,strict=True):
            u=row['update'];information_audit(u);context=None;gray=None;fresh=False
            if mode=='timing' and 'photometric' in u:
                gray=image(read,frame,True);rgb=frame['rgb_timestamp'];fresh=last_rgb is None or rgb>last_rgb;last_rgb=rgb
                context=dict(target=key_gray,source=gray,source_timestamp=rgb,reference_timestamp=key_rgb,qualified=False,fresh=fresh)
                history.augment(context,frame['depth_timestamp'])
            initial=model is None
            model,weights,c=reconstruct_update(model,weights,image(read,frame),CAMERA,u,image_context=context);counts.append(c)
            if mode=='timing' and u['accepted'] and (initial or u['keyframe_replaced']):key_gray=gray if fresh else None;key_rgb=frame['rgb_timestamp'] if fresh else None
            history.observe(u)
            if len(counts)%30==0:print(mode,'raw frames',len(counts),flush=True)
        write(out/'raw-audit.json',dict(pass_raw_mathematics=True,numeric_reference_poses_decoded=False,frames=180,
            geometric_systems=sum(x['iterations']+x['final_equations'] for x in counts),failure_witnesses=sum('information_failure' in row['update'] for row in rows)))
    frozen()


def score():
    _,manifest,read=inputs()
    for mode in MODES:require(json.loads((BASE/mode/'raw-audit.json').read_text())['pass_raw_mathematics'],'raw audit before numeric reference scoring')
    ref=module('mocap_score',ROOT/'scripts/check-opencv-rgbd.py');ref.controls();labels=np.loadtxt(io.BytesIO(read('groundtruth.txt')),comments='#')
    require(labels.ndim==2 and labels.shape[1]==8 and np.isfinite(labels).all() and np.all(np.diff(labels[:,0])>0),'whole chronological mocap')
    origin=np.linalg.inv(ref.truth_at(labels,manifest['frames'][0]['depth_timestamp']));results={}
    for mode in MODES:
        rows=json.loads((BASE/mode/'sensor-only.json').read_text())['frames'];accurate=accepted=0;errors=[]
        for i,(frame,row) in enumerate(zip(manifest['frames'],rows,strict=True)):
            u=row['update']
            if not i:require(u['accepted'],'first optical camera root established');continue
            if not u['accepted']:continue
            truth=origin@ref.truth_at(labels,frame['depth_timestamp']);error=np.linalg.inv(truth)@pose(u['first_from_current'])
            position=float(np.linalg.norm(error[:3,3]));r=error[:3,:3];skew=[r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]]
            angle=float(np.arctan2(np.linalg.norm(skew)/2,(np.trace(r)-1)/2));good=position<=.1 and angle<=.1;accepted+=1;accurate+=int(good);errors.append((position,angle))
            row.update(position_error_m=position,rotation_error_rad=angle,accurate=good)
        summary=dict(total_updates=179,accepted=accepted,root_accurate=accurate,lost=rows[-1]['update']['lost'],max_position_error_m=max((x[0] for x in errors),default=None),max_rotation_error_rad=max((x[1] for x in errors),default=None),complete_accuracy_pass=accurate==179 and not rows[-1]['update']['lost'])
        write(BASE/mode/'results.json',dict(summary=summary,frames=rows));results[mode]=summary
    write(BASE/'results.json',dict(first_frozen_heldout_trial=True,summaries=results,source_unchanged=frozen()['binary_sha256'],all_numeric_truth_deferred_until_both_raw_audits=True))
    print(json.dumps(results),flush=True)
    return 0 if all(s['complete_accuracy_pass'] for s in results.values()) else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('prepare','acquire','run','audit','score'))
    p.add_argument('--recording',choices=('plant','rpy'),default='plant');p.add_argument('--expected-archive-bytes',type=int);a=p.parse_args()
    RECORDING=a.recording
    if RECORDING!='plant':
        BASE=ROOT/'artifacts/projective-information'/('heldout-'+RECORDING)
        DATA=ROOT/'data'/('tum-fr1-'+RECORDING+'-heldout')
        ARCHIVE_ROOT='rgbd_dataset_freiburg1_'+RECORDING+'/'
        URL='https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_'+RECORDING+'.tgz'
    if a.action=='prepare':
        EXPECTED_BYTES=a.expected_archive_bytes if a.expected_archive_bytes is not None else EXPECTED_BYTES
        require(type(EXPECTED_BYTES) is int and 0<EXPECTED_BYTES<=LIMITS['compressed'],'bounded declared archive size')
    result=globals()[{'audit':'raw_audit'}.get(a.action,a.action)]()
    if type(result) is int:raise SystemExit(result)
