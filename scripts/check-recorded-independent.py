#!/usr/bin/env python3
"""Independent official-archive metadata and continuous measured-image motion oracle.

The pinned reprojection oracle supplies unchanged pixel reconstruction, Kabsch
coarse consensus, numerical-Jacobian/SVD refinement, clocks and physical scoring.
A static length-generalized state/physical audit keeps all179updates continuous.
Qualification is independently recomputed from timestamp columns and row arity;
no image pixels or ground-truth pose values enter metadata preregistration.
"""
import argparse
from bisect import bisect_left
import copy
from functools import lru_cache
import hashlib
import importlib.util
import json
import math
import gzip
import re
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
OLD_REPROJECTION_SHA = 'fd1425f9c73c21b104d9453a7c2b298c21fa22effd6ecb5f0015bbc41f79cef7'
ALGORITHM = 'bounded_visual_reprojection_independent_recording'
DATASET = 'tum-fr1-desk2-independent'
DESIGN_PATH = 'assets/recorded-independent/desk2-v1/design.json'
DESIGN_SHA = '6fa44837d27f6f1f4f69285780ccd5e4883d7b59ace15500699914aca323797a'
OFFICIAL_URL = 'https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz'
FINAL_URL = 'https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz'
ARCHIVE_ROOT = 'rgbd_dataset_freiburg1_desk2/'
CAMERA = dict(width=640,height=480,fx=517.306408,fy=516.469215,cx=318.643040,
              cy=255.313989,units_per_metre=5000,invalid_depth=0)
CALIBRATION_DOCUMENT = dict(repository='luigifreda/pyslam',
    revision='96019cfafcfc099ac9866884d7143a9ed1451a0d',source_path='settings/TUM1.yaml',
    file='camera-calibration.yaml',bytes=1615,
    sha256='5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf',
    role='source_calibration_documentation_only')
QUALIFICATION_ORDER = 'source-bound timestamp and row-arity metadata before estimator opens selected PNG files, headers, features or fits; opaque compressed archive acquisition/decompression occur earlier; numeric ground-truth pose values only after all fits'
FREEZE_PREPARATION = 'This metadata-only preparation reads manifest and qualification proof only; raw RGB/depth/mocap files are not opened, features not extracted and registration not run; earlier opaque archive acquisition/decompression are separate'
OLD_PATH = Path(__file__).with_name('check-recorded-reprojection.py')
if hashlib.sha256(OLD_PATH.read_bytes()).hexdigest() != OLD_REPROJECTION_SHA:
    raise ValueError('original reprojection checker must remain immutable')
_spec = importlib.util.spec_from_file_location('immutable_reprojection_oracle', OLD_PATH)
g = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g)
r, k, v = g.r, g.k, g.v


@lru_cache(maxsize=180)
def _temporal_rigid_fit(frozen):
    return v._robust_svd.__wrapped__(frozen)


def temporal_rigid_fit(pairs):
    frozen = tuple(tuple(pair['previous'])+tuple(pair['current']) for pair in pairs)
    return _temporal_rigid_fit(frozen)


@lru_cache(maxsize=180)
def _temporal_refinement(frozen, calibration, coarse):
    return g._refine_independent.__wrapped__(frozen, calibration, coarse)


def temporal_refinement(observations, camera, coarse):
    frozen = tuple(tuple(obs['previous_xyz'])+tuple(obs['current_pixel_xy']) for obs in observations)
    calibration = tuple(camera[key] for key in ('fx','fy','cx','cy'))
    initial = tuple(coarse[0])+tuple(coarse[1])
    return _temporal_refinement(frozen, calibration, initial)


def cache_contract_checks():
    """Exact keys retain full solver results, including rejection/work/trace."""
    previous = [(x/10., y/10., 1.5) for y in range(3) for x in range(4)]
    pairs = [dict(previous=list(p), current=[p[0]-.01, p[1], p[2]]) for p in previous]
    frozen = tuple(tuple(p['previous'])+tuple(p['current']) for p in pairs)
    uncached = v._robust_svd.__wrapped__(frozen)
    r.require(uncached[1] is None and temporal_rigid_fit(pairs) == uncached,
              'cached rigid solver differs from uncached positive control')
    before = _temporal_rigid_fit.cache_info().hits
    r.require(temporal_rigid_fit(pairs) == uncached
              and _temporal_rigid_fit.cache_info().hits == before+1,
              'exact rigid-fit cache key failed')
    camera = dict(fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989)
    observations = [dict(previous_xyz=p['previous'], current_pixel_xy=[
        camera['fx']*p['current'][0]/p['current'][2]+camera['cx'],
        camera['fy']*p['current'][1]/p['current'][2]+camera['cy']]) for p in pairs]
    coarse = uncached[0]['estimate']
    key = (tuple(tuple(o['previous_xyz'])+tuple(o['current_pixel_xy']) for o in observations),
           tuple(camera[k] for k in ('fx','fy','cx','cy')), tuple(coarse[0])+tuple(coarse[1]))
    direct = g._refine_independent.__wrapped__(*key)
    r.require(direct[1] is None and temporal_refinement(observations,camera,coarse) == direct,
              'cached refinement differs from uncached positive control')
    before = _temporal_refinement.cache_info().hits
    r.require(temporal_refinement(observations,camera,coarse) == direct
              and _temporal_refinement.cache_info().hits == before+1,
              'exact refinement cache key failed')
    _temporal_rigid_fit.cache_clear()
    _temporal_refinement.cache_clear()
    return dict(passed=['exact_cached_vs_uncached_rigid_result_including_work',
        'exact_cached_vs_uncached_refinement_including_trace',
        'repeated_exact_keys_hit_local_caches'], maxsize=180,
        immutable_helper_globals_modified=False, actual_sensor_data_read=False)

SOURCES = dict(g.v.SOURCES)
for _field in ('visual_source_sha256', 'reprojection_source_sha256',
               'reprojection_acquisition_source_sha256'):
    SOURCES.pop(_field)
SOURCES.update(
    evaluator_source_sha256='integrations/rgbd/src/bin/rustdriving-rgbd-independent.rs',
    independent_source_sha256='integrations/rgbd/src/independent.rs',
    independent_checker_sha256='scripts/check-recorded-independent.py',
    temporal_checker_sha256='scripts/check-recorded-temporal.py',
    reprojection_checker_sha256='scripts/check-recorded-reprojection.py',
    qualification_source_sha256='scripts/qualify-rgbd-independent.py',
    acquisition_source_sha256='scripts/fetch-independent-dataset.py')


def timestamp_rows(content, columns, image_kind=None):
    """Never convert or interpret the seven ground-truth pose tokens."""
    r.require(len(content) <= 4194304, 'metadata file exceeds declared byte bound')
    rows = []
    for number, raw_line in enumerate(content.decode('utf8').splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        fields = line.split()
        r.require(len(fields) == columns, f'metadata row arity line {number}')
        try:
            stamp = float(fields[0])
        except ValueError:
            raise ValueError(f'invalid metadata timestamp line {number}') from None
        r.require(math.isfinite(stamp) and (not rows or stamp > rows[-1][0]),
                  f'nonfinite or nonmonotonic metadata timestamp line {number}')
        if image_kind is not None:
            name = fields[1]
            r.require(name.startswith(image_kind+'/') and name.endswith('.png')
                      and '\\' not in name and len(Path(name).parts) == 2
                      and '..' not in Path(name).parts, 'unsafe image index path')
            rows.append((stamp, name))
        else:
            rows.append((stamp,))
        r.require(len(rows) <= (20000 if image_kind is not None else 30000), 'metadata source row bound')
    r.require(len(rows) >= 2, 'insufficient metadata timestamps')
    return rows


def raw_inventory_check(manifest,raw_path):
    expected={item['file'] for item in manifest['files']}|{CALIBRATION_DOCUMENT['file']}
    entries=list(raw_path.iterdir())
    r.require({entry.name for entry in entries}==expected
        and all(entry.is_file() and not entry.is_symlink() for entry in entries),
        'unexpected/missing raw files, directories or links; only pinned calibration sidecar permitted')


def metadata_inputs(manifest, raw_path):
    raw_inventory_check(manifest,raw_path)
    files = {item['file']: item for item in manifest['files']}
    result = {}
    total=0
    for name in ('depth.txt', 'rgb.txt', 'groundtruth.txt'):
        item = files[name]
        content = r.bounded(raw_path/name, 4194304)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'],
                  'changed full-source qualification metadata bytes')
        total+=len(content);r.require(total<=4194304,'aggregate metadata resource bound')
        result[name] = content
    return result


def independently_qualify(manifest, metadata):
    r.require(sum(len(x) for x in metadata.values())<=4194304,'aggregate metadata resource bound')
    depth = timestamp_rows(metadata['depth.txt'], 2, 'depth')
    rgb = timestamp_rows(metadata['rgb.txt'], 2, 'rgb')
    truth = timestamp_rows(metadata['groundtruth.txt'], 8)
    truth_times = [row[0] for row in truth]
    rgb_times = [row[0] for row in rgb]
    frames = manifest['frames']
    r.require(len(frames) == 180 and [f['source_index'] for f in frames] == list(range(100, 280)),
              'changed fixed consecutive source window')
    maximum_bracket, maximum_gap = 0., 0.
    for ordinal, frame in enumerate(frames):
        r.require(frame['split'] == ('initialization' if ordinal == 0 else 'independent_recording'),
                  'changed preregistered source split')
        for kind, index, rows in [('depth', frame['source_index'], depth),
                                  ('rgb', frame['rgb_source_index'], rgb)]:
            r.require(type(index) is int and 0 <= index < len(rows), 'invalid source index')
            stamp, filename = rows[index]
            r.require(stamp == frame[kind+'_timestamp']
                      and frame[kind+'_file'] == kind+'-'+filename[len(kind)+1:],
                      'changed original source acquisition')
        stamp = frame['depth_timestamp']
        right = bisect_left(rgb_times, stamp)
        choices = [i for i in (right-1, right) if 0 <= i < len(rgb)]
        nearest = min(choices, key=lambda i: (abs(rgb_times[i]-stamp), i))
        r.require(frame['rgb_source_index'] == nearest, 'RGB is not nearest timestamp with earlier tie')
        gap = abs(frame['depth_timestamp']-frame['rgb_timestamp'])
        r.require(gap <= .02, 'qualification RGB/depth gap exceeds exact gate')
        v.same_pair_gap(frame['pair_gap_seconds'], gap)
        maximum_gap = max(maximum_gap, gap)
        right = bisect_left(truth_times, stamp)
        if right < len(truth_times) and truth_times[right] == stamp:
            bracket = 0.
        else:
            r.require(0 < right < len(truth_times), 'qualification requires GT bracket without extrapolation')
            bracket = truth_times[right]-truth_times[right-1]
            r.require(0 < bracket <= .02, 'qualification GT bracket exceeds exact gate')
        maximum_bracket = max(maximum_bracket, bracket)
    return dict(depth_rows=len(depth), rgb_rows=len(rgb), ground_truth_rows=len(truth),
                frame_count=180, reference_bracketed_frames=180,
                unique_rgb_frames=len({(f['rgb_source_index'], f['rgb_timestamp'], f['rgb_file']) for f in frames}),
                maximum_observed_ground_truth_bracket_us=math.floor(maximum_bracket*1e6+.5),
                maximum_pair_gap_us=math.floor(maximum_gap*1e6+.5),
                summary_time_unit='rounded microseconds from source timestamps; qualification gates use unrounded seconds')


def manifest_check(manifest):
    r.require(set(manifest)=={'schema_version','dataset','official_archive','preregistration_sha256',
        'depth_calibration','calibration_source','frames','files'},'changed exact official manifest keys')
    r.require(type(manifest['schema_version']) is int and manifest['schema_version']==1
              and manifest['dataset']==DATASET and manifest['preregistration_sha256']==DESIGN_SHA,
              'changed prospectively fixed independent source/window design')
    archive=manifest['official_archive']
    r.require(set(archive)=={'url','final_url','bytes','sha256','md5','root','published_checksum'},
              'changed official archive identity keys')
    r.require(archive['url']==OFFICIAL_URL and archive['final_url']==FINAL_URL
              and archive['root']==ARCHIVE_ROOT and archive['published_checksum'] is None,
              'changed official source or invented publisher checksum')
    r.require(type(archive['bytes']) is int and 0<archive['bytes']<=536870912,
              'compressed archive resource bound')
    for key,length in [('sha256',64),('md5',32)]:
        r.require(type(archive[key]) is str and re.fullmatch('[0-9a-f]{'+str(length)+'}',archive[key]),
                  'invalid computed archive identity '+key)
    exact_json(manifest['depth_calibration'],CAMERA,'changed original source camera profile')
    exact_json(manifest['calibration_source'],CALIBRATION_DOCUMENT,'changed calibration documentation provenance')
    frames=manifest['frames']
    r.require(len(frames)==180 and [f['source_index'] for f in frames]==list(range(100,280)),
              'changed fixed consecutive independent window')
    names={name for f in frames for name in (f['depth_file'],f['rgb_file'])}|{'depth.txt','rgb.txt','groundtruth.txt'}
    for i,f in enumerate(frames):
        r.require(set(f)=={'source_index','depth_file','depth_timestamp','rgb_file','rgb_timestamp',
            'rgb_source_index','pair_gap_seconds','split'},'changed exact independent frame keys')
        r.require(f['split']==('initialization' if i==0 else 'independent_recording'),
                  'changed independent source role')
        for key in ('source_index','rgb_source_index'):
            r.require(type(f[key]) is int and f[key]>=0,'invalid original acquisition index')
        for key in ('depth_timestamp','rgb_timestamp','pair_gap_seconds'):
            r.require(type(f[key]) in (int,float) and math.isfinite(f[key]),'invalid source timestamp/gap')
        r.require(i==0 or f['depth_timestamp']>frames[i-1]['depth_timestamp'],'nonmonotonic depth acquisitions')
        r.require(i==0 or (f['rgb_timestamp']>=frames[i-1]['rgb_timestamp']
            and f['rgb_source_index']>=frames[i-1]['rgb_source_index']),'reversed RGB associations')
    files=manifest['files'];r.require(len(files)==len(names),'duplicate/missing archive inventory')
    r.require({f['file'] for f in files}==names,'changed exact selected archive inventory')
    total=CALIBRATION_DOCUMENT['bytes'];sources=set()
    for item in files:
        r.require(set(item)=={'file','source_path','bytes','sha256','role'},'changed exact file provenance keys')
        filename=item['file'];r.require(type(filename) is str and Path(filename).name==filename
            and '/' not in filename and chr(92) not in filename,'unsafe flat raw path')
        if filename in ('depth.txt','rgb.txt','groundtruth.txt'):
            source=filename;role={'depth.txt':'depth_index','rgb.txt':'rgb_index',
                'groundtruth.txt':'evaluation_only_mocap_ground_truth'}[filename]
        else:
            kind='depth' if filename.startswith('depth-') else 'rgb' if filename.startswith('rgb-') else None
            r.require(kind is not None and filename.endswith('.png'),'invalid selected image name')
            source=kind+'/'+filename[len(kind)+1:];role=kind+'_frame'
        r.require(item['source_path']==source and source not in sources and item['role']==role,
                  'changed original root-relative archive member')
        sources.add(source)
        r.require(type(item['bytes']) is int and 0<item['bytes']<=4194304
            and type(item['sha256']) is str and re.fullmatch('[0-9a-f]{64}',item['sha256']),
            'invalid member byte/hash bounds')
        total+=item['bytes']
    r.require(total<=134217728,'selected raw resource bound')


class BoundedArchiveReader:
    def __init__(self,stream):self.stream=stream;self.count=0
    def read(self,size=-1):
        r.require(size>=0,'unbounded decompression request')
        data=self.stream.read(min(size,2147483648-self.count+1));self.count+=len(data)
        r.require(self.count<=2147483648,'decompressed archive resource bound')
        return data


def archive_octal(field):
    raw=field.strip(b' \0')
    r.require(bool(raw) and re.fullmatch(b'[0-7]+',raw) is not None,'unsupported/invalid tar numeric field')
    return int(raw,8)


def archive_exact_read(stream,size):
    parts=[];remaining=size
    while remaining:
        data=stream.read(remaining);r.require(bool(data),'truncated tar stream')
        parts.append(data);remaining-=len(data)
    return b''.join(parts)


def archive_binding_check(path,manifest):
    """Hash compressed bytes and independently bind every selected source member.

    Tar payloads stay opaque. No blanket extraction, PNG inspection, or numeric
    ground-truth interpretation takes place during archive provenance checking.
    """
    manifest_check(manifest)
    r.require(path.is_file() and path.stat().st_size<=536870912,'compressed archive file/resource bound')
    sha=hashlib.sha256();md5=hashlib.md5();size=0
    with path.open('rb') as stream:
        while data:=stream.read(1024*1024):
            size+=len(data);r.require(size<=536870912,'compressed archive resource bound')
            sha.update(data);md5.update(data)
    archive=manifest['official_archive']
    r.require(size==archive['bytes'] and sha.hexdigest()==archive['sha256']
              and md5.hexdigest()==archive['md5'],'actual compressed archive identity mismatch')
    selected={ARCHIVE_ROOT+i['source_path']:i for i in manifest['files']}
    found=set();seen=set();members=0
    with gzip.open(path,'rb') as zipped:
        bounded=BoundedArchiveReader(zipped)
        while True:
            header=archive_exact_read(bounded,512)
            if header==bytes(512):
                r.require(archive_exact_read(bounded,512)==bytes(512),'incomplete tar end marker')
                while data:=bounded.read(65536):
                    r.require(not any(data),'nonzero trailing tar payload')
                break
            members+=1;r.require(members<=10000,'archive member count bound')
            checksum=archive_octal(header[148:156])
            r.require(checksum==sum(header[:148])+8*32+sum(header[156:]),'tar header checksum mismatch')
            kind=header[156:157]
            r.require(kind in (b'0',b'\0',b'5'),'archive extended/link/special types forbidden before payload')
            name=header[:100].split(b'\0',1)[0].decode('utf8')
            prefix=header[345:500].split(b'\0',1)[0].decode('utf8')
            if prefix:name=prefix+'/'+name
            r.require(chr(92) not in name and not name.startswith('/')
                and all(part not in ('','..','.') for part in name.rstrip('/').split('/'))
                and (name.rstrip('/')==ARCHIVE_ROOT.rstrip('/') or name.startswith(ARCHIVE_ROOT))
                and name.rstrip('/') not in seen,'unsafe/duplicate/foreign archive member')
            r.require(name.rstrip('/')!=ARCHIVE_ROOT.rstrip('/') or kind==b'5',
                      'archive root must be a directory')
            seen.add(name.rstrip('/'))
            count=archive_octal(header[124:136]);r.require(count<=4194304,'archive member size bound')
            if kind==b'5':r.require(count==0,'nonempty archive directory')
            digest=hashlib.sha256();remaining=count
            while remaining:
                data=archive_exact_read(bounded,min(65536,remaining));remaining-=len(data);digest.update(data)
            if name in selected:
                item=selected[name]
                r.require(kind!=b'5' and count==item['bytes'] and digest.hexdigest()==item['sha256'],
                          'selected raw descriptor not bound to actual archive member')
                found.add(name)
            padding=archive_exact_read(bounded,(-count)%512)
            r.require(not any(padding),'nonzero tar member padding')
    r.require(found==set(selected),'missing selected/full-source metadata archive members')
    return dict(archive_sha256=sha.hexdigest(),archive_md5=md5.hexdigest(),archive_bytes=size,
        archive_members_checked=members,decompressed_archive_bytes=bounded.count,
        selected_member_hashes_verified=len(found),pixels_read=False,image_headers_read=False,
        ground_truth_pose_values_parsed=False,published_checksum_available=False,
        computed_hash_is_publisher_signature=False)


def exact_json(actual, expected, message):
    # Treat JSON booleans separately from numbers (Python True equals 1).
    if isinstance(expected, bool):
        r.require(type(actual) is bool and actual == expected, message)
    elif isinstance(expected, dict):
        r.require(isinstance(actual, dict) and set(actual) == set(expected), message)
        for key in expected:
            exact_json(actual[key], expected[key], message+' '+key)
    elif isinstance(expected, list):
        r.require(isinstance(actual, list) and len(actual) == len(expected), message)
        for a, e in zip(actual, expected):
            exact_json(a, e, message)
    elif isinstance(expected, (int, float)):
        r.require(type(actual) in (int, float) and math.isfinite(actual) and actual == expected, message)
    else:
        r.require(actual == expected, message)


PREPROCESSING = dict(width=640, height=480,
    luma='(77*R+150*G+29*B)>>8; RGB/RGBA8; RGBA must be fully opaque',
    pixel_coordinates='nearest integer feature coordinate, ties round away from zero',
    min_depth_m=.3, max_depth_m=5., depth_units_per_metre=5000., patch_radius_pixels=1,
    patch_validity='all 9 depths valid and range-bounded', patch_max_spread_m=.05,
    point='centre depth at feature pixel; optical x-right y-down z-forward', maximum_pair_gap_s=.02)
FEATURE_POLICY = dict(max_features=400, max_matches=256,
    detector='fixed original FAST-9 threshold20 radius3, deterministic NMS,32pixel tile max2',
    descriptor='intensity-centroid oriented deterministic256bit BRIEF on5x5binomial blur',
    matching='both directional strict 5*best<4*second, maximumHamming64, mutual nearest; ties rejected')
TRACKING_POLICY = dict(max_unobserved_s=.20,
    reference='last accepted measured RGB features and depth image; initial root identity after measured geometry validation',
    pose='reference-root pose composed with measured refined previous_from_current pixel-reprojection pose',
    rejection='no root output or accepted clock/reference update; repeated/stale RGB timestamp versus last observed image rejects, including previously rejected fits; observed image clock advances without permission renewal; accepted-pose expiry checked before RGB freshness; evaluation never resets')
GT_POLICY = dict(method='linear translation and shortest-arc quaternion SLERP', max_bracket_s=.02,
                 extrapolation=False, max_source_rows=30000, max_source_bytes=4194304)

RESOURCE_POLICY = dict(max_raw_bytes=134217728,max_report_bytes=67108864,max_frames=180,
                       max_manifest_bytes=524288,max_freeze_bytes=524288,max_qualification_bytes=524288)

TEMPORAL_POLICY = dict(
    independent_window=dict(first_depth_index=100,last_depth_index=279,frames=180,updates=179),
    continuous_state=dict(initialization_frame_index=100,maximum_initializations=1,
                          chunk_resets=False,lost_recovery=False),
    evidence_scope='prospectively fixed independent FR1 desk2 recording; unchanged descriptor-based visual reprojection estimator; one continuous origin, no reset or parameter retuning')

IMMUTABLE_ALGORITHMS = {
    'feature_source_sha256': '5b5b103d4e798929f753a850387609405cad1a303d49ef8e81433f064e901106',
    'visual_pose_source_sha256': '718192d69b02de0a668c54e8469812b7c5f1903ab74de660e1b221e2ec874d24',
    'reprojection_pose_source_sha256': '2895e1c3eafefcfacdd156fd51706598521f0344f427f24edf420d159e5da486',
    'visual_checker_sha256': g.OLD_VISUAL_SHA,
    'reprojection_checker_sha256': OLD_REPROJECTION_SHA,
    'temporal_checker_sha256': '70cb0837aff6882e1d5d13798e4d7d52b6969c79d7ae49f5d3debc66404e06de',
}


def verify_sources(freeze, manifest_raw, qualification_raw, source_root):
    r.require(type(freeze['schema_version']) is int and freeze['schema_version'] == 1
              and type(freeze['protocol_version']) is int and freeze['protocol_version'] == 1
              and freeze['algorithm'] == ALGORITHM, 'unsupported temporal protocol')
    exact_json(freeze['qualification_order'],QUALIFICATION_ORDER,'changed acquisition/estimator exposure order')
    exact_json(freeze['freeze_preparation'],FREEZE_PREPARATION,'changed metadata-only preparation scope')
    exact_json(freeze['resources'],RESOURCE_POLICY,'changed temporal resource bounds')
    for field, expected in TEMPORAL_POLICY.items():
        exact_json(freeze[field],expected,'changed temporal continuity/evidence policy '+field)
    for field, path in SOURCES.items():
        r.require(freeze[field] == r.digest(r.bounded(source_root/path)), 'changed frozen source '+path)
    for field, expected in IMMUTABLE_ALGORITHMS.items():
        r.require(freeze[field] == expected, 'changed original measured-image mathematics '+field)
    design_raw=r.bounded(source_root/DESIGN_PATH,524288)
    r.require(r.digest(design_raw)==DESIGN_SHA
        and freeze['preregistration_source_sha256']==DESIGN_SHA
        and freeze['preregistration_sha256']==DESIGN_SHA,'changed prospective design bytes/source binding')
    r.require(freeze['manifest_sha256'] == r.digest(manifest_raw), 'changed frozen manifest')
    r.require(freeze['qualification_sha256'] == r.digest(qualification_raw), 'changed qualification proof bytes')
    qualification = json.loads(qualification_raw)
    exact_json(freeze['qualification'], qualification, 'changed frozen qualification object')
    manifest = json.loads(manifest_raw)
    manifest_check(manifest)
    r.require(manifest['dataset'] == freeze['dataset'] == DATASET,'changed official dataset identity')
    exact_json(freeze['official_archive'],manifest['official_archive'],'changed frozen official archive identity')
    r.require(type(freeze['regression_requested']) is bool, 'invalid temporal reporting role')
    r.require(freeze['kind'] == ('calibration_regression' if freeze['regression_requested'] else 'preregistered_independent_recording'),
              'misclassified temporal sequence')
    for field in ('depth_calibration', 'calibration_source'):
        exact_json(freeze[field], manifest[field], 'changed frozen acquisition '+field)
    r.require(len(freeze['frames']) == len(manifest['frames']), 'changed frozen frame count')
    for actual, expected in zip(freeze['frames'], manifest['frames']):
        v.same_acquisition(actual, expected)
    for field, expected in dict(preprocessing=PREPROCESSING, feature_policy=FEATURE_POLICY,
        tracking_policy=TRACKING_POLICY, registration_config=v.CONFIG, refinement_config=g.REFINEMENT_CONFIG,
        refinement_policy=g.REFINEMENT_POLICY, evaluation_label_policy=g.LABEL_POLICY,
        ground_truth_interpolation=GT_POLICY, accuracy_gates=dict(translation_m=.1, rotation_rad=.1),
        json_metadata_audit=v.JSON_METADATA_AUDIT).items():
        exact_json(freeze[field], expected, 'changed fixed policy '+field)
    for field in ('depth_calibration', 'calibration_source', 'preprocessing', 'feature_policy',
                  'tracking_policy', 'registration_config', 'refinement_config', 'refinement_policy',
                  'evaluation_label_policy'):
        expected = (g.refinement_config_digest(freeze[field]) if field == 'refinement_config'
                    else r.canonical(freeze[field]))
        r.require(freeze[field+'_sha256'] == expected, 'changed fixed policy hash '+field)
    calibration = manifest['depth_calibration']
    for key, value in dict(fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989,
                          width=640, height=480, units_per_metre=5000., invalid_depth=0).items():
        r.require(type(calibration[key]) in (int, float) and calibration[key] == value,
                  'changed pinned source camera profile '+key)
    return manifest, qualification


def qualification_check(freeze, manifest_raw, qualification, metadata):
    manifest = json.loads(manifest_raw)
    measured = independently_qualify(manifest, metadata)
    files = {item['file']: item for item in manifest['files']}
    expected = dict(schema_version=1, dataset=DATASET, manifest_sha256=r.digest(manifest_raw),
        official_archive=manifest['official_archive'],preregistration_sha256=DESIGN_SHA,
        window=dict(first_depth_index=100,last_depth_index=279,frames=180,updates=179),
        metadata_files={name:{key:files[name][key] for key in ('bytes','sha256')}
                        for name in ('depth.txt','rgb.txt','groundtruth.txt')},
        timestamps=dict(depth_index_rows=measured['depth_rows'], rgb_index_rows=measured['rgb_rows'],
            ground_truth_rows=measured['ground_truth_rows'], strict_depth_order=True, strict_rgb_order=True,
            strict_ground_truth_order=True, all_frame_brackets_valid=True, max_ground_truth_bracket_s=.02,
            max_pair_gap_s=.02, maximum_observed_ground_truth_bracket_us=measured['maximum_observed_ground_truth_bracket_us'],
            maximum_pair_gap_us=measured['maximum_pair_gap_us'], unique_rgb_acquisitions=measured['unique_rgb_frames'],
            duplicate_rgb_associations=180-measured['unique_rgb_frames'], summary_time_unit=measured['summary_time_unit']),
        passed=True, pixels_read=False, image_headers_read=False, ground_truth_pose_values_parsed=False,
        features_or_fits_run=False, helper_sha256=freeze['qualification_source_sha256'],
        acquisition_helper_sha256=freeze['acquisition_source_sha256'])
    exact_json(qualification, expected, 'invented qualification proof')
    return measured


def continuous_audit(report, manifest, all_features, depths, gt, label_failure=None):
    g.label_failure_check(report, label_failure)
    r.require(report['schema_version'] == 1 and report['ground_truth_operational'] is False
              and report['raw_redistributed'] is False
              and report['algorithm'] == ALGORITHM,
              'schema/truth/redistribution/algorithm')
    for name in ('dataset', 'official_archive'):
        exact_json(report[name], manifest[name], 'changed official source identity')
    r.require(report['preregistration_sha256']==DESIGN_SHA,'changed report prospective source/window design')
    frames, rows = manifest['frames'], report['frames']
    r.require(len(rows) == len(frames) == 180, 'omitted observed/rejected acquisition')
    r.require(report['freeze']['registration_config'] == v.CONFIG, 'weakened robust registration gates')
    r.require(report['freeze']['refinement_config'] == g.REFINEMENT_CONFIG, 'weakened pixel refinement gates')
    r.require(report['freeze']['tracking_policy']['max_unobserved_s'] == .20,
              'changed accepted-pose age')
    calibration = manifest['depth_calibration']
    origin = k.timed_pose(gt, frames[0]['depth_timestamp'])
    reference, root_reference, last_accepted, last_observed_rgb, lost = None, v.IDENTITY, None, None, False
    counters = dict(initialized_frames=0, accepted_updates=0, rejected_updates=0,
                    accurate_root_updates=0, reference_valid_updates=0)
    records = []
    for i, (row, frame, observed) in enumerate(zip(rows, frames, all_features)):
        r.require(type(row['feature_count']) is int,'invalid feature count type')
        for matches,key_names in [(row['matches'],('previous_index','current_index','hamming_distance')),
                                   (row['depth_matches'],('previous_index','current_index')),
                                   (row.get('initialization_depth_features',[]),('feature_index',))]:
            for item in matches:
                for key in key_names:r.require(type(item[key]) is int and item[key]>=0,'invalid feature association integer')
                if 'correspondence_index' in item:
                    r.require(type(item['correspondence_index']) is int and item['correspondence_index']>=0,
                              'invalid correspondence integer')
        for field in ('source_index', 'depth_file', 'depth_timestamp', 'rgb_file', 'rgb_timestamp',
                      'rgb_source_index', 'pair_gap_seconds', 'split'):
            if field == 'pair_gap_seconds':
                v.same_pair_gap(row[field], frame[field])
            else:
                r.require(row[field] == frame[field], 'changed paired observation/clock')
        r.require(type(row['accepted']) is bool and type(row['initialized']) is bool,
                  'invalid acceptance/initialization')
        r.require(isinstance(row['cpu_wall_seconds'], (int, float))
                  and math.isfinite(row['cpu_wall_seconds']) and row['cpu_wall_seconds'] >= 0,
                  'invalid CPU timing')
        r.require(row['last_accepted_stamp_before'] == last_accepted
                  and row['reference_frame_index_before'] == (frames[reference]['source_index']
                                                              if reference is not None else None)
                  and row['lost_before'] == lost
                  and row['last_observed_rgb_stamp_before'] == last_observed_rgb,
                  'invented before-acquisition pose/reference/clock state')
        stamp, rgb_stamp = frame['depth_timestamp'], frame['rgb_timestamp']
        blocked = v.clock_block_reason(lost, last_accepted, last_observed_rgb, stamp, rgb_stamp)
        computation_allowed = blocked is None
        r.require(row['features_computed'] is computation_allowed
                  and row['feature_count'] == (len(observed) if computation_allowed else 0),
                  'computed features before clock/freshness guard')
        v.feature_check(row['features'], observed if computation_allowed else [])
        truth = k.truth_relative(origin, k.timed_pose(gt, stamp))
        if i:
            counters['reference_valid_updates'] += int(truth is not None)
        rejected = None
        predicted = None
        refinement_attempted = False
        evidence = dict(source_index=frame['source_index'], feature_count=row['feature_count'],
                        features_computed=computation_allowed)
        if blocked:
            rejected = blocked
            lost = lost or 'localization lost' in blocked
        else:
            last_observed_rgb = rgb_stamp
            if reference is None:
                r.require(row['matches'] == [] and row['depth_matches'] == [] and row['correspondences'] == []
                          and not row.get('refinement_observations')
                          and not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement', 'relative_estimate')),
                          'invented initialization fit')
                good, ratio, point_count = v.initialized_geometry(row, observed, depths[i], calibration)
                evidence.update(initialization_geometry_ratio=ratio,
                                initialization_depth_points=point_count)
                if good:
                    predicted = v.IDENTITY
                    r.require(row['initialized'] is True, 'concealed initialization')
                    counters['initialized_frames'] += 1
                else:
                    lost = True
                    rejected = row.get('rejection')
                    r.require(bool(rejected), 'concealed initialization failure')
            else:
                r.require(row['reference_frame_index'] == frames[reference]['source_index']
                          and row['reference_stamp'] == frames[reference]['depth_timestamp'],
                          'invented reference acquisition')
                k.same_pose(r.pose(row['root_from_reference']), root_reference, 'invented reference root')
                matches = v.descriptor_matches(all_features[reference], observed)
                pairs = v.matched_geometry(row, matches, all_features[reference], observed,
                                         depths[reference], depths[i], calibration)
                independently, rejected = temporal_rigid_fit(pairs)
                evidence.update(descriptor_matches=len(matches), valid_depth_matches=len(pairs))
                if independently is not None:
                    r.require(row['initialized'] is False, 'silently changed root origin')
                    coarse = r.pose(row['coarse_relative_estimate'])
                    v.fit_check(row['fit'], coarse, pairs, independently)
                    observations = g.refinement_observations(row, independently, matches, observed, pairs)
                    refinement_attempted = True
                    refined, rejected = temporal_refinement(observations, calibration, coarse)
                    if refined is not None:
                        g.refinement_check(row['refinement'], r.pose(row['relative_estimate']), observations, calibration, coarse, refined)
                        predicted = k.compose(root_reference, r.pose(row['relative_estimate']))
                        evidence['refinement'] = refined
                    else:
                        r.require('refinement' not in row and 'relative_estimate' not in row, 'concealed refinement failure')
                        r.require(row['refinement_rejection'] == rejected, 'concealed backend refinement rejection')
                    evidence['fit'] = {field: value for field, value in independently.items() if field != 'estimate'}
                else:
                    r.require(not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement', 'relative_estimate'))
                              and not row.get('refinement_observations'), 'invented refinement without coarse consensus')
        accepted = predicted is not None
        r.require(row['refinement_attempted'] is refinement_attempted,
                  'invented or concealed refinement invocation')
        if not refinement_attempted or accepted:
            r.require('refinement_rejection' not in row, 'invented refinement rejection')
        r.require(row['accepted'] is accepted, 'changed independently reconstructed acceptance')
        evidence['accepted'] = accepted
        score = row['root_accuracy']
        if label_failure is not None:
            r.require(score.get('reference_rejection') == label_failure['reason'],
                      'concealed strict source-label failure')
        r.require(score['valid'] == accepted, 'invented root validity')
        if accepted:
            k.same_pose(r.pose(row['root_estimate']), predicted, 'invented root pose/composition')
            evidence['root'] = k.scored_pose(score, predicted, truth, 'visual root')
            reference, root_reference, last_accepted = i, predicted, stamp
            if i:
                counters['accepted_updates'] += 1
                counters['accurate_root_updates'] += int(score['within_accuracy_gates'])
            r.require('rejection' not in row, 'accepted pose retains contradictory rejection')
        else:
            r.require(row.get('rejection') == rejected and row['initialized'] is False
                      and not any(field in row for field in ('root_estimate', 'relative_estimate', 'refinement')),
                      'concealed failure/invented rejected pose')
            k.scored_pose(score, None, truth, 'rejected visual root')
            if i:
                counters['rejected_updates'] += 1
            evidence['rejection'] = rejected
        if rejected and ('lost' in rejected or 'duplicate or stale' in rejected):
            r.require(row['matches'] == [] and row['depth_matches'] == [] and row['correspondences'] == []
                      and not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement'))
                      and not row.get('refinement_observations'),
                      'performed visual fit after expired/duplicate acquisition')
        r.require(row['last_accepted_stamp_after'] == last_accepted
                  and row['reference_frame_index_after'] == (frames[reference]['source_index']
                                                             if reference is not None else None)
                  and row['lost_after'] == lost
                  and row['last_observed_rgb_stamp_after'] == last_observed_rgb,
                  'invented after-acquisition pose/reference/clock state')
        records.append(evidence)
    updates = len(frames)-1
    passed = (counters['initialized_frames'] == 1 and counters['accepted_updates'] == updates
              and counters['accurate_root_updates'] == updates
              and counters['reference_valid_updates'] == updates)
    expected = dict(frames=len(frames), updates=updates, **counters, lost=lost,
                    all_updates_passed=passed)
    r.require(report['summary'] == expected, 'summary hides rejected/unscorable acquisitions')
    return records



def audit(report, manifest, features, depths, gt, label_failure):
    r.require(report['qualification_verified'] is True, 'qualification gate not reported')
    exact_json(report['qualification'], report['freeze']['qualification'], 'report changed qualification proof')
    r.require(report['qualification_sha256'] == report['freeze']['qualification_sha256'],
              'report changed qualification proof hash')
    for field, expected in TEMPORAL_POLICY.items():
        exact_json(report['freeze'][field], expected, 'changed continuous temporal policy '+field)
    return continuous_audit(report, manifest, features, depths, gt, label_failure)


def mutation_checks(report, manifest, all_features, depths, gt, label_failure=None):
    audit(report,manifest,all_features,depths,gt,label_failure)
    tests = [
        ('omit_acquisition', lambda x: x['frames'].pop()),
        ('invent_summary', lambda x: x['summary'].__setitem__('accurate_root_updates', 999)),
        ('operational_truth', lambda x: x.__setitem__('ground_truth_operational', True)),
        ('source_archive', lambda x: x['official_archive'].__setitem__('sha256', '0'*64)),
        ('source_design',lambda x:x.__setitem__('preregistration_sha256','0'*64)),
        ('RGB_clock', lambda x: x['frames'][1].__setitem__('rgb_timestamp', 0.)),
        ('depth_clock', lambda x: x['frames'][1].__setitem__('depth_timestamp', 0.)),
        ('invent_derived_pair_gap', lambda x: x['frames'][1].__setitem__(
            'pair_gap_seconds', manifest['frames'][1]['pair_gap_seconds']+1e-12)),
        ('accepted_clock', lambda x: x['frames'][0].__setitem__('last_accepted_stamp_after', 0.)),
        ('observed_RGB_clock', lambda x: x['frames'][0].__setitem__('last_observed_rgb_stamp_after', 0.)),
        ('reference_clock', lambda x: x['frames'][0].__setitem__('reference_frame_index_after', 999)),
        ('weaken_consensus', lambda x: x['freeze']['registration_config'].__setitem__('min_inliers', 1)),
        ('weaken_ambiguity', lambda x: x['freeze']['registration_config'].__setitem__('ambiguity_support_ratio', 1.1)),
    ]
    if label_failure is not None:
        tests.extend([
            ('conceal_invalid_label_source', lambda x: x.pop('evaluation_label_failure')),
            ('invent_invalid_label_reason', lambda x: x['evaluation_label_failure'].__setitem__('reason', 'invented')),
            ('invent_invalid_label_hash', lambda x: x['evaluation_label_failure'].__setitem__('source_sha256', '0'*64)),
            ('invent_invalid_label_file', lambda x: x['evaluation_label_failure'].__setitem__('file', 'rgb.txt')),
            ('claim_mocap_for_invalid_source', lambda x: x['frames'][0]['root_accuracy'].__setitem__('reference_valid', True)),
            ('claim_accuracy_for_invalid_source', lambda x: x['frames'][0]['root_accuracy'].__setitem__('within_accuracy_gates', True)),
            ('invent_invalid_source_error', lambda x: x['frames'][0]['root_accuracy'].__setitem__('translation_error_m', 0.)),
            ('conceal_row_label_failure', lambda x: x['frames'][0]['root_accuracy'].__setitem__('reference_rejection', 'invented')),
        ])
    else:
        tests.append(('invent_label_failure_for_valid_source', lambda x: x.__setitem__(
            'evaluation_label_failure', dict(file='groundtruth.txt', source_sha256='0'*64, reason='invented'))))
    feature = next((i for i, row in enumerate(report['frames']) if row['features']), None)
    if feature is not None:
        tests.extend([
            ('wrong_feature_pixel', lambda x: x['frames'][feature]['features'][0].__setitem__('x', 0.)),
            ('wrong_FAST_score', lambda x: x['frames'][feature]['features'][0].__setitem__('score', 0)),
            ('wrong_BRIEF', lambda x: x['frames'][feature]['features'][0]['descriptor'].__setitem__(0, x['frames'][feature]['features'][0]['descriptor'][0] ^ 1)),
            ('wrong_orientation', lambda x: x['frames'][feature]['features'][0].__setitem__('orientation', 100.)),
        ])
    matched = next((i for i, row in enumerate(report['frames']) if row['matches']), None)
    if matched is not None:
        tests.extend([
            ('invent_descriptor_distance', lambda x: x['frames'][matched]['matches'][0].__setitem__('hamming_distance', 999)),
            ('wrong_feature_match_ID', lambda x: x['frames'][matched]['matches'][0].__setitem__('current_index', 999)),
            ('omit_depth_rejection', lambda x: x['frames'][matched]['depth_matches'].pop()),
        ])
    accepted = next((i for i, row in enumerate(report['frames']) if i and row['accepted']), None)
    if accepted is not None:
        tests.extend([
            ('invent_depth_point', lambda x: x['frames'][accepted]['correspondences'][0]['current'].__setitem__(0, 999.)),
            ('invent_relative_pose', lambda x: x['frames'][accepted]['relative_estimate']['translation_m'].__setitem__(0, 100.)),
            ('invent_root_pose', lambda x: x['frames'][accepted]['root_estimate']['translation_m'].__setitem__(0, 100.)),
            ('invent_reference_root', lambda x: x['frames'][accepted]['root_from_reference']['translation_m'].__setitem__(0, 100.)),
            ('invent_inlier_membership', lambda x: x['frames'][accepted]['fit']['inlier_indices'].pop()),
            ('invent_inlier_ratio', lambda x: x['frames'][accepted]['fit'].__setitem__('inlier_ratio', 0.)),
            ('invent_RMS', lambda x: x['frames'][accepted]['fit'].__setitem__('rms_m', 100.)),
            ('invent_hypothesis_work', lambda x: x['frames'][accepted]['fit'].__setitem__('point_checks', 0)),
            ('invent_consensus_refits', lambda x: x['frames'][accepted]['fit'].__setitem__('refits', 0)),
            ('invent_candidates', lambda x: x['frames'][accepted]['fit'].__setitem__('candidate_models', 999)),
            ('invent_competing_models', lambda x: x['frames'][accepted]['fit'].__setitem__('competing_models', 1)),
            ('invent_accuracy', lambda x: x['frames'][accepted]['root_accuracy'].__setitem__('translation_error_m', 999.)),
        ])
    rejected = next((i for i, row in enumerate(report['frames']) if not row['accepted']), None)
    if rejected is not None:
        tests.append(('invent_rejected_pose', lambda x: x['frames'][rejected].__setitem__(
            'root_estimate', dict(translation_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]))))
    coarse_failed = next((i for i, row in enumerate(report['frames'])
                         if i and row['features_computed'] and not row['accepted'] and 'fit' not in row), None)
    if coarse_failed is not None:
        tests.extend([
            ('forge_failed_coarse_witness', lambda x: x['frames'][coarse_failed].__setitem__('fit', {})),
            ('forge_failed_coarse_pose', lambda x: x['frames'][coarse_failed].__setitem__('coarse_relative_estimate',
                dict(translation_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]))),
            ('forge_failed_refinement_observation', lambda x: x['frames'][coarse_failed].__setitem__('refinement_observations', [{}])),
        ])
    blocked = next((i for i, row in enumerate(report['frames']) if not row['features_computed']), None)
    if blocked is not None:
        tests.extend([
            ('forge_blocked_coarse_witness', lambda x: x['frames'][blocked].__setitem__('fit', {})),
            ('compute_after_duplicate_guard', lambda x: x['frames'][blocked].__setitem__('features_computed', True)),
            ('renew_duplicate_accepted_clock', lambda x: x['frames'][blocked].__setitem__(
                'last_accepted_stamp_after', x['frames'][blocked]['depth_timestamp'])),
        ])
    refined = next((i for i, row in enumerate(report['frames']) if 'refinement' in row), None)
    if refined is not None:
        tests.extend([
            ('conceal_refinement_invocation', lambda x: x['frames'][refined].__setitem__('refinement_attempted', False)),
            ('invent_refinement_rejection', lambda x: x['frames'][refined].__setitem__('refinement_rejection', 'invented')),
            ('invent_coarse_pose', lambda x: x['frames'][refined]['coarse_relative_estimate']['translation_m'].__setitem__(0, 100.)),
            ('drop_refinement_support', lambda x: x['frames'][refined]['refinement_observations'].pop()),
            ('wrong_refinement_inlier_ID', lambda x: x['frames'][refined]['refinement_observations'][0].__setitem__('correspondence_index', 999)),
            ('wrong_refinement_pixel', lambda x: x['frames'][refined]['refinement_observations'][0]['current_pixel_xy'].__setitem__(0, 0.)),
            ('wrong_refinement_depth_point', lambda x: x['frames'][refined]['refinement_observations'][0]['previous_xyz'].__setitem__(0, 999.)),
            ('invent_pixel_initial_cost', lambda x: x['frames'][refined]['refinement'].__setitem__('initial_huber_cost', 9999.)),
            ('invent_pixel_final_cost', lambda x: x['frames'][refined]['refinement'].__setitem__('final_huber_cost', 9999.)),
            ('invent_pixel_RMS', lambda x: x['frames'][refined]['refinement'].__setitem__('final_rms_px', 9999.)),
            ('invent_pixel_work', lambda x: x['frames'][refined]['refinement'].__setitem__('point_checks', 0)),
            ('invent_pixel_support', lambda x: x['frames'][refined]['refinement'].__setitem__('valid_support', 1)),
            ('invent_pixel_convergence', lambda x: x['frames'][refined]['refinement'].__setitem__('converged', not x['frames'][refined]['refinement']['converged'])),
            ('weaken_pixel_Huber', lambda x: x['freeze']['refinement_config'].__setitem__('huber_delta_px', 1000.)),
            ('weaken_pixel_workcap', lambda x: x['freeze']['refinement_config'].__setitem__('max_point_checks', 1000000)),
        ])
        if report['frames'][refined]['refinement']['trace']:
            tests.extend([
                ('nonmonotonic_pixel_step', lambda x: x['frames'][refined]['refinement']['trace'][0].__setitem__('huber_cost_after', 99999.)),
                ('invent_pixel_increment', lambda x: x['frames'][refined]['refinement']['trace'][0]['increment'].__setitem__(0, 10.)),
                ('invent_pixel_line_search', lambda x: x['frames'][refined]['refinement']['trace'][0].__setitem__('line_search_trials', 999)),
            ])
    failures = []
    for name, change in tests:
        altered = copy.deepcopy(report)
        change(altered)
        r.require(altered!=report,'report mutation did not change baseline '+name)
        try:
            audit(altered, manifest, all_features, depths, gt, label_failure)
        except (ValueError, KeyError, IndexError):
            failures.append(name)
        else:
            raise ValueError('corrupted visual report accepted: '+name)
    return failures


def write_result(output, result):
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')


def qualification_mutations(freeze, manifest_raw, proof_raw, metadata, source_root):
    rejected = []
    for field in SOURCES:
        changed = copy.deepcopy(freeze); changed[field] = '0'*64
        r.require(changed!=freeze,'source mutation did not change baseline '+field)
        try:
            verify_sources(changed, manifest_raw, proof_raw, source_root)
        except (ValueError, KeyError):
            rejected.append('source_'+field)
        else:
            raise ValueError('changed source accepted '+field)
    policy_changes = [('refinement_config','max_iterations',100),
        ('registration_config','min_inliers',1), ('tracking_policy','max_unobserved_s',2.),
        ('preprocessing','patch_max_spread_m',1.), ('feature_policy','max_features',4000),
        ('ground_truth_interpolation','max_bracket_s',1.),
        ('evaluation_label_policy','invalid_source','deduplicate invalid reference poses')]
    for field, key, value in policy_changes:
        changed = copy.deepcopy(freeze); changed[field][key] = value
        r.require(changed!=freeze,'policy mutation did not change baseline '+field)
        if field+'_sha256' in changed:
            changed[field+'_sha256'] = (g.refinement_config_digest(changed[field]) if field == 'refinement_config'
                                        else r.canonical(changed[field]))
        try:
            verify_sources(changed, manifest_raw, proof_raw, source_root)
        except (ValueError, KeyError):
            rejected.append('policy_'+field)
        else:
            raise ValueError('weakened policy accepted '+field)
    proof = json.loads(proof_raw)
    changes = [ ('hide_metadata_failure',lambda x:x.__setitem__('passed',False)),
        ('invent_gt_row_order',lambda x:x['timestamps'].__setitem__('strict_ground_truth_order',False)),
        ('invent_gt_row_count',lambda x:x['timestamps'].__setitem__('ground_truth_rows',proof['timestamps']['ground_truth_rows']+1)),
        ('invent_gt_bracket_stat',lambda x:x['timestamps'].__setitem__('maximum_observed_ground_truth_bracket_us',proof['timestamps']['maximum_observed_ground_truth_bracket_us']+1)),
        ('invent_pair_gap_stat',lambda x:x['timestamps'].__setitem__('maximum_pair_gap_us',proof['timestamps']['maximum_pair_gap_us']+1)),
        ('weaken_pair_gap_gate',lambda x:x['timestamps'].__setitem__('max_pair_gap_s',2.)),
        ('invent_unique_rgb_count',lambda x:x['timestamps'].__setitem__('unique_rgb_acquisitions',999)),
        ('invent_qualification_window',lambda x:x['window'].__setitem__('first_depth_index',0)),
        ('invent_qualification_metadata_hash',lambda x:x['metadata_files']['groundtruth.txt'].__setitem__('sha256','0'*64)),
        ('invent_qualification_helper',lambda x:x.__setitem__('helper_sha256','0'*64)),
        ('qualification_parsed_pose_values',lambda x:x.__setitem__('ground_truth_pose_values_parsed',True)),
        ('qualification_decoded_pixels',lambda x:x.__setitem__('pixels_read',True)),
        ('qualification_opened_image_header',lambda x:x.__setitem__('image_headers_read',True)),
        ('qualification_ran_features',lambda x:x.__setitem__('features_or_fits_run',True)),
    ]
    for name, change in changes:
        altered_proof = copy.deepcopy(proof); change(altered_proof)
        r.require(altered_proof!=proof,'qualification mutation did not change baseline '+name)
        altered_raw = json.dumps(altered_proof,sort_keys=True).encode()
        altered_freeze = copy.deepcopy(freeze)
        altered_freeze['qualification'] = altered_proof
        altered_freeze['qualification_sha256'] = r.digest(altered_raw)
        try:
            verify_sources(altered_freeze,manifest_raw,altered_raw,source_root)
            qualification_check(altered_freeze,manifest_raw,altered_proof,metadata)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('self-consistently rehashed qualification accepted '+name)
    manifest = json.loads(manifest_raw)
    for name, field, value in [('rename_official_url','url','https://unverified.invalid/archive.tgz'),
                               ('rename_archive_root','root','different-recording/')]:
        changed = copy.deepcopy(manifest); changed['official_archive'][field] = value
        r.require(changed!=manifest,'official source mutation did not change baseline '+name)
        changed_raw = json.dumps(changed,sort_keys=True).encode()
        changed_freeze = copy.deepcopy(freeze); changed_freeze['manifest_sha256'] = r.digest(changed_raw)
        try:
            verify_sources(changed_freeze,changed_raw,proof_raw,source_root)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('self-consistently rehashed source accepted '+name)
    return rejected


def metadata_contract_checks():
    frames = []
    depth_rows, rgb_rows = [], []
    for i in range(280):
        stamp, rgb_stamp = i/64.+1/128., i/64.
        dep = f'{stamp:.9f}.png'; rgb = f'{rgb_stamp:.9f}.png'
        depth_rows.append(f'{stamp:.9f} depth/{dep}\n')
        rgb_rows.append(f'{rgb_stamp:.9f} rgb/{rgb}\n')
        if i >= 100:
            frames.append(dict(source_index=i,depth_timestamp=stamp,depth_file='depth-'+dep,
                rgb_source_index=i,rgb_timestamp=rgb_stamp,rgb_file='rgb-'+rgb,
                pair_gap_seconds=1/128.,split='initialization' if i==100 else 'independent_recording'))
    # Pose tokens deliberately are not numeric: qualification must treat them
    # as opaque while still enforcing eight-column metadata syntax.
    labels = ''.join(f'{i/128.:.9f} opaque opaque opaque opaque opaque opaque opaque\n' for i in range(570)).encode()
    metadata = {'depth.txt':''.join(depth_rows).encode(),'rgb.txt':''.join(rgb_rows).encode(),
                'groundtruth.txt':labels}
    manifest = dict(frames=frames)
    result = independently_qualify(manifest,metadata)
    r.require(result['unique_rgb_frames']==180 and result['maximum_pair_gap_us']==7813
              and result['maximum_observed_ground_truth_bracket_us']==0,
              'known earlier-index RGB tie or opaque-pose qualification changed')
    positive = ['opaque_pose_tokens_not_converted','earlier_rgb_index_on_exact_tie',
                'exact_reference_timestamp_zero_bracket','all_180_rows_retained']
    rejected = []
    corrupted = [ ('duplicate_gt_timestamp',dict(metadata,**{'groundtruth.txt':labels+labels.splitlines()[-1]+b'\n'})),
        ('nonfinite_gt_timestamp',dict(metadata,**{'groundtruth.txt':b'nan opaque opaque opaque opaque opaque opaque opaque\n'+labels})),
        ('wrong_gt_row_arity',dict(metadata,**{'groundtruth.txt':b'0 opaque\n'+labels})),
        ('duplicate_depth_timestamp',dict(metadata,**{'depth.txt':metadata['depth.txt']+depth_rows[-1].encode()})),
        ('oversized_gt_metadata',dict(metadata,**{'groundtruth.txt':b' '*(4194304+1)})),
        ('aggregate_metadata_cap',{'depth.txt':b'#'*(2*1024*1024),'rgb.txt':b'#'*(2*1024*1024),'groundtruth.txt':b'#'*(2*1024*1024)}) ]
    for name, altered in corrupted:
        r.require(altered!=metadata,'metadata mutation did not change baseline '+name)
        try:
            independently_qualify(manifest,altered)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('corrupted qualification metadata accepted '+name)
    for name, change in [('wrong_original_depth_index',lambda x:x['frames'][0].__setitem__('source_index',0)),
        ('wrong_nearest_rgb_index',lambda x:x['frames'][0].__setitem__('rgb_source_index',101)),
        ('invent_pair_gap',lambda x:x['frames'][0].__setitem__('pair_gap_seconds',.001)),
        ('omit_observed_depth_row',lambda x:x['frames'].pop())]:
        altered = copy.deepcopy(manifest); change(altered)
        r.require(altered!=manifest,'manifest mutation did not change baseline '+name)
        try:
            independently_qualify(altered,metadata)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('corrupted source association accepted '+name)
    return dict(passed=positive,mutations_rejected=rejected)


def temporal_report_mutations(report, manifest, features, depths, gt, label_failure):
    audit(report,manifest,features,depths,gt,label_failure)
    failures = []
    for name, change in [ ('conceal_qualification_gate',lambda x:x.__setitem__('qualification_verified',False)),
        ('conceal_report_qualification',lambda x:x.pop('qualification')),
        ('invent_report_qualification_hash',lambda x:x.__setitem__('qualification_sha256','0'*64)),
        ('invent_report_qualification_count',lambda x:x['qualification']['timestamps'].__setitem__('ground_truth_rows',report['qualification']['timestamps']['ground_truth_rows']+1)),
        ('invent_report_algorithm',lambda x:x.__setitem__('algorithm','unverified_algorithm')),
        ('drop_independent_tail',lambda x:x.__setitem__('frames',x['frames'][:36])),
        ('omit_final_observation',lambda x:x['frames'].pop()),
        ('reclassify_independent_recording',lambda x:x['frames'][36].__setitem__('split','viewed_prefix')),
        ('forge_midsequence_initialization',lambda x:x['frames'][36].__setitem__('initialized',True)),
        ('forge_boundary_clock_reset',lambda x:x['frames'][36].__setitem__('last_accepted_stamp_before',None)),
        ('forge_boundary_reference_reset',lambda x:x['frames'][36].__setitem__('reference_frame_index_before',None)),
        ('allow_chunk_resets',lambda x:x['freeze']['continuous_state'].__setitem__('chunk_resets',True)),
        ('allow_multiple_initializations',lambda x:x['freeze']['continuous_state'].__setitem__('maximum_initializations',2)),
        ('claim_fresh_generalization',lambda x:x['freeze'].__setitem__('evidence_scope','fresh environment generalization')),
        ('reverse_temporal_boundary',lambda x:x['frames'].__setitem__(slice(35,37),list(reversed(x['frames'][35:37])))) ]:
        changed = copy.deepcopy(report); change(changed)
        r.require(changed!=report,'continuous mutation did not change baseline '+name)
        try:
            audit(changed,manifest,features,depths,gt,label_failure)
        except (ValueError,KeyError):
            failures.append(name)
        else:
            raise ValueError('temporal report corruption accepted '+name)
    return failures


def verified_inputs(manifest, raw_path, report=None):
    raw_inventory_check(manifest,raw_path)
    if report is None:
        report = dict(calibration_source=manifest['calibration_source'], calibration_sha256_verified=True)
    v.calibration_check(report, manifest, raw_path)
    raw, total = {}, 0
    for item in manifest['files']:
        filename = item['file']
        r.require(Path(filename).name == filename and '/' not in filename and '\\' not in filename,
                  'unsafe raw filename')
        content = r.bounded(raw_path/filename)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'],
                  'measured RGB/depth/mocap byte mismatch')
        raw[filename] = content
        total += len(content)
    r.require(total+manifest['calibration_source']['bytes'] <= 128*1024*1024, 'raw acquisition total bound')
    if 'files' in report:
        r.require(report['files'] == [{key: item[key] for key in ('file', 'source_path', 'bytes', 'sha256', 'role')}
                                     for item in manifest['files']], 'changed input provenance')
    expected_names = {'depth.txt', 'rgb.txt', 'groundtruth.txt'} | {
        frame[key] for frame in manifest['frames'] for key in ('depth_file', 'rgb_file')}
    r.require(set(raw) == expected_names and len(manifest['files']) == len(expected_names),
              'unexpected/incomplete visual input inventory')
    return raw


@lru_cache(maxsize=1)
def continuous_contract_checks():
    """Known cumulative motion; same auditor, all179 denominators, no real input."""
    import random
    rng=random.Random(42)
    descriptors=[[rng.getrandbits(64) for _ in range(4)] for _ in range(12)]
    points=[(x,y) for y in (120,240,360) for x in (120,180,240,300)]
    depth=g.np.full((480,640),7500,dtype=g.np.uint16)
    identity=([0.,0.,0.],[1.,0.,0.,0.])
    pose=lambda value:dict(translation_m=value[0],quaternion_wxyz=value[1])
    proofs=[];mutants=[]
    for degraded in (False,True):
        all_features=[[dict(x=float(x+i),y=float(y),score=40,orientation=0.,descriptor=descriptors[j])
                       for j,(x,y) in enumerate(points)] for i in range(180)]
        frames=[dict(source_index=100+i,depth_file=f'depth-{i}.png',depth_timestamp=1.+i/64.,
                     rgb_file=f'rgb-{i}.png',rgb_timestamp=1.+i/64.,rgb_source_index=100+i,
                     pair_gap_seconds=0.,split='initialization' if i==0 else 'independent_recording') for i in range(180)]
        if degraded:
            frames[2].update(rgb_file=frames[1]['rgb_file'],rgb_timestamp=frames[1]['rgb_timestamp'],
                             rgb_source_index=101,pair_gap_seconds=1/64.)
            all_features[2]=all_features[1]
            all_features[4:]=[[] for _ in range(176)]
        source=dict(url='synthetic',final_url='synthetic',root='synthetic/',bytes=1,
                    sha256='a'*64,md5='b'*32,published_checksum=None)
        manifest=dict(dataset='hand-independent-motion',official_archive=source,
                      frames=frames,depth_calibration=CAMERA)
        freeze=dict(registration_config=v.CONFIG,refinement_config=g.REFINEMENT_CONFIG,
                    tracking_policy=dict(max_unobserved_s=.2),qualification={'timestamps':{'ground_truth_rows':180}},
                    qualification_sha256='hand',**TEMPORAL_POLICY)
        report=dict(schema_version=1,algorithm=ALGORITHM,dataset=manifest['dataset'],official_archive=source,preregistration_sha256=DESIGN_SHA,
                    ground_truth_operational=False,raw_redistributed=False,qualification_verified=True,
                    qualification=copy.deepcopy(freeze['qualification']),qualification_sha256='hand',
                    freeze=freeze,frames=[])
        reference=None;root=identity;accepted_stamp=None;observed_stamp=None;lost=False
        for i,(frame,observed) in enumerate(zip(frames,all_features)):
            row=dict(frame,accepted=False,initialized=False,features=[],feature_count=0,features_computed=False,
                     cpu_wall_seconds=0.,last_accepted_stamp_before=accepted_stamp,
                     reference_frame_index_before=100+reference if reference is not None else None,
                     lost_before=lost,last_observed_rgb_stamp_before=observed_stamp,
                     matches=[],depth_matches=[],correspondences=[],refinement_observations=[],refinement_attempted=False)
            blocked=v.clock_block_reason(lost,accepted_stamp,observed_stamp,frame['depth_timestamp'],frame['rgb_timestamp'])
            predicted=None;rejected=blocked
            if blocked:lost=lost or 'localization lost' in blocked
            else:
                observed_stamp=frame['rgb_timestamp'];row.update(features=observed,feature_count=len(observed),features_computed=True)
                if reference is None:
                    measured=[g.depth_point_status(f,depth,CAMERA)[0] for f in observed]
                    row['initialization_depth_features']=[dict(feature_index=j,accepted=True,point=p) for j,p in enumerate(measured)]
                    row['initialization_geometry_ratio']=v.scatter_ratio(measured)
                    row['initialized']=True;predicted=identity
                else:
                    row.update(reference_frame_index=100+reference,reference_stamp=frames[reference]['depth_timestamp'],
                               root_from_reference=pose(root))
                    matches=v.descriptor_matches(all_features[reference],observed);pairs=[];depth_rows=[]
                    for j,match in enumerate(matches):
                        previous=g.depth_point_status(all_features[reference][match['previous_index']],depth,CAMERA)[0]
                        current=g.depth_point_status(observed[match['current_index']],depth,CAMERA)[0]
                        pairs.append(dict(previous=previous,current=current))
                        depth_rows.append(dict(previous_index=match['previous_index'],current_index=match['current_index'],
                            accepted=True,previous_xyz=previous,current_xyz=current,correspondence_index=j))
                    row.update(matches=matches,depth_matches=depth_rows,correspondences=pairs)
                    fitted,rejected=temporal_rigid_fit(pairs)
                    if fitted is not None:
                        coarse=fitted['estimate'];row.update(fit={k:v for k,v in fitted.items() if k!='estimate'},
                                                            coarse_relative_estimate=pose(coarse),refinement_attempted=True)
                        observations=[dict(correspondence_index=j,previous_index=matches[j]['previous_index'],
                            current_index=matches[j]['current_index'],previous_xyz=pairs[j]['previous'],
                            current_pixel_xy=[observed[matches[j]['current_index']]['x'],observed[matches[j]['current_index']]['y']])
                            for j in fitted['inlier_indices']]
                        row['refinement_observations']=observations
                        refined,rejected=temporal_refinement(observations,CAMERA,coarse)
                        r.require(refined is not None,'nonzero synthetic refinement positive failed')
                        details={k:copy.deepcopy(v) for k,v in refined.items() if k!='estimate'}
                        for step in details['trace']:step['current_from_previous']=pose(step['current_from_previous'])
                        row.update(relative_estimate=pose(refined['estimate']),refinement=details)
                        predicted=k.compose(root,refined['estimate'])
            truth=([-i*1.5/CAMERA['fx'],0.,0.],[1.,0.,0.,0.])
            if predicted is not None:
                position=r.norm([a-b for a,b in zip(predicted[0],truth[0])]);angle=r.quaternion_error(predicted[1],truth[1])
                r.require(position<1e-8 and angle<1e-7,'synthetic root disagrees with analytic accumulated motion')
                row.update(accepted=True,root_estimate=pose(predicted),root_accuracy=dict(valid=True,estimate=pose(predicted),
                    reference_valid=True,evaluation_only_truth=pose(truth),translation_error_m=position,
                    rotation_error_rad=angle,within_accuracy_gates=True))
                reference=i;root=predicted;accepted_stamp=frame['depth_timestamp']
            else:
                row.update(rejection=rejected,root_accuracy=dict(valid=False,reference_valid=True,
                    evaluation_only_truth=pose(truth),within_accuracy_gates=False))
            row.update(last_accepted_stamp_after=accepted_stamp,reference_frame_index_after=100+reference,
                       lost_after=lost,last_observed_rgb_stamp_after=observed_stamp)
            report['frames'].append(row)
        accepted=sum(row['accepted'] for row in report['frames'][1:])
        report['summary']=dict(frames=180,updates=179,initialized_frames=1,accepted_updates=accepted,
            rejected_updates=179-accepted,accurate_root_updates=accepted,reference_valid_updates=179,
            lost=lost,all_updates_passed=accepted==179)
        labels=[(frame['depth_timestamp'],[-i*1.5/CAMERA['fx'],0.,0.],[1.,0.,0.,0.]) for i,frame in enumerate(frames)]
        depths=[depth]*180
        records=audit(report,manifest,all_features,depths,labels,None)
        r.require(len(records)==180,'synthetic continuous audit omits observations')
        if degraded:
            r.require(accepted==2 and report['frames'][2]['rejection']=='duplicate or stale RGB acquisition; no pose permission renewal'
                and report['frames'][2]['last_accepted_stamp_after']==frames[1]['depth_timestamp']
                and report['frames'][16]['lost_after'] and not report['frames'][15]['lost_after'],
                'synthetic duplicate/rejected-fit/expiry positive differs from declared physics')
            proofs.extend(['duplicate_RGB_does_not_renew_reference_or_clock','rejected_features_preserve_accepted_reference',
                           'expiry_at13_unaccepted_intervals_latches_Lost','all179_rejected_or_accepted_updates_preserved'])
        else:
            r.require(accepted==179 and root[0][0]<-.5,'whole179 nonzero accumulated motion control is vacuous')
            proofs.extend(['complete180_known_motion_passes_same_auditor','179_nonzero_measured_correspondence_updates',
                           'analytic_accumulated_translation_exceeds_half_metre','one_initialization_no_reset'])
        mutants.extend(('degraded:' if degraded else 'healthy:')+name for name in
            mutation_checks(report,manifest,all_features,depths,labels,None)+
            temporal_report_mutations(report,manifest,all_features,depths,labels,None))
    return dict(passed=proofs,mutations_rejected=mutants,hand_fixture_only=True,
                actual_recording_images_or_pose_values_read=False,terminal_known_translation_m=-179*1.5/CAMERA['fx'])


@lru_cache(maxsize=1)
def pixel_feature_contract_checks():
    """Actual generated grayscale, original FAST/BRIEF, nonzero physical motion."""
    previous=g.np.random.default_rng(42).integers(0,256,(480,640),dtype=g.np.uint8)
    current=g.np.roll(previous,3,axis=1)
    before=v.features(previous);after=v.features(current);matches=v.descriptor_matches(before,after)
    r.require(len(matches)>=12,'known translated image lacks measured descriptor correspondences')
    r.require(all(after[m['current_index']]['x']==before[m['previous_index']]['x']+3
        and after[m['current_index']]['y']==before[m['previous_index']]['y'] for m in matches),
        'measured synthetic descriptor correspondence disagrees with known pixel translation')
    depth=g.np.full((480,640),7500,dtype=g.np.uint16);pairs=[];depth_rows=[]
    for j,m in enumerate(matches):
        pp=g.depth_point_status(before[m['previous_index']],depth,CAMERA)[0]
        cp=g.depth_point_status(after[m['current_index']],depth,CAMERA)[0]
        pairs.append(dict(previous=pp,current=cp))
        depth_rows.append(dict(previous_index=m['previous_index'],current_index=m['current_index'],
            accepted=True,previous_xyz=pp,current_xyz=cp,correspondence_index=j))
    row=dict(matches=matches,depth_matches=depth_rows,correspondences=pairs)
    reconstructed=v.matched_geometry(row,matches,before,after,depth,depth,CAMERA)
    fit,error=temporal_rigid_fit(reconstructed)
    r.require(error is None and r.norm([fit['estimate'][0][0]+3*1.5/CAMERA['fx'],
        fit['estimate'][0][1],fit['estimate'][0][2]])<1e-8,'pixel-derived physical motion differs from known translation')
    failed=[]
    for name,change in [('invent_pixel_derived_Hamming',lambda x:x['matches'][0].__setitem__('hamming_distance',999)),
        ('invent_pixel_derived_feature_ID',lambda x:x['matches'][0].__setitem__('current_index',999)),
        ('invent_pixel_derived_depth',lambda x:x['depth_matches'][0]['current_xyz'].__setitem__(2,0.)),
        ('drop_pixel_derived_correspondence',lambda x:x['correspondences'].pop())]:
        altered=copy.deepcopy(row);change(altered);r.require(altered!=row,'pixel fixture mutation is no-op '+name)
        try:v.matched_geometry(altered,matches,before,after,depth,depth,CAMERA)
        except (ValueError,KeyError,IndexError):failed.append(name)
        else:raise ValueError('invented pixel-derived geometry accepted '+name)
    return dict(passed=['actual_grayscale_FAST_BRIEF_extraction','known3pixel_translation_matches',
        'pixel_derived_nonzero_physical_translation'],feature_counts=[len(before),len(after)],
        correct_measured_matches=len(matches),known_translation_m=-3*1.5/CAMERA['fx'],
        mutations_rejected=failed,actual_recording_read=False)


@lru_cache(maxsize=1)
def archive_contract_checks():
    """Opaque synthetic archive, independently hashed and member-bound; no PNGs."""
    import io
    import tempfile
    import tarfile
    frames=[];depth_rows=[];rgb_rows=[];payloads={}
    for i in range(280):
        stamp=i/64.+1/128.;rgb_stamp=i/64.;dep=f'{stamp:.9f}.png';rgb=f'{rgb_stamp:.9f}.png'
        depth_rows.append(f'{stamp:.9f} depth/{dep}\n');rgb_rows.append(f'{rgb_stamp:.9f} rgb/{rgb}\n')
        if i>=100:
            frames.append(dict(source_index=i,depth_file='depth-'+dep,depth_timestamp=stamp,
                rgb_file='rgb-'+rgb,rgb_timestamp=rgb_stamp,rgb_source_index=i,pair_gap_seconds=1/128.,
                split='initialization' if i==100 else 'independent_recording'))
            payloads['depth/'+dep]=f'opaque depth {i}'.encode();payloads['rgb/'+rgb]=f'opaque RGB {i}'.encode()
    payloads.update({'depth.txt':''.join(depth_rows).encode(),'rgb.txt':''.join(rgb_rows).encode(),
        'groundtruth.txt':''.join(f'{i/128.:.9f} opaque opaque opaque opaque opaque opaque opaque\n' for i in range(570)).encode()})
    files=[]
    for path,content in payloads.items():
        metadata=path in ('depth.txt','rgb.txt','groundtruth.txt')
        filename=path if metadata else path.replace('/','-',1)
        role={'depth.txt':'depth_index','rgb.txt':'rgb_index','groundtruth.txt':'evaluation_only_mocap_ground_truth'}.get(path,
            'depth_frame' if path.startswith('depth/') else 'rgb_frame')
        files.append(dict(file=filename,source_path=path,bytes=len(content),sha256=r.digest(content),role=role))
    archive=dict(url=OFFICIAL_URL,final_url=FINAL_URL,root=ARCHIVE_ROOT,bytes=1,sha256='a'*64,md5='b'*32,published_checksum=None)
    manifest=dict(schema_version=1,dataset=DATASET,official_archive=archive,preregistration_sha256=DESIGN_SHA,
                  frames=frames,files=files,depth_calibration=CAMERA,calibration_source=CALIBRATION_DOCUMENT)
    metadata={name:payloads[name] for name in ('depth.txt','rgb.txt','groundtruth.txt')}
    independently_qualify(manifest,metadata)
    def pack(extra=None,replacement=None):
        output=io.BytesIO()
        with tarfile.open(fileobj=output,mode='w',format=tarfile.USTAR_FORMAT) as tar:
            for path,data in payloads.items():
                if replacement and path==replacement[0]:data=replacement[1]
                item=tarfile.TarInfo(ARCHIVE_ROOT+path);item.size=len(data);tar.addfile(item,io.BytesIO(data))
            if extra is not None:tar.addfile(extra,io.BytesIO(b'X'*extra.size) if extra.size<=1 else None)
        return gzip.compress(output.getvalue(),mtime=0)
    rejected=[]
    with tempfile.TemporaryDirectory(prefix='independent-oracle-hand-',dir='/tmp') as directory:
        path=Path(directory)/'synthetic.tgz';content=pack();path.write_bytes(content)
        def identity(data):return dict(archive,bytes=len(data),sha256=r.digest(data),md5=hashlib.md5(data).hexdigest())
        valid=copy.deepcopy(manifest);valid['official_archive']=identity(content)
        proof=archive_binding_check(path,valid)
        raw_path=Path(directory)/'synthetic-raw';raw_path.mkdir()
        for item in files:(raw_path/item['file']).write_bytes(payloads[item['source_path']])
        (raw_path/CALIBRATION_DOCUMENT['file']).write_bytes(b'synthetic documentation placeholder; name control only')
        raw_inventory_check(valid,raw_path)
        for name,is_directory in [('extra_raw_file',False),('extra_raw_directory',True)]:
            extra=raw_path/'unexpected';extra.mkdir() if is_directory else extra.write_bytes(b'extra')
            try:raw_inventory_check(valid,raw_path)
            except ValueError:rejected.append(name)
            else:raise ValueError('unexpected on-disk raw inventory accepted '+name)
            extra.rmdir() if is_directory else extra.unlink()
        linked=raw_path/'unexpected';linked.symlink_to(raw_path/files[0]['file'])
        try:raw_inventory_check(valid,raw_path)
        except ValueError:rejected.append('extra_raw_symlink')
        else:raise ValueError('unexpected raw link accepted')
        linked.unlink()
        r.require(proof['selected_member_hashes_verified']==363,'archive positive drops fixed selection or metadata')
        for name,mutation in [('wrong_archive_SHA',lambda m:m['official_archive'].__setitem__('sha256','0'*64)),
            ('wrong_archive_MD5',lambda m:m['official_archive'].__setitem__('md5','0'*32)),
            ('wrong_archive_size',lambda m:m['official_archive'].__setitem__('bytes',m['official_archive']['bytes']+1)),
            ('unrelated_selected_payload_hash',lambda m:m['files'][0].__setitem__('sha256','0'*64)),
            ('invent_source_member',lambda m:m['files'][0].__setitem__('source_path','depth/invented.png')),
            ('invent_publisher_checksum',lambda m:m['official_archive'].__setitem__('published_checksum','signed')),
            ('different_official_source',lambda m:m['official_archive'].__setitem__('url','https://unverified.invalid/a')),
            ('changed_prospective_design',lambda m:m.__setitem__('preregistration_sha256','0'*64))]:
            changed=copy.deepcopy(valid);mutation(changed);r.require(changed!=valid,'archive mutation is no-op '+name)
            try:archive_binding_check(path,changed)
            except (ValueError,KeyError):rejected.append(name)
            else:raise ValueError('invented official archive provenance accepted '+name)
        fixtures=[]
        for name,member_name,kind in [('traversal_member',ARCHIVE_ROOT+'../escape',tarfile.REGTYPE),
            ('foreign_root_member','foreign/data',tarfile.REGTYPE),
            ('duplicate_member',ARCHIVE_ROOT+'groundtruth.txt',tarfile.REGTYPE),
            ('link_member',ARCHIVE_ROOT+'link',tarfile.SYMTYPE),
            ('device_member',ARCHIVE_ROOT+'device',tarfile.CHRTYPE),
            ('extended_header',ARCHIVE_ROOT+'extended',tarfile.XHDTYPE)]:
            member=tarfile.TarInfo(member_name);member.type=kind;member.size=0;member.linkname='untrusted'
            fixtures.append((name,pack(extra=member)))
        raw_header=tarfile.TarInfo(ARCHIVE_ROOT+'oversized');raw_header.size=4194305
        fixtures.append(('oversized_member_header_before_payload',gzip.compress(raw_header.tobuf(format=tarfile.USTAR_FORMAT)+bytes(1024),mtime=0)))
        bad_header=bytearray(gzip.decompress(content));bad_header[0]^=1
        fixtures.append(('bad_tar_header_checksum',gzip.compress(bad_header,mtime=0)))
        fixtures.extend([('selected_payload_not_bound',pack(replacement=('groundtruth.txt',b'unrelated full source metadata'))),
                         ('truncated_compressed_archive',content[:-8])])
        for name,data in fixtures:
            path.write_bytes(data);changed=copy.deepcopy(valid);changed['official_archive']=identity(data)
            r.require(data!=content,'hostile archive fixture is no-op '+name)
            try:archive_binding_check(path,changed)
            except (ValueError,KeyError,EOFError,OSError):rejected.append(name)
            else:raise ValueError('hostile synthetic archive accepted '+name)
        path.write_bytes(content)
        with path.open('r+b') as stream:stream.truncate(536870913)
        try:archive_binding_check(path,valid)
        except ValueError:rejected.append('actual_compressed_archive_cap_before_hashing')
        else:raise ValueError('oversized compressed archive accepted')
        bounded=BoundedArchiveReader(io.BytesIO(b'XX'));bounded.count=2147483648
        try:bounded.read(1)
        except ValueError:rejected.append('decompressed_header_payload_padding_aggregate_cap')
        else:raise ValueError('oversized decompressed stream accepted')
    return dict(passed=['compressed_SHA256_MD5_size_verified','all363_selected_members_bound_to_actual_archive',
        'whole_source_opaque_metadata_qualification','tar_headers_padding_and_payload_bytes_counted','exact_raw_inventory_with_permitted_documentation_sidecar'],
        mutations_rejected=rejected,actual_recording_read=False,synthetic_archive_only=True,
        computed_archive_identity_is_publisher_signature=False)


def source_binding_contract_checks():
    """Positive metadata-only synthetic freeze before all source mutations.

    These are test-generated bindings, not an archived operational trial or an
    assertion that the unavailable official compressed archive was acquired.
    """
    frames=[];depth=[];rgb=[];files=[]
    for i in range(280):
        stamp=i/64.+1/128.;rgb_stamp=i/64.;dep=f'{stamp:.9f}.png';im=f'{rgb_stamp:.9f}.png'
        depth.append(f'{stamp:.9f} depth/{dep}\n');rgb.append(f'{rgb_stamp:.9f} rgb/{im}\n')
        if i>=100:
            frames.append(dict(source_index=i,depth_file='depth-'+dep,depth_timestamp=stamp,
                rgb_file='rgb-'+im,rgb_timestamp=rgb_stamp,rgb_source_index=i,pair_gap_seconds=1/128.,
                split='initialization' if i==100 else 'independent_recording'))
            for kind,name in [('depth',dep),('rgb',im)]:
                content=f'opaque {kind} {i}'.encode()
                files.append(dict(file=kind+'-'+name,source_path=kind+'/'+name,bytes=len(content),
                    sha256=r.digest(content),role=kind+'_frame'))
    metadata={'depth.txt':''.join(depth).encode(),'rgb.txt':''.join(rgb).encode(),
        'groundtruth.txt':''.join(f'{i/128.:.9f} opaque opaque opaque opaque opaque opaque opaque\n' for i in range(570)).encode()}
    for name,content in metadata.items():
        files.append(dict(file=name,source_path=name,bytes=len(content),sha256=r.digest(content),
            role={'depth.txt':'depth_index','rgb.txt':'rgb_index','groundtruth.txt':'evaluation_only_mocap_ground_truth'}[name]))
    archive=dict(url=OFFICIAL_URL,final_url=FINAL_URL,root=ARCHIVE_ROOT,bytes=1,
                 sha256='a'*64,md5='b'*32,published_checksum=None)
    manifest=dict(schema_version=1,dataset=DATASET,official_archive=archive,preregistration_sha256=DESIGN_SHA,
        depth_calibration=CAMERA,calibration_source=CALIBRATION_DOCUMENT,frames=frames,files=files)
    manifest_raw=json.dumps(manifest,sort_keys=True).encode()
    freeze=dict(schema_version=1,protocol_version=1,algorithm=ALGORITHM,dataset=DATASET,
        manifest_sha256=r.digest(manifest_raw),preregistration_sha256=DESIGN_SHA,
        preregistration_source_sha256=DESIGN_SHA,official_archive=archive,regression_requested=False,
        kind='preregistered_independent_recording',frames=frames,resources=RESOURCE_POLICY,
        qualification_order=QUALIFICATION_ORDER,freeze_preparation=FREEZE_PREPARATION,**TEMPORAL_POLICY)
    freeze.update({key:r.digest(r.bounded(ROOT/path)) for key,path in SOURCES.items()})
    policies=dict(depth_calibration=CAMERA,calibration_source=CALIBRATION_DOCUMENT,
        preprocessing=PREPROCESSING,feature_policy=FEATURE_POLICY,tracking_policy=TRACKING_POLICY,
        registration_config=v.CONFIG,refinement_config=g.REFINEMENT_CONFIG,refinement_policy=g.REFINEMENT_POLICY,
        evaluation_label_policy=g.LABEL_POLICY,ground_truth_interpolation=GT_POLICY,
        accuracy_gates=dict(translation_m=.1,rotation_rad=.1),json_metadata_audit=v.JSON_METADATA_AUDIT)
    freeze.update(policies)
    for key in ('depth_calibration','calibration_source','preprocessing','feature_policy','tracking_policy',
                'registration_config','refinement_config','refinement_policy','evaluation_label_policy'):
        freeze[key+'_sha256']=g.refinement_config_digest(freeze[key]) if key=='refinement_config' else r.canonical(freeze[key])
    measured=independently_qualify(manifest,metadata)
    proof=dict(schema_version=1,dataset=DATASET,manifest_sha256=r.digest(manifest_raw),official_archive=archive,
        preregistration_sha256=DESIGN_SHA,window=dict(first_depth_index=100,last_depth_index=279,frames=180,updates=179),
        metadata_files={name:dict(bytes=len(content),sha256=r.digest(content)) for name,content in metadata.items()},
        timestamps=dict(depth_index_rows=measured['depth_rows'],rgb_index_rows=measured['rgb_rows'],
            ground_truth_rows=measured['ground_truth_rows'],strict_depth_order=True,strict_rgb_order=True,
            strict_ground_truth_order=True,all_frame_brackets_valid=True,max_ground_truth_bracket_s=.02,max_pair_gap_s=.02,
            maximum_observed_ground_truth_bracket_us=measured['maximum_observed_ground_truth_bracket_us'],
            maximum_pair_gap_us=measured['maximum_pair_gap_us'],unique_rgb_acquisitions=180,duplicate_rgb_associations=0,
            summary_time_unit=measured['summary_time_unit']),passed=True,pixels_read=False,image_headers_read=False,
        ground_truth_pose_values_parsed=False,features_or_fits_run=False,
        helper_sha256=freeze['qualification_source_sha256'],acquisition_helper_sha256=freeze['acquisition_source_sha256'])
    proof_raw=json.dumps(proof,sort_keys=True).encode();freeze.update(qualification=proof,qualification_sha256=r.digest(proof_raw))
    verify_sources(freeze,manifest_raw,proof_raw,ROOT)
    qualification_check(freeze,manifest_raw,proof,metadata)
    rejected=qualification_mutations(freeze,manifest_raw,proof_raw,metadata,ROOT)
    for name,mutate in [('source_prospective_design',lambda x:x.__setitem__('preregistration_source_sha256','0'*64)),
        ('invent_declared_prospective_design',lambda x:x.__setitem__('preregistration_sha256','0'*64)),
        ('hide_fresh_reporting_role',lambda x:x.__setitem__('kind','calibration_regression')),
        ('hide_prior_opaque_archive_exposure',lambda x:x.__setitem__('qualification_order','before any image bytes')),
        ('misrepresent_metadata_preparation_scope',lambda x:x.__setitem__('freeze_preparation','no archive bytes ever read')),
        ('claim_generalization_without_record',lambda x:x.__setitem__('evidence_scope','validated independent physical environment'))]:
        changed=copy.deepcopy(freeze);mutate(changed);r.require(changed!=freeze,'binding mutation is no-op '+name)
        try:verify_sources(changed,manifest_raw,proof_raw,ROOT)
        except (ValueError,KeyError):rejected.append(name)
        else:raise ValueError('invented prospective binding accepted '+name)
    return dict(passed=['synthetic_complete180_source_bound_metadata_freeze',
        'all21_code_and_exact_prospective_design_bindings','independent_opaque_whole_source_qualification'],
        mutations_rejected=rejected,formal_operational_source_freeze_verified=False,
        official_archive_acquired=False,actual_recording_read=False,synthetic_freeze_only=True)


def synthetic_sensor_contract_checks(path):
    """Audit cfg(test) Rust-produced PNG evidence, never source qualification."""
    packet_raw=r.bounded(path,67108864);packet=json.loads(packet_raw)
    r.require(set(packet)=={'kind','manifest','report','synthetic_motion','synthetic_raw_bytes','production_source_qualification_exercised'}
        and packet['kind']=='cfg_test_synthetic_sensor_fixture'
        and packet['production_source_qualification_exercised'] is False,'wrong synthetic Rust packet scope')
    manifest=packet['manifest'];report=packet['report'];frames=manifest['frames']
    r.require(manifest['dataset']=='synthetic-private-pinhole-plane'
        and manifest['official_archive'] is None and report['official_archive'] is None
        and report['qualification_verified'] is False,'synthetic packet invented official identity/qualification')
    exact_json(manifest['depth_calibration'],CAMERA,'synthetic pinhole profile changed')
    exact_json(packet['synthetic_motion'],dict(frames=180,updates=179,plane_depth_m=1.5,
        image_translation_per_frame_px=[1.,0.],root_translation_per_frame_m=[-1.5/CAMERA['fx'],0.,0.]),
        'synthetic known nonzero motion changed')
    r.require(len(frames)==180 and [f['source_index'] for f in frames]==list(range(100,280)),
              'synthetic sensor fixture dropped continuous observations')
    r.require(all(frame['depth_timestamp']==100.+i*.03125 and frame['rgb_timestamp']==frame['depth_timestamp']
        and frame['split']==('initialization' if i==0 else 'independent_recording') for i,frame in enumerate(frames)),
        'changed known synthetic acquisition timestamps/roles')
    raw=path.parent/'raw';expected_names={'groundtruth.txt'}|{f[key] for f in frames for key in ('depth_file','rgb_file')}
    r.require({p.name for p in raw.iterdir()}==expected_names and all(p.is_file() and not p.is_symlink() for p in raw.iterdir()),
              'unexpected synthetic raw inventory')
    features=[];depths=[];initial=None;raw_sha=hashlib.sha256();raw_total=0
    for i,frame in enumerate(frames):
        for key in ('depth_file','rgb_file'):
            r.require(Path(frame[key]).name==frame[key],'unsafe synthetic input path')
        image_raw=r.bounded(raw/frame['rgb_file']);depth_raw=r.bounded(raw/frame['depth_file'])
        raw_total+=len(image_raw)+len(depth_raw)
        r.require(raw_total<=134217728,'synthetic raw inventory exceeds unchanged128MiB bound')
        raw_sha.update(image_raw);raw_sha.update(depth_raw)
        gray=v.png_image(image_raw,False);depth=v.png_image(depth_raw,True)
        if initial is None:initial=gray
        r.require(g.np.array_equal(gray,g.np.roll(initial,i,axis=1)),'actual Rust PNG raster contradicts known pixel translation')
        r.require(g.np.all(depth==7500),'actual Rust measured depths contradict1.5m plane')
        features.append(v.features(gray));depths.append(depth)
    labels_raw=r.bounded(raw/'groundtruth.txt',4194304)
    r.require(raw_total+len(labels_raw)<=134217728,'synthetic raw inventory exceeds unchanged128MiB bound')
    r.require(type(packet['synthetic_raw_bytes']) is int and packet['synthetic_raw_bytes']==raw_total+len(labels_raw),
              'invented synthetic raw byte accounting')
    labels,failure=g.evaluation_labels(labels_raw)
    r.require(failure is None and len(labels)==180,'invalid synthetic reference source')
    for i,(stamp,translation,quaternion) in enumerate(labels):
        r.require(stamp==frames[i]['depth_timestamp'],'synthetic labels changed measured clock')
        v.vector_check(translation,[-i*1.5/CAMERA['fx'],0.,0.],'synthetic labels differ from analytic geometry')
        r.vector(quaternion,4,'synthetic label quaternion dimension/type')
        r.require(quaternion==[1.,0.,0.,0.],'synthetic label orientation differs from known motion')
    records=continuous_audit(report,manifest,features,depths,labels,None)
    r.require(report['summary']['accepted_updates']==report['summary']['accurate_root_updates']==179,
              'actual synthetic Rust motion failed all179 physical gates')
    rejected=[]
    targets=[('alter_actual_rust_root',lambda x:x['frames'][179]['root_estimate']['translation_m'].__setitem__(0,0.)),
        ('reset_actual_rust_origin',lambda x:x['frames'][36].__setitem__('initialized',True)),
        ('drop_actual_rust_tail',lambda x:x['frames'].pop()),
        ('alter_actual_rust_inliers',lambda x:x['frames'][1]['fit']['inlier_indices'].pop()),
        ('alter_actual_rust_feature',lambda x:x['frames'][1]['features'][0].__setitem__('x',0.)),
        ('alter_actual_rust_pixel_refinement',lambda x:x['frames'][1]['refinement'].__setitem__('final_huber_cost',999.)),
        ('alter_actual_rust_work',lambda x:x['frames'][1]['fit'].__setitem__('point_checks',0)),
        ('alter_actual_rust_accepted_clock',lambda x:x['frames'][1].__setitem__('last_accepted_stamp_after',0.)),
        ('alter_actual_rust_missing_reference',lambda x:x['frames'][179]['root_accuracy'].__setitem__('reference_valid',False))]
    for name,change in targets:
        altered=copy.deepcopy(report);change(altered);r.require(altered!=report,'Rust packet mutation is no-op '+name)
        try:continuous_audit(altered,manifest,features,depths,labels,None)
        except (ValueError,KeyError,IndexError):rejected.append(name)
        else:raise ValueError('invented actual Rust sensor witness accepted '+name)
    try:audit(report,manifest,features,depths,labels,None)
    except ValueError:rejected.append('unqualified_synthetic_packet_rejected_by_production_audit')
    else:raise ValueError('synthetic packet bypassed production qualification')
    return dict(passed=['actual180_Rust_PNG_rasters_match_known_nonzero_motion',
        'actual_FAST_BRIEF_depth_SVD_refinement_state_independently_reconstructed',
        'all179_analytic_root_scores_pass_same_continuous_auditor'],summary=report['summary'],
        mutations_rejected=rejected,fixture_sha256=r.digest(packet_raw),ordered_synthetic_PNG_digest=raw_sha.hexdigest(),
        synthetic_raw_bytes=raw_total+len(labels_raw),
        synthetic_only=True,production_source_qualification_exercised=False,official_archive_acquired=False,
        actual_recording_read=False,**g.maximum_scored_errors(records))


def main():
    if '--self-test' in sys.argv[1:]:
        parser=argparse.ArgumentParser(description='Synthetic independent-estimator controls only')
        parser.add_argument('--self-test',action='store_true');parser.add_argument('--output',type=Path)
        parser.add_argument('--synthetic-packet',type=Path)
        args=parser.parse_args()
        result=dict(schema_version=1,passed_integrity=True,continuous_contract_checks=continuous_contract_checks(),
            pixel_feature_contract_checks=pixel_feature_contract_checks(),archive_contract_checks=archive_contract_checks(),
            source_binding_contract_checks=source_binding_contract_checks(),
            metadata_contract_checks=metadata_contract_checks(),mathematical_contract_checks=g.mathematical_contract_checks(),
            missing_reference_score_mutations_rejected=k.missing_reference_contract_checks(),
            cache_contract_checks=cache_contract_checks(),actual_recording_read=False)
        if args.synthetic_packet:result['actual_rust_synthetic_sensor_contract_checks']=synthetic_sensor_contract_checks(args.synthetic_packet)
        if args.output:write_result(args.output,result)
        print(json.dumps(result));return
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'raw', 'freeze', 'qualification', 'archive', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--source-snapshot', type=Path)
    parser.add_argument('--preregister-only', action='store_true')
    args = parser.parse_args()
    manifest_raw = r.bounded(args.manifest, 512*1024)
    freeze_raw = r.bounded(args.freeze, 512*1024)
    qualification_raw = r.bounded(args.qualification, 512*1024)
    freeze = json.loads(freeze_raw)
    source_root = args.source_snapshot or ROOT
    manifest, qualification = verify_sources(freeze, manifest_raw, qualification_raw, source_root)
    archive_proof=archive_binding_check(args.archive,manifest)
    metadata = metadata_inputs(manifest, args.raw)
    measured = qualification_check(freeze, manifest_raw, qualification, metadata)
    common = dict(schema_version=1, passed_integrity=True, source_freeze_verified=True,
        frozen_source_count=len(SOURCES),preregistration_source_sha256=DESIGN_SHA,archive_binding=archive_proof, dataset=DATASET, kind=freeze['kind'],
        manifest_sha256=r.digest(manifest_raw), freeze_sha256=r.digest(freeze_raw),
        qualification_sha256=r.digest(qualification_raw), qualification_independently_recomputed=True,
        qualification_method='whole-source row arity and timestamp columns only; nearest RGB and strict brackets',
        temporal_timestamp_summary=measured, running_checker_sha256=r.digest(r.bounded(Path(__file__))),
        frozen_checker_sha256=freeze['independent_checker_sha256'],
        archived_source_snapshot=args.source_snapshot is not None)
    source_mutations = qualification_mutations(freeze,manifest_raw,qualification_raw,metadata,source_root)
    metadata_contracts = metadata_contract_checks()
    cache_contracts = cache_contract_checks()
    common.update(frozen_provenance_and_qualification_mutations_rejected=source_mutations,
                  metadata_contract_checks=metadata_contracts,
                  exact_solver_cache_contract_checks=cache_contracts,
                  continuous_contract_checks=continuous_contract_checks(),synthetic_controls_executed=True,
                  pixel_feature_contract_checks=pixel_feature_contract_checks(),archive_contract_checks=archive_contract_checks())
    if args.preregister_only:
        r.require(args.report is None, 'metadata preregistration must not consume fit reports')
        common.update(metadata_only=True,
                      evidence_scope=TEMPORAL_POLICY['evidence_scope'],
                      reference_source_previously_evaluated=False, metadata_files_read=['depth.txt','rgb.txt','groundtruth.txt'],
                      compressed_archive_bytes_read=True,archive_payload_decompressed=True,
                      opaque_image_payloads_hashed=True,selected_raw_png_bytes_opened=False,
                      image_bytes_read=True,image_headers_read=False,pixels_read=False,
                      actual_recorded_image_pixels_read=False,synthetic_image_controls_executed=True,
                      ground_truth_pose_values_parsed=False,features_or_fits_run=False,
                      negative_flags_scope='actual recording pixel/pose/fit interpretation; synthetic controls run separately; compressed archive and opaque payload exposure disclosed')
        write_result(args.output, common)
        print(json.dumps(common))
        return
    r.require(args.report is not None, 'fit audit requires --report')
    report_raw = r.bounded(args.report, 64*1024*1024)
    report = json.loads(report_raw)
    exact_json(report['freeze'], freeze, 'external freeze mismatch')
    r.require(report['freeze_sha256'] == r.digest(freeze_raw)
              and report['manifest_sha256'] == r.digest(manifest_raw), 'external source/freeze mismatch')
    raw = verified_inputs(manifest, args.raw, report)
    features, depths, cache = [], [], {}
    for frame in manifest['frames']:
        filename = frame['rgb_file']
        if filename not in cache:
            cache[filename] = v.features(v.png_image(raw[filename], False))
        features.append(cache[filename])
        depths.append(v.png_image(raw[frame['depth_file']], True))
    # Reference pose columns are parsed only after the frozen operational report
    # and the independent measured-feature/refinement reconstruction exist.
    gt, label_failure = g.evaluation_labels(raw['groundtruth.txt'])
    records = audit(report, manifest, features, depths, gt, label_failure)
    old_mutations = mutation_checks(report, manifest, features, depths, gt, label_failure)
    temporal_mutations = temporal_report_mutations(report,manifest,features,depths,gt,label_failure)
    common.update(summary=report['summary'], frames=records, report_sha256=r.digest(report_raw),
        raw_sha256_verified=True, evaluation_label_failure=label_failure,
        physical_pose_independently_scored=label_failure is None,
        unscorable_updates=report['summary']['updates']-report['summary']['reference_valid_updates'],
        mutations_rejected=old_mutations+temporal_mutations, mathematical_contract_checks=g.mathematical_contract_checks(),
        missing_reference_score_mutations_rejected=k.missing_reference_contract_checks(),
        pixel_features_descriptors_and_associations_independently_reconstructed=True,
        rigid_fit_independently_replayed='Kabsch SVD vs operational Horn quaternion/Jacobi',
        pixel_refinement_independently_replayed='central numerical Jacobian and SVD vs analytic Jacobian and Cholesky',
        fixed_original_inlier_support_independently_checked=True,
        monotonic_search_and_terminal_step_accounting_independently_checked=True,
        reference_and_clock_state_independently_reconstructed=True,
        evidence_role='viewed regression' if freeze['regression_requested'] else 'preregistered independent recording; prospectively fixed window and unchanged estimator',
        evidence_scope=TEMPORAL_POLICY['evidence_scope'],
        all_179_updates_audited_continuously=True,
        maximum_initializations=1,chunk_resets=False,lost_recovery=False,
        calibrated_covariance_or_root_confidence_claim=False,
        oracle_dependencies=dict(numpy=g.np.__version__, pillow=v.Image.__version__),
        continuous_audit_method='static copy of original state/physical audit with full180frame179update length; same pinned feature/3D/refinement math and no frame chunk resets',
        **g.maximum_scored_errors(records))
    write_result(args.output, common)
    print(json.dumps(report['summary']))


if __name__ == '__main__':
    main()
