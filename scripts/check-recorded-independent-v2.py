#!/usr/bin/env python3
"""Additive calibration-contract repair for the immutable desk2 first-trial auditor.

The original checker and its 21 frozen sources remain mandatory. Only input and
camera-document validation is replaced locally; no imported globals or motion
mathematics are changed. An integrity pass does not mean operational success.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import tempfile

ROOT = Path(__file__).resolve().parent.parent
ORIGINAL_SHA = '0a4696936103eda71824899607d6799c0f529deb036614bbab4053838d2e7af8'
ORIGINAL_PATH = Path(__file__).with_name('check-recorded-independent.py')
if hashlib.sha256(ORIGINAL_PATH.read_bytes()).hexdigest() != ORIGINAL_SHA:
    raise ValueError('original independent checker must remain immutable')
_spec = importlib.util.spec_from_file_location('immutable_independent_v1', ORIGINAL_PATH)
i = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(i)
r, v, g, k = i.r, i.v, i.g, i.k
REPAIR_REASON = ('Original v1 invoked the legacy visual calibration validator, which requires '
    'source_rgb_distortion; the prospectively frozen independent manifest intentionally has '
    'the exact eight-field FR1 projection/depth profile. V2 validates that contract and the '
    'pinned camera documentation without adding distortion fields or applying undistortion.')


def calibration_check(report, manifest, raw_path):
    i.exact_json(manifest['depth_calibration'], i.CAMERA, 'changed exact eight-field FR1 profile')
    i.exact_json(manifest['calibration_source'], i.CALIBRATION_DOCUMENT, 'changed pinned calibration descriptor')
    i.exact_json(report['freeze']['depth_calibration'], manifest['depth_calibration'], 'report/freeze calibration mismatch')
    for where in (report, report['freeze']):
        i.exact_json(where['calibration_source'], manifest['calibration_source'], 'report/freeze calibration source mismatch')
    r.require(report['calibration_sha256_verified'] is True, 'report omits verified calibration SHA256')
    source = manifest['calibration_source']
    path = raw_path / source['file']
    r.require(path.is_file() and not path.is_symlink(), 'unsafe calibration document')
    content = r.bounded(path, source['bytes'])
    r.require(len(content) == source['bytes'] and r.digest(content) == source['sha256'],
              'pinned camera calibration size/SHA256 mismatch')
    values = {}
    for line in content.decode('utf8').splitlines():
        match = re.fullmatch(r'\s*(Camera\.(?:fx|fy|cx|cy|width|height)|DepthMapFactor)\s*:\s*([-+0-9.eE]+)\s*(?:#.*)?', line)
        if match:
            r.require(match[1] not in values, 'duplicate camera parameter')
            value = float(match[2])
            r.require(math.isfinite(value), 'nonfinite camera parameter')
            values[match[1]] = value
    expected = {'Camera.'+key: i.CAMERA[key] for key in ('fx','fy','cx','cy','width','height')}
    expected['DepthMapFactor'] = i.CAMERA['units_per_metre']
    i.exact_json(values, expected, 'published YAML projection/dimensions/depth scale mismatch')
    r.require(type(manifest['depth_calibration']['invalid_depth']) is int
              and manifest['depth_calibration']['invalid_depth'] == 0, 'invalid depth sentinel changed')
    return dict(passed=True, exact_eight_field_profile=True, document_sha256=source['sha256'],
                document_bytes=len(content), yaml_projection_and_depth_scale_verified=True,
                invalid_depth=0, distortion_documentation_only=True,
                undistortion_applied=False, imported_checker_globals_modified=False)


def input_inventory_check(manifest, raw_path, report):
    i.manifest_check(manifest)
    i.raw_inventory_check(manifest, raw_path)
    i.exact_json(report['files'], manifest['files'], 'changed report input inventory')


def verified_inputs(manifest, raw_path, report):
    input_inventory_check(manifest, raw_path, report)
    calibration_check(report, manifest, raw_path)
    raw, total = {}, 0
    for item in manifest['files']:
        content = r.bounded(raw_path/item['file'], 4*1024*1024)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'],
                  'measured RGB/depth/mocap byte mismatch')
        raw[item['file']] = content
        total += len(content)
    r.require(total+manifest['calibration_source']['bytes'] <= 128*1024*1024,
              'raw acquisition total bound')
    return raw


def calibration_contract_checks(report, manifest, raw_path):
    baseline = calibration_check(report, manifest, raw_path)
    input_inventory_check(manifest, raw_path, report)
    rejected = []
    changes = [
        ('wrong_pinned_document_hash', 'manifest', lambda x: x['calibration_source'].__setitem__('sha256','0'*64)),
        ('wrong_pinned_document_size', 'manifest', lambda x: x['calibration_source'].__setitem__('bytes',1614)),
        ('wrong_manifest_fx', 'manifest', lambda x: x['depth_calibration'].__setitem__('fx',525.)),
        ('wrong_invalid_depth_sentinel', 'manifest', lambda x: x['depth_calibration'].__setitem__('invalid_depth',False)),
        ('wrong_freeze_fx', 'report', lambda x: x['freeze']['depth_calibration'].__setitem__('fx',525.)),
        ('unverified_report_flag', 'report', lambda x: x.__setitem__('calibration_sha256_verified',False)),
        ('wrong_report_document', 'report', lambda x: x['calibration_source'].__setitem__('sha256','0'*64)),
        ('invent_distortion_field', 'manifest', lambda x: x['depth_calibration'].__setitem__('source_rgb_distortion',{})),
        ('changed_report_inventory', 'report', lambda x: x['files'].pop()),
        ('changed_manifest_inventory', 'manifest', lambda x: x['files'].pop()),
    ]
    for name, kind, mutate in changes:
        changed = copy.deepcopy(report if kind == 'report' else manifest)
        mutate(changed)
        original = report if kind == 'report' else manifest
        r.require(r.canonical(changed) != r.canonical(original), 'calibration mutation is no-op '+name)
        m, p = (manifest,changed) if kind == 'report' else (changed,report)
        try:
            if 'inventory' in name:
                input_inventory_check(m,raw_path,p)
            else:
                calibration_check(p,m,raw_path)
        except (ValueError, KeyError):
            rejected.append(name)
        else:
            raise ValueError('calibration/inventory corruption accepted '+name)
    with tempfile.TemporaryDirectory(prefix='independent-v2-calibration-') as temporary:
        temporary = Path(temporary)
        content = (raw_path/i.CALIBRATION_DOCUMENT['file']).read_bytes()
        changed = bytes([content[0]^1])+content[1:]
        r.require(r.digest(changed) != r.digest(content), 'document mutation is no-op')
        (temporary/i.CALIBRATION_DOCUMENT['file']).write_bytes(changed)
        try:
            calibration_check(report,manifest,temporary)
        except ValueError:
            rejected.append('wrong_actual_document_bytes_same_size')
        else:
            raise ValueError('corrupt actual calibration document accepted')
        for item in manifest['files']:
            (temporary/item['file']).touch()
        (temporary/'unexpected-extra.txt').touch()
        try:
            input_inventory_check(manifest,temporary,report)
        except ValueError:
            rejected.append('extra_actual_raw_file')
        else:
            raise ValueError('extra actual raw file accepted')
    return dict(baseline=baseline, actual_calibration_document_checked=True,
                mutations_rejected=rejected, operational_report_or_source_modified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest','raw','freeze','qualification','archive','output','source-snapshot','report'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--calibration-self-test', action='store_true')
    args = parser.parse_args()
    r.require(not args.output.exists(), 'v2 output exists; preserve previous evidence')
    manifest_raw = r.bounded(args.manifest,512*1024)
    freeze_raw = r.bounded(args.freeze,512*1024)
    qualification_raw = r.bounded(args.qualification,512*1024)
    freeze = json.loads(freeze_raw)
    r.require(freeze['independent_checker_sha256'] == ORIGINAL_SHA, 'changed original frozen checker binding')
    manifest, qualification = i.verify_sources(freeze,manifest_raw,qualification_raw,args.source_snapshot)
    report_raw = r.bounded(args.report,64*1024*1024)
    report = json.loads(report_raw)
    i.exact_json(report['freeze'],freeze,'external freeze mismatch')
    r.require(report['freeze_sha256'] == r.digest(freeze_raw)
              and report['manifest_sha256'] == r.digest(manifest_raw), 'external source/freeze mismatch')
    controls = calibration_contract_checks(report,manifest,args.raw)
    common = dict(schema_version=1, running_checker_version=2,
        running_checker_sha256=r.digest(Path(__file__).read_bytes()), frozen_checker_sha256=ORIGINAL_SHA,
        original_checker_sha256=ORIGINAL_SHA, auditor_repair_reason=REPAIR_REASON,
        auditor_only_repair=True, estimator_or_math_changed=False, calibration_contract_checks=controls,
        source_freeze_verified=True, frozen_source_count=len(i.SOURCES), archived_source_snapshot=True,
        manifest_sha256=r.digest(manifest_raw), freeze_sha256=r.digest(freeze_raw),
        qualification_sha256=r.digest(qualification_raw), report_sha256=r.digest(report_raw),
        dataset=i.DATASET, kind=freeze['kind'], preregistration_source_sha256=i.DESIGN_SHA)
    if args.calibration_self_test:
        common.update(passed_integrity=True, calibration_only=True, motion_audit_performed=False)
        i.write_result(args.output,common)
        print(json.dumps(common))
        return
    archive_proof = i.archive_binding_check(args.archive,manifest)
    metadata = i.metadata_inputs(manifest,args.raw)
    measured = i.qualification_check(freeze,manifest_raw,qualification,metadata)
    source_mutations = i.qualification_mutations(freeze,manifest_raw,qualification_raw,metadata,args.source_snapshot)
    common.update(archive_binding=archive_proof, qualification_independently_recomputed=True,
        qualification_method='whole-source row arity and timestamp columns only; nearest RGB and strict brackets',
        temporal_timestamp_summary=measured,
        frozen_provenance_and_qualification_mutations_rejected=source_mutations,
        metadata_contract_checks=i.metadata_contract_checks(),
        exact_solver_cache_contract_checks=i.cache_contract_checks(),
        continuous_contract_checks=i.continuous_contract_checks(), synthetic_controls_executed=True,
        pixel_feature_contract_checks=i.pixel_feature_contract_checks(),
        archive_contract_checks=i.archive_contract_checks())
    raw = verified_inputs(manifest,args.raw,report)
    features, depths, cache = [], [], {}
    for frame in manifest['frames']:
        filename = frame['rgb_file']
        if filename not in cache:
            cache[filename] = v.features(v.png_image(raw[filename],False))
        features.append(cache[filename])
        depths.append(v.png_image(raw[frame['depth_file']],True))
    gt, label_failure = g.evaluation_labels(raw['groundtruth.txt'])
    records = i.audit(report,manifest,features,depths,gt,label_failure)
    old_mutations = i.mutation_checks(report,manifest,features,depths,gt,label_failure)
    temporal_mutations = i.temporal_report_mutations(report,manifest,features,depths,gt,label_failure)
    common.update(passed_integrity=True, summary=report['summary'], frames=records,
        raw_sha256_verified=True, evaluation_label_failure=label_failure,
        physical_pose_independently_scored=label_failure is None,
        unscorable_updates=report['summary']['updates']-report['summary']['reference_valid_updates'],
        mutations_rejected=old_mutations+temporal_mutations,
        mathematical_contract_checks=g.mathematical_contract_checks(),
        missing_reference_score_mutations_rejected=k.missing_reference_contract_checks(),
        pixel_features_descriptors_and_associations_independently_reconstructed=True,
        rigid_fit_independently_replayed='Kabsch SVD vs operational Horn quaternion/Jacobi',
        pixel_refinement_independently_replayed='central numerical Jacobian and SVD vs analytic Jacobian and Cholesky',
        fixed_original_inlier_support_independently_checked=True,
        monotonic_search_and_terminal_step_accounting_independently_checked=True,
        reference_and_clock_state_independently_reconstructed=True,
        evidence_role='viewed regression' if freeze['regression_requested'] else 'preregistered independent recording; prospectively fixed window and unchanged estimator',
        evidence_scope=i.TEMPORAL_POLICY['evidence_scope'], all_179_updates_audited_continuously=True,
        maximum_initializations=1, chunk_resets=False, lost_recovery=False,
        calibrated_covariance_or_root_confidence_claim=False,
        oracle_dependencies=dict(numpy=g.np.__version__,pillow=v.Image.__version__),
        continuous_audit_method='immutable original independent v1 state/physical audit and feature/3D/refinement math; local eight-field calibration validation repair only',
        **g.maximum_scored_errors(records))
    i.write_result(args.output,common)
    print(json.dumps(report['summary']))


if __name__ == '__main__':
    main()
