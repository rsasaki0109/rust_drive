#!/usr/bin/env python3
"""Reproduce viewed continuous multiscale comparisons without replacing first evidence.

Suite exit zero denotes exact source/protocol/numerical reproduction and an
independent audit, even when the preserved physical evaluator exits one. No
fresh holdout, driving fusion or uncertainty-calibration claim is made.
"""
import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent
ASSETS = ROOT/'assets/multiscale-temporal-v1'
DESIGN_SHA = 'd0b6a8f21ded03ce8125e30e47440d223c77567d0f2e41b6d9031c32e1ea34df'
DATASETS = ('tum-fr1-room-temporal', 'tum-fr1-desk2-independent')
PREFIXES = {'tum-fr1-room-temporal':'room', 'tum-fr1-desk2-independent':'desk2'}
MAX_JSON_BYTES = 64*1024*1024
CPU_TIMING_FIELDS = frozenset({'cpu_wall_seconds'})
AUDITOR_SHA = '231d6c5fd9a56d9925d7a52ad5a1affb36170275914fb275f199fcf385040e4e'
SOURCES = {
    'temporal_binary':'integrations/rgbd/src/bin/rustdriving-rgbd-multiscale-temporal.rs',
    'temporal_support':'integrations/rgbd/src/multiscale_temporal_support.rs',
    'design':'assets/multiscale-temporal-v1/design.json',
    'pair_binary':'integrations/rgbd/src/bin/rustdriving-rgbd-multiscale-pairs.rs',
    'multiscale_features':'crates/perception/src/multiscale_features.rs',
    'image_features':'crates/perception/src/image_features.rs',
    'registration3d':'crates/localization/src/registration3d.rs',
    'visual_odometry3d':'crates/localization/src/visual_odometry3d.rs',
    'reprojection3d':'crates/localization/src/reprojection3d.rs',
    'localization_lib':'crates/localization/src/lib.rs',
    'perception_lib':'crates/perception/src/lib.rs',
    'core_lib':'crates/core/src/lib.rs',
    'cargo_lock':'integrations/rgbd/Cargo.lock',
    'cargo_manifest':'integrations/rgbd/Cargo.toml',
    'rust_toolchain':'rust-toolchain.toml',
    'core_manifest':'crates/core/Cargo.toml',
    'perception_manifest':'crates/perception/Cargo.toml',
    'localization_manifest':'crates/localization/Cargo.toml',
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def new_directory(path):
    path = Path(path).absolute()
    require(not path.exists() and not path.is_symlink(),
            'output already exists or is a symlink: '+str(path))
    require(not any(parent.is_symlink() for parent in path.parents),
            'output parent is a symlink: '+str(path))
    path.mkdir(parents=True, exist_ok=False)
    return path


def read_json(path, limit=MAX_JSON_BYTES):
    require(path.is_file() and not path.is_symlink(), 'unsafe/missing input: '+str(path))
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rb') as stream:
        data = stream.read(limit+1)
    require(len(data) <= limit, 'JSON exceeds bounded decompression/read limit')
    return json.loads(data)


def non_timing(value):
    if isinstance(value, dict):
        return {key: non_timing(item) for key, item in value.items()
                if key not in CPU_TIMING_FIELDS}
    if isinstance(value, list):
        return [non_timing(item) for item in value]
    return value


def exact(actual, expected, context='evidence'):
    if isinstance(expected, dict):
        require(isinstance(actual, dict) and actual.keys() == expected.keys(), context+' fields changed')
        for key in expected:
            exact(actual[key], expected[key], context+'/'+key)
    elif isinstance(expected, list):
        require(isinstance(actual, list) and len(actual) == len(expected), context+' rows changed')
        for index, (a, e) in enumerate(zip(actual, expected)):
            exact(a, e, context+'/'+str(index))
    elif expected is None or isinstance(expected, bool):
        require(type(actual) is type(expected) and actual == expected, context+' value/type changed')
    elif isinstance(expected, (float, int)):
        require(type(actual) in (float, int) and math.isfinite(actual)
                and math.isfinite(expected) and actual == expected, context+' numeric value changed')
    else:
        require(type(actual) is type(expected) and actual == expected, context+' value changed')


def self_test(output):
    folder = new_directory(output)
    controls = []
    with tempfile.TemporaryDirectory(prefix='multiscale-suite-guards-') as temporary:
        base = Path(temporary)
        existing = base/'existing'
        existing.mkdir()
        sentinel = existing/'prior.json'
        sentinel.write_bytes(b'{"first":"preserve"}\n')
        prior = digest(sentinel)
        broken = base/'broken'
        broken.symlink_to(base/'missing', target_is_directory=True)
        parent_link = base/'parent-link'
        parent_link.symlink_to(existing, target_is_directory=True)
        for name, path in [('existing_directory', existing), ('broken_symlink', broken),
                           ('symlink_parent', parent_link/'child')]:
            try:
                new_directory(path)
            except ValueError:
                controls.append(name)
            else:
                raise ValueError('output admission corruption accepted: '+name)
        require(digest(sentinel) == prior and not (base/'missing').exists()
                and not (existing/'child').exists(), 'output guard modified prior evidence')
        original = {'source_sha256':'a'*64, 'protocol':{'age_s':.2},
                    'frames':[{'accepted':False,'rejection':'lost','error_m':.12}],
                    'summary':{'lost':True}, 'cpu_wall_seconds':1.}
        changed_cpu = json.loads(json.dumps(original))
        changed_cpu['cpu_wall_seconds'] = 900.
        exact(non_timing(changed_cpu), non_timing(original))
        controls.append('cpu_only_timing_variation_accepted')
        mutations = {
            'changed_source':lambda x:x.__setitem__('source_sha256','b'*64),
            'changed_gate':lambda x:x['protocol'].__setitem__('age_s',.3),
            'missing_rejection':lambda x:x['frames'][0].pop('rejection'),
            'changed_error':lambda x:x['frames'][0].__setitem__('error_m',.11),
            'removed_failed_frame':lambda x:x['frames'].clear(),
            'boolean_replaced_by_integer':lambda x:x['summary'].__setitem__('lost',1),
        }
        for name, mutate in mutations.items():
            changed = json.loads(json.dumps(original))
            mutate(changed)
            try:
                exact(non_timing(changed), non_timing(original))
            except ValueError:
                controls.append(name+'_rejected')
            else:
                raise ValueError('non-timing corruption accepted: '+name)
        compressed = base/'oversized.json.gz'
        with gzip.open(compressed, 'wb') as stream:
            stream.write(b' '*128)
        try:
            read_json(compressed, limit=64)
        except ValueError:
            controls.append('oversized_decompressed_json_rejected')
        else:
            raise ValueError('decompression budget ignored')
    proof = dict(passed=True, recorded_data_or_evaluator_invoked=False,
                 controls=controls, script_sha256=digest(Path(__file__)))
    (folder/'suite-self-test.json').write_text(json.dumps(proof, indent=2)+'\n')
    print(json.dumps(proof))
    return 0


def run_suite(binary, output):
    folder = new_directory(output)
    result_path = folder/'multiscale-temporal-regression-suite.json'
    status = dict(schema_version=1, regression_integrity_passed=False,
                  evidence_role='viewed regression', fresh_holdout_claim=False,
                  commands=[], cases=[])

    def save():
        result_path.write_text(json.dumps(status, indent=2, allow_nan=False)+'\n')

    def invoke(name, command, expected):
        log = folder/(name+'.log')
        started = datetime.now(timezone.utc).isoformat()
        with log.open('x') as stream:
            result = subprocess.run([str(x) for x in command], cwd=ROOT,
                                    stdout=stream, stderr=subprocess.STDOUT)
        status['commands'].append(dict(stage=name, command=[str(x) for x in command],
            exit_code=result.returncode, expected_exit_code=expected,
            started_utc=started, finished_utc=datetime.now(timezone.utc).isoformat(),
            log=log.name, log_sha256=digest(log)))
        save()
        require(result.returncode == expected,
                f'{name}: exit {result.returncode}, expected {expected}; retained {log}')

    save()
    try:
        require(digest(ASSETS/'design.json') == DESIGN_SHA, 'fixed viewed design changed')
        design = read_json(ASSETS/'design.json', 512*1024)
        require(tuple(design['datasets']) == DATASETS, 'fixed scene inventory changed')
        baseline = {}
        asset_hashes = {'design.json':DESIGN_SHA}
        source_hashes = {key:digest(ROOT/path) for key,path in SOURCES.items()}
        auditor = ROOT/'scripts/check-multiscale-temporal.py'
        auditor_sha = digest(auditor)
        require(auditor_sha == AUDITOR_SHA, 'first formal continuous auditor changed')
        for dataset in DATASETS:
            prefix = PREFIXES[dataset]
            first_freeze = ASSETS/(prefix+'-freeze.json')
            first_report = ASSETS/(prefix+'-report.json.gz')
            freeze = read_json(first_freeze, 512*1024)
            report = read_json(first_report)
            manifest = ROOT/'data'/dataset/'manifest.json'
            require(digest(manifest) == design['datasets'][dataset]['manifest_sha256'],
                    dataset+': original manifest bytes changed')
            exact(freeze['sources'], source_hashes, dataset+' first/current source hashes')
            exact(freeze['design'], design, dataset+' fixed design')
            require(freeze['dataset'] == dataset and freeze['design_sha256'] == DESIGN_SHA
                    and freeze['manifest_sha256'] == digest(manifest), dataset+': source/protocol identity')
            exact(report['freeze'], freeze, dataset+' preserved report/freeze')
            require(len(report['frames']) == 180 and set(report['summary']) == {'native','multiscale'},
                    dataset+': first report must preserve both180-frame branches')
            for mode in ('native','multiscale'):
                require(report['summary'][mode]['frames'] == 180
                        and report['summary'][mode]['updates'] == 179,
                        dataset+': failed rows removed from '+mode+' denominator')
            expected_exit = (2 if report.get('input_failure') or report.get('evaluation_label_failure') else
                0 if all(row['all_updates_passed'] for row in report['summary'].values()) else 1)
            baseline[dataset] = (freeze, report, expected_exit)
            asset_hashes[first_freeze.name] = digest(first_freeze)
            asset_hashes[first_report.name] = digest(first_report)
        binary = binary.absolute()
        require(binary.is_file() and not binary.is_symlink(), 'missing/unsafe executable')
        status.update(first_asset_sha256=asset_hashes, original_source_sha256=source_hashes,
                      binary_sha256=digest(binary), auditor_sha256=auditor_sha,
                      wrapper_sha256=digest(Path(__file__)))
        receipt_path = folder/'executable-receipt.json'
        invoke('executable-receipt', [binary, '--executable-receipt', '--output', receipt_path], 0)
        receipt = read_json(receipt_path, 512*1024)
        require(receipt['schema_version'] == 1 and receipt['executable_sha256'] == digest(binary),
                'executable receipt differs from the actual invoked bytes')
        exact(receipt['sources'], source_hashes, 'executable compiled source bindings')
        status['executable_receipt_sha256'] = digest(receipt_path)
        for dataset in DATASETS:
            prefix = PREFIXES[dataset]
            case = new_directory(folder/prefix)
            manifest = ROOT/'data'/dataset/'manifest.json'
            raw = manifest.parent/'raw'
            freeze_path, report_path = case/'freeze.json', case/'report.json'
            first_freeze, first_report, expected_exit = baseline[dataset]
            invoke(prefix+'-freeze', [binary, '--prepare-freeze', '--manifest', manifest,
                                    '--output', freeze_path], 0)
            exact(read_json(freeze_path, 512*1024), first_freeze, dataset+' full source/protocol freeze')
            invoke(prefix+'-evaluation', [binary, '--manifest', manifest, '--raw', raw,
                '--freeze', freeze_path, '--output', report_path], expected_exit)
            actual = read_json(report_path)
            exact(non_timing(actual), non_timing(first_report), dataset+' full non-timing report')
            audit_path = case/'audit.json'
            invoke(prefix+'-audit', [sys.executable, auditor, '--report', report_path,
                '--manifest', manifest, '--raw', raw, '--freeze', freeze_path,
                '--output', audit_path], 0)
            audit = read_json(audit_path)
            require(audit['kind'] == 'viewed_continuous_regression'
                    and audit['schema'] == 'rustdriving-multiscale-continuous-oracle-v1'
                    and len(audit['frames']) == 180
                    and all(audit[key] is True for key in
                        ('numeric_truth_checked_after_all_sensor_fits','one_initial_origin',
                         'no_reset_or_loss_recovery','all_acquisitions_retained'))
                    and audit['imported_globals_overridden'] is False,
                    dataset+': independent continuous integrity audit incomplete')
            exact(audit['summary'], actual['summary'], dataset+' independently reconstructed summaries')
            exact(audit['compiled_sources'], source_hashes, dataset+' independent auditor source bindings')
            require(audit['auditor_sha256'] == auditor_sha
                    and audit['design_sha256'] == DESIGN_SHA,
                    dataset+': formal auditor receipt differs from invoked fixed code/design')
            status['cases'].append(dict(dataset=dataset, evaluator_exit_status=expected_exit,
                physical_protocol_passed=expected_exit == 0, summary=actual['summary'],
                all_180_frames_and_179_updates_retained=True, independent_integrity_passed=True,
                full_source_protocol_and_non_timing_evidence_identical=True,
                freeze_sha256=digest(freeze_path), report_sha256=digest(report_path),
                audit_sha256=digest(audit_path)))
            save()
        require(source_hashes == {key:digest(ROOT/path) for key,path in SOURCES.items()}
                and auditor_sha == digest(auditor)
                and status['wrapper_sha256'] == digest(Path(__file__))
                and all(digest(ASSETS/name) == expected for name,expected in asset_hashes.items()),
                'first evidence, source or auditor changed during reproduction')
        status.update(regression_integrity_passed=True,
            all_physical_protocols_passed=all(case['physical_protocol_passed'] for case in status['cases']))
        save()
        print(json.dumps(dict(report=str(result_path), regression_integrity_passed=True,
                              all_physical_protocols_passed=status['all_physical_protocols_passed'])))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        status['failure'] = str(error)
        save()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        return self_test(args.output)
    require(args.binary is not None, '--binary required for viewed reproduction')
    return run_suite(args.binary, args.output)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError) as error:
        print('multiscale viewed regression: '+str(error), file=sys.stderr)
        sys.exit(2)
