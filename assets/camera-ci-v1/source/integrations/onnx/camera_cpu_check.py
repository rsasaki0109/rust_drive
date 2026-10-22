#!/usr/bin/env python3
"""Compare the unchanged parallel detector with an opt-in native CPU build.

Freeze ABBA order, original images, model and 200 ms policy before inference.
Native flags are not an accuracy guarantee: require complete numerical parity.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import statistics

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SUPPORT_SHA = 'f9d4e081b4989f3ffe3bd40f7f030f817797188161370b636276c127ac90d109'
ORDER = ['baseline-first', 'native-first', 'native-repeat', 'baseline-repeat']


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


build = module('native_cpu_build', HERE / 'build_cpu.py')
require, digest = build.require, build.digest
require(digest(HERE / 'camera_parallel_check.py') == SUPPORT_SHA,
        'pinned parallel numerical and timing auditor required')
previous = module('native_cpu_parallel_support', HERE / 'camera_parallel_check.py')
load, write, fresh = previous.load, previous.write, previous.fresh


def controls(args):
    receipt = build.verify(args.build)
    previous.controls(argparse.Namespace(binary=args.binary, output=args.output))
    # Exercise the new build binding and CPU admission independently of a model.
    rejected = []
    for name, change in [
        ('binary-hash', lambda row: row.update(executable_sha256='0' * 64)),
        ('cpu-features', lambda row: row['linux_cpu_flags'].append('rustdriving_impossible_cpu_feature')),
        ('missing-cpu-features', lambda row: row.update(linux_cpu_flags=[])),
        ('missing-sources', lambda row: row.update(compiled_sources={})),
        ('recipe', lambda row: row['compiler_overrides'].update(CARGO_PROFILE_RELEASE_OPT_LEVEL='2')),
        ('builder', lambda row: row.update(build_script_sha256='0' * 64)),
    ]:
        target = fresh(args.output / ('build-' + name))
        (target / 'bin').mkdir()
        shutil.copy2(args.binary, target / 'bin' / build.BINARY)
        row = json.loads(json.dumps(receipt))
        change(row)
        write(target / 'build.json', row)
        try:
            build.verify(target)
        except ValueError as error:
            rejected.append(dict(name=name, error=str(error)))
        else:
            raise ValueError('invalid build receipt accepted: ' + name)
        # Preserve the negative receipt and diagnostic, without five binaries.
        (target / 'bin' / build.BINARY).unlink()
        (target / 'bin').rmdir()
    write(args.output / 'cpu-summary.json', dict(
        pass_checks=True, native_input_rejections=25, existing_output_protections=2,
        build_admission_rejections=rejected, build_receipt_sha256=digest(args.build / 'build.json'),
        executable_sha256=digest(args.binary), checker_sha256=digest(__file__),
        build_script_sha256=digest(HERE / 'build_cpu.py')))


def run(args):
    receipt = build.verify(args.build)
    control = load(args.controls / 'cpu-summary.json')
    require(control['pass_checks'] and control['checker_sha256'] == digest(__file__)
            and control['build_receipt_sha256'] == digest(args.build / 'build.json')
            and control['executable_sha256'] == digest(args.binary), 'matching CPU controls')
    out = fresh(args.output)
    latency, freshness, tracking = previous.support()
    source_path = ROOT / 'assets/camera-sequence-v1/sequence.json'
    require(digest(source_path) == previous.SOURCE_SHA and digest(args.model) == previous.MODEL_SHA,
            'original source schedule and model')
    source = load(source_path)
    binaries = dict(baseline=args.baseline, native=args.binary)
    receipts = {}
    for name, binary in binaries.items():
        receipts[name], _ = latency.native(binary, ['--executable-receipt'], out / (name + '-executable.json'))
        require(receipts[name]['compiled_sources'] == receipt['compiled_sources'], 'exactly the same 35 compiled sources')
        require(receipts[name]['executable_sha256'] == digest(binary), 'executable hash')
    require(receipts['baseline']['executable_sha256'] != receipts['native']['executable_sha256'],
            'distinct baseline and optimized executable required')
    selection = []
    for index, event in enumerate(source['events']):
        selection.append((f'tum-{index}', source_path.parent / event['input']['path'],
                          ROOT / f'assets/camera-sequence-v1/recorded/exports/first-{index}.json', 'TUM'))
    for saved in sorted((ROOT / 'assets/camera-analysis-v1/publication-repair/regression/first').glob('*.json')):
        selection.append((saved.stem, args.bdd / (saved.stem + '.jpg'), saved, 'BDD'))
    require(len(selection) == 16, 'all original eight TUM and eight BDD images')
    for _, image, saved, _ in selection:
        require(digest(image) == load(saved)['source_image_sha256'], 'original image bytes')
    pins = dict(receipt['compiled_sources'])
    for path in (Path(__file__), HERE / 'camera_history.py', HERE / 'build_cpu.py', HERE / 'camera_parallel_check.py',
                 HERE / 'camera_latency_check.py', ROOT / 'integrations/camera-tracking/freshness_check.py',
                 ROOT / 'integrations/camera-tracking/check.py'):
        pins[str(path.relative_to(ROOT))] = digest(path)
    write(out / 'before-inference.json', dict(
        order=ORDER, source_sequence_sha256=previous.SOURCE_SHA, model_sha256=previous.MODEL_SHA,
        policy=freshness.POLICY, executable_receipts=receipts, native_build_receipt=receipt,
        source_sha256=pins, image_selection=[dict(name=name, data=data, image_sha256=digest(image),
        reference_report_sha256=digest(saved)) for name, image, saved, data in selection],
        viewed_development_data=True, independent_fresh_holdout=False,
        repeated_image_runs=2,planned_same_host_reference_forwards=64, hardware_speedup_not_a_correctness_gate=True))
    inspections = {}
    for name, binary in binaries.items():
        inspections[name], _ = latency.native(binary, ['--inspect-sequence', source_path], out / (name + '-inspection.json'))
    trials = []
    historical = latency.camera_history.Comparison()
    forwards = 0
    for label in ORDER:
        name = label.split('-')[0]
        print('Timing ' + label, flush=True)
        report, wall = latency.native(binaries[name], ['--model', args.model, '--sequence', source_path], out / (label + '.json'))
        require(report['compiled_sources'] == receipt['compiled_sources'], 'timed complete source receipt')
        require(report['source_sequence_sha256'] == previous.SOURCE_SHA, 'timed original source identity')
        require(report['startup_before_schedule_wall_s'] + report['schedule_wall_s'] <= wall + .01,
                'external elapsed time covers all reported stages')
        manifest, _, _ = previous.audit_parallel(report, source, inspections[name], freshness, tracking)
        durations = []
        for event in report['events']:
            analysis = event['measured']['analysis']
            if analysis is not None:
                saved = ROOT / f"assets/camera-sequence-v1/recorded/exports/first-{event['source']['event_index']}.json"
                previous.parallel_analysis(analysis, load(saved), historical)
                require(analysis['compiled_sources'] == load(ROOT / 'assets/camera-parallel-v1/verification/parity/tum-0-first.json')['compiled_sources'],
                        'complete unchanged 26-source analyzer receipt')
                durations.append(event['measured']['timing']['read_analysis_and_probe_wall_s'])
        path = out / (label + '-guard-input.json')
        write(path, manifest)
        guard, _ = latency.native(args.guard, ['--sequence', path], out / (label + '-guard-result.json'))
        require(guard['results'] == [item['output'] for event in report['events'] for item in event['measured']['guard_events']],
                'exact independent guard replay')
        trials.append(dict(label=label, external_wall_s=wall, processing_s=durations,
                           actual_model_forwards=report['model_forwards'], summary=report['summary']))
        forwards += report['model_forwards']
    write(out / 'trials.json', trials)
    parity = fresh(out / 'parity')
    expected_analyzer_sources = load(ROOT / 'assets/camera-parallel-v1/verification/parity/tum-0-first.json')['compiled_sources']
    parity_forwards = 0
    reference_forwards = 0
    references = fresh(out / 'current-baseline')
    for name, image, saved, data in selection:
        print('Exact comparison ' + data + ' ' + name, flush=True)
        current, _ = latency.native(args.baseline, ['--model', args.model, '--image', image], references / (name + '.json'))
        reference_forwards += current['execution']['model_forwards']
        results = []
        for repeat in ('first', 'repeat'):
            actual, _ = latency.native(args.binary, ['--model', args.model, '--image', image], parity / (name + '-' + repeat + '.json'))
            require(actual['compiled_sources'] == expected_analyzer_sources, 'complete analyzer source receipt')
            previous.parallel_analysis(actual, load(saved), historical)
            require(previous.stripped(actual) == previous.stripped(current), 'exact current-host native/baseline output')
            results.append(previous.stripped(actual))
            parity_forwards += actual['execution']['model_forwards']
        require(results[0] == results[1], 'exact repeated numerical output')
    require(all(digest(ROOT / name) == pin for name, pin in pins.items()), 'sources unchanged during verification')
    require(all(digest(binaries[name]) == row['executable_sha256'] for name, row in receipts.items()), 'binaries unchanged')
    build.verify(args.build)
    baseline = [row['processing_s'][0] for row in trials if row['label'].startswith('baseline')]
    native = [row['processing_s'][0] for row in trials if row['label'].startswith('native')]
    quality = load(ROOT / 'assets/camera-analysis-v1/publication-repair/regression/results.json')
    write(out / 'summary.json', dict(
        pass_checks=True, order=ORDER, baseline_processing_s=baseline, native_processing_s=native,
        mean_speedup=statistics.mean(baseline) / statistics.mean(native),
        mean_processing_reduction_fraction=1 - statistics.mean(native) / statistics.mean(baseline),
        timing_speedup_required_for_execution_success=False, age_limit_s=.2,
        observed_deadline_gates={row['label']: row['summary']['deadline_gate_pass'] for row in trials},
        original_image_count=16, repeats_per_image=2, complete_numerical_parity=True,
        benchmark_actual_model_forwards=forwards, parity_actual_model_forwards=parity_forwards,
        same_host_reference_forwards=reference_forwards,same_host_native_baseline_exact=True,
        total_actual_model_forwards=forwards + parity_forwards + reference_forwards,
        inherited_bdd_quality=quality['overall'], inherited_quality_gates_pass=quality['quality_gates']['pass'],
        historical_comparison=historical.report(),quality_metrics_inherited_from_same_boxes_classes_and_order=True,
        new_accuracy_improvement_measured=False,
        source_sha256=pins, executable_receipts=receipts, native_build_receipt_sha256=digest(args.build / 'build.json')))
    print(json.dumps(dict(pass_checks=True, mean_speedup=statistics.mean(baseline) / statistics.mean(native),
                          original_image_count=16, complete_numerical_parity=True)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['controls', 'run'])
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, default=HERE / 'target/release/rustdriving-camera-parallel')
    parser.add_argument('--guard', type=Path, default=ROOT / 'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--bdd', type=Path, default=ROOT / 'artifacts/bdd-camera')
    parser.add_argument('--controls', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    for key in ('build', 'baseline', 'guard', 'model', 'bdd', 'output'):
        setattr(args, key, getattr(args, key).absolute())
    args.binary = args.build / 'bin' / build.BINARY
    if args.mode == 'controls':
        controls(args)
    else:
        require(args.controls is not None, '--controls required')
        run(args)
