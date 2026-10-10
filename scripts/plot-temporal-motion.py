#!/usr/bin/env python3
"""Export recorded continuous root errors; rejected poses remain missing."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--preview', type=Path, help='Optional PNG export for visual review')
    args = parser.parse_args()
    if args.output.suffix.lower() != '.svg':
        raise ValueError('output must be a standalone SVG')
    content = args.report.read_bytes()
    report = json.loads(content)
    if report['algorithm'] != 'bounded_visual_reprojection_temporal':
        raise ValueError('requires the continuous temporal report')
    rows = report['frames']
    if [row['source_index'] for row in rows] != list(range(100, 280)):
        raise ValueError('requires all 180 original acquisitions')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['svg.hashsalt'] = 'rustdriving-temporal-motion-v1'
    origin = rows[0]['depth_timestamp']
    times = [row['depth_timestamp'] - origin for row in rows]
    boundary = times[36]
    figure, axes = plt.subplots(3, 1, figsize=(10, 6.8), sharex=True,
                                gridspec_kw={'height_ratios': [3, 3, 1]})
    for axis, field, label, gate in zip(
            axes[:2], ('translation_error_m', 'rotation_error_rad'),
            ('Root position error (m)', 'Root rotation error (rad)'),
            (report['freeze']['accuracy_gates']['translation_m'],
             report['freeze']['accuracy_gates']['rotation_rad'])):
        values = [row['root_accuracy'].get(field) if row['accepted'] else None for row in rows]
        values = [value if isinstance(value, (float, int)) and math.isfinite(value)
                  else float('nan') for value in values]
        axis.plot(times, values, color='#176998', linewidth=1.3, marker='.', markersize=3,
                  label='Accepted sensor estimate vs evaluation-only reference')
        axis.axhline(gate, color='#ba2525', linestyle='--', linewidth=1.2,
                     label=f'Frozen gate: {gate:g}')
        axis.axvspan(0, boundary, color='#d8e4ed', alpha=.65)
        axis.axvline(boundary, color='#667785', linestyle=':', linewidth=1)
        axis.set_ylabel(label)
        axis.set_ylim(bottom=0)
        axis.grid(alpha=.2)
    axes[0].legend(loc='upper right', fontsize=8)
    failures = [time for row, time in zip(rows[1:], times[1:]) if not row['accepted']]
    axes[2].scatter(failures, [0] * len(failures), marker='x', color='#ba2525', s=24,
                    label='Rejected acquisition: no current root pose')
    axes[2].axvspan(0, boundary, color='#d8e4ed', alpha=.65)
    axes[2].axvline(boundary, color='#667785', linestyle=':', linewidth=1)
    axes[2].set_yticks([])
    axes[2].set_xlabel('Elapsed acquisition time (s)')
    axes[2].legend(loc='upper left', fontsize=8)
    axes[2].set_xlim(0, times[-1])
    figure.suptitle('Continuous room motion: one initialization, no resets\n'
                    'Shaded prefix was viewed previously; same room, no automotive claim', fontsize=12)
    figure.tight_layout(rect=(0, 0, 1, .93))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, metadata={'Date': None})
    # SVG path coordinates allow line whitespace; normalize the generated text.
    args.output.write_text('\n'.join(line.rstrip() for line in
                                    args.output.read_text().splitlines()) + '\n')
    if args.preview is not None:
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.preview, format='png', dpi=120)
    plt.close(figure)
    metadata = dict(schema_version=1, report_sha256=hashlib.sha256(content).hexdigest(),
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    plot_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
                    matplotlib_version=matplotlib.__version__, frames=180, updates=179,
                    rejected_pose_values_plotted=False, no_interpolation_across_rejections=True,
                    time_source='original depth acquisition timestamp relative to index 100',
                    scope='Same-room partly viewed temporal extension')
    args.metadata.parent.mkdir(parents=True, exist_ok=True)
    args.metadata.write_text(json.dumps(metadata, indent=2)+'\n')


if __name__ == '__main__':
    main()
