# Recorded RGB-D local registration evaluator

This optional CPU-only Rust executable tests the bounded SE(3) matcher against
two recorded registered-depth frames. The previous frame is the map; the current
frame is the scan. Every estimate starts at identity. Motion-capture camera poses
are parsed only after **all** sensor-only registrations finish and are used only
to score the resulting transforms.

The preregistered selection contains 12 TUM Freiburg 1 XYZ depth frames and all
11 consecutive selected pairs. Fixed depth calibration, sampling, point bounds,
configuration, accuracy gates, input hashes and compiled source hashes are
written to the report. Rejected pairs remain in the denominator and retain their
rejection reason. The first three frames are calibration and the remaining nine
are temporal held-out frames in the same room. This does not measure independent
scene generalization, driving, global localization, SLAM or covariance coverage.

From the repository root, acquire the hash-pinned ignored raw data as described
in [the dataset documentation](../../data/tum-fr1-xyz/SOURCE.md), then run:

```sh
cargo test --release --locked --manifest-path integrations/rgbd/Cargo.toml
mkdir -p artifacts/tum-rgbd-tight
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --manifest data/tum-fr1-xyz-tight/manifest.json \
  --prepare-freeze artifacts/tum-rgbd-tight/freeze.json
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --manifest data/tum-fr1-xyz-tight/manifest.json \
  --raw data/tum-fr1-xyz-tight/raw \
  --freeze artifacts/tum-rgbd-tight/freeze.json \
  --output artifacts/tum-rgbd-tight/results.json
```

Create the output directory before preparing the freeze. Freeze preparation
reads only manifest metadata and compiled source/configuration hashes. It does
not decode depth, parse motion capture or run registration. Preparation refuses
to overwrite an existing freeze. Evaluation requires the saved freeze to match
exactly before opening raw inputs. The initial fast subset selected indices
120–131, about 0.033 seconds apart, after the first 0.333-second subset failed.
Its first evaluation exhausted the global work budget on all 11 pairs. Exact
per-cell AABB ordering/pruning now reduces point comparisons while preserving
nearest-neighbor results and index ties. Re-evaluation accepted 10/11 fast pairs
under unchanged thresholds and the same budget; these already viewed frames
are calibration/regression data. The tight subset selects unused indices
140–151 as a separate temporal protocol in the same environment. It cannot
establish new-scene generalization. The final source and protocol must be frozen
before its first evaluation. See each manifest for source and selection details.

Exit status is `0` if all 11 pairs meet the fixed 0.1 m / 0.1 rad accuracy gates,
`1` for a valid evaluation with rejected or inaccurate pairs, and `2` for invalid
inputs or I/O errors. The frozen first evaluation rejected all 11 pairs because
of insufficient unique overlap or the shared distance-work budget. These
failures are a baseline for future work, not evidence of working recorded-motion
localization. The original report, manifest and exact evaluator source are saved
in `baselines/`; its parameters were communicated before scoring, while its
source-hash snapshot is retrospective. See
[recorded-data measurements](../../docs/recorded-rgbd.md).

Dependencies and the lockfile are isolated from the default workspace; no GPU,
Python, ROS, ONNX model or custom runtime is needed. Raw data are ignored and are
not redistributed; the public source mirror is not a redistribution license.

## Recorded odometry and measured fixed-map baseline

The additional `--motion` mode preserves the original pair evaluator and its
failure history. It uses actual recorded depth for natural 6DoF pair odometry and
localizes each scan against the first measured depth cloud, without mocap inputs.
The original separately frozen temporal 260–271 trial accepted 11/11 accurate
pairs and 9/11 accurate fixed-map fits; two map ambiguities rejected, and full
protocol exit 1 remains reported. This viewed data is now explicitly
calibration/regression for the keyframe feature. The first 200–211 trial's exit 2 missing-reference failure and its exact
source/freeze are retained, with a later explicit unscorable-fit regression.
See [recorded-motion behavior, uncertainty and reproduction](../../docs/recorded-motion.md).
No SLAM, loop closure, independent-room or automotive localization claim follows.

## Bounded measured keyframe localization

`--keyframes` tracks against a rolling accepted measured cloud and composes
poses into the first depth-camera frame. It preserves registration guards,
issues no pose on rejection and latches loss after a 0.20-second gap; recovery
requires explicit reset. The evaluator never resets and parses mocap only after
all sensor-only fits. Its original preregistered protocol selected 36 consecutive
indices 340–375. Current protocol version 6 labels every supplied interval as
viewed calibration/regression; the original first-trial freeze is retained.
See [protocol, frame contract and reproduction](../../docs/recorded-keyframes.md).
The first frozen held-out interval accepted 33/35 updates but only 13/35 met the
root-frame accuracy gates; the complete trial remains **failed, exit 1**. Two
ambiguities reject and chained error reaches 0.170314 m / 0.163528 rad. Its
independent audit preserves this failure and rejects 16 report corruptions.
Later reruns treat the interval as viewed calibration/regression. Per-fit
covariance does not describe accumulated root-frame uncertainty. This is not
map fusion, SLAM or automotive localization.

## Bounded fused measured-map localization

`--submaps` adds a measured voxel map in the fixed first accepted depth-camera
frame. Accepted scans can update bounded voxel means and observation counts;
registration rejects preserve the map and accepted-pose clock. Atomic fusion
rejects preserve the map while allowing an otherwise valid pose. Direct root
fits avoid composing a replacement-keyframe chain, but ambiguity, fused-map
error and local drift remain explicit limitations. The evaluator runs every
fit before parsing motion-capture references and never resets a lost stream.
See [the frame, fusion, freeze and reproduction contract](../../docs/recorded-submaps.md).
This is an optional recorded-depth experiment, separate from the driving EKF.
Both frozen new recordings failed complete acceptance: desk accepted 29/35
updates with 8/35 accurate, while office accepted 7/35 with 5/35 accurate and
latched loss. Their independent audits retain every failure. Protocol 8 supports
precise pinned Freiburg 1/3 camera profiles and `--regression` for explicitly
viewed reruns; it does not loosen registration, accuracy or expiry gates.

## Measured RGB-D visual motion

The separate `--visual` path extracts original Rust FAST-9/binary image features,
matches descriptors, projects valid registered depth patches and robustly fits
known 3D correspondences. Only accepted observations replace the reference.
Repeated RGB acquisitions reject even when their preceding fit failed;
accepted-pose expiry latches loss before decoding another image. This optional
mode adds the existing local perception crate while preserving every registry
version/checksum and the previous algorithms. The published independent audit
reconstructs pixels, matching, depth geometry, rigid fits and clock state.
[Results, source freezes and reproduction](../../docs/recorded-visual-odometry.md).

## Pixel reprojection refinement

`--visual-reprojection` adds bounded Rust pixel optimization after the unchanged
robust measured 3D consensus. Only original inlier landmarks and current feature
pixels enter refinement. Failed refinement preserves the accepted reference
and clock and emits no current pose. Use `--regression` for the already viewed
desk, office, sitting and Freiburg 2 selections. Protocol 3 retains sensor traces
when full-source label validation fails, while returning exit 2 and marking
every accuracy reference unscorable. The original `--visual` results remain
numerically reproducible.
[Independent audit, results and commands](../../docs/recorded-reprojection.md).
