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
