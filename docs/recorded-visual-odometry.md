# Recorded RGB-D visual odometry

The optional `--visual` evaluator estimates measured camera motion from recorded
RGB feature correspondences and registered depth. It uses the Rust-native image
feature extractor and bounded robust 3D correspondence fitter. Ground-truth
motion-capture poses are parsed only after every operational fit has finished.
This is a short indoor camera-odometry experiment, not automotive localization.

Each new RGB image produces at most 400 FAST-9 features and oriented binary
descriptors. Matching requires mutual nearest neighbors, a strict ratio test in
both directions and a maximum Hamming distance. At most 256 matches become 3D
correspondences. Both endpoint depth patches must contain nine valid depths
between 0.3 and 5 m, with a maximum 0.05 m spread. A deterministic bounded RANSAC
fit and consensus refit reject insufficient support, degenerate geometry,
excessive motion and competing models. Known correspondences need rank-two
geometry; noncollinear planes are permitted.

Initialization validates measured feature/depth geometry and establishes the
first camera frame as the identity origin. Each accepted relative pose is
composed with the last accepted camera pose. Rejections produce no current root
pose and preserve the accepted reference and timestamp. There is no calibrated
covariance or accumulated root confidence estimate.

Recorded depth and RGB acquisition timestamps must differ by at most 0.02 s.
Before decoding pixels, the evaluator checks its loss state, the 0.20 s maximum
accepted-pose age, and RGB freshness. Freshness compares against the last
**observed** RGB timestamp, including images whose pose fit rejected. A duplicate
image cannot renew a pose by being associated with another depth frame. Skipped
images have `features_computed: false` and empty feature/match lists. Exceeding
the accepted-pose age latches loss; this evaluator never resets automatically.

The explicit Freiburg 1 and Freiburg 3 published camera profiles are pinned
alongside their calibration-document bytes. Registered depth uses pinhole
projection. The evaluator does not compensate acquisition time differences,
apply additional RGB undistortion, measure infrared extrinsics, or establish a
vehicle calibration. These limitations remain relevant even when a pose passes
the evaluation gates.

## Development evidence

The first viewed Freiburg 1 desk development run accepted 24 of 35 updates, and
all 24 accepted root poses met the fixed 0.1 m / 0.1 rad gates. Ten repeated RGB
acquisitions rejected; one consensus did not stabilize within the refit bound.
The largest accepted root errors were 0.048007 m and 0.055847 rad. All 35 updates
had valid motion-capture reference brackets and stayed in the denominator.
The complete trial therefore failed, with exit status 1.

The first development source, freeze and report remain preserved in ignored
`artifacts/visual-desk-development-v1/`. A later clock-before-decode refinement
preserved every accepted root estimate exactly; its separate viewed result is
in `artifacts/visual-desk-development-v2/`. Neither is fresh holdout evidence.
The office interval is also viewed development. The sitting interval is a
separately preregistered recording. Its first frozen result is retained, and
subsequent runs are regression measurements.

The office diagnostic used the same fixed configuration and accepted 32 of 35
updates, but only nine met the root accuracy gates. Two consensus fits did not
stabilize and one competing-model ambiguity rejected. The largest accepted
errors were 0.155641 m and 0.065158 rad, with all 35 reference brackets valid and
no loss latch. Its complete result also failed with exit 1. Improved continuity
alone does not establish accurate odometry.

## First preregistered sitting trial

The fixed sitting window selected original depth indices 100–135 using source
availability and timestamp association, before decoding its pixels or examining
reference pose values. The source/configuration/checker freeze, all 16 declared
source files, manifest and a preregistration journal were saved and independently
verified before the first operational run, which began at
2026-10-10 02:29:04.643229 UTC. The first freeze's SHA-256 is
`cb98437e063e248dc71a09ce4c6450344cc43d9c20473396f4f1d204babfbf56`.

The first trial accepted **31 of 35 updates**, and all 31 accepted root poses met
the fixed accuracy gates. Three frames had no bounded correspondence consensus;
one repeated RGB acquisition rejected. All 35 reference brackets were valid,
initialization succeeded and loss did not latch. Maximum accepted root errors
were **0.081783 m / 0.028519 rad**. The complete physical protocol still
**failed, exit 1**, because four selected updates had no accepted current pose.
No fit or accuracy threshold changed after viewing this result.

The independent checker passed both the current-source and archived-source first
trial audits. It reconstructed every image feature, descriptor, association,
depth correspondence and robust fit using NumPy Kabsch SVD, and verified
reference/clock state and motion-capture scoring. It rejected 34 report
corruptions and 11 frozen provenance/policy corruptions. This integrity result
preserves the failed physical protocol; all rejected updates remain visible.

| Recording | Role | Accepted / 35 | Accurate / 35 | Maximum accepted root error | Physical exit |
| --- | --- | ---: | ---: | --- | ---: |
| Freiburg 1 desk | Viewed development | 24 | 24 | 0.048007 m / 0.055847 rad | 1 |
| Freiburg 3 office | Viewed development | 32 | 9 | 0.155641 m / 0.065158 rad | 1 |
| Freiburg 3 sitting | First preregistered sequence | 31 | 31 | 0.081783 m / 0.028519 rad | 1 |

First evidence is saved in
[`assets/recorded-visual/first-sitting-v1/`](../assets/recorded-visual/first-sitting-v1/),
with exact source bytes in
[`integrations/rgbd/baselines/visual-sitting-first-v1/`](../integrations/rgbd/baselines/visual-sitting-first-v1/).
The three current numerical baselines use `--regression` and live under
[`assets/recorded-visual/current-regression/`](../assets/recorded-visual/current-regression/).
The [complete numerical regression proof](../assets/recorded-visual/visual-regression-suite.json)
compares all non-timing operational evidence and audits the immutable first
trial against its source snapshot.
A different recording and scene do not establish another physical room or
camera-domain generalization; office and sitting share the Freiburg 3 profile.

## Reproduction

To reproduce all three viewed baselines and independently verify the original
first trial:

```sh
python3 -m pip install -r scripts/requirements-visual.txt
python3 scripts/fetch-visual-datasets.py tum-fr1-desk-visual
python3 scripts/fetch-visual-datasets.py tum-fr3-office-visual
python3 scripts/fetch-visual-datasets.py tum-fr3-sitting-visual
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml
python3 scripts/check-recorded-visual-suite.py \
  --binary integrations/rgbd/target/release/rustdriving-rgbd-evaluate \
  --output artifacts/recorded-visual
```

The suite exits zero for unchanged results and independent integrity, while
preserving each evaluator's failed physical exit 1. CI runs this same suite
on CPU and uploads reports only; raw inputs remain excluded.

From the repository root, fetch the selected hash-pinned recording using
`scripts/fetch-visual-datasets.py`; consult the corresponding `data/*-visual/SOURCE.md`
for the exact source, file inventory and acquisition command. Raw RGB, depth and
motion capture are ignored and are not redistributed.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml
mkdir -p artifacts/visual-desk-reproduction
integrations/rgbd/target/release/rustdriving-rgbd-evaluate \
  --visual --regression --manifest data/tum-fr1-desk-visual/manifest.json \
  --prepare-freeze artifacts/visual-desk-reproduction/freeze.json
integrations/rgbd/target/release/rustdriving-rgbd-evaluate \
  --visual --regression --manifest data/tum-fr1-desk-visual/manifest.json \
  --raw data/tum-fr1-desk-visual/raw \
  --freeze artifacts/visual-desk-reproduction/freeze.json \
  --output artifacts/visual-desk-reproduction/results.json
```

The binary location follows `CARGO_TARGET_DIR` when configured. Freeze preparation
refuses to overwrite a freeze and reads only manifest metadata. It records
source, acquisition, independent checker, dependency-lock, calibration and all
algorithm/policy hashes before pixel decoding. Evaluation checks the entire
freeze before opening raw inputs and checks each input's size and SHA-256.
Use `--regression` for viewed data; an exact historical first-trial reproduction
does not constitute another fresh trial.

The isolated legacy JSON parser can round a small derived `pair_gap_seconds`
value one floating-point unit differently from Python. The independent audit
allows at most 1e-15 s difference only for that metadata-derived field when
comparing the source manifest, parsed freeze and report. The source manifest
SHA-256, acquisition timestamps and source indices remain exact, and the gap is
independently recomputed from the pinned timestamps. This serialization allowance
does not change the 0.02 s association limit or any physical accuracy/fit gate;
historical dependency features and old-mode numerics remain unchanged.

Exit 0 requires every selected update to have an accepted pose, a valid
motion-capture bracket and an accurate root pose. Exit 1 preserves legitimate
rejections and inaccurate or unscorable updates. Exit 2 indicates invalid inputs
or an I/O failure. The native feature and fit implementation needs no GPU,
learned model or ROS. The independent checker separately reconstructs pixels,
descriptors, correspondences and state using pinned Python dependencies and a
NumPy SVD pose fitter; checking integrity cannot turn a failed trial into a
successful one.

There is no scale-invariant feature pyramid, loop closure, relocalization, visual
map optimization, dynamic-object handling or planning/control integration in
this experiment. Execution timings are offline process measurements, not
real-time guarantees.
