# Recorded-depth keyframe localization

The optional `integrations/rgbd` Rust executable adds `--keyframes` to evaluate a
bounded, stateful SE(3) localizer against measured Kinect depth. It keeps one
accepted measured cloud as its reference and composes local transforms into the
first depth-camera frame. The existing pair-motion and fixed-first-cloud
regressions remain separate modes and retain their earlier evidence.

The operational input is a timestamp, the source frame index and measured XYZ
points. Motion-capture poses enter only after **every** sensor-only fit has
finished. They do not supply initial guesses, map coordinates, correspondence
choices or recovery poses. An accepted pose maps the current camera frame into
the initial camera frame; lengths are metres and rotations are radians.

The first scan self-registers at identity under the unchanged numerical guards.
Accepted scans replace the reference after at least 0.10 seconds. Registration
uses the last accepted relative pose as its prior; replacing the reference
resets that prior to identity. Rejected scans issue no pose and do not update the
reference. A gap longer than 0.20 seconds since the last accepted observation
latches tracking loss until an explicit reset. This evaluator never resets or
fills gaps with truth, interpolation or zero motion. A failed initialization
also requires explicit reset, preventing a silent change of root frame. Finite
chronological acquisition timestamps advance the observed clock before cloud
validation, so malformed scans cannot conceal expiry or permit retrograde input.

Preprocessing retains the existing bounded, CRC-checked 640×480 uint16 PNG
reader, 8-pixel sampling, 0.3–5.0 m depth range, 0.03 m initial voxel sampling
and 0.06 m additional voxel sampling. The nearest-neighbour correspondence
radius is 0.15 m. All existing geometry, ambiguity, convergence, covariance,
point-count and work-budget guards remain active.

## Protocol and reproduction

Already viewed `fast` 120–131, `tight` 140–151 and `motion-v2` 260–271 subsets
are calibration/regression data for this feature. The original temporal selection
contains exactly 36 consecutive source indices **340–375**, without selecting
frames by their registration outcome. Index 340 initializes the map; all 35
subsequent observations were held out for the original frozen implementation.
This interval shares the same indoor room and sensor as calibration, so it does
not establish independent-environment accuracy.

The original version-5 protocol was frozen before decoding those fresh frames.
Current version 6 marks every supplied interval as calibration/regression; the
original first-trial freeze and sources remain immutable. To reproduce current
behavior, save a new metadata-only external protocol:

```sh
source scripts/env.sh
cargo test --release --locked --manifest-path integrations/rgbd/Cargo.toml
mkdir -p artifacts/recorded-keyframes
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --keyframes --manifest data/tum-fr1-xyz-keyframes/manifest.json \
  --prepare-freeze artifacts/recorded-keyframes/freeze.json
```

Freeze preparation reads metadata only. It hashes the manifest, matcher,
keyframe core, executable sources, lockfile, independent checkers and numerical
configuration. It refuses to overwrite an existing freeze. Acquire raw data
using the dataset's source instructions, then evaluate with the saved freeze:

```sh
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --keyframes --manifest data/tum-fr1-xyz-keyframes/manifest.json \
  --raw data/tum-fr1-xyz-keyframes/raw \
  --freeze artifacts/recorded-keyframes/freeze.json \
  --output artifacts/recorded-keyframes/results.json
```

Evaluation verifies the external freeze before opening sensor or label files,
then checks every file's bytes and SHA-256 against the manifest. It checks frame
selection against the original depth index. The output retains initialization,
all update observations, every rejection and missing reference. Successful rows
include their reference index and timestamp, initial pose, local registration,
root transform, reference replacement and independent root accuracy. Both local
and chained translation and angular errors are scored; a missing reference is
never an accuracy pass.

Exit 0 requires one valid initialization and all 35 updates accepted and within
the fixed 0.1 m / 0.1 rad root accuracy gates with valid evaluation references.
Exit 1 retains a valid evaluation with rejected or inaccurate updates. Exit 2
indicates invalid inputs, protocol mismatch or I/O failure. Diagnostic timings
measure this CPU execution only; they do not establish real-time performance.

For the complete current regression suite and stronger independent audit of the
archived first result, fetch the five earlier subsets as documented in
[recorded motion](recorded-motion.md#reproduction-and-independent-checks), then:

```sh
python3 scripts/fetch-keyframe-dataset.py
python3 scripts/check-recorded-keyframe-suite.py \
  --binary integrations/rgbd/target/release/rustdriving-rgbd-evaluate \
  --output artifacts/recorded-keyframe-suite
```

The Python oracle requires Pillow. Suite exit 0 means all known numerical
outcomes and integrity checks reproduced, including expected evaluator exit 1
for the temporal drift, expiry and missing-reference cases. It explicitly
records that the original temporal accuracy protocol failed. Published
[current reports](../assets/recorded-keyframes/current-regression/validation.json)
retain all six denominators and source-bound freezes.

## First frozen outcome and failure retention

The first untouched temporal trial covered 1.167668 seconds. It initialized one
root frame and scored all **35 updates**: **33 accepted**, **13 within both root
accuracy gates**, **35 valid mocap references** and **10 reference replacements**.
Indices **359 and 364 rejected as ambiguous**. The complete protocol returned
**exit 1**; individual accepted fits are not evidence that the chained trajectory
passed. The maximum root-frame translation error was **0.170314 m**, and the
maximum root-frame angular error was **0.163528 rad**. Accepted local fits had
maximum errors of 0.063922 m and 0.041874 rad against their individual reference
frames, illustrating accumulation across reference replacements.

An independent checker reconstructed measured clouds, active references,
initial priors, root-transform composition and mocap scores. The original
checker integrity passed, and all **16** deliberate report/order corruptions
rejected. This checks the
reported failure; it neither reruns the optimizer nor turns the result into an
accuracy pass. The [original report and freeze](../assets/recorded-keyframes/first-temporal-v1/results.json)
and [exact source snapshot](../integrations/rgbd/baselines/keyframes-first-v1/SOURCE.json)
retain the chronological acquisition/run journal and audit before subsequent
changes. The subsequent chronology/expiry guard correction uses a new version-6 freeze
and treats this now viewed interval as **calibration/regression**, never another
untouched trial. All six current viewed reruns preserve every deterministic
pose, decision, rejection and score from their original outcomes, excluding
CPU timings and freeze identities. Input selection, preprocessing, numerical
configuration, accuracy gates, reference policy and per-fit uncertainty
allowances also remain identical. No numerical thresholds were retuned from
this outcome. All six current reports passed independent integrity checks;
**85** deliberate report mutations and **12** missing-reference score-contract
negative cases rejected.

The three already viewed positive calibration subsets each passed all 11 root
updates with three replacements; their independent audits each rejected 15
report/order mutations. Two recorded-data negative runs remain failures:

| Viewed input | Observed outcome | Meaning |
| --- | --- | --- |
| Original 0–110/10 with roughly 0.333-second gaps | Initialization accepted; all 11 updates rejected, exit 1 | Accepted-pose expiry latches loss without silent recovery. |
| Motion 200–211 | 10 updates accepted, seven root-accurate, eight valid references, one rejection, exit 1 | Three missing references remain unscorable rather than counted as passes. |

Both negative reports passed independent integrity audits. They exercise
recorded inputs and failure retention, rather than synthetic pose labels.

## Limits

This is one-reference-cloud localization with root-frame transform chaining,
not map fusion, SLAM, loop closure, global relocalization or integration with
vehicle localization/control. Errors can accumulate after each replacement.
The emitted covariance and the previously frozen engineering allowance describe
an individual reference-frame fit only. They are not accumulated root-frame
confidence and do not establish statistically calibrated coverage. The upstream
registered-depth reader's teaching calibration is not measured automotive
sensor calibration. Raw data and model assets are not redistributed.
