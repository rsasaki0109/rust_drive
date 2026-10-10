# Bounded measured-depth submap localization

The optional `rustdriving-rgbd-evaluate --submaps` executable localizes actual
recorded depth clouds against a measured map fused in the first accepted
depth-camera frame. It uses the reusable
`rustdriving_localization::submap3d::SubmapLocalizer3d`; it is separate from the
driving EKF and cannot actuate a vehicle. This is a bounded local mapping
experiment, without loop closure, global relocalization or automotive
validation.

## Operational contract

The first measured cloud must pass the existing guarded self-registration at
identity before it establishes an origin. Each subsequent accepted pose is a
direct scan-to-root-map fit, with the last accepted root pose as its prior.
It does not repeatedly compose poses through replacement reference frames.
Motion-capture labels are parsed only after every operational fit and map
update has finished; labels never initialize, update, reset or repair the map.

Sampling retains the existing 640 × 480 registered-depth interpretation,
8-pixel stride, 0.3–5.0 m range, first 0.03 m voxel representative and an
additional 0.06 m coarse voxel representative. The scan and map are bounded
to 5,000 points. Registration preserves a 0.15 m correspondence limit,
0.4 unique overlap fraction, 0.08 m maximum RMS, the shared 20-million
distance-check budget and all 12 alternative-fit ambiguity probes. The
existing 0.1 m / 0.1 rad accuracy gates remain evaluation-only criteria.

The historical XYZ regressions preserve their original 525.0/525.0 focal
lengths and 319.5/239.5 principal point. The separate desk profile uses the
precise pinned Freiburg 1 values 517.306408/516.469215 and
318.643040/255.313989. Its YAML source is hash-verified before depth decoding
and copied into the freeze/report provenance. These published RGB intrinsics
serve as an explicit pinhole model for registered depth; RGB distortion
coefficients are retained as documentation and are not applied to depth pixels.
Physical native-infrared extrinsics and vehicle calibration are unavailable.
The separate Freiburg 3 office profile uses 535.4/539.2 focal lengths and
320.1/247.6 principal point from its independently pinned TUM3 YAML. Its source
reports zero RGB distortion; additional depth pixel correction is still not
performed. Both new profiles keep 5,000 depth units per metre.

After an accepted fit, a map update is eligible at least 0.10 seconds after
the last successful fusion. Measured points are transformed into the root and
grouped by `floor(coordinate / 0.06 m)` voxel keys. Each voxel stores an online
mean and its raw-observation count, in deterministic lexicographic key order.
An update first builds a bounded proposed copy; any invalid coordinate,
5,000-voxel capacity violation or 1,000-observation per-voxel count violation
rejects the entire proposal. Such a skipped fusion can leave a valid pose
accepted and renew its age. A rejected registration issues no pose, changes no
map and does not renew the accepted-pose clock.

More than 0.20 seconds since the last accepted pose latches loss. Invalid
acquisitions cannot conceal elapsed time or renew permission. An explicit
`reset` establishes a separate origin; it cannot reconnect the previous
trajectory. This recorded evaluator never resets. Map means/counts and the
last accepted pose may remain inspectable after loss as historical state;
they are not current valid output.

## Evidence and audit

Protocol version 8 freezes the manifest, sampling, registration parameters,
fusion policy, source files, checker/acquisition scripts, Cargo manifests and
locks, and pinned toolchain before raw access. Preparing a freeze reads metadata
only and refuses to replace an existing file. Evaluation compares the entire
freeze with the compiled protocol before opening raw inputs. Every frame,
including initialization, rejected updates and missing reference brackets,
appears in the report.
The original first desk trial retains its version 7 freeze and exact archived
sources. Version 8 adds the separate office input/calibration profile and a
reporting-only `--regression` flag. Already viewed XYZ and desk intervals are
always labeled calibration/regression. The first office freeze was prepared
before decoding; subsequent office reruns can use `--regression` with a new
calibration freeze, without changing the operational algorithm or thresholds.
Both first-trial freezes, journals, results and audits are retained under
`assets/recorded-submaps/first-desk-v7/` and `first-office-v8/`; exact frozen
sources are archived under `integrations/rgbd/baselines/submap-*-first-v*/`.

The independent Python checker reconstructs measured clouds, voxel fusion,
map geometry/count digests, before/after clocks and priors, root transforms,
registration residuals and overlap, and motion-capture scoring. It also tests
deliberate report corruptions. A successful integrity audit verifies the report
and its retained failures; it does not turn a failed physical-accuracy trial
into a pass.

The first frozen desk recording selected 36 consecutive original depth indices
100–135 before decoding. It accepted 29/35 updates, but only 8/35 passed both
root accuracy gates, despite all 35 valid mocap references. Six ambiguity
rejections remain explicit. The maximum accepted errors reached
0.427439 m / 0.477560 rad; the complete protocol **failed, exit 1**. It performed
10 map updates and retained 1,433 representatives without capacity failures or
loss. The independent audit verified map/count SHA digests and all state/score
rows, rejected 28 report corruptions and six source-freeze corruptions, and ran
four fusion and two missing-reference examples. The desk recording differs in
sequence and clutter from the XYZ regressions; a different physical room or
camera is not established. Accepted local ICP fits alone do not establish
accurate motion or trustworthy fused mapping.

The separately frozen Freiburg 3 long-office-household trial also selected
indices 100–135 before decoding. It accepted 7/35 updates and passed both root
accuracy gates on 5/35, with all 35 references valid. Five ambiguity rejects
were followed by one accepted-pose age expiry and 22 explicit latched-loss
rejects. Its maximum accepted errors were 0.128414 m / 0.030981 rad. The
complete protocol **failed, exit 1**; three map updates retained 3,553 points.
The independent audit verified every retained failure and rejected 28 report
and six source corruptions. This distinct recording and camera profile expose
limited coverage; one short office window cannot establish broad environment
generalization. Neither first-trial result was used to loosen matching or
expiry gates.

Current viewed regression outcomes preserve every update in the denominator:

| Recording / original indices | Updates | Accepted | Accurate root | Valid references | Exit |
|---|---:|---:|---:|---:|---:|
| XYZ original 0–110 / 10 | 11 | 0 | 0 | 11 | 1 |
| XYZ fast 120–131 | 11 | 11 | 11 | 11 | 0 |
| XYZ tight 140–151 | 11 | 11 | 11 | 11 | 0 |
| XYZ motion 200–211 | 11 | 10 | 7 | 8 | 1 |
| XYZ motion-v2 260–271 | 11 | 11 | 11 | 11 | 0 |
| XYZ keyframes 340–375 | 35 | 24 | 24 | 35 | 1 |
| Freiburg 1 desk 100–135 | 35 | 29 | 8 | 35 | 1 |
| Freiburg 3 office 100–135 | 35 | 7 | 5 | 35 | 1 |

The sparse original interval exceeds the 0.20-second accepted-pose age limit;
its rejection is expected. The 200–211 interval preserves three missing root
reference brackets as unscorable. The viewed 340–375 interval has more accurate
accepted root poses than the previous replacement-keyframe chain (24 versus
13), while accepting fewer updates (24 versus 33). Both complete protocols
still fail; better conditional accuracy does not establish better coverage.

The 6 × 6 per-fit covariance excludes correlated fused-map errors and
association uncertainty. The previously declared 0.02 m / 0.03 rad diagonal
engineering allowance remains provisional, without statistical coverage or
root-confidence claims. No covariance is fed to the driving pipeline.

## Reproduction

From the repository root, install the pinned Rust toolchain and acquire the
hash-pinned ignored raw files described by the chosen dataset's `SOURCE.md`.
For the already viewed 340–375 XYZ calibration interval:

```sh
source scripts/env.sh
cargo test --locked --manifest-path integrations/rgbd/Cargo.toml
mkdir -p artifacts/submap-example
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --submaps --regression --manifest data/tum-fr1-xyz-keyframes/manifest.json \
  --prepare-freeze artifacts/submap-example/freeze.json
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --submaps --regression --manifest data/tum-fr1-xyz-keyframes/manifest.json \
  --raw data/tum-fr1-xyz-keyframes/raw \
  --freeze artifacts/submap-example/freeze.json \
  --output artifacts/submap-example/results.json
python3 scripts/check-recorded-submaps.py \
  --manifest data/tum-fr1-xyz-keyframes/manifest.json \
  --raw data/tum-fr1-xyz-keyframes/raw \
  --freeze artifacts/submap-example/freeze.json \
  --report artifacts/submap-example/results.json \
  --output artifacts/submap-example/independent-check.json
```

Exit status `0` requires one initialization and every update accepted with a
valid reference and both accuracy gates satisfied. A valid report with rejected,
inaccurate or unscorable updates exits `1`; invalid input or I/O exits `2`.
The 340–375 calibration example is expected to exit `1`; keep its report and
run the checker anyway. Raw depth and mocap are ignored and not redistributed.
The original dataset's license must be considered separately from the Rust
implementation's Apache-2.0 license.

After acquiring all eight pinned optional inputs, reproduce and independently
audit the complete viewed suite:

```sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml
python3 scripts/check-recorded-submap-suite.py \
  --binary integrations/rgbd/target/release/rustdriving-rgbd-evaluate \
  --output artifacts/submap-regression-suite
```

The runner audits both immutable first trials, prepares current
`--regression` freezes for all eight datasets, preserves their recorded exit
statuses and compares every deterministic non-timing report field with the
published baselines. Exit `0` verifies reproducible integrity and expected
outcomes; `all_physical_protocols_passed=false` preserves the five failed
physical-accuracy protocols. It does not claim a new holdout from repeated
inputs. The two new recordings can be acquired with
`scripts/fetch-submap-datasets.py`; the six historical XYZ datasets retain
their existing acquisition instructions.
