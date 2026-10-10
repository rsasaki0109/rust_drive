# Recorded depth odometry and fixed-map localization

The optional Rust RGB-D executable now computes **natural 6DoF pair odometry**
and **local pose against a measured fixed first-frame map**. Camera motion comes
from depth alone. Independent TUM motion-capture labels enter only after all
sensor fits finish. This is a bounded offline indoor-camera baseline: no driving
fusion, vehicle calibration, SLAM, loop closure or global localization.

## Working behavior and retained failures

The separate temporal trial uses original depth-index entries **260–271**, twelve
previously unfitted depth frames spanning 1305031110.835248–1305031111.199395 s
(0.364 s). Numerical settings were frozen before first depth decoding or fitting.
All eleven consecutive pairs and all eleven fixed-map fits remain in the result.

| Partition / stage | Accurate accepted pair fits | Accurate accepted fixed-map fits | Continuous odometry |
|---|---:|---:|---:|
| Viewed original 0..110/10 regression | 4/11 | 3/11 | Invalid after first rejection |
| Viewed fast 120..131 regression | 11/11 | 11/11 | 11 frames |
| Viewed tight 140..151 regression | 11/11 | 9/11 | 11 frames |
| Viewed first temporal 200..211, invalid reference interval | 7/11 physically scored; 11 sensor fits | 8/11 physically scored; 11 sensor fits | 11 frames, incomplete scoring |
| **Fresh temporal 260..271** | **11/11** | **9/11** | **11 frames** |

The fresh pair errors are **0.00134–0.01072 m**, **0.00386–0.01594 rad**.
The nine accepted fixed-map errors are **0.00375–0.03689 m**,
**0.00419–0.03915 rad**. Fixed-map fits at source indices **270 and 271 reject
comparable distinct local solutions**. The unchanged ambiguity guard keeps these
rejections; choosing a plausible-looking solution would hide a real limitation.
The full frozen pair-plus-map protocol therefore **fails**, evaluator exit **1**,
although pair odometry completes. This does not become an all-passed benchmark.

Composed odometry accumulates error: the final 0.364 s trajectory error is
**0.05303 m and 0.06177 rad** relative to independent mocap. This is greater than
any one accepted pair error. Pair covariance is not propagated into a trajectory
confidence bound. No long-term drift, mapped-room consistency or real-time claim
follows. Per-fit synchronous CPU measurements of this trial are about
0.237–0.413 s, excluding depth decoding and reference scoring.

## Sensor-only algorithm

The locked optional integration reuses the core bounded SE(3) matcher. It first
uses the established uint16 PNG decoder, 640×480 registered-depth/default camera
model, eight-pixel sampling and 0.03 m voxel preprocessing, then a deterministic
**0.06 m voxel**. Its representative is the first point in original lexicographic
voxel order. The correspondence radius is tightened from 0.30 to **0.15 m**.
Point counts and all final residual/overlap checks are independently reconstructed
from actual raw depth; no reference pose selects points or correspondences.

All other original safety guards remain: 20,000,000 distance-comparison budget,
0.4 retained overlap, 0.08 m RMS ceiling, bounded jumps, degeneracy rejection and
twelve ambiguity probes. No guard was weakened to improve acceptance. The pair
initialization is identity. Its current-camera-to-previous-camera transform is
composed only while every preceding pair is accepted; an earlier rejection makes
continuous odometry invalid, without a truth reset or invented bridge.

The first measured cloud supplies a fixed local map in the first optical frame.
Each subsequent scan starts from the last accepted sensor-only map estimate,
initially identity. Rejected fits do not update that estimate. The first cloud
is neither a motion-capture map nor a retrospectively transformed ground-truth
map. It is not updated, so changing view/occlusion can lose overlap. This baseline
does not implement map construction, keyframes or relocalization.

## Uncertainty remains provisional

The conditional least-squares covariance is retained unchanged and still badly
understates measured error. Fresh accepted-pair NEES is **3,197–22,739**;
accepted-map NEES is **1,319–51,984**, compared with the ideal six-dimensional
Gaussian 95% reference value of about 12.59.

A separate **provisional covariance** adds diagonal engineering allowances of
0.02 m position standard deviation and 0.03 rad rotation standard deviation.
These values were fixed before the first new temporal trial, and unchanged after
any new outcome. Viewed fast/tight calibration errors fit within the nominal reference ellipsoid;
they account conservatively for otherwise omitted correlated-depth/map and
association effects. This is an engineering floor, **not a fitted statistical
noise model or established confidence guarantee**.

The independent oracle measures provisional NEES **0.021–0.461** for eleven fresh
pairs and **0.069–5.099** for nine fresh map fits. All twenty accepted, scored fits
fall below the nominal 95% reference. This small, correlated room sequence cannot
establish 95% coverage or independent-scene calibration; rejection availability
is 11/11 for pairs and 9/11 for map fits. Rejected fits receive no invented
covariance. The allowance is not fused into the driving EKF.

## Preregistration and missing-label failure

Previously viewed original, fast and tight windows are calibration/regression
only. The first new trial selected **200–211** and froze sources/settings before
sensor fitting. It failed validity with evaluator exit **2**: depth timestamps
201–203 lie in a **0.11010 s** mocap gap, exceeding the fixed 0.02 s interpolation
limit. Its exact evaluator, motion module, checker, manifest, external freeze and
failure log remain in [the first-trial archive](../integrations/rgbd/baselines/motion-temporal-v1/failure.json).
No accuracy claim follows from that failed original trial.

The reporting correction subsequently retains every sensor fit and marks missing
physical references explicitly. The now-viewed 200–211 regression has four
unscorable pair references and three unscorable fixed-map references. The
remaining 7/11 and 8/11 fit scores pass the unchanged pose gates. Missing labels
are never accuracy passes, and the regression evaluator exits 1.

The second selection **260–271** was declared separately, using **timestamp
bracket availability only**, before acquisition/decoding/fitting. No depth,
reference transform or measured fit error was used to choose it. All twelve
brackets fit the unchanged 0.02 s limit. The new evaluator/checker freeze and
[pre-evaluation timestamp record](../assets/recorded-motion-v2-pre-evaluation.json)
explicitly preserve this sequence of events. Registration, preprocessing,
uncertainty numerical settings and accuracy gates equal those of the first trial.
No parameters were tuned after inspecting the second temporal outcome.

Both trials use the **same physical room and camera sequence**. They cannot
establish independent-environment or automotive localization. The new frames
retain the historical manifest's first-three-frame `calibration` label, but no
new frame tuned either frozen protocol. Nine pairs belong to its later
`held_out` partition; all eleven second-trial pairs were freshly unfitted here.

## Reproduction and independent checks

Raw data stay ignored and are not redistributed. Public transfer URLs do not
grant a dataset licence; an official TUM terms request returned HTTP403 through
the configured proxy. See [source and licence limits](../data/tum-fr1-xyz-motion-v2/SOURCE.md).
The optional integration and its existing pinned PNG/SHA dependencies retain
separate locks; the driving workspace adds no image dependencies.

```sh
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz-fast
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz-tight
python3 scripts/fetch-recorded-motion.py --dataset tum-fr1-xyz-motion
python3 scripts/fetch-recorded-motion.py --dataset tum-fr1-xyz-motion-v2
cargo test --release --locked --manifest-path integrations/rgbd/Cargo.toml
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml
python3 scripts/check-recorded-motion-suite.py \
  --binary integrations/rgbd/target/release/rustdriving-rgbd-evaluate \
  --output artifacts/recorded-motion
```

The suite requires Pillow (the existing demo requirements suffice). On Windows,
append `.exe` to the binary path. A custom `CARGO_TARGET_DIR` requires its actual
binary path. The suite checks frozen sources, independent raw hashes, depth
geometry, timestamp interpolation, relative transform direction, pose errors,
final correspondences/residuals/overlap, initializations, composed odometry and
conditional/provisional SPD covariance/NEES. Eleven adversarial report mutations
are rejected in the fresh trial. The oracle does **not rerun the ICP optimizer**.

Suite exit0 means **integrity and known regression outcomes reproduced**; it
explicitly records `all_physical_protocols_passed=false`. Its new temporal
physical protocol still exits1 because two map fits reject. The archived first
trial check reconstructs the missing-reference timestamp witness and verifies
exact frozen-source hashes; it does not claim to rerun the historical optimizer.
Full reports, all failures and measured uncertainty accompany the compact
[recorded-motion evidence](../assets/recorded-motion-results.json).
