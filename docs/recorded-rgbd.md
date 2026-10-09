# Recorded RGB-D pair registration

The optional `integrations/rgbd` executable evaluates the Rust 6-DoF local
point-to-point matcher on real TUM RGB-D depth frames. Independent motion-capture
poses score accepted motion estimates. **Both original frozen benchmarks reject
all eleven pairs**. Exact nearest-neighbour search optimization subsequently
accepts 10/11 already viewed regression pairs and **7/11 later preregistered
temporal pairs**, including 5/9 in the temporal held-out partition. Rejections
and severely overconfident conditional covariance remain documented below.
This is a sensor-only **local pair matcher**;
it does not implement SLAM, global localization, camera AI or driving-pipeline
fusion. Recorded sensor processing and physical pose scoring complement the
older apartment experiment's imposed semi-synthetic SE(2) transforms.

## Source, frames and calibration

The [SHA-pinned manifest](../data/tum-fr1-xyz/manifest.json) specifies twelve
measured frames from `rgbd_dataset_freiburg1_xyz`, acquired through a public TUM
teaching mirror. Selection was fixed before scoring: every tenth original
depth-index entry from index 0 through 110. The selected timestamps span
1305031102.160407–1305031105.830008 seconds, about 3.67 seconds.

All eleven consecutive selected-frame pairs are evaluated. The current frame
defines the partition: two calibration pairs and nine temporal held-out pairs.
The held-out frames share the same room and acquisition as calibration; they
do not establish independent-scene generalization. No difficult pair is omitted.

The upstream registered-depth teaching reader specifies 640 × 480 uint16 depth
PNGs, `fx = fy = 525`, `cx = 319.5`, `cy = 239.5`, and depth scale 5,000 units per
metre. Zero depth is invalid. Back-projection gives the optical frame with x
right, y down and z forward. This is a documented registered-depth/default
calibration model; separately measured native depth-camera extrinsics, vehicle
extrinsics and a calibration uncertainty budget are not supplied.

The original TUM trajectory stores world-from-camera translations in metres
and quaternions in xyzw order. Its full file was verified byte-identical against
a second public mirror; this verifies transfer rather than physical measurement
accuracy. The original dataset's redistribution terms were not independently
retrieved here. Raw depth, index and trajectory files stay ignored and are not
redistributed. A mirror's code licence cannot replace dataset rights. See the
[source and licence limitations](../data/tum-fr1-xyz/SOURCE.md).

## Frozen evaluation

Preprocessing samples every eight pixels in each axis from pixel (0, 0), at
most 4,800 pixels per image. Valid depths must be 0.3–5.0 m. A deterministic
0.03 m XYZ voxel retains the first valid sampled pixel in row-major order;
voxel keys determine final point order. The previous frame supplies the map
and the current frame the scan. Matcher defaults and identity initialization
are fixed; no motion-capture pose provides an initial guess.

All matches run before pose labels are parsed. Ground truth cannot level clouds,
select points, choose pairs, set correspondence or tune configuration. Natural
roll, pitch and height changes are preserved in the 6-DoF match. To score the
current-camera-to-previous-camera transform, the evaluator composes the inverse
previous world pose with the current world pose.

Ground-truth interpolation uses linear translation and shortest-arc quaternion
SLERP, requires a bracket no wider than 0.02 s and forbids extrapolation. Fixed
accuracy gates are translation error ≤ 0.1 m and rotation error ≤ 0.1 rad.
Every rejection remains in the denominator and full report. Conditional local
ICP covariance excludes map uncertainty, correlated depth errors and association
uncertainty; it is not statistically calibrated uncertainty.

## Reproduce

Run from the repository root:

```sh
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz
mkdir -p artifacts/tum-rgbd
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --manifest data/tum-fr1-xyz/manifest.json \
  --prepare-freeze artifacts/tum-rgbd/retrospective-freeze.json
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --manifest data/tum-fr1-xyz/manifest.json --raw data/tum-fr1-xyz/raw \
  --freeze artifacts/tum-rgbd/retrospective-freeze.json \
  --output artifacts/tum-rgbd/rerun.json
```

Freeze preparation reads only metadata and records compiled matcher/evaluator,
lockfile and manifest hashes, configuration, preprocessing and scoring policy;
it performs no depth decoding, pose parsing or matching. Evaluation requires an
external matching freeze and rejects changed protocol hashes. The original
failure remains known when reproducing it; generating a fresh protocol file
cannot make already inspected data unseen again. Original-data protocols are
explicitly marked `baseline_retrospective`; preparation refuses to overwrite an
existing freeze. Reuse its matching freeze when repeating the same executable.

The optional integration has its own locked PNG decoder and SHA implementation;
these do not add image-decoding dependencies to the core driving workspace.
The evaluator verifies the manifest's input sizes/hashes and records source,
evaluator and lockfile hashes, parameters, point counts, all pair results and
per-pair synchronous CPU wall time. The timing excludes acquisition and is not
a latency distribution or real-time guarantee. A short indoor local-pair result
cannot establish vehicle motion accuracy, robust global initialization, map
consistency, loop closure or long-term drift performance.

## Original frozen results

The original eleven pairs all fail acceptance: six reject insufficient unique
overlap and five exhaust the fixed 20,000,000 distance-check budget. No pair is
accepted, so translation/rotation accuracy and covariance coverage are not
established. The actual preprocessed clouds contain 1,117–2,864 points. The full
[`integrations/rgbd/baselines/tum-first-v1.json`](../integrations/rgbd/baselines/tum-first-v1.json)
report retains every rejection and all eleven
evaluation-only reference transforms. The CLI exits 1 for failed fixed gates,
2 for invalid input and 0 only when all pairs pass.

The archived
[`tum-fast-v1.json`](../integrations/rgbd/baselines/tum-fast-v1.json) records the
separate 120–131 fast-frame experiment: **all eleven pairs reject the fixed
20,000,000 distance-check budget**, including all nine temporal held-out pairs.
The matching original source, evaluator, manifest and external protocol archive
are retained alongside it. A small frame interval alone does not fix bounded
matching. No accepted-pose error claim follows from either failed stage.

This is a measured failure baseline. It is not hidden by the earlier apartment
semi-synthetic accuracy result. The interval between selected frames is about
0.333 seconds, appreciably longer than native Kinect depth intervals. After
inspecting this failure, a later **separate temporal comparison** was declared
before any new fits: original depth-index entries 120–131, twelve consecutive
frames about 0.029–0.036 seconds apart. See
[the new fixed selection](../data/tum-fr1-xyz-fast/SOURCE.md). It shares the same
physical room/sequence and is not independent-environment validation; its result
cannot replace the original failed benchmark. The first fast stage kept matcher
defaults, identity initialization, calibration and point preprocessing unchanged.
Later exact nearest-neighbour search optimization addresses distance work while
preserving geometric matching parameters and the budget. The already viewed
fast interval then becomes **regression data** for that optimized matcher; its
historic `held_out` labels cannot establish fresh generalization again.

The optimized fast regression result in
`artifacts/tum-rgbd-fast-pruned/results.json` accepts **10/11** pairs, all ten
within the unchanged 0.1 m / 0.1 rad accuracy gates; one pair still exhausts the
global distance budget. Accepted translation errors are **0.00122–0.00794 m**
and rotation errors **0.00621–0.01797 rad** against independent interpolated
motion capture. All pairs remain in the denominator, and this partial acceptance
does not turn the original failed fast stage into a pass. The report/protocol
explicitly label this role `calibration_regression`.

The measured optimized regression CPU times are about **0.33–0.80 seconds per
pair**, each a single synchronous matcher wall-clock measurement. They exclude
depth decoding and reference scoring and do not establish real-time operation.

To reproduce the declared shorter-interval comparison independently of the
original failed output, use separate paths:

```sh
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz-fast
mkdir -p artifacts/tum-rgbd-fast
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --manifest data/tum-fr1-xyz-fast/manifest.json \
  --prepare-freeze artifacts/tum-rgbd-fast/freeze.json
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --manifest data/tum-fr1-xyz-fast/manifest.json --raw data/tum-fr1-xyz-fast/raw \
  --freeze artifacts/tum-rgbd-fast/freeze.json \
  --output artifacts/tum-rgbd-fast/results.json
```

After both failures, a further separate interval was declared before new fits:
original indices **140–151**, twelve consecutive frames spanning
1305031106.830652–1305031107.198208 seconds. These were withheld from the optimizer
until its source/configuration freeze. They are **temporal withheld frames of
the same physical sequence**, rather than an independent environment. All eleven
pairs must remain in the report, with the same first-three-frame calibration and
nine-frame temporal partition. [Selection and retained failure history](../data/tum-fr1-xyz-tight/SOURCE.md).

```sh
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz-tight
mkdir -p artifacts/tum-rgbd-tight
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --manifest data/tum-fr1-xyz-tight/manifest.json \
  --prepare-freeze artifacts/tum-rgbd-tight/freeze.json
cargo run --release --locked --manifest-path integrations/rgbd/Cargo.toml -- \
  --manifest data/tum-fr1-xyz-tight/manifest.json --raw data/tum-fr1-xyz-tight/raw \
  --freeze artifacts/tum-rgbd-tight/freeze.json \
  --output artifacts/tum-rgbd-tight/results.json
```

## Later frozen temporal result

[The externally recorded protocol](../assets/recorded-rgbd-tight-freeze.json)
fixes the optimized source, unchanged matcher settings, exact manifest, optical
calibration, preprocessing and scoring before the 140–151 interval is evaluated.
The actual `artifacts/tum-rgbd-tight/results.json` result accepts **7/11** pairs;
all seven are within the unchanged accuracy gates. Four pairs reject: three
exhaust the global distance budget and one has a comparable distinct local fit
and rejects ambiguity. The temporal held-out partition passes **5/9**, not 9/9.
This is partial local-pair success, not a passed protocol or complete localization.

Accepted translation errors are **0.00112–0.00853 m** and rotation errors
**0.00525–0.02104 rad**. Measured matcher CPU time is **0.699–1.327 seconds per
pair**, one synchronous measurement per pair, excluding decoding and scoring.
The actual preprocessed clouds contain 1,306–1,798 points in this interval.
These correlated frames occupy one room and cover about 0.368 seconds;
millimetre errors on accepted local pairs do not establish driving accuracy.

The independent Python checker reconstructs depth geometry from the raw PNGs,
verifies frozen input/source hashes, recomputes time-bounded physical reference
transforms and pose errors, and retains all rejections. It rejects fourteen
mutations, including altered timestamps, ground-truth initialization, omitted
failed pairs, invented estimates/errors and invalid covariance. It does not
independently rerun the ICP optimizer. The completed tight oracle is
`/tmp/recorded-rgbd-tight-oracle.json`.

**Covariance is not calibrated.** Independently reconstructed conditional
normalized estimation error squared (NEES) ranges from **704–6,010** on accepted
viewed-fast regression pairs and **1,589–15,189** on accepted later temporal
pairs. These are far above the ideal six-dimensional Gaussian 95% reference
threshold of about 12.59. The nominal local least-squares covariance substantially
understates measured pose error in these cases. Depth/map correlation, association
error and calibration uncertainty are excluded; there is no uncertainty coverage
or safety-bound claim, and this result is not fused into the driving EKF.

With Pillow installed, independently check the tight report:

```sh
python3 scripts/check-recorded-rgbd.py \
  --report artifacts/tum-rgbd-tight/results.json \
  --freeze artifacts/tum-rgbd-tight/freeze.json \
  --manifest data/tum-fr1-xyz-tight/manifest.json --raw data/tum-fr1-xyz-tight/raw \
  --output artifacts/tum-rgbd-tight/oracle.json
```

The [compact stage history](../assets/recorded-rgbd-results.json) retains the two
failed original stages, viewed regression improvements, partial preregistered
temporal acceptance and conditional uncertainty failures together. Improving
future acceptance or calibrating covariance requires a new recorded freeze and
disclosure of all previously inspected data; these temporal frames must then be
treated as regression data.
