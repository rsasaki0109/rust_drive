# Measured RGB-D pixel reprojection refinement

The optional `--visual-reprojection` evaluator adds a Rust-native pose refinement
stage to measured image-feature odometry. It uses the original FAST features,
mutual descriptor matches, registered depth checks and robust 3D consensus.
The original `--visual` evaluator remains available with its numerical baselines
and immutable first trial unchanged.

The refinement projects previous-camera measured landmarks into the current
camera and minimizes image residuals with a vector-norm Huber loss. It optimizes
`current_from_previous` internally and returns `previous_from_current` for root
composition. An analytic projection Jacobian, bounded six-variable normal solve
and monotonic line search avoid building a custom optimizer runtime. A maximum
of 256 original consensus inliers and eight iterations bound the operation.
Invalid geometry, ill-conditioned solves and failed line searches reject the
update. Rejection preserves the accepted reference and clock and produces no
current pose. The observed RGB clock still advances, so a repeated image cannot
retry a failed fit or renew localization permission.

The method reduces the influence of noisy current depth on pose refinement;
previous depth and the original 3D consensus still affect the result. It does not
establish scale invariance, calibrated covariance, global relocalization, loop
closure, vehicle calibration or driving integration. Reaching the iteration cap
is reported explicitly and does not certify optimizer stationarity. Camera
projection uses the documented pinhole profile without additional undistortion
or RGB/depth time compensation.

## Diagnosis and evidence roles

The viewed office diagnostic found much larger true residuals along depth than
laterally. Its pair translation and rotation errors were strongly coupled:
changing orientation about a distant feature centroid can compensate depth
residuals with translation. This is evidence for testing pixel refinement, not
proof that sensor noise is the sole cause. Calibration, depth registration and
integer feature localization remain possible contributors.

All desk, office and sitting RGB-D selections used for the earlier visual
evaluator are now viewed data. Repeating them with the new algorithm supplies
development/regression evidence. Every selected update, including repeated
images and failed fits, remains in the denominator; motion-capture labels are
parsed only after all operational estimates finish.

| Recording | Role | Accepted / 35 | Accurate / 35 | Maximum accepted root error | Physical exit |
| --- | --- | ---: | ---: | --- | ---: |
| Freiburg 1 desk | Viewed development | 24 | 24 | 0.041360 m / 0.051885 rad | 1 |
| Freiburg 3 office | Viewed development | 32 | 32 | 0.042368 m / 0.015501 rad | 1 |
| Freiburg 3 sitting | Viewed development | 31 | 31 | 0.034400 m / 0.014818 rad | 1 |
| Freiburg 2 desk | Viewed after first input failure | 24 | Unscorable | Unavailable: invalid labels | 2 |

The office result improves from nine accurate updates and 0.155641 m maximum
accepted position error under the original 3D-only estimator. Every accepted
refinement in these three development runs converged under the fixed defaults.
The original three-dimensional consensus and repeated-image rejection schedules
are unchanged; all three full accuracy/availability protocols still fail.

The additional Freiburg 2 desk selection fixes original depth indices 100–135
using only availability and timestamps. Its published profile differs from
Freiburg 1/3 and uses **5208 depth units per metre**, exactly as recorded in the
pinned calibration document and crosschecked published calibration. This is a
source-model assumption, not independently measured sensor calibration; RGB
distortion is documented but not additionally corrected. The interval has 36
depth observations and 25 unique RGB images. Its 11 repeated images remain
selected; completeness cannot be established by deleting them.

The first frozen Freiburg 2 attempt **failed input validation, exit 2**. Its
complete motion-capture file contains two unequal poses at the same timestamp,
1311868229.576, on physical lines 10862–10863. The duplicate lies after the
selected interval, but the frozen contract validates the whole source. No row
was deleted, deduplicated or normalized. All sensor fits had finished before
the label reader rejected the file; that original adapter wrote no report.
The exact first source archive, freeze, before/after journals and error log are
retained in [the first-trial archive](../assets/recorded-reprojection/first-fr2-desk-v1/phase1.json).
There is no first-trial accuracy result or recoverable first sensor trace.
The regression suite rebuilds the exact archived sources in a temporary source
directory, reproduces the original freeze byte for byte, and verifies exit 2,
the identical error log and absence of a results file. Rebuilt binary identity
is reported separately from the original executable identity.

The subsequent **viewed protocol 3** records sensor traces even when the strict
evaluation-only label reader fails. It retains exit 2 and an explicit
`evaluation_label_failure`; every accuracy reference is unscorable, with zero
valid or accurate updates. Invalid labels cannot supply poses or renew tracking.
This reporting correction does not modify the estimator, calibration, fitting
gates or full-source label validation. The Freiburg 2 recording is now viewed
and requires `--regression`; it cannot become a fresh trial again.

The evaluation-only label reader has explicit limits of 4 MiB and 30,000 rows,
with finite values, unit-quaternion validation and strictly increasing times.
The source has 20,957 rows and 1,420,075 bytes. These capacity limits were fixed
before the first evaluation and do not exempt its duplicate timestamp.

The independent checker reconstructs measured image features, descriptors,
matches, depth correspondences and coarse consensus with the immutable visual
oracle. It independently computes pixel refinement with numerical projection
Jacobians and SVD, then checks costs, diagnostics, rejection state, root
composition and motion-capture scores. Integrity success does not turn a failed
physical protocol into a passed one.

For the failed first trial, a separate input-failure audit verifies the original
20 archived sources, manifest, all acquired file hashes, strict label rejection
and original error log. It does not reconstruct or invent first-trial sensor
results. The current checker separately reconstructs subsequent viewed traces.

## Reproduction

```sh
python3 -m pip install -r scripts/requirements-visual.txt
python3 scripts/fetch-visual-datasets.py tum-fr1-desk-visual
python3 scripts/fetch-visual-datasets.py tum-fr3-office-visual
python3 scripts/fetch-visual-datasets.py tum-fr3-sitting-visual
python3 scripts/fetch-reprojection-dataset.py tum-fr2-desk-reprojection
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml
python3 scripts/check-recorded-reprojection-suite.py \
  --binary integrations/rgbd/target/release/rustdriving-rgbd-evaluate \
  --output artifacts/recorded-reprojection
```

The suite checks source/configuration freezes and all non-timing numerical
results. It exits zero for intact reproduction, while each evaluator keeps its
physical exit status or invalid-label exit 2. The archived-source replay builds
offline using the dependencies cached by the preceding locked build. Use the
binary path selected by `CARGO_TARGET_DIR` when
that environment variable is configured. Raw RGB, depth and motion-capture
files remain ignored and are not redistributed or relicensed.
