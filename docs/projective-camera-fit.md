# Raw camera correction probes and scale ambiguity

New offline tools fit and independently audit a shared focal/depth correction
before considering a camera change in the tracker. They also repair a discovered
scale ambiguity in the first fitting objective. **Neither attempted correction
passes trial eligibility, so no camera profile is exported or applied.** Native
Rust code, its compiled tracker and all operational defaults remain unchanged.

The [fixed design](../assets/projective-camera-fit-v1/design.json),
[normalization correction](../assets/projective-camera-fit-v1/normalized-design.json)
and [source-bound packet](../assets/projective-camera-fit-v1/summary.json) preserve
both complete failures. This study supplies conditional calibration diagnostics,
without a physical camera calibration certificate or an accuracy increase.

## Fixed data and model

The original [registered-grid model](registered-grid-camera.md) remains the
operational baseline: K=525, center 319.5/239.5, axial PNG depth divided by 5,000,
and no further manufacturer scale factor or distortion warp. The earlier
published-K-only comparison already fails the
[continuous adoption gate](multireference-rgbd.md). This experiment estimates
two shared perturbations rather than selecting a different published camera.

Parameters `a` and `b` multiply both focal lengths by `exp(a)` and nominal depth
by `exp(b)`. Principal point, focal aspect ratio, distortion, depth offset and
all source/reference timestamps stay fixed. Bounds are ±0.05 in log units.
With `A=diag(exp(-a),exp(-a),1)`, nominal points become `exp(b)*A*p`.
Target normals follow the normalized inverse transpose of that transformation;
the derivative includes this moving normal.

Training uses fifteen room and fifteen desk2 pairs at original window offsets
`(0,6), (12,18), …, (168,174)`, approximately 0.2 s apart. Validation uses fifteen
pairs per recording at `(3,9), …, (171,177)`, including RPY. Its frame endpoints
are disjoint from training endpoints. All recordings are already viewed FR1
data, with temporal correlation and the same camera; this is not a fresh
held-out or automotive result.

Correspondences use the original fine-scale projection, normal and distance
gates at original K and offline reference motion. Every eighth admitted pair
is retained in row-major order, with at least 200 samples in each fixed pair.
Pixel identities, raw points, base normals and reference motions then stay
frozen during fitting. Each pair has equal weight, and the two training records
have equal weight. Reference labels enter these offline probes only.

## The first failure and its correction

The first objective minimizes physical point-to-plane Huber loss with a 10 mm
cutoff. When reference translation is zero, multiplying all depths by `s`
also multiplies residual noise by `s`. The objective can prefer smaller depths
even though that motion supplies no metric scale baseline.

A fixed control with known pure rotations and independent 3 mm noise on all
three point coordinates reproduces the problem. The first numerical information
test reports condition 160.863 and accepts apparent scale sensitivity. Reducing
the log scale by 0.01 lowers cost from 8.466226e-6 to 8.300992e-6 m²; fitting reaches
the lower scale bound. This is a constructed gauge counterexample, not a camera
noise calibration. The first probe receipt's `axial_coordinate` field names
only part of the injected three-coordinate noise; its source and original
receipt are retained.

The corrected objective expresses the complete robust residual in original
nominal raw metres: `r_fit = r_physical / exp(b)`. Its focal derivative is the
physical derivative divided by `exp(b)`, and its scale derivative simplifies to
`-n(a)·t_reference / exp(b)`. It is exactly zero for zero reference translation,
including noisy points. The corrected control rejects scale estimation and
keeps the objective invariant under a uniform scale change.

This repairs the demonstrated scale ambiguity. It remains a provisional
normalization and a conditional sensitivity test. Focal estimation, surface
association, measured-normal errors, image registration, reference timing and
unmodeled noise can still affect the fitted parameters. No covariance or
physical identifiability certificate is produced.

## Complete outcomes and the application gate

Both versions fit room, desk2 and one joint correction, plus all thirty
leave-one-pair-out fits. The iteration cap of 12, bounds, step cap, line search,
pair identities, validation metrics and eligibility thresholds stay fixed.
All **75 derived sample caches reproduce byte-for-byte** between versions.

| Joint objective | Focal multiplier | Depth multiplier | Joint status | Eligible for trajectory trial |
|---|---:|---:|---|---|
| First physical-distance loss | 1.005325 | 0.951229 | Lower depth bound | No |
| Corrected raw-unit loss | 1.017104 | 1.014614 | Converged | No |

The first separate room fit reaches its bound. Corrected room fitting reaches
the work cap, and twelve of its fifteen leave-one-out folds also reach the cap.
The corrected room/desk2 parameter difference is 0.006467 in maximum log units,
within the predeclared 0.01 agreement threshold. Maximum leave-one-out deviations
are 0.017976 and 0.019616, exceeding the same 0.01 stability threshold. The cap is
not increased after viewing these outcomes.

Validation below reports physical raw RMS in millimetres, with identical
frozen samples. The original physical-metre metrics and reference clocks remain
the application gate even after training normalization changes.

| Viewed recording | Original RMS | First correction RMS | Corrected objective RMS |
|---|---:|---:|---:|
| Room | 17.1001 | 16.8016 | 17.6077 |
| Desk2 | 16.2230 | 15.1756 | 16.4276 |
| RPY | 13.6113 | 12.7401 | 14.0089 |

Lower RMS from the first correction also fails to improve all reference-pose
corrections. The next table shows median translation / rotation magnitudes
in millimetres / milliradians. They are single linearized raw-pair probes,
not estimated root trajectories.

| Viewed recording | Original | First correction | Corrected objective |
|---|---|---|---|
| Room | 10.239 / 7.423 | 11.572 / 8.478 | 10.889 / 8.325 |
| Desk2 | 11.626 / 12.007 | 10.275 / 12.407 | 11.219 / 11.846 |
| RPY | 8.439 / 5.269 | 8.944 / 5.749 | 9.531 / 5.895 |

Eligibility requires every full and leave-one-out fit to converge inside the
bounds, parameter agreement and stability within 0.01 log units, and nonincreasing
validation robust cost, RMS and median pose corrections in all three records.
Both versions fail. No `trial-profile.json` is written, no new operational
trajectory is run, and the original three-recording adoption gates remain.
The result does not rule out calibration with a richer model or independent
measurements; it rejects applying these fitted corrections under this protocol.

## Independent verification and reproduction

The maintained checker constructs transformed normals from triangle cross
products, independently of the fitter's dual-normal formula. Central finite
differences check focal and scale derivatives, absolute H/g and line-search
costs. Two known synthetic corrections are recovered, including a complete
JSON round trip through the fit-trace auditor. Unexcited motion, noiseless and
noisy pure-rotation scale gauges reject; joint H/g rescaling is detected.

For recorded checks, the auditor decodes original PNGs, independently rebuilds
central normals and the complete original match population, verifies frozen
pixel identities and all caches, and checks every fit and fold. The two attempts
contain **390 and 406** fitted/final calibration systems, respectively, and
**90 physical pose systems each** for validation. All pass independent
reconstruction while both eligibility decisions remain false.

The first full audit exposed an `abs(JSON list)` handling bug. It is repaired
with array conversion and serialized-trace controls; the original failed log,
diagnostic source and repaired audit are retained. A later extra closing
parenthesis was caught before the normalized process read data. That failed
source epoch and syntax log are retained separately from the actual normalized
run. Neither issue changes the preserved numeric first trial.

The root suite passes 371 Rust tests and 49 scenario replays, retaining all 245
deterministic hashes. The unchanged optional tracker passes 59 Rust tests:
**430 Rust tests** in total. Native source, Cargo pins and the release-binary
hash match the preceding visibility packet. CI includes dataset-free camera
math controls; recorded audits were run locally on this Linux host.

Use the [existing inputs and environment](projective-depth-rust.md#reproduction)
and fresh output paths:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-camera.py \
  --output artifacts/camera-controls
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-projective-camera.py \
  --output artifacts/camera-diagnostic
# Expected exit 1 for retained calibration eligibility failures; continue auditing.
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-camera.py \
  --reports artifacts/camera-diagnostic --output artifacts/camera-audit
```

`--source-archive` lets the independent checker bind historical diagnostic source
pins to the appropriate retained archive while using a separately frozen
auditor. The default objective for a historical report without an objective
field is the original physical-distance loss. The packet includes all source
epochs and derived reports; original recordings, sparse point caches and
compiled binaries are not redistributed. Subjective maturity versus mature
driving OSS remains **about 20%**, with the **30% waypoint unmet**.
