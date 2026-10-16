# Rust projective depth odometry and fusion

The optional `rustdriving-projective-depth` library implements dense
point-to-plane odometry with projective depth fusion. It ports the promising
[Python development variant](projective-depth-diagnostics.md) into an
independent bounded Rust implementation. Its two viewed recordings reproduce
the prototype's **64/179 room and 161/179 desk2** root-accurate updates.
Both still fail complete accuracy. The implementation remains offline research;
it does not feed vehicle localization or controls.

The maintained tracker now has explicit policy configuration and a
[complete-trajectory regression gate](native-depth-regression.md). Native
reprojection, matching, residual and planar-normal changes fail its adoption
criteria; the default retains the original
64/179 and 161/179 outcomes, with exact complete sensor-ledger replay.
The subsequent [XYZ model](projective-vertices.md) preserves measured geometry
through view changes but regresses desk2 continuous accuracy; depth remains the
default representation.

## Algorithm and interface

The standalone [crate](../integrations/projective-depth/Cargo.toml) has a
`Tracker::new(Camera)` / `Tracker::step(timestamp, depth_metres)` API.
The default algorithm accepts optical axial-depth samples, pinhole calibration
and acquisition time. An explicit [photometric mode](projective-rgbd-fusion.md)
also accepts associated measured grayscale images and independent RGB timestamps.
Neither mode accepts reference poses, an external map or simulator state.
The only library dependency is serde for explicit diagnostics;
the CLI additionally uses correctly rounded serde_json input.

Coordinates are x right, y down, z forward, in metres. The initial depth camera
defines one retained root. Returned poses map the current optical camera into
that first camera. They supply neither vehicle extrinsics nor a covariance.

Camera dimensions are multiples of four, width 32–640 and height 32–480.
Focal lengths are finite 20–5,000 pixels; the principal point is inside the
image. The input has exactly width × height finite, nonnegative samples, each
at most 100 m; zero means missing depth. Geometric support uses axial depth
strictly between **0.3 and 5 m**. Initializing requires 200 valid coarse-level
surface normals; initialization fixes an origin rather than measuring a
world pose. Blank or malformed initial data do not establish a pose.

The pyramid uses scales **4, 2, 1** with at most **10, 7, 5** iterations.
Two-by-two reduction rejects missing or discontinuous support; central surface
normals reject depth jumps. Projective matching uses nearest pixel centers
with ties rounded to even, transformed normal agreement above 0.8 and bounded
Euclidean separation. The finest level samples every second row/column.
Each iteration checks at most 76,800 source sites at full camera resolution;
total matching work is bounded by 1,113,600 source sites, plus 76,800 for final
admission. Image/model preparation and fusion are separately linear in image
size. These are operation bounds, not measured deadlines or memory guarantees.

Huber point-to-plane residuals with 0.01 m transition generate a 6×6 normal
system. Diagonal scaling, bounded symmetric Jacobi eigenvalues and a Cholesky
solve reject nonpositive or ill-conditioned systems above condition 1e6.
An increment cannot exceed 0.1 m / 0.15 rad. The returned pose must have at
least 200 final correspondences, final RMS at most 0.03 m, and displacement
from its reference at most 0.5 m. Final RMS is recomputed at the returned pose;
the exploratory prototype used its previous iteration's residual.

Keyframes change after relative translation exceeds 0.1 m or rotation exceeds
0.1 rad. The projective model moves into the accepted camera view while its
root transform is preserved. Rasterization selects nearest positive camera Z;
an exact depth tie keeps the lowest original row-major source index. Compatible
depths within `0.03 + 0.01 * model_depth` metres are averaged with weight capped
at 16. Closer incompatible returns replace old depth; farther ones are retained
as ignored observations. Every selected sample belongs to a reported fusion
category. This is a single-view model, without TSDF, loop closure or global map
optimization.

## Acquisition and failure behavior

Acquisition timestamps must be finite, nonnegative and strictly newer than
every previously observed valid timestamp. A rejected acquisition never renews
the accepted-pose timestamp. Invalid sizes/samples, geometric rejection and
duplicates return no new pose; model and reference remain unchanged. A valid
new timestamp advances the observed clock even when its depth is malformed,
so a sequence of bad measurements cannot hide accepted-pose expiry.

Age beyond **0.20 s** latches loss, with no implicit reset. A rejected fit
cannot commit a partial map, reference replacement or pose. Expiry is checked
when `step` receives a valid new acquisition timestamp. This library has no
wall-clock watchdog and supplies no permission to drive through a silent
sensor; a future live integration must independently check freshness.

The subsequent optional [explicit-clock supervisor](projective-depth-supervision.md)
now provides caller-driven idle ticks, fault holds and post-fault acquisition
qualification. It leaves this original estimator's fitting behavior unchanged;
callers still own the tick clock and a future driving integration.

The synchronous JSON-lines CLI configures one camera then handles at most
200 frame requests. Every input line, including newline, is capped at 8 MiB;
unknown fields, missing configuration, reconfiguration, malformed JSON or
missing final newline exit 2. Successful transport execution exits 0 even when
individual updates are rejected. The recorded evaluator separately exits 1
when complete physical accuracy fails.

## Actual validation

The [design](../assets/projective-depth-rust-v1/design.json) was fixed before
the Rust fits. It retains the same viewed depth indices 100–279, registered-grid
camera model, 0.1 m / 0.1 rad root gates, 20 ms reference bracket and one origin
as the earlier experiments. RGB is not decoded. Source, lockfile, binary,
manifest and raw-file digests are recorded before fitting. All 180 sensor-only
outcomes are saved before numeric reference labels or prototype scores enter
scoring. Every rejection remains in the 179-update denominator.

| Recording | Accepted | Root-accurate | Max position error | Max rotation error |
|---|---:|---:|---:|---:|
| Room | 179 | 64 | 0.296290 m | 0.141286 rad |
| Desk2 | 179 | 161 | 0.114800 m | 0.090386 rad |

The independent NumPy audit checks normal solves, eigenvalue condition,
increments, root composition, reference changes, accepted clocks, expiry and
all root scores. It verifies **6,861** normal steps across both recordings.
The complete first trajectories agree with the archived prototype to within
**1.77e-14 m** in translation; the trace/arccos rotation disagreement is at most
1.34e-7 rad. Every acceptance decision and root-accuracy count agrees.
This audit does not independently reconstruct every raw correspondence matrix.

The subsequent [raw-depth reconstruction audit](projective-depth-raw-audit.md)
now independently rebuilds all recorded correspondences, normal equations and
prescribed fused models from PNG depth. It closes that explicit audit gap while
preserving the original physical accuracy failures.

Four analytic ray/sphere scenes exercise stationary and nonzero motion about
all three axes, under fixed **0.004 m / 0.004 rad** gates. The checker also
rejects ten transport violations and ten deliberate report corruptions.
Six Rust tests verify degeneracy, map retention after failure, initialization,
clock expiry, exact raster ties/occlusion and the fusion weight cap.
CI runs these analytic checks without downloading TUM recordings. Complete
recorded runs and replay are local Linux evidence.
[Validation receipt](../assets/projective-depth-rust-v1/validation.json).

## Reproduction

Use the existing [room](temporal-recorded-motion.md) and
[desk2](independent-recorded-motion.md) acquisition instructions and pinned
`scripts/requirements-visual.txt` environment. No OpenCV is needed.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/projective-depth/local-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/projective-depth/local-recorded
```

The latter command retains results and exits **1** for these accuracy failures.
Select fresh output directories. Archived first/final sources and compressed
outcomes contain no redistributed raw depth or reference files.

The implementation improves executable Rust capability, while measured
accuracy remains the prototype's known failure. It supplies no independent
new environment, measured vehicle calibration, calibrated uncertainty,
automotive generalization or driving integration. The subjective maturity
estimate remains **about 20%**, with the **30% waypoint unmet**.
