# Native depth policies and continuous regression gate

**No accuracy improvement is adopted.** Four changes were implemented in the
maintained Rust tracker and evaluated over both complete 179-update recordings.
None meets the acceptance rule fixed before fitting. The default remains point
reprojection, forward nearest-pixel matching and target-plane residuals, with
**64/179 room and 161/179 desk2** root-accurate updates.

This work adds configurable native implementations, raw reconstruction and a
reusable adoption checker. It does not resolve sustained localization accuracy
or add a driving capability. Earlier local-window convergence results are not
used as substitutes for these complete physical trajectories.

## Complete outcomes

The separate [immutable measured-map baseline](projective-fixed-map.md) removes
all rolling fusion and reference changes. Its complete viewed room/desk2/RPY
results are 24/20/17 accurate updates out of 179 each, followed by latched loss.
Independent raw equations match, but availability and RPY accuracy fail.
It remains a separate experimental API/binary and does not replace the default.

The later [raw camera correction study](projective-camera-fit.md) tests focal
and depth-scale perturbations before running another trajectory comparison.
Both attempts fail the frozen application gate; their raw audits pass, and
the native camera, fitting policies and prior outcomes remain unchanged.

The later [measured free-space update](projective-visibility.md) removes a
controlled disappearing foreground but scores67/161/74 on the same viewed
room/desk2/RPY windows. Maximum position error increases in room and desk2,
so the original adoption rule keeps this option disabled.

The later [visible-surface fusion study](projective-fusion.md) separately tests
nearest-Z selection bias. Its opt-in mean raster scores67/161/74 on the viewed
room/desk2/RPY windows and increases maximum position errors in all three.
It fails the same adoption rule and leaves the original default unchanged.

All configurations accept all 179 updates in each recording without expiry or
resets. Every complete accuracy protocol still fails. Counts use the original
**0.1 m / 0.1 rad** first-camera root gates and include all rejected or inaccurate
updates in the denominator. Values below compare the same viewed depth indices
100–279, calibration and acquisition timestamps.

| Native configuration | Room accurate / 179 | Desk2 accurate / 179 | Max position error, room / desk2 (m) | Max rotation error, room / desk2 (rad) |
|---|---:|---:|---|---|
| Original default | 64 | 161 | 0.296290 / 0.114800 | 0.141286 / 0.090386 |
| Tangent-plane model reprojection | 64 | 162 | 0.299407 / 0.113487 | 0.142044 / 0.090094 |
| Reciprocal pixel matching | 60 | 161 | 0.296900 / 0.115724 | 0.141722 / 0.093232 |
| Symmetric measured-plane residuals | 62 | 79 | 0.297918 / 0.204397 | 0.145455 / 0.338936 |
| Bilinear target sampling | 64 | 159 | 0.295584 / 0.126713 | 0.140018 / 0.090489 |

Tangent reprojection slightly improves desk2 while worsening room's maximum
errors. Reciprocal matching worsens room and maximum errors. Symmetric residuals
substantially worsen desk2. Bilinear sampling reduces room's maximum errors but
loses two desk2 accurate updates. No candidate improves room's accurate count.
The baseline and every attempted result remain preserved.

The [retained comparison](../assets/projective-reprojection-v1/summary.json)
and [validation receipt](../assets/projective-reprojection-v1/validation.json)
link the complete source-bound results.

The adoption rule requires room accuracy above 64, desk2 at least 161, all 179
updates accepted in each without loss, and no worse maximum position or rotation
error in either recording. Mathematical checks and default-interface regression
must also pass. Full 179/179 physical accuracy is reported separately; a passing
relative adoption gate would not establish that stronger requirement.

## Rust API and transport

The existing `Tracker::new(camera)` and camera-only JSON `Configure` keep their
original behavior. The current default binary reproduces **all 360 original
sensor outcomes exactly**, including poses, equations, fusion and clocks.

`Tracker::with_config(camera, TrackerConfig)` explicitly selects experimental
policies. The default configuration is equivalent to:

```rust
TrackerConfig {
    reprojection: Reprojection::Point,
    correspondence: Correspondence::Forward,
    residual: Residual::TargetPlane,
    normals: NormalEstimation::CentralDifference,
    geometry: ModelGeometry::Depth,
    photometric: Photometric::Disabled,
    step_control: StepPolicy::Disabled,
    motion_model: MotionModel::Stationary,
    timing: Timing::Disabled,
}
```

The JSON-lines `Configure` request accepts optional `reprojection`,
`correspondence`, `residual`, `normals`, `geometry`, `photometric`, `step_control`, `motion_model` and `timing` fields. Missing fields use
the original defaults; unknown fields and enum values reject. The version-1
acknowledgment, frame limits, optical coordinate convention and metre/second
units are unchanged. There are no new dependencies or Cargo lockfile changes.

* `reprojection`: `point` or `tangent_plane`. The latter intersects a rounded
  destination ray with the transformed measured tangent plane during fusion
  and model-view warps. Missing normals, grazing rays, out-of-range depths or
  corrections outside the existing fusion band retain the original point
  splat. `tangent_adjusted` counts adjusted projected samples when nonzero;
  it is not a count of unique target pixels or additional measurements.
* `correspondence`: `forward`, `reciprocal` or `bilinear`. Reciprocal matching
  requires a selected target to project back to the original sampled source
  pixel. Bilinear sampling requires four valid depths/normals, bounded depth
  spread and a nondegenerate blended normal; missing corners and discontinuities
  reject without filling or nearest-pixel fallback.
* `residual`: `target_plane` or `symmetric_planes`. Symmetric fitting averages
  two Huber losses. The source-plane Jacobian includes rotation of its normal,
  giving a target-point lever arm. Its RMS averages the two squared residuals
  per correspondence, which is counted once. This option is not recommended
  by the recorded comparison.
* `normals`: `central_difference` or `inverse_depth_plane`. The latter uses
  a uniform 5×5 inverse-depth plane at native resolution, with complete measured
  support and axial-depth fit RMS at most 0.002 m. Unsupported or curved patches
  retain their original central normals; scales 2 and 4 always retain central
  normals. This option also fails recorded accuracy and remains experimental.
* `geometry`: `depth` or `vertices`. The latter retains full XYZ in model cells,
  including fusion, view warps and source/model pyramids. It requires the other
  default policies and rejects unsupported combinations before acknowledgment.
  Its [geometric preservation control and failed continuous comparison](projective-vertices.md)
  retain all outcomes; it is not promoted to the default.
* `photometric`: `disabled` or `keyframe`. The latter requires associated measured
  grayscale images and the original depth/fitting policies. It uses fresh,
  phase-qualified intensity constraints against a measured reference image;
  repeated RGB is ignored while new admissible depth remains usable. The
  [fused RGB-D implementation and complete failures](projective-rgbd-fusion.md)
  and the maintained [exposure correction](projective-rgbd-exposure.md)
  are retained; depth-only tracking remains the default.
* `step_control`: `disabled` or `fixed_support`; the latter requires `keyframe`
  photometric mode. It backtracks on fixed measurements and weights, with every
  trial reported. Its [complete regression and independent checks](projective-step-control.md)
  retain the failed 83/179 room and 67/179 desk2 result; it is explicitly opt-in.
* `motion_model`: `stationary` or `constant_velocity`. The latter uses the last
  two accepted depth poses and actual acquisition intervals for SE(3) prediction.
  Its [controls and complete comparison](projective-motion.md) retain mixed
  accuracy outcomes; stationary remains the default for both depth and RGB-D.

* `timing`: `disabled` or `previous_motion`; the latter requires `keyframe`,
  `constant_velocity` and disabled step control. It projects each measured image
  at its own acquisition time using causal accepted-pose motion, and reports the
  source/reference time warps. Its [controls and complete comparison](projective-timing.md)
  give 80/179 room and 171/179 desk2, but maximum rotation error still fails adoption.

Only the individual configurations documented here have recorded trajectory
results. Their combinations have no physical accuracy qualification. All modes
retain bounded pyramid work, normal/distance/support/condition/increment guards,
accepted acquisition clocks, atomic failure behavior and latched expiry.
They remain offline research, outside vehicle localization and controls.

## Reproduction and automatic rejection of regressions

Use the original [room/desk2 inputs and tool environment](projective-depth-rust.md#reproduction).
Select fresh output directories. For example, compare the baseline with bilinear
target sampling:

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/native-baseline
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --correspondence bilinear --output artifacts/native-bilinear
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --correspondence bilinear --output artifacts/native-bilinear-analytic
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --analytic-reports artifacts/native-bilinear-analytic --correspondence bilinear \
  --recorded-reports artifacts/native-bilinear --output artifacts/native-bilinear-raw
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth-comparison.py \
  --baseline artifacts/native-baseline --candidate artifacts/native-bilinear \
  --raw-audit artifacts/native-bilinear-raw --output artifacts/native-bilinear-gate.json
```

Both evaluators exit **1** after saving these complete physical failures. The
comparison checker also saves its result and exits **1** when adoption fails;
this differs from a transport or mathematical-audit error. Run these commands
separately rather than chaining later checks on a quality exit code of zero.

The comparison checker binds the baseline to the original archived native sensor
ledger, checks identical inputs/calibration and no resets, verifies the scored
updates against the pre-label ledger, and recomputes all root classifications.
It uses an `atan2` rotation check; reference interpolation is shared with the
existing qualified reference helper. It also binds the raw audit to the exact
candidate freeze and sensor-ledger hashes. Changed counts, poses, gates or
unrelated audit files cannot supply a passing result.

## Validation and limits

All four candidates retain their sources and full first outcomes. The original
source and binary were saved before editing the maintained Rust component.
Historical source-bound experiments retain their original archives and reports;
replaying those versions requires their archived sources rather than replacing
their recorded source hashes with hashes of the updated component.
Numeric labels enter scoring only after every operational outcome in a recording
has been written. Hash-qualified PNGs, axial-Z `/5000` conversion, first optical
origin and acquisition timestamps remain unchanged. These are viewed indoor
development records, without held-out or automotive generalization.

Independent raw reconstruction checks **28,036 fitted equations plus 1,432
returned-pose equations**, rebuilding all 1,432 prescribed candidate model
generations. Counts and fusion categories agree exactly. The four full comparisons
reuse the same observations. They do not represent four independent acquisitions.

The four-policy packet records **23 tests**, including a transformed-plane
witness, missing/grazing-normal fallback, an incorrect reverse-pixel witness,
all six source-normal Jacobian axes and missing bilinear support. Each policy
also passes four known-motion checks, malformed-input/clock/expiry checks and
report-corruption controls. CI runs the analytic checks without downloading
recordings. The existing supervisor's analytic checks pass.

Core checks pass **371 Rust tests and 49 scenarios with sensor-log replays**.
No driving pipeline or vehicle integration was changed. The original accuracy
failure remains the open problem; subjective maturity stays **about 20%** versus
mature driving OSS. Test counts alone do not justify increasing it.

## Relative axial-depth weights and standardized robust loss

The [distance-stratified study](projective-range.md) observes increasing residual
dispersion in all four axial-depth bins on three viewed recordings. Its frozen
relative-variance mode scores 79/74/115 accurate room/desk2/RPY updates; the
separately frozen standardized-Huber revision scores 77/75/107. Both retain all
updates but regress desk2 and fail the fixed adoption rule. Original raw-RMS,
support, conditioning, fusion and accepted-clock gates remain, and the default
continues scoring 64/161/74. These improvements in two recordings do not supply
camera/noise calibration or justify increasing maturity beyond about 20%.

## Midpoint surfaces and bidirectional association

The later [surface comparison](projective-surface.md) evaluates all three viewed
recordings with the original indices, root gates and update denominator. Midpoint
normals score 66/86/75 accurate room/desk2/RPY updates; searching correspondences
in both directions scores 64/159/75. The original default remains 64/161/74.
Both candidates fail adoption. The native implementation, exact moving-normal
derivative, separate-direction support and independent reverse-equation transport
remain reproducible optional research, without a maturity increase.

## Normal-noise diagnosis and rejected native change

The subsequent native normal-estimation experiment also makes **no default
change**. On 12 evenly spaced frames from each already viewed recording,
adjacent central normals differ by a median 0.310 rad in room and 0.348 rad in
desk2, measured on complete 5×5 patches whose inverse-depth plane fits within
2 mm. The plane estimates differ by 0.078 and 0.081 rad respectively. Selection
uses that same fitted patch, so this is a planar-support noise diagnostic,
not independent normal truth or proof of the cause of trajectory drift.

For a plane, inverse axial depth is linear in pixel coordinates:
`1/Z = a*du + b*dv + c`. The camera normal is proportional to
`[fx*a, fy*b, c-(u-cx)*a-(v-cy)*b]`. The Rust implementation fits all 25 measured
values without changing depth vertices, fusion, correspondence policy, Huber
weights, iteration budgets, clocks or the fixed physical accuracy gates.

| Normal estimation | Room accurate / 179 | Desk2 accurate / 179 | Max position, room / desk2 (m) | Max rotation, room / desk2 (rad) |
|---|---:|---:|---|---|
| Original central differences | 64 | 161 | 0.296290 / 0.114800 | 0.141286 / 0.090386 |
| First: strict 5×5 support at all scales | 59 | 77 | 0.302753 / 0.138466 | 0.143199 / 0.376426 |
| Revised: native-resolution planar support, central fallback | 63 | 85 | 0.299755 / 0.184243 | 0.142434 / 0.322477 |

Both attempts accept all updates without expiry, but fail the unchanged adoption
rule. The first also fails two of four existing known-motion controls because
strict wide support removes small curved geometry at coarse levels. Its source,
complete physical outcomes and failed controls are retained; it has **no complete
raw-equation audit**. The maintained revision preserves coarse and curved-surface
support and passes all four controls. It still substantially regresses desk2.
Smoother planar normals did not establish better continuous localization.

The [normal experiment packet](../assets/projective-normal-v1/summary.json)
and [validation receipt](../assets/projective-normal-v1/validation.json) retain
both attempts and sources. The revised raw audit reconstructs every fitted and
returned-pose equation and all 358 prescribed fusion generations without parsing
pose truth. Camera-only default input reproduces all 360 original outcomes.

The current optional crate passes **27 tests**, adding exact tilted-plane,
known-noise, unsupported-patch and coarse/curved-support checks. The independent
oracle uses a NumPy pseudo-inverse, with direct plane/support controls. CI runs
the new known-motion, transport and raw analytic checks without downloading
recordings. To reproduce the maintained policy, add
`--normals inverse_depth_plane` to the evaluator, analytic checker and raw auditor
commands above. Use fresh paths and pass the new recorded reports to the auditor
and adoption checker. Physical evaluation and the adoption checker save these
failures and exit 1. These are viewed indoor experiments, not held-out or vehicle
evidence; maturity remains about 20%.
