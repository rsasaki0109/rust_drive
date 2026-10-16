# Measured surface residuals and bidirectional projective matching

Two optional native depth-fitting modes distinguish symmetric plane residuals
from actually searching correspondences in both directions. Neither meets the
frozen adoption rule, so the original target-plane tracker remains the default.
This is offline RGB-D research, with no driving or calibrated-confidence claim.

## Complete viewed comparisons

All three recordings retain their original depth indices 100–279, one first
optical-camera origin, 179 update denominator, K525, axial metres and 0.1 m /
0.1 rad root-error gates. Every mode accepts all 179 updates without expiry or
reset. Numeric mocap labels enter trajectory scoring after the recording's
complete sensor-only ledger is saved. RPY was viewed in the preceding
[information-guard trial](projective-information.md); it is regression data here.

| Mode | Room / desk2 / RPY accurate updates, each out of 179 |
|---|---|
| Original target plane | 64 / 161 / 74 |
| Symmetric midpoint | 66 / 86 / 75 |
| Bidirectional midpoint | 64 / 159 / 75 |

| Mode and recording | Maximum position error (m) | Maximum rotation error (rad) |
|---|---:|---:|
| Original, room | 0.296290 | 0.141286 |
| Midpoint, room | 0.297634 | 0.142046 |
| Bidirectional, room | 0.300067 | 0.143001 |
| Original, desk2 | 0.114800 | 0.090386 |
| Midpoint, desk2 | 0.171308 | 0.316872 |
| Bidirectional, desk2 | 0.124217 | 0.088299 |
| Original, RPY | 0.087348 | 0.173375 |
| Midpoint, RPY | 0.088627 | 0.155312 |
| Bidirectional, RPY | 0.088044 | 0.155537 |

The rule was fixed before each candidate: room must exceed 64 accurate updates,
desk2 retain at least 161, RPY retain at least 74, and all three retain every
update without worse maximum position or rotation error. Both candidates fail;
both complete physical protocols exit 1 after saving their results. Reduced RPY
rotation error does not compensate for regressions in another recording. No
new recording is used to bypass these failed development gates. Maturity stays
**about 20%** versus mature driving OSS, with the 30% waypoint unmet.

## Projection and local objective diagnosis

The official TUM file-format page recommends the ROS default model without
undistorting the pre-registered depth. Its uncalibrated K525 interpretation and
the previously failed published-intrinsics replacement are retained; no new
depth multiplier, intrinsic fit or distortion correction is introduced.
[Camera contract](registered-grid-camera.md); [previous comparison](multireference-rgbd.md).

The maintained `diagnose-projective-surface-bias.py` probes 15 fixed consecutive
raw-depth pairs in each already viewed recording, in both directions and at
all three scales. Mocap supplies only the offline probe pose, never a tracker
initializer or measurement. At native resolution the original objective's
median one-step rotation correction away from the reference pose is 3.545 mrad
in room, 3.948 mrad in desk2 and 3.096 mrad in RPY. Midpoint gives 3.523, 4.200 and
3.158 mrad respectively. Thus averaging measured normals does not consistently
remove the local mismatch. These sparse probes do not identify a unique cause,
calibrate the camera or establish a complete trajectory result.

## Residual and derivative

For source point `p`, fitted source-to-target transform `(R,t)`, target point
`v`, target normal `nt`, and measured source normal `ns`, let `q=R*p+t` and
`n=R*ns`. The new signed residual is

```
r = (nt+n) dot (q-v) / 2
J = [(nt+n)/2, (q cross nt + v cross n)/2]
```

The source normal rotates with the fitted pose; freezing it gives the wrong
rotation derivative. The mean normal is not renormalized. The factor one half
retains the ordinary signed-plane scale when the normals agree. The existing
10 mm Huber rule applies to this one residual. For the unrobust squared losses,
the old average of two squared plane residuals additionally includes
`(rt-rs)^2/4`; averaging before squaring removes that difference penalty. This
algebra is not a proof of smaller physical error, and robust tails differ too.

`symmetric_midpoint` uses the existing current-to-model projective matches.
`bidirectional_midpoint` also projects supported model vertices into the current
depth image and applies the original distance/normal/border gates there.
The reverse-selected fixed pairs contribute the same residual and target-frame
derivative above. These are separate association sets, without a reciprocal
matching requirement. Each contributes full H/g and squared residuals; reported
RMS divides by their total count. Shared points and fused-history correlations
are not independent measurements for covariance purposes.

Every bidirectional normal equation reports `direction_counts: [forward,reverse]`.
Both directions require at least 200 samples before every solve and at the
returned pose; a large total cannot substitute for an unsupported direction.
All existing work caps, physical information, solver, increment, returned-RMS,
fusion, keyframe and accepted-clock rules remain. Rejection cannot commit state.

## Interface and independent validation

Select `TrackerConfig.residual` / JSON `Configure.residual` as
`symmetric_midpoint` or `bidirectional_midpoint`. Both require the original
depth/point/forward/central-difference policies, without photometric tracking.
Unsupported combinations reject before acknowledging configuration. Old modes
omit the optional directional counts and retain their successful output schema.

Rust controls check all six derivatives by finite differences at a nonzero
pose, fixed-pair endpoint reversal, the difference-penalty identity and
per-direction support admission. The raw NumPy auditor computes reverse-frame
normal equations and transports them with `-Ad(T^-1)`; the native implementation
instead accumulates direct target-frame derivatives. All fitted and returned
equations and model generations are reconstructed from original depth PNGs,
without numeric reference-pose decoding. Altered/missing directional counts and
substituting a one-direction objective must fail the raw check.

An unequal-visibility ray-cast control checks reversed scalar objectives,
swapped directional counts and transported H/g at all three scales. Its direction
counts are 651/636, 3421/3391 and 3925/3816. These checks validate implementation
and frame conventions. Native flat-plane controls independently reconstruct both
weak-information failure witnesses and verify that rejection cannot renew the
accepted clock or bypass latched expiry. Summing two directions does not create
missing physical constraints. The physical results above still fail. No runtime noise,
probability calibration, automotive qualification or new dependency is added.

Sources, the two frozen designs, complete ledgers and failures are preserved in
[the surface evidence packet](../assets/projective-surface-v1/summary.json).
Historical evidence and the first midpoint-only source remain separate.

All 420 Rust tests (49 optional-native and 371 core), Clippy, formatting and 49
core scenario/replays pass. The 245 core outputs remain unchanged. Replaying the
original depth, affine RGB-D and motion/time-compensated configurations reproduces
all 1,440 historical sensor outcomes exactly across eight complete runs, without
numeric pose-label reads. Across the six candidate runs, independent raw checks
reconstruct 21,088 fitted/returned normal systems and 1,074 model generations.
The mathematical and compatibility checks pass while all six complete physical
accuracy protocols fail. CI exercises the synthetic modes and negative controls
without downloading recorded datasets.

## Reproduction

With the existing pinned source datasets and NumPy/Pillow environment:

```bash
source scripts/env.sh
cargo test --all-targets --locked --manifest-path integrations/projective-depth/Cargo.toml
cargo build --release --bins --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --residual bidirectional_midpoint --output artifacts/surface-analytic
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-surface-math.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/surface-math
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --residual bidirectional_midpoint \
  --case tum-fr1-room-temporal --case tum-fr1-desk2-independent \
  --case tum-fr1-rpy-heldout --output artifacts/surface-recorded
# Expected exit 1: complete physical accuracy fails.
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --analytic-reports artifacts/surface-analytic --residual bidirectional_midpoint \
  --recorded-reports artifacts/surface-recorded \
  --case tum-fr1-room-temporal --case tum-fr1-desk2-independent \
  --case tum-fr1-rpy-heldout --output artifacts/surface-raw
```

Use fresh output paths. Replacing the residual with `symmetric_midpoint` repeats
the first candidate. Omitting `--case` preserves the original two-recording
evaluator/auditor selection; RPY has no earlier fused-prototype comparison file.
