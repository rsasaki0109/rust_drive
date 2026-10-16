# Physical directional information guard

The maintained Rust projective tracker now rejects pose fits whose information
is effectively missing in a physical direction. This guard is mandatory before
every normal solve and again at the returned pose, before any accepted state,
fused model or motion-history commit. It repairs a false-acceptance case rather
than adding another optional accuracy variant.

The previous solver checked condition only after dividing each row and column
by its own diagonal scale. This is useful for numerical solution, but it can make
an almost unobserved pose direction look as strong as a measured direction.
A diagonal matrix with entries `[1e-12,1,1,1,1,1]` has equilibrated condition 1,
while its physical condition is 1e12. Both numerical stability and directional
information must pass; one cannot substitute for the other.

## Contract

The combined geometric and actually used brightness-marginalized image Hessian
is evaluated at fixed parameter scales: 1 metre translation and 1 radian rotation.
A single maximum-diagonal factor scales the entire matrix for the bounded 6×6
Jacobi eigensolve. Unlike diagonal equilibration, it preserves the relative
strength of different pose directions. The spectrum must be positive definite
with largest/smallest eigenvalue at most 1e6, the existing solver's condition
ceiling. No threshold is fitted to the physical reference trajectories.

The existing scaled SPD solver, update calculation, support, residual, increment,
keyframe/fusion and acquisition-expiry policies remain. The guard is invariant
to common residual scaling and rotations of the translation/rotation axes at
these fixed units; it is not a calibrated uncertainty model or an arbitrary
SE(3) reference-origin invariance claim. Relative spectrum alone does not supply
absolute information, sensor-noise, association-error or covariance calibration.
Noise or modeling error can still create apparent constraints.

A rejection emits an optional `information_failure` witness containing its
pyramid scale, candidate depth-frame pose, raw geometric/image equations and
spectrum. `final_pose` identifies rejection at the final admission check. The
spectrum reports the fixed physical units, common matrix scale, all sorted
eigenvalues and an optional condition. Nonpositive or numerically singular
spectra cannot authorize a pose. Successful report payloads retain their original
structure, and failures retain atomic accepted-state behavior.

## Verified repair and regressions

The own-time plane control first acquires two synchronous textured RGB-D frames,
then varies image offsets from −14 to +14 ms. Previously the uncompensated mode
accepted all 8 acquisitions, although six poses failed 4 mm / 0.004 rad accuracy.
The guard now admits only the first two and rejects all six weak-information
fits. Those rejections cannot refresh the accepted clock: subsequent expiry
latches loss, without extrapolation or silent restart. The properly compensated
mode continues to pass 8/8, as does its 10-acquisition nonplanar positive control.
Nonzero-phase plane bootstrap without an observable motion rate still rejects.

The independent NumPy auditor reconstructs every rejected witness from raw
pixels, compares the physical spectrum using `eigvalsh`, and verifies the pose
against the last reported iterate or the independently predicted initializer.
Near machine precision the sign of an effectively zero eigenvalue may differ
between Jacobi and NumPy; both must reject, and the eigenvalues must agree within
the documented numerical tolerance. Three corrupted failure reports reject:
a fabricated eigenvalue, an accepted weak pose and a missing required witness.
The original timing provenance mutations and brightness-null-space controls
also pass. Rust tests check an artificially equilibrated weak direction, rotated
physical axes and global residual scaling from 1e-20 to 1e20.

All 416 Rust tests (45 optional-native and 371 core), Clippy, formatting and 49 core
scenario/replays pass. The 245 core outputs remain unchanged. Complete depth,
ordinary RGB-D and compensated RGB-D runs reproduce all 1,080 historical sensor
outcomes exactly, including their known physical accuracy failures:

| Mode | Accurate room / desk2, out of 179 updates |
|---|---|
| Depth default | 64 / 161 |
| Ordinary affine RGB-D | 84 / 124 |
| Motion and time-compensated RGB-D | 80 / 171 |

The stronger rejection contract does not repair those real-motion drift results.
The design and sources were frozen before fitting or acquiring new validation
data. Historical source/evidence archives remain intact.

## First frozen original-source validation

The first new candidate was the official FR1 plant recording, indices 100–279.
The complete original archive was hashed and its whole metadata tables checked
before any PNG header, pixel or numeric mocap-pose decoding. Reference brackets
at six fixed indices 232–237 exceed the unchanged 20 ms limit. The trial stops as
an input-qualification failure, without a pose score, easier replacement window
or relaxed reference gate. Its original design, source archive and failure remain.

The separately frozen official FR1 RPY recording passes complete metadata
qualification: 722 depth rows, 723 RGB rows and 2,742 mocap rows; maximum selected
reference bracket 10.700 ms and RGB/depth association 14.338 ms. The same fixed
indices 100–279 provide 180 acquisitions and 179 updates, without repeated RGB.
This is a new recording for this estimator, with the same FR1 camera family and
no established automotive or independent-room generalization. The source is
[attributed and byte-pinned](../data/tum-fr1-rpy-heldout/SOURCE.md).

Both declared modes finish their complete sensor-only ledgers before numerical
reference scoring. Independent processes then reconstruct all 7,333 geometric
and 3,829 intensity systems and all 358 fused-model generations without decoding
numeric reference poses. Both modes accept all 179 updates without expiry/reset.

| Frozen RPY mode | Root accurate / 179 | Maximum position (m) | Maximum rotation (rad) |
|---|---:|---:|---:|
| Depth default | 74 | 0.087348 | 0.173375 |
| Previous-motion / time-compensated RGB-D | 61 | 0.104537 | 0.184782 |

Both complete physical protocols fail. The new information guard does not
reject these biased but well-conditioned fits, and temporal image compensation
worsens this new recording's outcome. No runtime parameter, source calibration
or selection was retuned after these scores. The new recording is now viewed;
subsequent development must treat it as regression data. All 360 sensor outputs
also reproduce byte-for-byte from its archived source and pinned native binary.

## Conditional uncertainty and empirical calibration failure

An additional offline checker tests a deliberately provisional conditional
model, `Sigma0=(0.005 m)^2 * H^-1`, with the final combined information matrix.
The model fixes the maintained map and motion warps; it omits map/pose history,
association, calibration, brightness-weight uncertainty and sample correlation.
It is not a reported or calibrated runtime covariance.

The first-camera root error is expressed in the actual maintained reference:
`xi=Log(Gkey^-1 * Gest * Gtruth^-1 * Gkey)`. The conditional NEES is
`xi^T * Sigma0^-1 * xi`, compared with nominal six-dimensional Gaussian 95%
threshold 12.591587. For position, the Jacobian is
`Rkey * [I, -[t_relative]_cross]`, including the reference lever arm. Projected
joint-ellipsoid radii use this positional covariance and the rotational block.
Six pure-axis error controls in a rotated/translated reference, and independent
finite differences of the position projection, pass.

A mode-specific empirical variance factor is frozen from the higher 95th
percentile of both previously viewed recordings’ 358 accepted updates, before
numeric RPY pose scoring. It changes no runtime estimate. This is a diagnostic
calibration attempt on correlated frames from two recordings, without an
exchangeability or probability-calibration claim.

| RPY conditional diagnostic | Depth | Time-compensated RGB-D |
|---|---:|---:|
| Nominal ellipsoid coverage, 179 accepted updates | 0% | 0% |
| Frozen empirical variance factor | 2,718,306.748 | 16,957,683.728 |
| Inflated ellipsoid coverage | 100% | 100% |
| Maximum projected position radius (m) | 1.553465 | 2.711021 |
| Maximum projected rotation radius (rad) | 2.344250 | 2.463849 |

The preregistered diagnostic requires at least 90% held-out coverage and both
projected radii at most 0.1 m / 0.1 rad. Both modes fail. Large inflated radii also
leave a useful small-error linear approximation, so 100% observed coverage is
not a qualified probabilistic error bound. Neither the nominal inverse Hessian
nor scalar inflation is supplied to the driving EKF or used to authorize motion.

## Evidence integrity and maintained helper repairs

The RPY inventory writer initially repeated `source_path` while expanding a
verified archive member. The acquisition-only repair reverified every original
selected byte and wrote that key once. Original selection, estimator, scoring
and pre-acquisition design stayed fixed; no pixels or numeric poses were decoded
by the repair. The maintained writer now explicitly copies `bytes` and `sha256`.
A full real-archive regression reproduces the exact repaired manifest.

The conditional report initially failed on a NumPy scalar Boolean, leaving an
incomplete JSON file. Its source, partial file and error are preserved. A
serialization-only repair converts scalars to JSON primitives and recomputes all
values against the same frozen factor; there is no numerical retuning. The
maintained conditional checker now casts its gate to a Python Boolean.

Shared JSON evidence writes now publish only complete files, by writing and
flushing a temporary file and linking it exclusively into the final name.
Serialization, flush and publication faults leave no final report; an existing
report cannot be overwritten, including by concurrent writers. Six fault/race
and canonical-byte controls pass. This prevents a failed evaluation from
leaving an apparent completed report. CLI acknowledgments and successful report
bytes remain unchanged. The maintained physical scorer and empirical-calibration
checker return exit 1 for completed quality failures; source/input validation or
serialization exceptions remain distinct from a saved numerical failure. Their
final helper regressions reproduce all prior values and failed gates. The first
cold sources and their original exit/serialization behavior remain archived.

The [versioned packet](../assets/projective-information-v1/summary.json) contains
before/candidate/final sources, both cold source freezes, first failures,
complete RPY ledgers/results and raw audits, empirical factors/results, default
replay receipts and validation logs. Raw images and compiled binaries are not
redistributed. Cold replay requires its exact archived source/configuration;
current helper fixes and documentation are separately preserved in the final
source archive. The working source never retroactively becomes the source of
an earlier trial.

This work repairs a genuine false-acceptance and report-publication defect,
while preserving prior results and establishing a new failed validation.
The 30% waypoint is still unmet. Useful real-motion accuracy, calibration and
broader perception/localization evidence remain incomplete; the subjective
maturity estimate stays about 20% versus mature driving OSS.

## Reproduction

Use a fresh output directory for controls:

```sh
source scripts/env.sh
cargo test --manifest-path integrations/projective-depth/Cargo.toml --offline
cargo build --release --manifest-path integrations/projective-depth/Cargo.toml --offline
python scripts/check-projective-timing.py --binary integrations/projective-depth/target/release/rustdriving-projective-depth --output artifacts/information-controls
python scripts/check-projective-report-write.py --output artifacts/report-write-controls
python scripts/check-projective-calibration-math.py --output artifacts/calibration-math
bash scripts/check.sh
```

The one-time `projective-heldout.py` protocol has separate `prepare`, `acquire`,
`run`, `audit` and `score` stages. It refuses existing trial paths and changed
frozen sources. The first RPY inventory uses the separately hashed repair.
Replay the published cold source snapshot with the recorded raw manifest and
native binary; all 360 recorded outcomes were reproduced that way locally.
The current acquisition writer and conditional serializer have regression
repairs, while the preceding exact cold sources remain immutable. An internal
same-environment replay does not establish external or cross-platform success.
