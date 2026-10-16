# Advancing past an inadmissible coarse depth proposal

This optional NumPy experiment changes the multiscale schedule of the
[local-plane normal tracker](patch-keyframe-depth.md). When a coarse proposal
fails its measured-support, condition or increment guard, the fitter applies
no update for that proposal and advances to the next finer level. Finest-level
iterations, returned-pose admission and marginalization retain every original
guard. The Rust driving pipeline and default fused-depth estimator are unchanged.

## Recorded outcome

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Terminal loss |
|---|---:|---:|---|
| Original Rust fused depth, preceding evidence | 179 / 64 | 179 / 161 | Neither |
| Symmetric Huber control | 179 / 72 | 179 / 74 | Neither |
| Preceding patch normals | 142 / 77 | 120 / 75 | Both |
| Patch normals with selective coarse skips | 179 / 77 | 179 / 76 | Neither |

Tracking continuity recovers on both recordings. Room's maximum position/
rotation errors are **0.293017 m / 0.186621 rad**; desk2's are
**0.187061 m / 0.205988 rad**. Both complete physical-accuracy protocols still
fail. Room gains no additional accurate updates, and desk2 gains one. Room's
rotation maximum is larger than the preceding truncated trial's 0.143939 rad;
the two trials cover different accepted-pose populations.

The [matched comparison](../assets/selective-keyframe-depth-v1/comparison-with-parent.json)
preserves both full summaries. The
[prefix receipt](../assets/selective-keyframe-depth-v1/prefix-parity.json)
verifies identical complete sensor-ledger rows for the first **143 room** and
**121 desk2** acquisitions, including initialization. The first skipped
proposals match the preceding failures at source indices 243 and 221, including
their exact successful trace prefixes and rejection reasons. Earlier physical
errors therefore remain in this schedule-only comparison.

Room skips **38** coarse proposals: 32 by condition and six by increment.
Desk2 skips **two**, both by increment. There are no fit rejections or expired
acquisitions. All original accepted-pose, finest/final and Schur requirements
remain active; the Rust default and subjective maturity estimate of
**about 20%** remain unchanged.

All accepted fits still reach the finest-level five-iteration cap. The
[convergence record](../assets/selective-keyframe-depth-v1/convergence.json)
reports median maximum-node proposed translation steps of about 0.168 mm in
room and 0.288 mm in desk2 at returned poses. These local quantities do not
establish convergence or physical accuracy.

## Diagnostic and fixed contract

The [preceding-failure diagnostic](../assets/selective-keyframe-depth-v1/level-diagnostic.json)
reconstructs all twelve pre-expiry fit failures from the frozen sensor-only
ledgers and original hashed depth PNGs. A separate centered-patch SVD estimates
normals; independent matrix products rebuild the residual systems. The script
verifies every saved iteration and reproduces the original rejection first.
It then evaluates levels 4/2/1 at both acquisition-entry and rejected poses.
It fits no new trajectory and reads no numeric ground truth.

| Preceding patch-normal failures | Room | Desk2 |
|---|---:|---:|
| Pre-expiry rejected fits | 6 | 6 |
| Finest system passes support, condition and increment guards | 4 | 2 |
| Failed level 4 | 4 | 6 |
| Failed level 2 | 2 | 0 |

The finest feasibility counts agree at entry and rejected poses. A single
admissible linearization does not establish subsequent fitting success or
physical accuracy. The diagnostic distinguishes the actual failed level from
the last logged successful iteration; they can differ after convergence.
Both recordings were already viewed development data.

The [design](../assets/selective-keyframe-depth-v1/design.json) was fixed before
new recorded fitting. Levels remain 4/2/1 with at most 10/7/5 attempts. Only
levels 4 and 2 may be skipped, and only for these existing guard failures:

- No admitted raw pair incident to the current acquisition.
- Zero/nonfinite diagonal or scaled condition above 1e6, including nonpositive
  smallest scaled eigenvalue.
- A proposed node increment exceeding 0.10 m translation or 0.15 rad rotation.

Each skip records its raw equation, reason, unchanged before/after poses, zero
applied increment, and proposed increment/condition if the solve succeeded.
It consumes one attempt and ends that level. Previously valid local iterations
remain in the attempted poses; nothing commits to the tracker until the whole
fit, final admission and elimination succeed. Chart errors and unexpected
errors remain failures. There is no damping, pseudoinverse or rank filling.

Level 1 cannot be skipped. Its measured support, condition and increment
requirements remain strict. Returned-pose unweighted RMS remains at most
0.03 m, accepted displacement at most 0.50 m, and incident-only Schur guards
and unique factor ownership are unchanged. Accepted-pose age above 0.20 s
still latches terminal loss. Initialization still needs 200 measured normals
at level 4. There are no pose resets or idle watchdog.

Depth pixels, vertices, 5×5 PCA normal estimation, symmetric measured-normal
residuals, Huber0.01m weights and the prior are unchanged. The source checker
proves that all engine text outside `fit()` matches the preceding frozen
implementation. Trace length remains at most 22 events, including skips.

## Independent verification

The private auditor reconstructs every skipped equation from original pixels
using the preceding independent SVD and matrix-based raw factor oracle. A
spectral solve independently classifies the reason and checks proposed
increments. It requires identical before/after poses, exactly zero applied
increments, the prescribed next level and completion of the finest budget or
convergence. Existing state, solve, prior and Schur checks remain active.

A rendered depth fixture puts one missing pixel in every 2×2 block: valid
finest neighborhoods remain, but both decimated coarse depths are empty.
The preceding fitter rejects it; the new tracker skips both coarse levels and
recovers the known motion with position error about **0.028 mm**. Explicit
coarse-layer fixtures also exercise rank-deficient tilted normals and an
oversized proposal; these are injected layers, not claimed sensor outputs.
The checker verifies **18** fixture trace/final systems. Six new report
corruptions are rejected, including changed skip poses, nonzero applied
increments, a false reason, finest-level skipping, skipping a valid system
and an unjustified level jump.

Both inherited fifteen-frame motion branches pass the 4 mm / 4 mrad bounds,
with **452** iteration/returned-pose equations and nine retained marginal
generations checked. All nine inherited corruptions are rejected. Missing
finest support, planar degeneracy, terminal expiry and late elimination
failure remain failures; omission recovery preserves the clean prior.

All sensor fitting finishes before recorded auditing. Two workers each own
a private audit module and callbacks, while operational fitter state remains
untouched. Both audits finish successfully before numeric reference labels
are read. Each recording retains all 179 updates, identity initialization
outside the denominator, physical gates 0.10 m / 0.10 rad and reference
interpolation brackets at most 20 ms. RGB is not decoded by the estimator.

The recorded audit checks **7,813** systems: **7,140** applied-iteration
systems, **40** skipped coarse systems, **358** returned-pose systems and
**275** elimination-input systems, plus all 275 Schur-result priors. Room
consumes 513 unique raw factors across 137 marginal generations; desk2 consumes
595 across 138. Reported-system counts explicitly include coarse skips.

`bash scripts/check.sh` passes **371** Rust tests and **49** scenario executions
with full sensor-log replays. All **245** deterministic outputs and **278**
preceding evidence files keep their previous hashes. CI adds the new optional
scheduling and syntax checks.

## Reproduction and limits

Use the pinned NumPy/Pillow environment and original hash-pinned TUM inputs:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-levels.py \
  --output artifacts/depth-level-diagnostic-replay
OPENBLAS_NUM_THREADS=1 python scripts/check-selective-keyframes.py \
  --output artifacts/selective-keyframes-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-selective-keyframes.py \
  --output artifacts/selective-keyframes-recorded
# Expected exit 1: complete physical accuracy remains unmet.
```

Directories must be fresh. Completed summaries distinguish physical quality
failure from an audit exception. The packet archives frozen source/context,
sensor-only ledgers and compact physical scores; original images and reference
files are not redistributed. [TUM attribution](tum-source-provenance.md).

The [packet manifest](../assets/selective-keyframe-depth-v1/files.json) records
file sizes and SHA-256 digests. Merging compact score fields into matching
sensor-ledger rows and serializing `{summary, frames}` with two-space JSON
indentation and a final newline reconstructs the original scored reports byte
for byte. The [validation receipt](../assets/selective-keyframe-depth-v1/validation.json)
separates implementation integrity from physical quality, and the
[source comparison](../assets/selective-keyframe-depth-v1/previous-rust-source-parity.json)
checks the 16 preceding pinned sources including the Rust estimator.

Advancing to finer geometry can still fail or enter a poor correspondence
basin. The schedule does not establish calibrated uncertainty, independent
environment performance, real-time operation, long-duration mapping or vehicle
integration. A smaller accepted-only maximum after early loss does not prove
improvement over a complete trajectory.
