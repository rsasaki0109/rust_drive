# Fixed-support RGB-D step control

The optional Rust RGB-D tracker now has an explicit `step_control: fixed_support`
research policy. It checks the nonlinear effect of each proposed increment on
the same measurements and weights, shrinks bad steps, and stops a pyramid level
when none of five trials passes. **It is not adopted:** complete recorded pose
accuracy regresses. Both depth-only and ordinary RGB-D defaults are preserved.

## Recorded decision

All results retain the original 180 acquisitions per recording, first optical
camera origin, K=525 camera, 0.1 m / 0.1 rad accuracy gates and zero resets.
Every run accepts all 179 updates without expiry.

| Tracker | Accurate, room / desk2 | Maximum position, room / desk2 (m) | Maximum rotation, room / desk2 (rad) |
|---|---|---|---|
| Depth default | 64 / 161 | 0.296290 / 0.114800 | 0.141286 / 0.090386 |
| Ordinary exposure-aware RGB-D | 84 / 124 | 0.267082 / 0.121676 | 0.102433 / 0.149427 |
| Fixed-support step control | 83 / 67 | 0.360512 / 0.181417 | 0.120893 / 0.194282 |

The original cross-record adoption gate fails for both recordings. Room's
maximum position error worsens relative to depth-only, and desk2 regresses in
count and both maxima. Both also regress relative to the maintained RGB-D
predecessor. Checking local descent does not establish root accuracy.
Numeric pose truth enters scoring only after operational ledgers are saved.
These remain already-viewed indoor development recordings.

The guarded ledgers contain 3,819 / 3,801 step reports and 4,566 / 4,335 trial
costs. Steps shrink 495 / 374 times, with 747 / 534 unavailable trials and no
level halts. No finite full-step cost exceeds its initial cost: every reduction
in these runs is caused by retained measurements leaving the admissible domain,
rather than an observed increase in the finite surrogate. This distinguishes
the tested failure mechanism from a demonstrated need to correct uphill steps.
It does not prove a unique cause of accumulated root drift.

The first implementation applied the guard to all RGB-D fits. After its complete
failure was saved, the maintained correction made it explicitly opt-in rather
than changing the algorithm or tuning its constants. The first and corrected
guarded runs retain identical operational results. Depth default retains all
360 original outputs, and RGB-D with step control disabled retains all 360
exposure-aware outputs. The [source-bound packet](../assets/projective-step-control-v1/summary.json)
contains complete ledgers, sources, controls, audits and failed adoption.

## Fixed measurement objective

At each iteration, original projective correspondence, normal and distance
admission select source queries and reference surface planes. The guard freezes
these pairs and their Huber IRLS weights. If a photometric equation is usable,
it additionally freezes its accepted source pixels and Tukey weights from the
reported exposure estimate. No residual or gradient cutoff is reapplied while
testing a trial.

Each trial evaluates the weighted squared point-to-plane residual plus
`0.005²/100` times the weighted squared image residual, with the common 1/2
factor. The image's gain and offset are re-fit by weighted least squares on
exactly those pixels. This is a local fixed-weight surrogate; exposure bounds
apply to original equation admission, while the surrogate nuisance fit is
unconstrained. The constant, centered brightness columns use the same source
values and weights for every trial. All retained image samples must remain
sampleable; any image-domain or near-plane failure makes the whole trial
unavailable rather than dropping its pixels.

The full proposed normal solve keeps its original support, scaled SPD condition
and 0.1 m / 0.15 rad increment gates. Proposed scales are exactly
`1, 1/2, 1/4, 1/8, 1/16`; the first cost no greater than the initial cost plus
`1e-12*(1+initial_cost)` is accepted. No passing trial leaves the pose unchanged
and ends that resolution level. This is not a frame rejection: finer levels and
the original final residual, displacement and clock checks still apply.

Each trace retains the unscaled normal solution in `delta`, and separately
reports `step_control.applied_scale`, fixed counts, initial cost and all tried
costs. `after` composes the scaled increment. A zero scale explicitly records a
level halt. Geometry-only intervals within RGB-D use the same depth surrogate,
including when reference color has been cleared after a repeated image.
`step_control` defaults to `disabled` and requires `photometric: keyframe` when
enabled. Unsupported configuration rejects before acknowledgment.

## Independent validation

NumPy rebuilds fixed correspondences, image samples and weights from measured
PNG data and reported linearization poses. It uses weighted SVD least squares
for trial exposure rather than Rust's centered sums. It checks every reported
cost, unavailable trial, selected scale and scaled pose, plus the existing raw
normal equations and prescribed depth fusion. Separate mutation controls detect
fabricated costs, false support counts and a zero-cost claim for pixels that
left the image.

Four additional ray-cast motions pass 4 mm / 0.004 rad gates. Three exercise
image-boundary backtracking: their full steps are unavailable and their half
steps keep all retained pixels. Rust hand controls also verify an overshoot
shrinks, wholly uphill proposals halt, and a single lost pixel cannot lower
cost. Existing motion, exposure and asynchronous reference lifecycle controls
pass with the guard enabled. CI covers these without recording downloads.

Complete recorded reconstruction passes for 7,978 geometric systems, 7,638 image
systems, 7,620 initial surrogate costs and 8,901 trial costs, with all 358
prescribed depth model generations. Numeric pose truth is never decoded by this
audit. The optional crate passes 39 Rust tests; core checks retain 371 tests,
49 scenario/replay checks and all 245 deterministic output hashes. The 410
passing Rust tests and correct local calculations do not reverse the failed
physical accuracy decision. Earlier evidence files remain unchanged.

## Reproduction and limitations

Use the [existing input/environment setup](projective-depth-rust.md#reproduction)
and fresh output directories:

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-step-control.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/step-controls
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --photometric keyframe --step-control fixed_support --output artifacts/step-recorded
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --photometric keyframe --step-control fixed_support --output artifacts/step-analytic
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --photometric keyframe --analytic-reports artifacts/step-analytic \
  --recorded-reports artifacts/step-recorded --output artifacts/step-raw
```

The physical evaluator and adoption checker save failures and exit 1. Surrogate
descent can converge toward a biased or ambiguous pose, and correspondence sets
can change between iterations. A single boundary sample can restrict a whole
trial. The policy does not add calibrated uncertainty, temporal compensation,
loop closure or vehicle localization. Subjective maturity remains about 20%
versus mature driving OSS; this work supplies no qualified accuracy gain.
