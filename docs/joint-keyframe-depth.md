# Joint keyframe pose comparison

The optional NumPy research tool now compares fixed reference poses with
simultaneous adjustment of a bounded set of measured-depth camera poses. The
synthetic five-camera test recovers perturbed reference poses, but the viewed
recorded trial fails: joint fitting scores **72/179** accurate room updates
and **51/179** desk2 updates, with tracking loss in both. Desk2 reaches
**1.706278 m** maximum accepted position error. This is an offline diagnostic;
it does not replace the Rust fused-depth tracker or feed vehicle localization.
The subjective maturity estimate remains **about 20%**.

## Fixed trial

The [design](../assets/joint-keyframe-depth-v1/design.json) fixes the method,
window, work bounds and accuracy gates before execution. Both recordings use
their original selected depth indices 100–279, all 180 acquisitions and one
first-camera origin. Rejected and post-loss frames remain in the 179-update
denominator. Numeric labels are decoded only after all four operational
branches, followed by independent integrity checks, have completed. RGB pixels
are never decoded by this comparison. These are already viewed recordings,
without a held-out claim.

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Loss |
|---|---:|---:|---|
| Original Rust fused depth, preceding evidence | 179 / 64 | 179 / 161 | Neither |
| Matched fixed-reference control | 91 / 71 | 125 / 74 | Both |
| Joint reference-pose adjustment | 92 / 72 | 95 / 51 | Both |

The matched control and joint variant use the same raw-depth factor definition,
window size, reference selection and admission bounds. The original fused-depth
baseline has a different map representation; the matched control isolates
the effect of revising reference poses within this new experiment. The first
prototype sources and physical failures remain in the
[evidence packet](../assets/joint-keyframe-depth-v1/).

## Estimator and limits

The window holds the first frame, the newest two motion-selected keyframes,
the last accepted frame if distinct, and the new frame, for at most five nodes.
The first pose remains exactly identity. Fixed mode optimizes the new pose
only; joint mode optimizes all other poses, at most 24 scalar variables.
All poses map optical camera coordinates into the first optical camera:
x right, y down, z forward, metres, proper right-handed rotations.

For each newer-to-older node pair, raw projective point-to-plane correspondences
must supply at least 200 matches. Retained measured factors must connect every
node to the first. The source left-increment Jacobian is
`[n_root, q_source_root × n_root]`; the target Jacobian is its negative.
The joint normal system includes both diagonal blocks and their negative
cross blocks. No numeric reference pose, authored motion prior or synthetic
measurement enters this system.

The original depth pyramid and correspondence gates are retained, with scales
4/2/1, source strides 1/2/4, Huber distance 0.01 m and iteration caps 10/7/5.
The diagonally scaled system must be SPD with condition at most 1e6. Each
node's increment is bounded to 0.10 m / 0.15 rad. Final admission re-evaluates
all active raw factors at the returned poses and requires global geometric
RMS at most 0.03 m; displacement from the prior accepted current pose must not
exceed 0.50 m. The primary physical accuracy gates remain 0.10 m / 0.10 rad,
with reference interpolation brackets at most 20 ms.

Keyframes change above 0.10 m or 0.10 rad relative to the newly refined latest
selected key. Accepted updates may revise stored reference poses; previous
published outcomes are not rewritten. Failed fitting/admission leaves stored
poses, selected references and the accepted acquisition clock unchanged.
Age above 0.20 s on a later acquisition latches loss; there is no automatic
reset or new origin. This batch tool has no sensor-silence watchdog.

This is pose-only local adjustment, without landmark bundle adjustment,
marginalization, loop-closure discovery, relocalization or calibrated
uncertainty. When a reference leaves the window, its incident measured
constraints are discarded. Preserving their information through a correctly
derived marginal prior remains unimplemented. The experiment does not isolate
which modeling limitation causes the recorded drift. Connected constraints,
good conditioning and low local residuals do not guarantee accurate root poses.

## Independent validation

An independent checker differentiates source and target residuals by central
finite differences, checks the unanchored gauge directions, and reconstructs
normal systems through incidence matrices. It uses separate raw equation
code and a spectral solve rather than the optimizer's block-slice assembly
and direct solve.
The optimizer and checker share the preceding depth-pyramid and rotation
exponential helpers;
correspondence equations, block assembly and solve verification use separate
code. The preceding [raw-depth audit](projective-depth-raw-audit.md) checks
that frontend against the independent Rust implementation.

Five analytic views use independently ray-cast spheres and a plane, with known
translations/rotations and deliberately perturbed reference estimates. Joint
adjustment recovers all five camera poses to at most **0.000076887 m** position
error, against **0.012411 m** remaining in the fixed-reference control. Every
joint node must be within 4 mm / 4 mrad. Original input poses remain unchanged
by fitting. Unobservable planes and disconnected graphs are rejected.

Six corruptions include changed increments, first-origin movement, missing
constraints, wrong variable sets, changed conditioning and coherent H/g
scaling. Five invalid pose/mode/clock cases are also rejected. CI runs these
analytic checks using the existing optional NumPy/Pillow requirements, without
downloading the recordings.

The recorded evaluator retains partial traces for rejected fits and checks
each reported matrix, increment, edge count and residual against independently
decoded depth. Its state audit reconstructs reference selection, internal
pose revisions and accepted clocks, verifies terminal loss, and reproduces
the actual fit-rejection guards. This mathematical/state validation is
separate from the failed physical accuracy assessment. Exact counts, source
snapshots and receipts are in
[validation.json](../assets/joint-keyframe-depth-v1/validation.json).

All **8,880** reported normal systems agree: **8,477** iteration systems and
**403** returned-pose final systems. All **24** fit rejections are independently
reproduced: 17 disconnected graphs, six conditioning failures and one oversized
increment. The remaining 289 rejected updates occur after expiry and perform
no fitting. Accepted poses and acceptance/loss decisions reproduce the first
frozen prototype exactly. These integrity checks pass while all four complete
physical-accuracy protocols fail with exit 1.

The existing `scripts/check.sh` also passes all 371 core Rust tests and 49
scenario executions/replays, with all 245 deterministic outputs unchanged.
Previous Rust estimators, optional checks, Cargo pins and evidence packets
retain their preceding hashes.

## Reproduction

The optimizer is Python research code in
`scripts/joint_keyframe_depth.py`, separate from the Rust estimator. Use the
existing optional environment with `scripts/requirements-visual.txt`:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-joint-keyframe-depth.py \
  --output artifacts/joint-keyframe-analytic
```

With the already hash-pinned TUM recordings prepared as described in the
[preceding diagnostic](projective-depth-diagnostics.md):

```bash
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-joint-keyframe-depth.py \
  --output artifacts/joint-keyframe-recorded
# Expected exit 1: complete physical accuracy fails.
```

Outputs require a fresh directory. Integrity exceptions stop before completed
physical scores; a completed `comparison.json` retains the separate quality
outcome. Source/input digests bind each branch, and packet `files.json` binds
the archived evidence. Original raw images and motion-capture files are not
redistributed. No real-time, automotive accuracy or safety claim is made.
