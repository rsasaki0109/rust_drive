# Causal RGB/depth acquisition-time compensation

The maintained Rust RGB-D tracker can now compensate the different acquisition
times of a measured grayscale image and its associated depth. Select
`photometric: keyframe`, `motion_model: constant_velocity`, and
`timing: previous_motion`. Timing defaults to `disabled`. Fixed-support step
control is rejected with this option because its retained measurement cost has
not been extended to temporal image warps.

This addresses an implemented measurement-model gap. It does not resolve
sustained localization accuracy: the complete viewed recordings still fail
physical accuracy and the fixed cross-record adoption rule. No default changes.

## Complete recorded outcomes

All rows use the same 180 original acquisitions, indices 100–279, K=525, first
depth-camera root, zero resets and 0.1 m / 0.1 rad root gates. Every tracker below
accepts all 179 updates in each recording without expiry. Numeric reference
poses are decoded only after each operational ledger is saved.

| Tracker | Accurate room / desk2, out of 179 | Max position room / desk2 (m) | Max rotation room / desk2 (rad) |
|---|---|---|---|
| Original depth default | 64 / 161 | 0.296290 / 0.114800 | 0.141286 / 0.090386 |
| Ordinary exposure-aware RGB-D | 84 / 124 | 0.267082 / 0.121676 | 0.102433 / 0.149427 |
| RGB-D with previous motion initialization | 75 / 169 | 0.270406 / 0.098134 | 0.117406 / 0.110973 |
| RGB-D with previous motion and temporal image compensation | **80 / 171** | 0.282444 / 0.090224 | 0.111927 / 0.113717 |

Compensation improves both accurate-update counts over its immediate motion
initializer predecessor, but room remains worse than ordinary RGB-D. Relative
to original depth, room passes the relative adoption gate. Desk2 exceeds the
unchanged maximum-angle bound, so the combined adoption decision fails. Counts
alone do not establish a general accuracy gain; the candidate also worsens
room's maximum position error and desk2's maximum angle versus its immediate
predecessor. These already-viewed indoor records are development regressions,
not held-out or automotive evidence. Subjective maturity stays about 20%.

The [source-bound packet](../assets/projective-timing-v1/summary.json) retains
the design frozen before implementation, before/candidate/final source archives,
complete sensor-only and scored trajectories, independent raw audits, controls,
failed adoption decision and default replay receipts. No parameters were changed
after candidate scores were viewed.

## Coordinate and history contract

All depth poses use optical x-right/y-down/z-forward and axial Z metres.
Depth remains the primary acquisition clock. Image association must stay within
20 ms, image timestamps cannot regress, and repeated RGB never contributes an
intensity constraint. Spatially registered RGB/depth with the existing shared
pinhole camera is assumed; this adds no extrinsic or distortion calibration.

For the two last accepted depth poses `G0,G1` at `t0,t1`, the causal body twist is
`xi=Log(G0^-1*G1)/(t1-t0)`. A measured depth point at `td` projects into its RGB
image at `tr` through `Image_from_Depth=Exp(-xi*(tr-td))`. Translation and rotation
are coupled by the SE(3) exponential. Each warp reports both acquisition times,
both accepted motion-source timestamps, a status and its optional transform.

For a source depth point `p`, fitted depth transform `T`, reference warp `A`, and
source warp `B`, the image residual is
`I_reference(project(A*T*p))-gain*I_source(project(B*p))-offset`.
Both image samples are bilinear. Geometric admission, normals, depth distance
and final geometric gates remain in the depth-reference frame. If `q=T*p` and
`g_image` is the projected reference-image gradient, the depth-pose Jacobian is
`[R_A^T*g_image, q cross (R_A^T*g_image)]`. Both warps are fixed before the current
fit; the current fitted pose never supplies its own motion rate. The existing
brightness gain/offset fit, nuisance projection and intensity scale are retained.

The source rate uses only poses accepted before the current observation. The
reference rate stays attached to its depth-reference acquisition, frozen when
that reference is accepted. The initial reference has no motion history; it can
be backfilled only after the first accepted interval starting at that reference.
This helps later observations, never the fit that supplied the missing rate.
Rejected fits and malformed or regressing observations never advance accepted
history or reference motion. A reference change with repeated RGB still clears
the color anchor until a subsequent fresh reference can be established.

An exact zero time offset has a known identity warp without history. Nonzero
offset with unavailable/ambiguous motion, or a warp exceeding 0.1 m or 0.15 rad,
disables image constraints while keeping admissible depth available. Valid
compensated pairs replace the old 5 ms difference-of-phases gate. These bounds
are model guards, not calibrated synchronization-error or acceleration bounds.
The existing 0.2 s accepted-depth expiry still latches loss before prediction.

## Independent checks and limits

The own-time renderer generates depth at the depth acquisition and grayscale
at the RGB acquisition from one static analytic scene. Poses generate and score
measurements only; runtime receives depth, gray bytes and their clocks.

* A nonplanar sequence passes ten known-motion acquisitions through a reference
  change and a rejected zero-depth observation, within 4 mm / 0.004 rad. Image
  constraints are withheld at nonzero-phase bootstrap and resume after causal
  history becomes available. Rejection never supplies a motion endpoint.
* A planar sequence bootstraps with two simultaneous RGB/depth pairs, then varies
  RGB offsets from -14 to +14 ms. Compensated tracking meets 4 mm / 0.004 rad on 8/8
  acquisitions; the uncorrected control meets those gates on 2/8. Both accept 8/8,
  so mere availability would hide the uncorrected drift. Without synchronous
  bootstrap or observable depth, nonzero-phase plane motion is rejected rather
  than inventing a rate.
* All 183 geometric and 183 intensity systems in these controls are reconstructed
  from raw images. Six-axis finite differences independently check the nonidentity
  reference-warp Jacobian. Signed offsets, missing history, near-pi motion and
  bounded-warp fallback have Rust controls. Four corrupted provenance reports
  are rejected, including current-frame motion and a newer reference rate.
* The color lifecycle control checks ten acquisitions, 84 raw geometric systems,
  repeated RGB, reference clearing/resumption, five clock/image faults and
  latched expiry. All 34 malformed transport/configuration cases reject; invalid
  temporal combinations reject before configuration acknowledgment.

The recorded raw audit independently rebuilds accepted history and reference
ownership from the complete sensor ledger. It computes SE(3) logarithms with a
quaternion eigensolve and Jacobian quadrature, exponentials with a matrix series,
and image systems with NumPy weighted SVD nuisance projection. After independent
causal verification within 1e-12, equations consume the reported rounded warp
matrices, as they consume the verified reported per-step poses. An initial raw
audit stopped at room index 126: independent warp differences below 3.9e-16 were
amplified by a nearly canceling Schur gradient beyond its unchanged tolerance.
The rounded-input correction reproduces that frame without changing any H/g,
residual or physical accuracy tolerances. The first audit failure and its source
version are retained. The complete auditor verifies every
geometric/image normal system and depth-model generation without decoding
numeric reference poses. Full geometric and photometric residual populations
remain explicit. The two recordings pass 7,969 geometric and 7,478 image systems, 358 prediction
reports and 358 fused-model generations. Source/binary/input hashes bind the
audits to these runs.

All 414 Rust tests (43 optional-native, 371 core), Clippy, formatting and 49 core
scenario/replays pass. The 245 core outputs stay unchanged. Both stationary
defaults reproduce all 720 historical sensor outcomes exactly. These checks
establish the bounded implementation and regression behavior, not real-time
performance, moving-scene robustness, acceleration accuracy or vehicle safety.
The motion rate comes from drifting estimates and assumes locally constant body
motion; compensation cannot be treated as measured synchronization truth.

## Reproduction

Use the existing [recorded inputs and pinned tools](projective-depth-rust.md#reproduction).
Use fresh output directories. The evaluator exits 1 after saving a complete
physical-accuracy failure; an exception before results is a different failure.

```sh
source scripts/env.sh
cargo test --manifest-path integrations/projective-depth/Cargo.toml --offline
cargo build --release --manifest-path integrations/projective-depth/Cargo.toml --offline
python scripts/check-projective-timing.py --binary integrations/projective-depth/target/release/rustdriving-projective-depth --output artifacts/timing-controls
python scripts/check-projective-rgbd-lifecycle.py --binary integrations/projective-depth/target/release/rustdriving-projective-depth --timing previous_motion --output artifacts/timing-lifecycle
python scripts/check-projective-depth.py --binary integrations/projective-depth/target/release/rustdriving-projective-depth --photometric keyframe --motion-model constant_velocity --output artifacts/timing-analytic
python scripts/evaluate-projective-depth.py --binary integrations/projective-depth/target/release/rustdriving-projective-depth --photometric keyframe --motion-model constant_velocity --timing previous_motion --output artifacts/timing-recorded
python scripts/audit-projective-depth-raw.py --analytic-reports artifacts/timing-analytic --photometric keyframe --recorded-reports artifacts/timing-recorded --output artifacts/timing-raw
python scripts/check-projective-depth-comparison.py --baseline artifacts/depth-default --candidate artifacts/timing-recorded --raw-audit artifacts/timing-raw --output artifacts/timing-adoption.json
bash scripts/check.sh
```

Use the camera-only evaluator to produce `artifacts/depth-default` first. The
comparison must bind every baseline outcome to the original frozen depth ledger;
its nonzero exit preserves the failed adoption result. The NumPy/Pillow versions
and native binary are pinned in the saved run files; no dataset is redistributed.
