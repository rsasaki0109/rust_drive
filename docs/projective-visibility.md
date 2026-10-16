# Measured free-space evidence for fused maps

The optional Rust tracker now implements `visibility: "free_space_decay"`.
When a new supported depth observation places an old surface in measured free
space, its stored support decreases by one. Exhausted support clears that cell
before the original warp and fusion. A controlled disappearing foreground is
removed correctly. **The mode remains disabled by default:** viewed scores are
67/161/74, and maximum position errors increase in room and desk2.

The [fixed design](../assets/projective-visibility-v1/design.json),
[source-bound packet](../assets/projective-visibility-v1/summary.json) and
[adoption decision](../assets/projective-visibility-v1/adoption.json) preserve
the complete trial. These are the same three viewed Freiburg 1 recordings,
without new held-out qualification or a maturity increase.

## What the diagnostic separates

The preceding [fusion study](projective-fusion.md) compares models that can admit
different points and normals. This diagnostic adds paired probes on **identical
source pixels and projected target pixels**. The raw physical keyframe and a
model fused with offline reference transforms must both admit each pair under
the original fine-scale gates. There are fifteen fixed probes per recording,
with 221,833 / 208,282 / 236,991 common matches in room/desk2/RPY.

For each probe, raw-keyframe normals, Jacobians and raw-keyframe Huber weights
are frozen. If raw and fused target points are `p0` and `p1`, then
`r_fused = r_raw + n_raw · (p0 - p1)`. The same normal matrix gives a raw
correction, a response to the map-point shift, and their summed correction.
Every six-dimensional vector sum is verified. The table summarizes median
magnitudes; translation is in millimetres and rotation in milliradians.

| Common-support response | Room translation / rotation | Desk2 translation / rotation | RPY translation / rotation |
|---|---|---|---|
| Raw physical keyframe | 6.238 / 7.122 | 6.663 / 4.006 | 7.461 / 6.362 |
| Map-point shift under frozen raw normals | 6.611 / 4.958 | 8.239 / 6.889 | 8.317 / 5.906 |
| Fused points under frozen raw normals | 8.792 / 8.047 | 13.320 / 10.205 | 12.413 / 9.819 |
| Fused points with updated normals and robust weights | 7.734 / 6.829 | 9.466 / 7.170 | 12.130 / 6.418 |

Median RMS of the plane displacement is 6.958 / 7.856 / 6.505 mm. The snapshot
calculation isolates the algebraic effect of changing stored points under a
fixed system. Camera calibration, reference timing, raw measurement errors and
association still contribute; the result does not identify an additive physical
error budget or a unique drift cause. Full systems and per-probe vectors are
retained. Dataset-free controls recover six prescribed linearized translation
and rotation axes, and independently indexed matches reproduce the original
full-support equations.

## Visibility observations and the update

The diagnostic also replays all original baseline model generations. Each old
cell is projected into the current raw depth view with the archived estimated
pose. Only finite, in-range depth with a supported original central-difference
normal can supply evidence. With projected old depth `z` and current measured
depth `d`, the existing fusion band `b = 0.03 + 0.01*z` metres defines:

- `d - z > b`: the old point lies in observed free space.
- `d - z < -b`: a nearer measurement occludes the old point.
- Otherwise: the depths are consistent within the band.

Missing, out-of-range or unsupported observations supply no evidence. A nearer
occluder leaves background support intact during this check. Each accepted newer
depth acquisition decreases a contradicted old cell's support by one; zero
support clears depth. Decay occurs on cloned state **after all pose-admission
checks**, before any reference-view warp or positive fusion. Rejected fits spend
no support and cannot renew the accepted clock.

| Viewed recording | Contradicting observations / tested baseline cells | Fraction | Counterfactual reference-pose fraction |
|---|---:|---:|---:|
| Room | 1,081,684 / 35,983,635 | 3.006% | 9.977% |
| Desk2 | 1,363,025 / 36,091,930 | 3.777% | 15.001% |
| RPY | 1,470,519 / 38,803,241 | 3.790% | 15.508% |

These totals count repeated cell observations, with substantial correlation.
The reference-pose channel belongs only to the offline diagnostic. The larger
fractions there reinforce the sensitivity to pose, map construction and the
uncalibrated registered grid. Contradicting cells are not labeled dynamic
objects or independently proven false geometry.

In the candidate's changing models, the corresponding fractions are about
1.36% / 1.60% / 1.69%, with 129,166 / 137,126 / 152,704 removals over all updates.
Later eligible populations differ because the map changes. Lower internal
contradiction does not establish better root accuracy: the complete trajectory
test below still fails adoption.

Support remains a bounded heuristic, capped at the original 16. Correlated rays,
warped cells and calibration errors prevent interpreting it as independent
evidence counts or a probability. Incorrect accepted poses can also create
apparent free space. The original fitting objective, physical information guard,
clock/expiry rules, keyframe thresholds and four positive-fusion categories
remain unchanged.

`TrackerConfig.visibility` and JSON `Configure.visibility` accept `disabled`
(default) or `free_space_decay`. Decay requires original nearest/depth/point/
forward/target-plane/central-difference policies, stationary motion, disabled
photometric tracking, disabled step control and disabled timing. Unsupported
combinations reject before configuration acknowledgment.

## Complete recorded results

All runs use original indices 100–279, K=525 intrinsics, acquisition timestamps,
one first-camera optical root and fixed 0.1 m / 0.1 rad gates. Each accepts all
179 updates without expiry or resets. Native sensor outcomes are saved before
numeric reference poses are decoded for scoring.

| Recording | Accurate, original / decay | Maximum position, original / decay (m) | Maximum rotation, original / decay (rad) |
|---|---|---|---|
| Room | 64 / 67 | 0.296290 / 0.301377 | 0.141286 / 0.140646 |
| Desk2 | 161 / 161 | 0.114800 / 0.117715 | 0.090386 / 0.090279 |
| RPY | 74 / 74 | 0.087348 / 0.087346 | 0.173375 / 0.172995 |

The frozen rule requires room above 64, desk2 at least 161, RPY at least 74,
complete accepted tracking and no worse maximum position or rotation in any
recording. Counts and rotation maxima pass; room/desk2 position maxima fail.
The default stays unchanged. No parameters or gates are retuned after scoring.

## Independent controls and reproduction

A known stationary scene first measures a 144-pixel foreground three times,
then observes its background. A missing patch between observations leaves the
support intact. Three actual contradictions remove all 144 cells; the legacy
mode retains all of them. A new near occluder is classified separately. The
native ledgers and independently reconstructed subsequent equations confirm
the model transition. Substituting the retained ghost map while keeping a valid
removal receipt is detected. An injected rejected empty frame leaves all nine
valid outputs exactly identical.

A separate twelve-frame known-motion scene with 2 mm axial noise exercises a
reference-view warp and passes the existing 10 mm / 0.01 rad gates. Original
four-motion controls retain their 4 mm / 0.004 rad gates. The optional crate
passes **59 Rust tests**, and transport checks reject **75** malformed or
unsupported requests. Raw audits reconstruct **10,741 fitted/returned systems**
and **537 model generations** from the original PNGs without reference-pose
decoding. New mutations reject false visibility counts, false removal counts
and missing evidence receipts.

Core checks retain **371 Rust tests**, **49 scenario replays** and **245**
unchanged deterministic outputs, for **430 Rust tests** including the optional
crate. All **1,440** historical default depth, RGB-D and timing outputs remain
exactly identical. CI includes the dataset-free controls; full recorded audits
were run locally on this Linux host. The packet preserves both diagnostic and
implementation source epochs, with unchanged diagnostic math checked explicitly.

Use the [existing inputs and environment](projective-depth-rust.md#reproduction)
and fresh output paths:

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-projective-visibility.py \
  --output artifacts/visibility-diagnostic
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-visibility.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/visibility-controls
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --visibility free_space_decay --output artifacts/visibility-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --visibility free_space_decay --case tum-fr1-room-temporal \
  --case tum-fr1-desk2-independent --case tum-fr1-rpy-heldout \
  --output artifacts/visibility-recorded
# Expected exit 1 for retained physical accuracy failures; continue auditing.
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --visibility free_space_decay --analytic-reports artifacts/visibility-analytic \
  --recorded-reports artifacts/visibility-recorded --case tum-fr1-room-temporal \
  --case tum-fr1-desk2-independent --case tum-fr1-rpy-heldout \
  --output artifacts/visibility-raw
```

No raw recordings or compiled binaries are redistributed. This remains offline
research, with no held-out accuracy, calibrated uncertainty or vehicle
integration claim. Subjective maturity versus mature driving OSS remains
**about 20%**, with the **30% waypoint unmet**.
