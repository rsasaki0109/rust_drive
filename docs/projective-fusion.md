# Visible-surface fusion and signed raster bias

The Rust tracker now has an optional `fusion_sampling: "visible_mean"` policy.
It removes most of a demonstrated nearest-Z bias in a prescribed noisy-plane
control. Complete viewed tracking scores are **67/161/74** accurate updates in
room/desk2/RPY, compared with the original **64/161/74**. Maximum position errors
increase in all three recordings, so **the default remains `nearest`**.

The [fixed design](../assets/projective-fusion-bias-v1/design.json),
[source-bound packet](../assets/projective-fusion-bias-v1/summary.json) and
[adoption decision](../assets/projective-fusion-bias-v1/adoption.json) preserve
the diagnostic, full outcomes and unchanged acceptance rule. These are the same
three already viewed Freiburg 1 windows, with no held-out accuracy claim.

## The measured problem and implementation

The original raster keeps the smallest axial Z when multiple transformed points
round into one target pixel. Even observations from one noisy surface can then
select preferentially nearer depths. The earlier [XYZ representation](projective-vertices.md)
preserves transformed coordinates, and tangent reprojection compensates for
pixel-center ray changes; both retain this nearest-Z selection.

A separate fixed control uses forty independent noisy observations of a 2 m
fronto-parallel plane, 10 mm Gaussian axial noise, seed 1702 and a supplied
0.10 m axial transform. It does not estimate a trajectory. Both selectors have
the same 693,112 occupied output cells, including 71,509 collision cells:

| Selection | Mean error, all cells (mm) | Mean error, collision cells (mm) | Collision-cell RMS (mm) |
|---|---:|---:|---:|
| Nearest Z | -0.57546 | -5.70939 | 10.04215 |
| Visible-surface mean | 0.01535 | 0.01711 | 7.04933 |

These pixels are correlated samples within a constructed noise model. The
result establishes a conditional raster bias, without calibrating physical
sensor noise or establishing its contribution to recorded trajectory drift.

The new raster makes two passes. The first retains the original nearest depth
`z_min` for visibility. The second admits only points with
`z - z_min < 0.03 + 0.01*z_min` metres and returns their support-weighted mean.
The band is always anchored to the original minimum: intermediate points cannot
form a chain that admits a farther surface. More distant points never enter
the average. A nearer outlier can still determine the band, and surfaces closer
than the band can mix; this remains a provisional fusion policy.

Raw points have unit support, and their aggregate retains support one. A warped
stored cell retains the maximum contributing support, capped at 16, rather than
summing frame support from overlapping rays. These weights remain heuristic.
Compatible temporal averaging, nearer replacement, farther rejection and the
cap of 16 retain their original rules. Fitting equations, information checks,
root composition, accepted clocks and expiry are unchanged.

`TrackerConfig.fusion_sampling` and JSON `Configure.fusion_sampling` accept
`nearest` (default) or `visible_mean`. Visible mean currently requires original
depth/point/forward/target-plane/central-difference fitting, stationary motion,
disabled photometric tracking, disabled step control and disabled timing.
Unsupported combinations reject before configuration acknowledgment.

## Separating supplied pose errors from map construction

The diagnostic replays all 180 raw frames in each recording and probes fifteen
fixed offsets, 1 through 169 in steps of 12, before fusion. It compares the raw
physical keyframe, a model reconstructed with the archived estimated transforms,
and two counterfactual models fused with interpolated reference transforms.
All models use exactly the archived keyframe-change schedule. Reference labels
enter this offline diagnostic only; the native tracker receives depth and its
acquisition timestamp.

The table reports median magnitudes of one linearized correction at the
reference pose, using the original fine-scale point-to-plane matcher. Translation
is in millimetres; rotation is in milliradians. These are objective probes,
not recovered trajectories or calibrated error estimates.

| Model | Room translation / rotation | Desk2 translation / rotation | RPY translation / rotation |
|---|---|---|---|
| Raw physical keyframe | 6.521 / 6.928 | 6.295 / 3.806 | 7.516 / 5.929 |
| Fusion with estimated transforms, nearest Z | 4.991 / 7.019 | 5.719 / 4.358 | 7.129 / 7.424 |
| Fusion with reference transforms, nearest Z | 8.058 / 6.647 | 10.239 / 6.548 | 11.969 / 5.548 |
| Fusion with reference transforms, visible mean | 8.241 / 6.693 | 10.423 / 6.607 | 11.772 / 5.470 |

Surface averaging changes these corrections only slightly. Using reference
transforms does not consistently reduce them. Different models can admit
different correspondences, and the registered-grid camera remains uncalibrated.
This experiment therefore does not isolate a unique drift cause or treat
reference-transform fusion as a physically exact map. All probe summaries
and replayed fusion partitions are retained in the packet.

## Complete trajectory outcomes and adoption

All runs keep original indices 100–279, K=525 intrinsics, acquisition clocks,
first-camera optical root, and 0.1 m / 0.1 rad accuracy gates. Each accepts all
179 updates without expiry or pose resets. Numeric reference poses are decoded
only after each recording's complete native sensor ledger is saved.

| Recording | Accurate, original / mean | Maximum position, original / mean (m) | Maximum rotation, original / mean (rad) |
|---|---|---|---|
| Room | 64 / 67 | 0.296290 / 0.299154 | 0.141286 / 0.142723 |
| Desk2 | 161 / 161 | 0.114800 / 0.117292 | 0.090386 / 0.090102 |
| RPY | 74 / 74 | 0.087348 / 0.088212 | 0.173375 / 0.173699 |

The frozen rule requires room above 64, desk2 at least 161, RPY at least 74,
all 179 updates accepted in each recording, and no worse maximum position or
rotation error in any recording. Counts pass; maximum-error requirements fail.
There is no parameter sweep or changed default. Complete physical accuracy
remains false for every recording.

## Verification and reproduction

The optional crate passes **56 Rust tests**. Hand controls verify weighted
surface means, background exclusion, minimum-anchored bands and frame-support
retention. Four existing ray-cast motions retain their original 4 mm / 0.004 rad
gates. A twelve-frame known-scene sequence with 2 mm axial noise exercises a
stored-model reference change and passes fixed 10 mm / 0.01 rad gates. Injecting
a rejected empty depth leaves all twelve valid outcomes exactly identical.
The separate NumPy implementation reconstructs subsequent fitted equations,
so the check also exercises the internal fused map across updates.

Recorded raw audits reconstruct **10,778 fitted and returned-pose systems** and
**537 fused-model generations**, without decoding reference poses. Analytic
mutations reject substitution of nearest-Z sampling and altered merge counts,
along with the original matrix, residual and fusion-category corruptions.
Transport tests reject **64** malformed or unsupported requests. The **1,440**
historical native sensor outputs remain exactly identical across default depth,
RGB-D and timing modes. Core verification retains **371 Rust tests**, **49**
scenario replays and **245** unchanged deterministic outputs: **427 Rust tests**
including the optional crate. CI runs dataset-free analytic controls; the full
recorded audit was run locally on this Linux host.

Use the [existing input provenance and environment](projective-depth-rust.md#reproduction)
and fresh output paths:

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-projective-fusion.py \
  --output artifacts/fusion-diagnostic
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-fusion.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/fusion-controls
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --fusion-sampling visible_mean --output artifacts/fusion-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-depth.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --fusion-sampling visible_mean --case tum-fr1-room-temporal \
  --case tum-fr1-desk2-independent --case tum-fr1-rpy-heldout \
  --output artifacts/fusion-recorded
# Expected exit 1 for retained physical accuracy failures; continue the audit.
OPENBLAS_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --fusion-sampling visible_mean --analytic-reports artifacts/fusion-analytic \
  --recorded-reports artifacts/fusion-recorded --case tum-fr1-room-temporal \
  --case tum-fr1-desk2-independent --case tum-fr1-rpy-heldout \
  --output artifacts/fusion-raw
```

The packet retains the exact diagnostic source epoch as well as the subsequent
implementation epoch. Encoded raw recordings and compiled binaries are not
redistributed. This change supplies no held-out localization, calibrated
uncertainty or vehicle integration. Subjective maturity versus mature driving
OSS remains **about 20%**, with the **30% waypoint unmet**.
