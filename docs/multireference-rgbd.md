# Optional multi-reference RGB-D experiment

The optional `multireference` module in `integrations/projective-depth` adds
bounded registration against raw measured keyframes, with a separately selected
phase-qualified intensity term. It can recover textured planar motion that
depth-only fitting rejects as unobservable. The viewed recorded experiments
do **not** improve accuracy across both recordings. The original fused-depth
tracker remains the default, and the engineering maturity estimate stays
**about 20%**.

## Fixed comparisons and retained failures

All comparisons use the same 180 original depth acquisitions, indices 100–279,
one first-camera origin, 179 update denominator, 0.10 m / 0.10 rad root gates,
and at most 20 ms reference interpolation brackets. Depth units are divided
by 5000 once. Numeric reference labels do not enter fitting. These recordings
were already viewed during development; no held-out claim is made.

| Fixed development variant | Room accurate / 179 | Desk2 accurate / 179 |
|---|---:|---:|
| Original fused depth, K525 | 64 | 161 |
| Fused depth plus phase-qualified intensity | 78 | 75 |
| First NumPy raw multi-reference geometry | 70 | 74 |
| First NumPy raw multi-reference geometry plus intensity | 62 | 72 |
| Canonical pixel-knot NumPy/Rust geometry plus intensity repeat | 69 | 72 |
| Original fused depth, published TUM1 intrinsics without distortion | 63 | 132 |

Every branch accepts all 179 updates without expiry. Acceptance and low local
residuals therefore do not establish accurate root poses. No candidate in this
table improves both records, and none passes complete accuracy. Their fixed
designs, source snapshots, sensor-only ledgers and scored failures remain in
[phase-aware RGB-D](../assets/phase-aware-rgbd-v1/),
[multi-reference RGB-D](../assets/multireference-rgbd-v1/) and
[camera interpretation](../assets/depth-camera-profile-v1/).

The phase-plus-fusion and multi-reference prototypes finish both records, or
all four branches respectively, before numeric label scoring. The separate
camera diagnostic finishes each record's operational run before its scoring;
it retains the original evaluator's ordering. This diagnostic uses published
`fx=517.306408, fy=516.469215, cx=318.64304, cy=255.313989` without applying
distortion or another depth correction. It is a calibration interpretation
comparison, without a claim of newly measured camera calibration.

## Implemented estimator contract

`MultiReferenceTracker::new(camera, Mode)` explicitly selects `Geometry` or
`PhaseGatedRgbd`. `step(depth_timestamp, depth_m, image)` returns acquisition
identity, acceptance/loss, the accepted pose and clock, selected reference
timestamps, keyframe changes, all combined normal equations and increments,
and a final equation re-evaluated at the returned pose. Poses map the current
optical camera into the first optical camera: x right, y down, z forward,
metres, right-handed proper rotation.

References retain the first frame, up to three newest motion-selected
keyframes, and the last accepted frame if distinct, for at most five views.
Keyframes change above 0.10 m or 0.10 rad relative to the latest selected key.
When four selected keys are already retained, the oldest key after the first
is evicted. References contain their original measured depth and associated
image; there is no depth fusion or synthesized image template.

Point-to-plane Huber residuals use a 0.005 m engineering scale. Optional image
residuals use exact bilinear values and derivatives, a 10-level engineering
scale, Tukey cutoff 30, gradient magnitude at least 1, and at least 200 retained
image correspondences per reference. These scales are tuning constants, not
calibrated noise or uncertainty estimates. Both Jacobians are expressed for
left increments of the first-camera pose before accumulation.

The image term requires a strictly new RGB timestamp relative to **every
observed RGB timestamp**, and a difference in RGB-minus-depth offsets no larger
than 5 ms against each reference. This phase test does not compensate exposure
or capture-time motion. On these recordings the RGB/depth offsets are roughly
15–17 ms and associated RGB images sometimes repeat. Repeated RGB contributes
no photometric equation. Association must be within 20 ms, RGB timestamps must
be finite, nonnegative and nondecreasing, and grayscale arrays must match the
camera. A malformed image can consume an observed RGB timestamp, without
refreshing accepted state. Geometry mode requires depth alone and rejects an
image argument.
`photo_enabled` reports frame-level RGB freshness permission; each reference's
phase qualification and actual contributing counts are reported separately.

The pyramid has scales 4/2/1, source strides 1/2/4, and iteration caps 10/7/5.
Geometry requires at least 200 total correspondences, diagonally scaled SPD
condition at most 1e6, increments at most 0.10 m / 0.15 rad, final geometric
RMS at most 0.03 m and a displacement from the last accepted pose at most
0.50 m. Correspondences use the original tracker's depth/surface gates and
ties-to-even projection. Work and reference counts are bounded; deadlines
and real-time performance have not been measured.

Depth acquisition timestamps are strictly new. Rejected data cannot modify
accepted poses, selected references or the accepted clock. Age above 0.20 s
on a subsequent depth acquisition latches loss; there is no automatic reset.
This experimental tracker has no idle-tick watchdog. The existing
[explicit-clock supervisor](projective-depth-supervision.md) continues wrapping
the original fused-depth tracker. Neither path supplies vehicle controls.

## Numerical convention repair

The first Rust geometry-only port matches the NumPy trajectory within
1.91e-13 m, but the first image-enabled port differs by up to 0.05917 m.
The first Rust image-enabled result is 66/179 room and 73/179 desk2, rather
than the first NumPy prototype's 62/179 and 72/179. An independent raw-pixel reconstruction rejects the combined matrix at the
third acquisition even though the geometric matrix and image support counts
agree. The first Rust sources, all four results, the failing raw diagnostic
and its output are retained in this packet.

At an integer pixel coordinate, a bilinear interpolant has different one-sided
spatial derivatives. Forward/inverse pose composition can move an otherwise
integer sample by a few floating-point bits and select the adjacent derivative
cell. The [declared numerical repair](../assets/multireference-rgbd-v1/numeric-convention.json)
canonicalizes image sample coordinates within 1e-10 pixel of an integer before
bounds, interpolation and differentiation, consistently choosing the right/down
cell. This changes the numerical convention at pixel knots, without changing
physical pose gates, reference policy or source selection. The separately
retained canonical NumPy repeat uses the same convention. The first prototype
and first Rust trajectories are not overwritten.

## Independent checks and evidence

Six added Rust tests cover exact interpolation and pixel-knot derivatives, textured-plane observability,
repeated/phase-incompatible images, malformed input, accepted-state retention
and terminal expiry. All 18 optional library tests pass.

The analytic CLI checker reconstructs **270** geometric/combined equations
from independently generated pixels, across seven sequences and **241** fitted
increments. Four two-frame sphere/plane scenes test optical directions,
rotation and metres; a textured plane adds previously unobservable motion;
an 18-frame scene reaches the five-reference budget and evicts old keys while
retaining the first origin. A further textured-plane sequence rotates and
translates image references, exercising nonidentity image Jacobians and the
pixel-knot convention. Known poses must agree within 4 mm / 4 mrad.
Ten report corruptions and thirteen malformed transport cases are rejected.
The raw check catches joint scaling of H/g that a solve-only check would miss.

The recorded Rust evaluator finishes all four operational branches before
decoding numeric ground truth or comparing the saved NumPy prototype. It
independently checks every reported combined solve, Rodrigues increment,
reference partition, key eviction and acquisition clock. The canonical repeat
also reconstructs every reported geometric and combined matrix from the
original decoded depth/image pixels and retained reference poses before numeric
reference scoring. Recorded Rust results, prototype
disagreement and this scope distinction are retained in the
[validation receipt](../assets/multireference-rgbd-v1/validation.json).

The final recorded check reconstructs **15,915** equations: **15,199** fitted
iteration systems and **716** returned-pose final systems. All geometric and
image support counts agree exactly. Every canonical Rust trajectory agrees
with the independent NumPy repeat within **2.11e-13 m**; the maximum
trace/arccos rotation disagreement is **1.40e-7 rad**. Geometry scores 70/179
room and 74/179 desk2; image-enabled fitting scores 69/179 and 72/179. The
image-enabled desk2 maximum position error is **0.525899 m**. These integrity
checks pass while all four complete physical-accuracy protocols fail with
exit 1. There is no default promotion or maturity increase.

The original estimator, supervisor, transports and Cargo pins match their
previous source archives, apart from exporting the new module. Their analytic
checks still pass. `bash scripts/check.sh` passes all 371 core Rust tests and
49 scenario executions/replays; all 245 deterministic core output files match
their preceding hashes. The optional CI job adds the new analytic checker
without downloading these recorded datasets.

## Reproduction

```bash
source scripts/env.sh
cargo test --all-targets --locked --manifest-path integrations/projective-depth/Cargo.toml
cargo build --release --bins --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/check-multireference-rgbd.py \
  --binary integrations/projective-depth/target/release/rustdriving-multireference-rgbd \
  --output artifacts/multireference-check
```

The separate CLI accepts one `Configure` request with `camera` and `mode`
(`geometry` or `phase_gated_rgbd`), then `Frame` requests with `timestamp`,
row-major `depth_m`, and an optional `image: {timestamp, gray}`. Grayscale is
an array of bytes; recorded RGB conversion is `(77R+150G+29B)>>8`. Input JSON
lines are bounded to 8 MiB and at most 200 acquisitions. Unknown fields and
invalid transport terminate with exit 2. Estimator rejection is a returned
measurement outcome. Successful transport exits 0 independently of accuracy.

With the existing hash-pinned dataset manifests and raw files prepared by
the [original diagnostic](projective-depth-diagnostics.md), restore the frozen
prototype comparison ledgers at repository root, then use a fresh output:

```bash
tar -xzf assets/multireference-rgbd-v1/prototype-evidence.tar.gz
tar -xzf assets/multireference-rgbd-v1/canonical-prototype-evidence.tar.gz
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-multireference-rgbd.py \
  --binary integrations/projective-depth/target/release/rustdriving-multireference-rgbd \
  --prototype-root artifacts/multireference-rgbd-canonical-dev/first \
  --output artifacts/multireference-recorded-repeat
# Expected exit 1: the complete physical-accuracy gates fail.
```

Source archives retain the ignored prototype programs and the exact first-run
Rust/evaluator files. Packet `files.json` inventories bind their digests; raw
dataset images and numeric reference files are not redistributed in these
packets. Broader environments, driving integration, loop closure, measured
extrinsics and calibrated uncertainty remain unverified.
