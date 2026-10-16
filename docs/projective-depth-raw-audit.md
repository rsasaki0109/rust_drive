# Independent raw-depth reconstruction of projective odometry

A separate NumPy implementation now reconstructs depth pyramids, surface
normals, projective correspondences, robust normal equations and fused depth
models from the original PNG files. It audits the archived Rust operational
reports on both viewed recordings. All **6,861 iteration equations**, **358
returned-pose equations** and **358 fused model generations** pass, with exact
discrete correspondence and raster/fusion counts.

This closes the earlier [port audit's](projective-depth-rust.md) explicit
raw-correspondence gap. It establishes mathematical agreement with the declared
algorithm, rather than additional physical position accuracy. The original
room **64/179** and desk2 **161/179** accuracy failures remain unchanged.
The subjective maturity estimate stays **about 20%**; the **30% waypoint
remains unmet**.

## Inputs and independent reconstruction

[`audit-projective-depth-raw.py`](../scripts/audit-projective-depth-raw.py) reads
the immutable original sensor-only ledgers and freezes its sources and input
digests before recorded pixel decoding. Each recording retains original depth
indices 100–279: one initialization and 179 updates. Prior evidence hashes,
manifest hashes, PNG hashes, frame identities and dimensions are verified.
Numeric ground truth and RGB pixels are not decoded.

The [design](../assets/projective-depth-raw-audit-v1/design.json) fixes the
scope and tolerances before the audit. Axes are optical x right, y down, z
forward; PNG axial depth is divided by 5,000 to obtain metres. The fixed
registered-grid pinhole model remains the documented uncalibrated default.

[`projective_depth_raw.py`](../scripts/projective_depth_raw.py) implements the
geometry independently with vectorized arrays. It imports no operational
fitter and does not execute the exploratory Python algorithm. For every frame
it rebuilds the depth/support model carried forward from the preceding frame.
It reconstructs all three pyramid levels, discontinuity masks and surface
normals. For every reported iteration pose, it computes the nearest-pixel
associations, support/normal gates, Huber residuals, **H**, **g** and RMS directly
from those reconstructed surfaces. It repeats that calculation at the returned
pose for final admission.

Reference-view changes reproject the model using the reported accepted
transform. An independent lexicographic sort selects the nearest valid axial
depth, breaking exact depth ties by original source index. The checker
reconstructs compatible-depth averages, capped support weights, replacements
and ignored observations, verifying every reported raster/fusion category.
Its reconstructed model supplies the next frame's geometry.

The existing independent linear-algebra/state audit also verifies solves,
increments, root composition, reference changes and acquisition/expiry clocks.
Reported poses are therefore checked mathematical witnesses; they are not
supplied as independent physical motion truth.

| Quantity | Fixed numerical comparison |
|---|---|
| Correspondence count | Exact, zero count tolerance |
| Raster and fusion counts | Exact, zero count tolerance |
| Normal matrix H | Absolute 1e-7 plus relative 1e-10 |
| Gradient g | Absolute 1e-9 plus relative 1e-10 |
| Residual RMS | Absolute 1e-10 m |

The first complete audits require no tolerance or operational gate changes.

## Results and controls

| Recording | Raw frames | Iteration equations | Final-pose equations | Fused generations | Reference-view warps |
|---|---:|---:|---:|---:|---:|
| Room | 180 | 3,411 | 179 | 179 | 44 |
| Desk2 | 180 | 3,450 | 179 | 179 | 42 |

The [validation receipt](../assets/projective-depth-raw-audit-v1/validation.json)
and per-recording audits retain all frame identities and model-support counts.
Original depth, fitting sources, acceptance gates and preceding evidence remain
unchanged. This audit adds no estimator/runtime dependency.

Seven hand-computable controls verify metre/axis/normal conventions, half-pixel
pyramid centers, invalid/discontinuous reduction, raster ties and occlusion,
all four fusion categories, capped-weight arithmetic and a 225-point plane's
normal equation. Four analytic ray/sphere motions also reproduce all raw
equations and fusion counters. The CI invocation additionally audits fresh
reports from the current locked Rust binary.

Six deliberate report corruptions are rejected. One rescales both H and g,
leaving the solved increment unchanged: the preceding solve-only auditor
accepts it, while raw-pixel reconstruction rejects it. Other controls alter
support, per-iteration or returned-pose RMS, or move a fusion sample between
categories while preserving the total count. This demonstrates why valid
linear algebra alone does not bind a fit to its input image.

## Reproduction and limits

Acquire the existing pinned [room](temporal-recorded-motion.md) and
[desk2](independent-recorded-motion.md) inputs and use the environment from
`scripts/requirements-visual.txt`:

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python scripts/audit-projective-depth-raw.py \
  --output artifacts/projective-depth-raw/local
```

The output directory must be fresh. Exit 0 means the selected mathematical
audit passes; it is not an accuracy acceptance result. `--analytic-only`
requires no recordings. To audit fresh analytic Rust reports instead of the
archived controls, add `--analytic-reports <check-projective-depth-output>`.
The full recorded mode audits the fixed archived original sensor ledgers.

The checker reconstructs the model prescribed by the algorithm and checks its
reported consequences at every following iteration. Rust's internal model
buffer is not exported, so this is not a direct byte comparison of that
buffer. Pillow performs PNG decoding in both data paths; the independent
portion is the geometry, association and fusion calculation. The audit does
not establish a globally optimal pose, correct physical calibration,
calibrated uncertainty, new-environment accuracy or driving integration.
All recordings remain viewed development data. Only local Linux execution is
claimed; no real-time or real-vehicle safety result follows from these checks.
