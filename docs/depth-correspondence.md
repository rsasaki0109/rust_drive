# Pixel correspondence stability

This optional diagnostic tracks the measured pixel pairs between consecutive
saved finest-level updates. On **100 archived continuation updates**, every
step lowers its fixed-pair Huber-plus-prior cost. Re-selecting measured pixels
then produces a higher cost than before the update in **49/100** cases. This
demonstrates local cost variation caused by reassociation on these saved
poses. It does not establish the cause of physical trajectory drift.

The tracker, its iteration caps, Rust driving pipeline and default estimator
remain unchanged. No new poses are fitted or committed, and no numeric
physical reference is parsed. The latest full reciprocal trajectory scores
remain **88/179 room** and **75/179 desk2** accurate; this diagnostic supplies
no new physical-accuracy measurement. The subjective maturity estimate remains
**about 20%** against mature driving OSS.

## Fixed inputs and observed result

Use the same source indices **120, 160, 200, 240, 279** from each already viewed
recording as the preceding
[continuation diagnostic](depth-convergence.md). That packet retains ten
additional finest-level updates from each saved accepted fit. This diagnostic
replays those exact 100 before/after pose windows and incoming priors; it does
not refit a trajectory or reinterpret them as 100 new acquisitions.

| Fifty saved updates per recording | Room | Desk2 |
|---|---:|---:|
| Fixed-pair cost decreases | 50/50 | 50/50 |
| Reassociated cost higher than the original cost | 23/50 | 26/50 |
| Before-pair occurrences retained exactly | 91.73% | 94.24% |
| Same source pixel changes its target, occurrences | 16,627 | 14,189 |
| Raw factors crossing the 200-sample admission gate | 0 | 0 |

Retention is the sum of exact retained pairs divided by the sum of before-pair
counts, over all raw node pairs and iterations, before factor admission. The
before populations contain **524,818** room and **646,674** desk2 occurrences;
**481,417** and **609,450** are retained. These overlapping counts are not
unique pixels, independent observations, known outliers or physical errors.
The aggregate intersection-over-union values are 0.8471 and 0.8911.

The [per-sample and per-edge record](../assets/depth-correspondence-v1/summary.json)
retains all populations, hashes and cost terms. Factor-admission changes do
not explain the measured cost changes in these selected intervals: every
edge remains on the same side of its admission threshold. This observation
does not exclude such changes elsewhere.

## Pair identities and cost accounting

For each raw node pair, identify a match by its flattened target pixel ID and
flattened sampled source pixel ID. Preserve the node-pair identity: the same
pixel numbers in another image pair are different measurements. Classify each
transition into exact retained pairs, source pixels that acquire a different
target, disappearing source pixels and appearing source pixels. Reciprocal
projection makes both columns one-to-one, which is checked explicitly.

The two selectors are frozen copies of the
[descent variant's](descent-keyframe-depth.md) operational and independent
selectors, adding only returned pixel IDs. All depth, normal, distance,
projection, reciprocity and stride rules remain unchanged. The geometric
counts include pairs with fewer than 200 samples; cost includes only admitted
factors. A raw node pair crossing that threshold contributes or removes its
entire cost, even when many geometric pixel pairs remain unchanged.

| Cost | Poses | Pixel pairs |
|---|---|---|
| `C0` | Saved before poses | Before selection A |
| `C1` | Saved after poses | Same selection A |
| `C2` | Same after poses | Fresh selection B |

Each cost sums the symmetric normalized-normal Huber loss with cutoff
**0.01 m**, plus the unchanged incoming prior's nonlinear transported cost.
Its additive prior constant follows the original zero convention. Record:

```text
fixed-step change   = C1 - C0
reassociation change = C2 - C1
total change        = C2 - C0
```

At the same after poses, retained pixel pairs have identical measured
points/normals and losses. Therefore the reassociation change equals the
loss of added admitted pairs minus the loss of removed admitted pairs. The
prior is identical in `C1` and `C2` and cancels in that difference. Check this
partition for every edge and for the complete window.

All 100 fixed-step changes are negative. The reported increase count requires
`C2 - C0 > 1e-12 * max(1, abs(C0))`. The largest increases are approximately
**0.00086035** room and **0.00220832** desk2 in the summed objective's units.
Since `C2` uses a different population, this is not a comparison of the same
fixed objective or a calibrated likelihood. It does not identify which
correspondences are physically correct, distinguish calibration/model/prior
bias, or prove reassociation causes accumulated pose error.

## Independent verification

The operational path uses explicit `(target ID, source ID)` dictionary keys
and root-frame losses. The separate oracle uses structured integer arrays,
array intersections, relative-camera losses, separately branched Huber
evaluation and quaternion-based prior coordinates. It independently rebuilds
normals with patch SVD rather than operational covariance eigendecomposition.
Original depth decimation, vertices, intrinsics and the already validated
selector code remain shared dependencies.

All **960** raw node-pair transitions are independently compared: 460 room and
500 desk2. The checker also reconstructs the **100** saved iteration systems
and **10** saved final systems, including the recorded proposals and pose
chaining. These are reconstructions of existing evidence, not new fits.

Controls cover identity and 2 cm shifts of rendered depth at all three scales.
A hand-counted equal-population fixture contains 197 retained pairs, one
retargeted source, two disappeared sources and two appeared sources. Separate
199-to-200 and 200-to-199 fixtures verify whole-factor admission accounting;
four empty-population transitions are checked. These injected ID fixtures
test bookkeeping and are not claimed as image measurements. Both paths
reject duplicate reciprocal IDs. Eight report corruptions are rejected,
covering counts, retargeting, admission, costs, hashes and prior accounting.

The mandatory core check passes **371** Rust tests and **49** scenarios with
complete sensor-log replays. All **245** deterministic core outputs, **357**
previous evidence files and 16 pinned preceding native sources keep their
hashes. CI includes the new controls and syntax checks.

## Reproduction

Use the pinned optional NumPy/Pillow environment, existing hash-pinned TUM
depth images and archived continuation packet. Output directories must be
fresh:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-depth-correspondence.py \
  --output artifacts/correspondence-controls-replay
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-correspondence.py \
  --output artifacts/correspondence-recorded-replay
```

The [design](../assets/depth-correspondence-v1/design.json) and source archive
freeze **41** source/context members before the real-image replay. Both
commands return zero when integrity checks pass, regardless of whether the
reassociated cost decreases. The
[validation receipt](../assets/depth-correspondence-v1/validation.json) and
[manifest](../assets/depth-correspondence-v1/files.json) retain source/input
hashes, controls, counts and failure limits. Original images are not
redistributed; [TUM attribution and terms](tum-source-provenance.md).
