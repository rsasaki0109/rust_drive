# Sparse planar circle hypotheses

`fit_sparse_circles` is an optional planar perception policy. A cluster with
three or four returns can now use a circular hypothesis before the existing
five-return fit becomes available. This reduces changes between a visible
surface centroid and a fitted object center, which otherwise enter velocity
estimation as apparent motion. The policy defaults to false.

This is a geometric hypothesis within authored circular-target simulation.
Three points alone cannot establish an object's shape or calibrate uncertainty.
The unchanged 0.12 m radius padding is a heuristic; it does not guarantee unseen
physical-body containment. Noncircular objects, segmentation errors, ego pose
error and future motion remain separate problems.

## Admission and timing

Existing clustering, identity association, acquisition clocks, dense fitting and
fallback proxies remain in use. A sparse hypothesis requires all of these:

- Three or four returns in the original cluster, using every return.
- Maximum point separation of at least 0.5 m.
- Maximum triangle twice-area divided by that separation of at least 0.05 m.
- The existing algebraic circle fit, radius between 0.4 and 2.5 m, and maximum
  radial residual of 0.12 m.
- Every return on the near visible branch, allowing at most 0.12 m of outward
  radial-normal projection along its measured ray.

These are declared geometry/degeneracy heuristics selected before the physical
comparison. Unsupported or rejected sparse clusters retain the previous proxy;
points are never dropped to manufacture a successful triplet. Five or more
returns use the existing fit and clusters below three returns remain excluded.
The option rejects multi-height and 3D configurations.

Detection uses the scan's acquisition pose in continuous wheel/gyro coordinates.
Delayed scans retain their original stamps. Absolute localization corrections
only reproject published geometry; they cannot change the fitted body-frame
circle or become acquired object velocity. The ordinary tracker, measured-disc
enclosure, forecast guard and stopped clear-sweep recovery retain their duties.

## Independent checks

The Rust numerical path solves the original centered 3×3 normal equations.
`scripts/sparse_circle_oracle.py` instead eliminates the intercept and evaluates
centered sums through a direct 2×2 inverse. It reconstructs clusters and geometry
from original sensor returns, then checks each raw associated measurement in its
acquisition frame. Filtered track geometry and simulator actors do not seed it.
The preceding tracker/enclosure, prediction, braking, recovery, physical road
and clearance oracles also remain active.

Analytic Rust tests cover visible arcs at different bearings, degeneracy, the far
surface, a contradictory fourth return, order/pose invariance, sparse-to-dense
stationary tracking, delayed turning-ego acquisition and replay policy/output
corruption. Thirteen Python checks independently cover numerical geometry,
measurement corruption, lost policy and unchanged frozen worlds/error budgets.
The normal check script and both plant CI jobs include the new checks.

## Complete frozen comparison

All 32 preceding enabled [clear-sweep conditions](clear-sweep-recovery.md) retain
their original input values except `fit_sparse_circles: true`. Both plants and
seeds 7/42 remain included, with original goals, deadlines, sensor noise, error
budgets, accuracy, clearance and all five coverage horizons. Initial acquisitions
remain scored. Inputs and gates were frozen before the new physical runs.

| Measure | Preceding policy | Sparse circles |
|---|---:|---:|
| Conditions retaining every previously passed gate | — | 32/32 |
| Strict complete qualification | 0/32 | 0/32 |
| Goal arrivals | 16 | 16 |
| Original incomplete missions | 8 | 8 |
| Forecast-coverage failures | 28 | 26 |
| Localization-accuracy failures | 5 | 5 |
| Acquired proxy misses against older forecasts | 60 | 25 |
| Forecast-fault braking control ticks | 504 | 116 |
| Complete control ticks | 28,054 | 27,416 |

The two newly passing coverage conditions are reference mission seed 7 and
seed 42 with acquired heading. They still miss the original mission deadline.
Across 160 per-condition horizon comparisons, coverage fractions improve in 87,
worsen in 10 and stay equal in 63. Eight-second fractions worsen by up to
0.038748 in a release case. Counts and denominators remain in the result packet;
changed driving/termination changes the acquired sample sets, so pooled coverage
fractions are not a fixed-sample causal estimate.

The new recordings contain 13,719 acquired scans and 12,310 associated detections,
including 76 accepted sparse fits. Every one is independently reconstructed;
maximum center/radius differences are below 2e-13 m. Minimum physical swept
clearance is 2.916569 m and minimum actual road margin is 0.639097 m, with zero
collisions or road violations. All final forecast guards return to observing.

### Fixed original sensor samples

A separate post-hoc audit uses the preceding, unchanged raw returns to separate
ego/world pose error from sensor-local shape error. It keeps the legacy
association hypothesis, uses physical actor geometry only in the scorer, and
retains rejected/unmatched cases. All 156 original sparse clusters qualify in
this dataset and have unambiguous matches under the declared 2 m/0.5 m rule.

| Sensor-local metric on the same 156 clusters | Existing proxy | Circular hypothesis |
|---|---:|---:|
| Mean center error | 0.617277 m | 0.029549 m |
| Maximum center error | 0.950809 m | 0.142453 m |
| Complete physical-disc containment misses | 117 | 7 |
| Maximum containment shortfall | 0.907530 m | 0.106786 m |

The seven remaining shape misses are retained. This audit is evidence for a
bounded circular-target improvement, not a calibrated shape guarantee or a
replacement for the original world-coordinate forecast gates. The initial
decomposition also found that only 26 of 556 matched 0.2 s forecast misses
coincide with sensor-local shape misses; most short-horizon misses require
addressing ego pose/world projection as well. Long-horizon velocity, unobserved
acceleration and association failures remain.

## Evidence and remaining work

The [result packet](../assets/sparse-circle-fits-results.json) contains the full
matrix, frozen inputs, unchanged gates, source/binary hashes, fixed-sample audit
and verification receipts. Local raw records are retained under
`artifacts/sparse-circle-fits-v1`, with compressed and decoded stream hashes.
Native RNE remains pinned to `df6007aa40315e81d12ae00fc1f60369e393a178`.

The required `bash scripts/check.sh` passes 917 Rust tests, 207 Python tests and
49 standard scenario/replay pairs; all 245 standard output files retain exact
bytes. Native formatting, 67 release tests and strict all-target Clippy pass.
All 560 supported preceding logs replay with exact output bytes across 384,024
ticks. All 160 current physical/replay archives have both compressed and decoded
SHA-256 verified. Operational sources remain unchanged throughout the final
required check; Cargo locks and pinned toolchains remain unchanged.

Default adoption remains unqualified because strict acceptance is 0/32. The eight
original missions, five accuracy failures, 26 forecast-coverage failures and
seven fixed-sample shape misses remain open. No KITTI/automotive sensor data or
real-vehicle validation was added. Comparative maturity remains **about 29%**,
and the 30% waypoint is unmet.
