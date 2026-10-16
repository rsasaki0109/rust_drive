# C1 inverse-depth interpolation and initial support margins

The first cubic comparison retains all **32,507** initial source occurrences
but regresses continuous-objective convergence from **2/10 to 0/10**. Every
observable window eventually loses complete cubic sampling support. A separate
frozen follow-up qualifies neighboring cubic stencils before fitting. It
converges on the previously stalled **room source 240 in nine corrections**,
but retains only **49.94%** of initial source occurrences and converges on
**1/10** windows overall. Neither branch meets the progression rule or becomes
the tracking default.

## Recorded comparisons

| Ten viewed windows | Bilinear deeper-search control | Cubic center support | Cubic initial margin |
|---|---:|---:|---:|
| Continuous-objective convergence | 2 | 0 | 1 |
| Original-discrete fresh convergence | 0 | 0 | 0 |
| Missing-current-support stops | 4 | 4 | 7 |
| Line-search stops | 1 | 6 | 2 |
| Budget stops | 3 | 0 | 0 |
| Total applied corrections | 134 | 40 | 50 |
| Initial source occurrences | 32,507 | 32,507 | 16,234 |
| Initial factor occurrences | 36 | 36 | 19 |

The first cubic run's entire source metadata, including every original metric
normalizer, equals the bilinear control at all ten initial states. Its six
failed final searches each reject all **21 candidates** for lost sampling
support. Across fitting there are **729 invalid / 40 accepted / zero valid
nonpassing** candidates. Each accepted step strictly lowers its own objective.
All windows stop before correction ten, so later checkpoints explicitly repeat
their actual terminal counts.

The follow-up changes only initial admission to require nine neighboring cubic
stencils. Retention is **1,939/10,156 = 19.09% room** and **14,295/22,351 = 63.96%
desk2**. This includes the minimum-200 factor admission rule; it is not just
a per-pixel rejection rate. Room source 160 retains 427 older-view observations
but no measured current-frame factor. Desk2 source 120 likewise retains 299
older observations and still lacks current support. Older measurements cannot
stand in for a new acquisition.

| Source index | Cubic center room | Margin room | Cubic center desk2 | Margin desk2 |
|---|---|---|---|---|
| 120 | Search after 8 | Missing support | Missing support | Missing support |
| 160 | Search after 6 | Missing support | Search after 6 | Search after 16 |
| 200 | Missing support | Missing support | Search after 8 | Search after 25 |
| 240 | Search after 6 | Converged after 9 | Missing support | Missing support |
| 279 | Search after 6 | Missing support | Missing support | Missing support |

Room source 240's remaining **full** proposal is **0.009401 mm / 9.0722e−6 rad**,
below the unchanged strict **0.01 mm / 1e−5 rad** thresholds. All nine accepted
steps have multiplier one. Original-discrete freshly associated convergence
still fails there. Because both interpolation and source selection differ
from the original bilinear fit, this success does not isolate a benefit of
smooth interpolation or identify physical outliers among excluded samples.

The margin run has **282 invalid / 50 accepted / zero valid nonpassing**
candidates, with 23 shortened steps and no accepted cost increases. Its two
failed final searches also have 21 invalid trials. New cap-10/20/30 continuous
convergence is **1/1/1**; desk2 attempts continue until their later domain stops.

All original-discrete fresh support, conditioning and 0.03 m RMS gates pass.
Room median fresh RMS is **2.454214 mm** in the bilinear control, **2.477670 mm**
with cubic center support and **2.483834 mm** with the cubic margin. Desk2's
median stays **2.740464 mm**. These are fitting residuals, without physical
pose-error scoring. Numeric real reference labels are never parsed and no
continuous trajectory is refit. Per-window selection of different algorithms
would be another policy; successes are not combined into a better headline.

## Cubic model and first frozen protocol

The [first design](../assets/cubic-depth-refinement-v1/design.json) and **91
source/context members** are archived before fitting. Use the same initial
poses, timestamps, incoming nonlinear priors and sample indices
**120/160/200/240/279** in both recordings as the
[deeper-search control](deep-search-depth-refinement.md).

Tensor Catmull-Rom interpolation samples a **4×4 inverse-axial-depth stencil**.
It preserves measured values at grid points and provides continuous first
derivatives across jointly qualified adjacent cells. Exact projective
derivatives form the existing root-frame incidence blocks. There is no integer
snapping, bilinear fallback or clipping of interpolated values.

All sixteen depth values must be between **0.3 and 5 m**, with valid measured
normals. Each of the nine overlapping 2×2 subpatches must meet the original
`max_depth - min_depth <= 0.05 + 0.01 * min_depth` condition. Interpolated
inverse depth must be finite and positive. Negative cubic weights can produce
overshoot, so continuity alone does not certify physically correct surface
shape. The field is C1 where adjacent stencils are qualified; holes and domain
boundaries still reject candidates.

Start with the unchanged original sampling-domain population and its geometric
matching and bilinear nine-probe qualification. Filter for cubic center support,
requiring at least 200 sources per factor. Survivors preserve their original
bilinear center gradient normalizers. Those weights remain engineering metric
scales rather than calibrated noise or covariance estimates. The actual first
run loses no initial source, while candidate-time qualification becomes stricter.

Freeze that population and its weights for the whole solve. The incoming
nonlinear prior, Huber loss, full GN proposal, **21** Armijo multipliers through
`2^-20`, numerical slack, maximum **30** corrections and exact 10/20/30
checkpoints remain unchanged. Current-frame support, SPD condition **1e6**,
full proposal **0.10 m / 0.15 rad** and strict full-proposal convergence remain
active. No source is removed during fitting to lower the cost, and no damping,
posterior accumulation, tracker or Schur commit occurs.

## Separately frozen initial-margin follow-up

After preserving the first failure, the
[follow-up protocol](../assets/cubic-margin-refinement-v1/design.json) and **95
source/context members** are frozen before another fit. This is an
outcome-informed development follow-up, not held-out validation.

At initialization only, test the cubic sampler at the projected center and
eight target-image offsets of one pixel, holding target optical Z fixed.
Retain the intersection of source identities and factors with at least 200
survivors. Probes are admission checks, without new factors or weights. The
original normalizers, point coordinates and normals are checked exactly for
all survivors. Finite probes do not certify every intervening or future pose.

Private unchanged solver/objective/checkpoint functions bind this new initial
population. A separate private verifier uses homogeneous-image offsets and the
independent cubic sampler. Original center-only cubic verification stays
separate. Thus the follow-up isolates an initial selection change within the
cubic method, while comparison with bilinear still changes the objective and
sampling domain.

## Independent evidence and reproduction

The operational sampler uses explicit four-weight tensor products. The oracle
uses sequential cubic Hermite interpolation with central endpoint slopes and
homogeneous projection. Separate relative-camera residuals, SVD normals,
incidence/prior matrices, spectral solves and Rodrigues increments check every
reported system, candidate, stop and checkpoint. PNG hashes and axial Z `/5000`
conversion precede both constructions; image vertices remain shared.

Analytic checks cover quadratic inverse-depth reproduction, integer values,
left/right derivatives at an authored kink, XYZ finite differences, missing
outer-corner rejection, and all **12** nonzero-prior objective-gradient axes.
Rendered center-only fitting fails its search after nine corrections; the
margin fit retains **887/988** sources and converges in one. Degenerate images
retain guards. Four center-only and three additional margin corruptions are
rejected. AST/dependency proofs retain unchanged solver and audit bodies.

The first run checks **60 new-report / 154 control / 60 checkpoint systems**.
The follow-up checks **70 new-report / 60 control / 60 checkpoint systems**.
These roles share observations, replay controls and repeat terminal states;
they are not independent measurements or entirely new equations. Original
source archives, complete first reports, selection receipts and failed outcomes
remain separately frozen.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-cubic-depth-refinement.py \
  --output artifacts/cubic-depth-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-cubic-depth-refinement.py \
  --output artifacts/cubic-depth-refinement/recorded-repeat
OPENBLAS_NUM_THREADS=1 python scripts/check-cubic-margin-refinement.py \
  --output artifacts/cubic-margin-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-cubic-margin-refinement.py \
  --output artifacts/cubic-margin-refinement/recorded-repeat
bash scripts/check.sh
```

Use the pinned NumPy/Pillow environment, new output directories and previously
acquired hash-qualified PNGs. Inspect the frozen
[first summary](../assets/cubic-depth-refinement-v1/summary.json),
[first validation](../assets/cubic-depth-refinement-v1/validation.json),
[follow-up summary](../assets/cubic-margin-refinement-v1/summary.json) and
[follow-up validation](../assets/cubic-margin-refinement-v1/validation.json).
CI includes both analytic suites. Core checks pass **371 Rust tests and 49
scenarios with complete sensor-log replays**, retaining all 245 core-output and
16 pinned native-source digests. Neither experiment establishes new physical
accuracy, operational integration or timing. Subjective maturity remains
**about 20%** versus mature driving OSS; 30% and 50% remain unmet.
