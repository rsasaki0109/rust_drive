# Why fixed and fresh depth convergence disagree

The saved [admission experiment](slope-admission-refinement.md) converges on
**9/10 fixed continuous objectives**. At those same terminal poses, both the
original discrete fresh objective and a freshly selected version of the same
continuous model converge on **0/10**. This diagnostic applies no pose updates.

The original fresh check changes source membership, rounded target pixels and
the residual model. It is therefore a different stationarity test. Fresh
selection within the continuous model also changes source membership and
bilinear-center normalizers. Neither result establishes physical pose error.
All previous convergence requirements and failed progression evidence remain.

## What changes at the same saved pose

Across the ten windows, the continuous fit contains **60,288 source
occurrences**; the original discrete fresh check contains **116,383**:

| Membership at each saved terminal pose | Occurrences |
|---|---:|
| Shared source identities | 55,814 |
| Fixed sources absent from the fresh check | 4,474 |
| Fresh sources absent from the fixed fit | 60,569 |
| Shared sources assigned a different rounded target pixel | 16,532 |

An identity includes target node, source node and linear source-image pixel.
Shared identities retain exactly the same measured source point and normal.
The target pixel can change independently. Counts overlap across factors and
windows; fresh-only sources are observations from existing frames, without
new independent acquisitions. In particular, they can have been excluded by
the continuous fit's stricter initial admission policy.

The room source-200 window still has no admitted continuous factor, at either
initial or terminal selection. Its original discrete fresh check has 817
source occurrences. This guard remains a failed window, rather than a
converged prior-only fit.

## An exact, ordered gradient bridge

At each saved pose, subtract the fixed continuous raw gradient from the fresh
discrete raw gradient. The same nonlinear prior appears once at each endpoint
and cancels. The implementation reconstructs this difference in five steps:

1. Remove fixed sources absent from fresh selection.
2. On shared sources, replace the continuous residual's Huber influence with
   that of the original-target symmetric point-to-plane residual, holding the
   continuous Jacobian fixed.
3. Replace that Jacobian with the rotating symmetric measured-normal Jacobian.
4. Replace the original rounded target pixel with the terminal rounded target.
5. Add fresh-only sources.

The residual-influence term includes the different residual model, frozen
engineering metric normalization and Huber clipping. The Jacobian term changes
the projective field gradient to the measured-normal differential. Intermediate
combinations serve additive accounting; they are not solved as new objectives.
Subsets below 200 sources never bypass the original factor-admission threshold.

The maximum gradient partition error is **2.56e−15** over all ten windows.
This is observed floating-point agreement, without a certified error bound.
The decomposition depends on the declared order; it does not identify unique
physical causes or justify removing any observation.

To compare components without mixing raw metre/radian derivatives, use one
common coordinate scale: the reciprocal square root of the complete fresh
system's Hessian diagonal. The signed projection of each scaled component
onto the scaled total difference gives the following fractions over the
**nine locally converged windows**:

| Ordered term | Minimum | Median | Maximum |
|---|---:|---:|---:|
| Removed fixed sources | −0.210 | 0.003 | 0.252 |
| Shared residual influence | −0.208 | 0.059 | 0.450 |
| Shared Jacobian | −0.322 | −0.014 | 0.293 |
| Shared target reassignment | −0.583 | −0.027 | 0.160 |
| Fresh-only sources | 0.750 | 0.974 | 1.452 |

Fractions sum to one within each window, with cancellation permitting negative
values or values above one. Column medians need not sum to one. These are
engineering-scaled vector projections, not causal percentages, calibrated
uncertainty or physical accuracy. The room source-200 control has only the
fresh-only term, with projection one, and is excluded from this table.

## Selection changes stationarity even within the same model

Reselect the original continuous admission policy at the saved final pose,
including its usual bilinear-center normalizers, then evaluate one guarded
full proposal. No proposal is applied. The nine converged windows contain
60,352 reselected continuous sources versus 60,288 fixed sources, but none
retains strict convergence after reselection.

The following values are maximum full-proposal translation in **mm** and
rotation in **mrad** across active nodes. They are remaining numerical
corrections, without reference-pose error scoring:

| Window | Original discrete fresh: mm / mrad | Reselected continuous: mm / mrad |
|---|---:|---:|
| Room / 120 | 1.532 / 1.385 | 0.381 / 0.337 |
| Room / 160 | 0.987 / 0.850 | 0.303 / 0.733 |
| Room / 200 | 0.875 / 0.409 | Current-support guard |
| Room / 240 | 0.304 / 0.307 | 0.237 / 0.148 |
| Room / 279 | 1.849 / 1.275 | 0.697 / 0.435 |
| Desk2 / 120 | 1.279 / 1.717 | 0.798 / 0.977 |
| Desk2 / 160 | 0.699 / 0.459 | 0.475 / 0.329 |
| Desk2 / 200 | 1.966 / 1.428 | 0.336 / 0.257 |
| Desk2 / 240 | 4.498 / 1.921 | 6.045 / 2.211 |
| Desk2 / 279 | 2.269 / 0.964 | 0.674 / 0.346 |

The unchanged strict convergence limits are **0.01 mm / 0.01 mrad**. All
original discrete fresh solves pass their support, condition and increment
guards. The nine reselected continuous solves also pass those guards, while
the room source-200 support guard remains. Reselection changes both membership
and weights; this control does not isolate either change alone. Additional
fixed-objective iterations cannot by themselves establish stationarity for
these changed objectives. No reselection-and-refit algorithm is implemented
or validated here.

## Independent evidence and reproduction

The [design](../assets/depth-objective-bridge-v1/design.json) and **126
source/context members** are archived before recorded diagnosis. Inputs are
the same viewed room/desk2 windows at source indices 120/160/200/240/279,
with initial poses, original frozen populations, terminal poses and incoming
prior preserved. This is outcome-informed development, without held-out data.

Operational per-factor array joins and direct local-vector terms are checked
against dictionary identity sets, global incidence Jacobians and differences
between six independently accumulated gradient stages. Separate relative-camera
reciprocal selection, SVD normals, polynomial Hermite interpolation,
projector-based rotating-normal derivatives, prior reconstruction and spectral
solves check **30 endpoint systems**. These include **20 existing archived
systems**, rather than 50 independent systems or new measurements. Original
PNG digests and axial-Z conversion `/5000` once precede both constructions;
decoded observations remain shared.

Authored controls force every term to be nonzero, exercise Huber tails,
retargeting, source removal/addition and two moving endpoints. Identical/empty
populations give zero differences and undefined projection fractions. Four
receipt corruptions and four invalid inputs are rejected. Rendered surfaces
with a nonzero prior supply three endpoint checks and **24 numerical gradient
axes**, with correspondences fixed during differentiation. CI runs these
controls without recorded data.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-objectives.py \
  --analytic-only --output artifacts/depth-objective-bridge/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-objectives.py \
  --output artifacts/depth-objective-bridge/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, existing hash-qualified PNGs and new output directories.
The [summary](../assets/depth-objective-bridge-v1/summary.json),
[statistics](../assets/depth-objective-bridge-v1/statistics.json) and
[validation](../assets/depth-objective-bridge-v1/validation.json) preserve the
first complete diagnosis. Core checks pass **371 Rust tests and 49 scenarios
with complete sensor-log replays**; all 245 core-output and 16 pinned native
source digests remain unchanged.

There is no new recorded pose fitting, trajectory, numeric real reference
label, physical accuracy score, tracker/prior/Schur commit or native/default
adoption. Vehicle integration and timing remain unverified. Subjective maturity
stays **about 20%** versus mature driving OSS; **30% and 50% remain unmet**.
