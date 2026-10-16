# Deeper bounded search on the same local objective

This optional refiner extends the search from **8 to 21 candidates**, reaching
`2^-20` of the full proposal. All other fitting math, observations, guards and
the maximum **30 applied corrections** stay unchanged. In the same ten viewed
windows, line-search failures fall from **4 to 1**, while continuous convergence
stays **2/10** and original-discrete fresh convergence stays **0/10**. Three
room windows now spend their correction budget without converging. The new
variant remains an offline local experiment.

## First recorded outcome

| Outcome across ten windows | Previous eight-candidate search | Twenty-one candidates |
|---|---:|---:|
| Continuous-objective convergence | 2 | 2 |
| Original-discrete fresh convergence | 0 | 0 |
| Missing-current-support stops | 4 | 4 |
| Line-search stops | 4 | 1 |
| Correction-budget stops | 0 | 3 |
| Total applied local corrections | 86 | 134 |

All **86** old trace entries match exactly. The **six** previous non-search
terminal reports remain completely equal. Each of the four old search failures
continues from its identical saved pose, matrix and full proposal, reproducing
all eight failed candidates before trying a smaller multiplier. Its first new
accepted multiplier matches the preceding frozen
[directional diagnosis](depth-search-diagnostic.md): 1/256 for sources
120/160/240 and 1/1024 for source 279.

| Room source | New applied corrections | New stop | Remaining full translation proposal | Remaining full rotation proposal |
|---|---:|---|---:|---:|
| 120 | 30 | Budget | 0.027937 mm | 5.1881e−6 rad |
| 160 | 30 | Budget | 0.082965 mm | 7.9869e−5 rad |
| 200 | 0 | Missing current support | Unavailable | Unavailable |
| 240 | 30 | Budget | 0.015796 mm | 1.0502e−5 rad |
| 279 | 9 | Search | 1.165563 mm | 1.1210e−3 rad |

Translation and rotation entries are maxima over non-origin nodes. Convergence
requires **every full unscaled** proposal strictly below **0.01 mm / 1e−5 rad**.
Tiny applied steps therefore do not grant convergence. For example, source
279's last applied translation is only **2.22e−6 mm**, while its remaining full
translation proposal is about **1.17 mm**. Its final search has all **21**
candidates invalid because at least one fixed source loses sampling support.
The diagnostic retains the guard and the original sources.

The desk2 recording is unchanged in full: sources 160 and 200 converge after
**16 and 19** corrections, and 120/240/279 retain missing-current-support stops.
At correction caps 10/20/30, continuous convergence is **0/2/2** across the ten
windows. Early terminal states explicitly retain actual counts.

All ten original-discrete fresh guards and 0.03 m RMS gates pass, but their
remaining proposals still fail fresh convergence. The room median fresh RMS
is **2.454214 mm**, versus previous **2.454380 mm**; desk2 stays **2.740464 mm**.
These fitting residuals do not score physical pose error. The predeclared
progression rule fails, and no new full trajectory is fit or scored. The latest
optional reciprocal trajectory scores remain **88/179 room / 75/179 desk2**
accurate, with complete physical protocols unmet.

## Numerical behavior retained

The new runs apply **48 additional** corrections, including **40** accepted
multipliers below the former 1/128 bound. Across all runs there are **97**
shortened steps and **807** candidate trials: **110 invalid**, **563 valid
nonpassing** and **134 accepted**. These work counts include the old prefixes
and are neither tracker admissions nor timing measurements.

The original Armijo slack, `1e-12 * max(1, abs(before_cost))`, remains unchanged.
It admits **two small cost increases**, both in room source 240 at updates
**23 and 27**: approximately **7.67e−13 and 9.87e−13**. Both use multiplier
**1/16384** and depend on that slack. All other accepted steps strictly lower
the reported objective. The two increases are retained explicitly; they
supply neither strict descent nor convergence evidence. Changing that
tolerance would be a separate numerical policy experiment.

## Fixed contract and independent audit

The [protocol](../assets/deep-search-depth-refinement-v1/design.json) and **79
source/context members** are frozen before recorded fitting. Use the same
source indices **120, 160, 200, 240, 279** in both recordings, the same original
initial poses and timestamps, source IDs, center normalizers and nonlinear
incoming priors. Numeric real reference labels are never parsed. This is
already viewed development evidence.

Only the operational search loop changes from `range(8)` to `range(21)`.
It still tries positive powers of one half in order and accepts the first
passing candidate. The objective, Huber weights, ownership, source retention,
prior transport, matrix, scaled solve and entire refiner body are unchanged.
There is no source reselection at any checkpoint, posterior accumulation,
damping, rank fill or state reset. Minimum factor support **200**, current-frame
measured support, SPD condition limit **1e6**, full proposal bounds **0.10 m /
0.15 rad** and strict convergence thresholds remain active. Invalid fixed
sampling returns a guard with no cost; exhausted search stops the attempt.

The independent search's only source change is likewise its loop bound.
A private instance of the frozen verifier uses this search while retaining
its original relative-camera/polynomial objective, SVD normals, incidence and
prior matrix construction, spectral solve and Rodrigues increments. The old
eight-candidate verifier is separately instantiated to recheck every control.
The checkpoint implementation and its independent audit remain unchanged.
AST source proofs and dependency identities enforce this boundary.

The audit checks **154 new-report systems** (134 iterations and 20 final
continuous/discrete systems), **106 old-control systems** and **60 checkpoint
systems**. These **320 matrix checks** include repeated control prefixes and
checkpoint/terminal states, using the same observations. They do not represent
320 independent measurements or entirely new systems. Each accepted search,
exhausted search, candidate validity and pose chain is verified. Original PNG
hashes and axial-Z conversion (`/5000` once) precede separate PCA/SVD normal
construction; image vertex construction remains shared.

Rendered measured-image initialization still converges in one correction and
its whole report equals the old control. Three additional reported systems
and six endpoint systems are checked. Two authored image-domain fixtures
verify first passage at the ninth candidate and complete 21-candidate
exhaustion; their injected directions are explicitly outside a guarded-solve
accuracy claim. Four new receipt corruptions are rejected, and the unchanged
prior controls recheck guards, full-proposal convergence, derivatives and
checkpoint ordering. CI runs the analytic checks without recorded images.

## Reproduce and inspect

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-deep-search-depth-refinement.py \
  --output artifacts/deep-search-depth-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-deep-search-depth-refinement.py \
  --output artifacts/deep-search-depth-refinement/recorded-repeat
bash scripts/check.sh
```

Use the pinned NumPy/Pillow environment, new output directories and previously
acquired hash-qualified room/desk2 PNGs. The frozen
[summary](../assets/deep-search-depth-refinement-v1/summary.json),
[statistics](../assets/deep-search-depth-refinement-v1/statistics.json) and
[validation](../assets/deep-search-depth-refinement-v1/validation.json) retain
the first complete result with source archives and a digest manifest.
Core checks pass **371 Rust tests and 49 scenario runs with complete log
replays**; all 245 core outputs and 16 pinned native estimator sources remain
unchanged. No tracker, prior, Schur or default driving state is committed by
this experiment. Automotive sensing, physical accuracy, calibrated uncertainty
and real-time performance remain unverified. Subjective maturity stays **about
20%** versus mature driving OSS; the 30% and 50% goals remain unmet.
