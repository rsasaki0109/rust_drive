# Conditional damped-direction comparison

This optional local refiner tries alternative directions when the existing
Gauss-Newton (GN) search accepts a multiplier below **1/16** or exhausts its
21 candidates. It selects **32** damped directions in the viewed room windows,
but continuous convergence stays **2/10** and original-discrete fresh convergence
stays **0/10**. All three previous budget stops remain. Their final objectives
are slightly higher than the control, and the room median fresh RMS regresses.
The progression rule fails; this branch is retained as a failed comparison.

## First recorded outcome

| Outcome across ten windows | Deeper-search control | Damped alternatives |
|---|---:|---:|
| Continuous-objective convergence | 2 | 2 |
| Original-discrete fresh convergence | 0 | 0 |
| Missing-current-support stops | 4 | 4 |
| Line-search stops | 1 | 1 |
| Thirty-correction budget stops | 3 | 3 |
| Total applied local corrections | 134 | 139 |
| Accepted tiny cost increases within original GN slack | 2 | 9 |

Desk2 is completely unchanged after removing the additional receipt wrapper:
sources 160 and 200 converge at **16 and 19** corrections, and three windows
retain missing-current-support guards. Room source 200 is likewise unchanged.
These **six complete reports** equal the control. Across the changed room fits,
**44** trace entries precede the first selected alternative and equal their
control entries exactly; desk2 contributes **35** more unchanged entries.
Later states differ by design and are independently reconstructed.

| Room source | Applied corrections | Stop | Remaining full translation proposal | Remaining full rotation proposal | Final objective minus control |
|---|---:|---|---:|---:|---:|
| 120 | 30 | Budget | 0.026595 mm | 1.9485e−5 rad | +6.9830e−10 |
| 160 | 30 | Budget | 0.114279 mm | 8.7408e−5 rad | +2.9989e−9 |
| 200 | 0 | Missing support | Unavailable | Unavailable | 0 |
| 240 | 30 | Budget | 0.046173 mm | 3.6688e−5 rad | +1.6040e−12 |
| 279 | 14 | Search | 1.233076 mm | 1.1995e−3 rad | −1.5375e−6 |

Proposal entries are maxima over non-origin nodes. Convergence still requires
every **undamped full** proposal strictly below **0.01 mm / 1e−5 rad**.
Source 279 advances from nine to fourteen corrections and lowers its objective,
while its remaining full proposal grows. Its last search rejects all **21 GN
and eight damped candidates** for lost fixed-source sampling support. Neither
more accepted updates nor a smaller fitting cost establishes pose accuracy.

Room median original-discrete fresh RMS is **2.454814 mm**, versus control
**2.454214 mm**; desk2 stays **2.740464 mm**. All ten original fresh support,
conditioning and 0.03 m RMS guards still pass, with no fresh convergence.
Numeric reference labels are never parsed and no full trajectory is refit.
The latest optional reciprocal trajectory scores remain **88/179 room /
75/179 desk2** accurate, with complete physical protocols unmet.

## Bounded direction policy

The [design](../assets/damped-depth-refinement-v1/design.json) and **83**
source/context members are frozen before recorded fitting. Use the same ten
original local windows, initial poses, optical origin, timestamps, observations,
center normalizers and incoming nonlinear priors as the
[deeper-search control](deep-search-depth-refinement.md). This is development
on already viewed data, without a new held-out test.

First construct the **original undamped** system and require its measured
current-frame support, SPD condition limit **1e6** and full proposal bounds
**0.10 m / 0.15 rad**. Check original full-proposal convergence before any
direction alternatives. Degenerate information cannot be repaired by damping.
The original 21-multiplier GN search runs unchanged.

Only a failed search or accepted GN multiplier below 1/16 triggers the eight
fixed values **0.001, 0.01, 0.1, 1, 10, 100, 1000, 10000**. For each value solve

```text
(H + lambda * diag(diag(H))) d = -g
```

This changes the numerical direction, adding no measured information, posterior
prior or objective penalty. Require the same condition and increment bounds
on each direction, and evaluate its full left-root increment. All fixed sources
must remain sampled. There are at most **29 candidate attempts per correction**
and **30 corrections**; the grid never expands after seeing outcomes.

The existing passing GN candidate is the initial incumbent. In grid order,
replace it only when a supported alternative satisfies the original Armijo
bound and lowers both the initial and incumbent objectives by more than
`1e-12 * max(1, abs(before_cost))`. Compare the same original nonlinear prior
and fixed-source Huber objective. Each selected alternative therefore makes
a resolved local improvement, but changing the sequence of states does not
guarantee a better objective after a fixed number of corrections.

The fallback GN policy retains its original numerical slack. It admits **nine
tiny positive cost changes** in this run, all from GN choices, versus two in
the control. Each is exposed in the evidence; none counts as strict descent
or convergence. The stricter comparison for alternative directions does not
remove increases from the preserved fallback.

## Independent checks and work counts

The original refiner body changes only its search call to provide the complete
system. Its fixed objective, population, weights, undamped solve, guards,
convergence and checkpoints remain unchanged. The verifier likewise changes
only that dependency. AST proofs and dependency identities check this boundary.

The audit uses the existing independent relative-camera polynomial objective,
SVD normals, incidence/prior system, eigen-decomposition solves and Rodrigues
increments. It independently forms each damped diagonal, checks every proposed
direction and rejected candidate, and replays the selection order. Raw PNG
hashes and axial Z `/5000` conversion precede both surface constructions;
image vertices and original observations remain shared.

There are **63 triggered searches / 504 damped solve checks**, all in room.
The applied updates select **32 damped / 107 GN** directions. The audit checks
**159 new GN-system roles** (139 iteration systems and 20 final systems),
**154 control systems**, and **60 checkpoint systems**. Those **373 matrix
roles** include repeated prefixes and endpoints. The 504 damped systems modify
the same equations numerically and are not independent measured information.
There are **921 GN candidate trials** (149 invalid, 633 valid nonpassing,
139 passing), plus **504 damped trials**, including **71 invalid**. A passing
GN candidate can be replaced by an alternative; passing-candidate counts and
applied-direction counts are distinct.

Analytic controls check **21 rendered reported systems**, six checkpoint
systems and 72 damped solves along applied traces; the perturbed rendered
terminal search adds eight more solve checks. The rendered warm start converges
in one correction, while the 2 cm perturbed start stops after twelve corrections
without convergence. No alternative is selected in these rendered controls.
Missing and planar images retain their original guards before damping.

A separate authored quadratic exercises direction selection with eight more
spectral solves. Its chosen damping **100** matches a hand-derived two-variable
inverse and improves on the shortened GN step. Private cost substitutions are
isolated from measured-data modules. This is algebraic policy evidence rather
than image-derived pose accuracy. Five new receipt corruptions are rejected;
unchanged inherited controls cover prior derivatives, support and full-proposal
convergence. CI runs these checks without the recordings.

## Reproduce and inspect

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-damped-depth-refinement.py \
  --output artifacts/damped-depth-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-damped-depth-refinement.py \
  --output artifacts/damped-depth-refinement/recorded-repeat
bash scripts/check.sh
```

Use the pinned NumPy/Pillow environment, fresh output directories and previously
acquired hash-qualified room/desk2 depth PNGs. The frozen
[summary](../assets/damped-depth-refinement-v1/summary.json),
[statistics](../assets/damped-depth-refinement-v1/statistics.json) and
[validation](../assets/damped-depth-refinement-v1/validation.json) retain the
first recorded result with source archives and a file manifest. Core checks
pass **371 Rust tests and 49 scenarios with complete log replays**; all 245 core
outputs and 16 pinned native sources remain unchanged. This branch commits no
tracker, prior, Schur or default driving state. Physical localization accuracy,
vehicle integration and timing remain unverified. Subjective maturity stays
**about 20%** versus mature driving OSS; the 30% and 50% goals remain unmet.
