# Initial admission using the fitted depth field

Using the actual fitted depth field for initial neighborhood admission improves
local convergence from **6/10 to 9/10** viewed windows. All five desk2 windows
now converge; room source 200 still lacks 200 center-qualified sources in any
current-frame factor. The original **32,507 source occurrences are all retained**,
with their original point/normal values and metric normalizers unchanged.

Another **27,781 source occurrences** from the same existing frames enter the
fit, for **60,288 total**. All **124 applied corrections are full steps** and
strictly lower each fixed objective. Original-discrete fresh convergence
remains **0/10**, so the frozen progression rule still fails. There is no
physical trajectory score or native/default adoption.

| Ten viewed windows | Extended-budget control | Fitted-field admission |
|---|---:|---:|
| Own continuous-objective convergence | 6 | 9 |
| Original-discrete fresh convergence | 0 | 0 |
| Initial source occurrences | 32,507 | 60,288 |
| Previous source occurrences retained / lost | — | 32,507 / 0 |
| Initial factor occurrences | 36 | 51 |
| Initial-current-support stops | 4 | 1 |
| Search / budget stops | 0 / 0 | 0 / 0 |
| Applied corrections | 106 | 124 |
| Shortened / invalid / valid nonpassing trials | 0 / 0 / 0 | 0 / 0 / 0 |
| Accepted cost increases | 0 | 0 |

Retention of the old population is **100%**. The expanded population is
**1.854616 times** the old population; that separate ratio is not a retention
percentage. Counts are source occurrences across factors/windows, with shared
frames and repeated points, rather than independent new sensor measurements.

## Why current-frame support was lost

The [control](extended-slope-refinement.md) uses one-sided depth-slope Hermite
sampling during fitting, while its initial nine neighboring probes still call
an older bilinear sampler requiring measured normals at all probe corners.
An otherwise valid depth field can lose admission because an unused neighboring
normal is absent. The new policy changes those initial probes to the actual
fitted sampler, preserving the original probe offsets and finite-gradient gate.

All earlier selection remains: raw reciprocal matching, bilinear center
geometry, center target-normal eligibility, source-normal agreement greater
than 0.8, point-distance bounds and the **minimum 200 sources per factor**. Source
coordinates, source normals and bilinear-center normalizers come from that
same original center-qualified seed. They are sliced, without recomputation.

The recorded evaluator independently reconstructs all **61 raw-pair stage
counts**, including center/probe counts below the admission threshold. These
current-factor examples show the restored support:

| Window | Target / current-source index | Geometric center | Old bilinear probes | New depth-field probes |
|---|---|---:|---:|---:|
| Room / 200 | 2 / 4 | 107 | 34 | 106 |
| Room / 200 | 3 / 4 | 155 | 36 | 151 |
| Desk2 / 120 | 1 / 4 | 289 | 190 | 288 |
| Desk2 / 120 | 2 / 4 | 391 | 182 | 389 |
| Desk2 / 120 | 3 / 4 | 411 | 198 | 407 |
| Desk2 / 240 | 1 / 4 | 178 | 41 | 177 |
| Desk2 / 240 | 2 / 4 | 233 | 55 | 232 |
| Desk2 / 240 | 3 / 4 | 319 | 104 | 317 |
| Desk2 / 279 | 1 / 4 | 212 | 70 | 208 |
| Desk2 / 279 | 2 / 4 | 384 | 132 | 383 |
| Desk2 / 279 | 3 / 4 | 398 | 130 | 396 |

Raw current-pair counts in room source 200 are 217 and 292, but original center
geometry retains only 107 and 155. Its older pair likewise has 141 center-qualified
points. None reaches 200, so the new admission still selects no factor there. The two
current factors are not pooled to bypass the per-factor requirement. Older
measurements cannot replace current-frame support. Desk2 source 240's 178-point center
factor also remains excluded; its other two current factors qualify.

## Fixed local outcomes

| Window | Control corrections / stop | New corrections / stop | New source / factor occurrences |
|---|---|---|---:|
| Room / 120 | 12 / converged | 7 / converged | 5,788 / 8 |
| Room / 160 | 10 / converged | 16 / converged | 6,067 / 6 |
| Room / 200 | 0 / initial support | 0 / initial support | 0 / 0 |
| Room / 240 | 7 / converged | 5 / converged | 8,592 / 6 |
| Room / 279 | 12 / converged | 13 / converged | 3,001 / 3 |
| Desk2 / 120 | 0 / initial support | 10 / converged | 3,116 / 7 |
| Desk2 / 160 | 39 / converged | 19 / converged | 9,667 / 6 |
| Desk2 / 200 | 26 / converged | 27 / converged | 21,153 / 6 |
| Desk2 / 240 | 0 / initial support | 19 / converged | 792 / 3 |
| Desk2 / 279 | 0 / initial support | 8 / converged | 2,112 / 6 |

All nine new converged states satisfy the unchanged strict maximum **full**
proposal norms **0.01 mm / 1e−5 rad**, across active variables. These are remaining
numerical corrections, without physical pose-error scoring. No shortened step
is used as a convergence shortcut. All newly converged windows terminate by 27
corrections; cap 30/60/90 convergence is **9/9/9**, with actual terminal states
carried forward. The source population is frozen throughout each complete
attempt, without resets or reselection at checkpoints.

All ten original-discrete fresh support/conditioning and **0.03 m RMS** gates
pass, while fresh proposal convergence fails everywhere. Room median fresh RMS
slightly worsens **2.463668→2.463832 mm**; desk2 improves
**2.740464→2.711780 mm**. These fitting residuals do not establish physical
trajectory accuracy. The original complete-data progression rule fails both
the remaining room support requirement and fresh convergence, retaining the
small room residual regression as well.

Additional residuals change objective composition and its balance with the
unchanged incoming prior. Absolute costs across these different populations
are not accuracy comparisons. Original prior ownership checks still prevent
reusing already consumed raw factors. No numeric real reference labels are
parsed, continuous trajectory refit, or tracker/prior/Schur state committed.

## Frozen policy and independent evidence

The [design](../assets/slope-admission-refinement-v1/design.json) and **123
source/context members** are archived before fitting. This is outcome-informed
development on the same viewed source indices **120/160/200/240/279** in both
recordings, not held-out validation.

Probe the projected center and eight offsets of one target pixel, holding
target optical Z fixed. Four measured interpolation corners, depth continuity,
prepared one-sided slopes, positive finite inverse depth, finite derivatives
and image/front bounds retain the fitted sampler's original guards. Missing
depth remains missing. Probes supply admission tests, without additional
residual terms or weights. Finite probes do not certify all intervening or
future poses.

Private unchanged extended refiner/objective/search/checkpoint functions bind
only the new initial population. Prepared fields, Huber 0.01 m loss, original
nonlinear prior, maximum 90 corrections, 21 Armijo candidates through `2^-20`,
numerical slack, current support, scaled SPD condition limit 1e6, full-proposal bounds
0.10 m / 0.15 rad and strict convergence remain unchanged. AST proofs retain the
same nine-probe function body with a different sampler binding, and the same
source slicing/minimum 200 rule. Original control bindings stay isolated.

The independent audit retains a separate unmodified center selector and legacy
probe oracle. It uses homogeneous target-image offsets, independently prepared
one-sided fields, polynomial interpolation, SVD source-selection normals,
prior/incidence matrices, spectral solves and Rodrigues increments. It compares
all stage counts and verifies every new report, candidate and checkpoint;
the original extended control is separately rechecked. Original PNG digests
and axial-Z `/5000` conversion precede both constructions; decoded geometry
remains shared.

An authored depth patch passes new probes despite an absent unused neighboring
normal. Missing depth still rejects, and original center-normal/source-normal
agreement gates still reject their fixtures. Inherited interpolation/eight
side-availability controls pass. Rendered fitting retains **998 sources versus
988 previously**, converging in one correction. Degenerate images retain
guards; all **12 nonzero-prior gradient axes** pass, and four report corruptions
are rejected. Analytic roles are **7 report / 6 checkpoint systems plus 1
separate prior system**.

Recorded roles are **144 new-report / 126 control / 60 checkpoint systems**.
They reuse observations and terminal states, without independent new sensor
data. All fits finish before 30, so the optional future-state checkpoint
corruption branch is not triggered in this run. See the
[summary](../assets/slope-admission-refinement-v1/summary.json),
[stage counts and statistics](../assets/slope-admission-refinement-v1/statistics.json)
and [validation](../assets/slope-admission-refinement-v1/validation.json).

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-slope-admission-refinement.py \
  --output artifacts/slope-admission-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-slope-admission-refinement.py \
  --output artifacts/slope-admission-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, new output directories and existing hash-qualified raw
PNGs. CI includes the new analytic suite. Core checks pass **371 Rust tests
and 49 scenarios with complete sensor-log replays**, retaining 245 core-output
and 16 native-source digests. Vehicle integration, sustained physical trajectory
accuracy and timing remain unverified. Subjective maturity stays **about 20%**
versus mature driving OSS; **30% and 50% remain unmet**.
