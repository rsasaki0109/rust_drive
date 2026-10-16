# Two-pixel sampling admission

Wider initial qualification removes all **138 unsupported candidate trials**
seen in the preceding comparison and improves terminal fixed convergence from
**7/10 to 8/10** on the ten viewed windows. It retains **99.0774%** of initial
source occurrences, all 57 initial factors and exact retained metric values.

New two-pixel, original one-pixel and original-discrete fresh convergence all
remain **0/10**, while both recordings' median fresh residuals worsen relative
to the preceding three-round control. The original progression rule fails;
this variant is not adopted and supplies no physical pose-accuracy score.

This separately frozen research variant tests wider initial support qualification
in the [bounded fitted-center reselection loop](reselected-center-refinement.md).
It changes admission only; candidate support, objective, prior, solver, strict
convergence and the three-round work bounds retain their original policies.

## The archived support crossing

A read-only reconstruction independently checks 24 systems in desk2 source 200's
second fixed fit, then reconstructs all 21 failed candidate poses without
applying a new update. Each candidate loses the same source: **pixel 279616**
in factor **target 3 / source 4**. Its projection in the target optical image is:

| Saved state | Target u, pixels | Target v, pixels |
|---|---:|---:|
| Round-start qualification | 579.9282705720 | 424.8199610797 |
| Last accepted pose | 580.9999999745 | 424.6520091622 |
| Failed candidate at 2^-20 | 581.0000000001 | 424.6520091601 |

Cell **(581, 424)** lacks measured support. The source has moved **1.0717294
pixels horizontally** from the round start, beyond the original ±1-pixel probes.
At that round-start pose, the original probes accept it and the proposed
±2-pixel probes reject it. The original guard correctly rejects the later
candidates; no source is removed during that fixed fit to force acceptance.

The [reader](../assets/interior-center-refinement-v1/read-only-preflight.py),
[complete queries](../assets/interior-center-refinement-v1/read-only-preflight.json)
and [preflight output](../assets/interior-center-refinement-v1/read-only-preflight.log)
preserve the operational and independent sampling agreement. This is a diagnosis
of a saved viewed failure, without a physical pose-error score or an extra fit.

## Admission and retained metric ownership

Start from the unchanged legacy/fitted center union after its original nine
probes. At the same initial pose, sample **all 25 integer image offsets in
[-2, 2] × [-2, 2]**. Retain only sources whose samples and finite nonzero
gradients pass every probe. Keep the original minimum **200 sources per factor**;
whole subthreshold factors are excluded and their losses remain explicit.

This is a strict subset of the original one-pixel population at the same pose.
Every retained source ID, measured point, normal and initial gradient-length
metric remains exact. Legacy qualification still owns its metric in the
legacy/C1 intersection. Additional probes supply no extra measured factors.

The field remains the original one-sided C1 inverse-depth interpolation with
the conservative ±1e−9-pixel incident-cell support guard. No depth, slope or
normal is filled, and no unsupported region is extrapolated. During fitting,
the original candidate sampler evaluates the original raw projected coordinates.
Any selected source that loses support invalidates the complete candidate.

The two-pixel grid is an engineering admission choice informed by the saved
1.07-pixel crossing. Finite probes do not certify a later continuous pose path,
interpolation positivity between probes or universal floating-point stability.
Removing sources changes objective composition and its balance with the same
incoming nonlinear prior; residuals and costs across populations are not
physical accuracy comparisons.

## Bounded comparison and original-domain checks

The [design](../assets/interior-center-refinement-v1/design.json) and **153
source/context members**, including the read-only diagnosis and fixture-development
logs, are frozen before recorded fitting. This is outcome-informed development
on the same ten viewed room/desk2 windows at indices 120/160/200/240/279,
without held-out data.

Use the same original poses, timestamps, PNGs and prior as the archived
three-round one-pixel control. Recheck every original control report independently.
New first populations have explicit subset/metric receipts; their fit reports
need not equal the original reports. Complete original center-stage counts
remain unchanged before the added interior filter.

Private bindings reuse the original fixed objective, search and inner refinement
bodies, plus the original transition, stop and outer refinement bodies. Sources
and metrics stay fixed within each attempt and are reselected only between
fits. The same original nonlinear prior appears once per objective without
accumulation, remarginalization or tracker/prior/Schur commits.

At most **three rounds of 30 corrections** permit **90 applied corrections per
window**. Preserve support, condition and full-proposal guards, strict
**1e−5 m / 1e−5 rad** convergence, Huber loss and 21 Armijo candidates. Stop
precedence and final-state choice remain unchanged; tiny accepted steps cannot
substitute for a small full proposal.

At every final pose, separately reconstruct both the **original one-pixel C1
fresh system** and the original rounded-discrete fresh system. Convergence of
a smaller two-pixel selection cannot replace either original check. Progression
requires all five fixed-and-fresh two-pixel windows and all five original
one-pixel checks in each recording, improved original-discrete fresh convergence,
no local/discrete guard, passing 0.03 m RMS gates and no worse median discrete
RMS than the archived outer control.

## Recorded result and observation losses

| Ten viewed windows | One-pixel outer control | Two-pixel admission |
|---|---:|---:|
| Terminal fixed-objective convergence | 7/10 | 8/10 |
| Same-policy continuous fresh convergence | 0/10 | 0/10 |
| Original one-pixel continuous fresh convergence | 0/10 | 0/10 |
| Original discrete fresh convergence | 0/10 | 0/10 |
| Initial source / factor occurrences | 80,428 / 57 | 79,686 / 57 |
| Applied corrections / association rounds | 378 / 27 | 446 / 28 |
| Inner guard / inner budget / outer budget stops | 1 / 2 / 7 | 0 / 2 / 8 |
| Invalid / valid nonpassing candidate trials | 138 / 0 | 0 / 0 |
| Accepted shortened steps | 11 | 0 |
| Accepted fixed-objective cost increases | 0 | 0 |

The extra **68 corrections** do not establish a runtime improvement or new
information. All 446 accepted corrections use a full step and lower their own
fixed objective. Across reselection boundaries, **17 of 28** equal-pose costs
increase because the populations and metrics change; these include terminal
diagnostics, without a monotonic-optimization interpretation.

| Window | Corrections in rounds 1 / 2 / 3 | Terminal stop |
|---|---|---|
| Room / 120 | 14 / 12 / 19 | Outer budget |
| Room / 160 | 16 / 9 / 24 | Outer budget |
| Room / 200 | 22 / 13 / 4 | Outer budget |
| Room / 240 | 25 / 17 / 12 | Outer budget |
| Room / 279 | 8 / 30 / — | Inner budget |
| Desk2 / 120 | 11 / 12 / 26 | Outer budget |
| Desk2 / 160 | 17 / 16 / 13 | Outer budget |
| Desk2 / 200 | 20 / 30 / — | Inner budget |
| Desk2 / 240 | 23 / 12 / 9 | Outer budget |
| Desk2 / 279 | 14 / 11 / 7 | Outer budget |

Room source 160 now completes three converged fixed fits instead of stopping
at its second inner budget. Desk2 source 200 no longer hits the recorded support
barrier, but exhausts its second inner budget; room source 279 also remains
unconverged at that budget. The other eight terminal fixed systems converge.
All ten first fixed fits converge, with **170 first-round corrections** versus
122 in the control. Their complete reports differ because their source sets
change; none is claimed as an exact control-report reproduction.

Fresh maximum full translation proposals range **0.157–4.489 mm** with the new
two-pixel selector and **0.155–4.598 mm** with the original one-pixel selector,
above the unchanged **0.01 mm** threshold. Both sets have no proposal guard
failure. All original-discrete support/condition and 0.03 m RMS gates also pass,
but fresh convergence remains zero under every check.

Median original-discrete fresh RMS worsens **2.434341 → 2.457182 mm room** and
**2.676903 → 2.709491 mm desk2**. These compare the two bounded outer experiments,
not the earlier single-fit baseline. The numerical correction and residual
values do not measure physical pose error. Neither numerical progression nor
full-trajectory follow-up is established.

The initial filter removes **742 occurrences**: 310 room and 432 desk2. It adds
none and changes no retained point, normal or metric; all 57 original initial
factors still meet 200 sources. New initial retention is **79,686 / 80,428 =
99.0774%**. The authored identity fixtures lose a larger fraction, as recorded
below; the retention claim applies only to these ten viewed windows.

Relative to the new initial population, final reselection has **79,700**
occurrences, retains **63,067 (79.1444%)**, removes **16,619** and adds **16,633**.
Across all 28 transitions, cumulative removal/addition counts are **41,382 /
41,396**, and **169,768 shared normalizers change**. Those counts repeat sources
across factors, rounds and windows; they are not independent acquisitions.

## Independent checks and reproduction

The independent selector uses homogeneous relative transforms, calibrated image
shifts and unprojection, separate polynomial interpolation and SVD normals to
reconstruct the 25 probes. Full stage counts and selected metadata agree at
each round's start and end. Unchanged independent incidence/prior matrices,
spectral solves, Rodrigues updates, objectives, search receipts, dictionary
source transitions and outer-stop logic verify the reports. Original solver
and audit bindings remain isolated.

Recorded checks cover **530 new-report / 459 rechecked control / 10 original
one-pixel terminal system roles** and **348 raw-pair stage rows**. The 24
read-only preflight systems also occur in the archived control. Adjacent round
boundaries repeat states, and all roles share the original measured frames.

Authored controls include a missing outer measured corner that passes the old
one-pixel probes and rejects under the two-pixel policy. The first unperturbed
rendered warm case converges in one round and fails the fixture's requirement
to exercise multiple rounds; that
[development failure](../assets/interior-center-refinement-v1/analytic-preflight-failure.log)
is retained. An explicit left perturbation of the warm initial pose, recorded
as **(6, −2, 1) mm / (0.0008, −0.0006, 0.0004) rad**, exercises two actual
rounds and three corrections. Its
[preflight reader](../assets/interior-center-refinement-v1/analytic-fixture-preflight.py)
and [output](../assets/interior-center-refinement-v1/analytic-fixture-preflight.log)
are retained. No recorded-fit policy is tuned through these authored fixtures.

A separate nonzero-prior fixture runs three rounds and preserves its incoming
prior. Missing/plane data keep their guards. Unquantized and 2 mm quantized
identities each agree on **820 sources**, versus the original **939**, and
converge in one round without corrections. These explicit source losses do not
affect the historical controls. The analytic report checks **35 system roles**,
seven scheduler cases, three input guards and six outer corruptions; inherited
inner controls retain 12 nonzero-prior gradient axes. Repeating the analytic
suite at recorded-run start matches the final standalone result exactly.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-interior-center-refinement.py \
  --output artifacts/interior-center-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-interior-center-refinement.py \
  --output artifacts/interior-center-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, existing hash-qualified PNGs and new output directories.
Decode original axial Z with `/5000` once, then construct operational and
independent geometry from the shared observations. Acquisition timestamps stay
unchanged. CI includes the new analytic suite.

The [analytic evidence](../assets/interior-center-refinement-v1/analytic.json),
[summary](../assets/interior-center-refinement-v1/summary.json),
[statistics](../assets/interior-center-refinement-v1/statistics.json) and
[validation](../assets/interior-center-refinement-v1/validation.json) preserve
the first full result in a **17-file evidence manifest**.

Core checks pass **371 Rust tests and 49 scenarios with complete sensor-log
replays**. All 245 core-output hashes, 16 pinned native-source hashes and 642
previous evidence-manifest entries remain unchanged. No numeric real reference
label is parsed, physical trajectory scored or native/default setting changed.
Vehicle integration and timing remain unverified. Subjective maturity stays
**about 20%** versus mature driving OSS; **30% and 50% remain unmet**.
