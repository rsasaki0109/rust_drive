# Fitted-center reselection: lower residuals, failed convergence

Putting the [fitted-center union](fitted-center-refinement.md) into the original
bounded reselection scheduler does **not** repair fresh convergence on the ten
viewed windows. Both same-model and original-discrete fresh convergence remain
**0/10**. Terminal fixed-objective convergence falls from **10/10 to 7/10**,
despite lower median fresh residuals in both recordings. This separately frozen
research variant fails the original progression rule and is not adopted.

Every complete first-round fit reproduces the preceding fixed-union control
exactly except its unused cap, including all **122 corrections**. Its first
fresh population, matrix and proposal also reproduce the archived terminal
fresh check exactly. The regression follows the added reselection-and-fit
rounds; the original single-fit result remains unchanged.

## Unchanged bounds and explicit selection changes

The [design](../assets/reselected-center-refinement-v1/design.json) and **143
source/context members** are frozen before recorded fitting. Private bindings
reuse the exact transition, stop and refinement function bodies of the
[earlier three-round experiment](reselected-slope-refinement.md), with the
fitted-center solver as their sole changed operational dependency.

Use at most **three rounds of 30 corrections**, for at most **90 applied
corrections per window**. The first control fits finish by 19, so the first
30-cap round retains their complete reports. Later rounds start exactly at
the preceding terminal poses, without resets or choosing an earlier result.

Each round uses the unchanged conservative boundary support, one-sided C1
field, legacy/fitted center union, raw reciprocal nearest measured-normal
checks, nine initial probes and minimum 200 sources per factor. Legacy center
qualification keeps precedence over C1-only qualification for metric ownership.
Within a fit, every selected source and metric stays fixed. Only between fits
does the same selector construct a new population and ordinary center metrics.

The same original nonlinear prior appears once per objective with unchanged
anchors, information and consumed-pair ownership. It is neither accumulated nor
remarginalized. Huber loss, condition/increment guards, strict 1e−5 m / 1e−5 rad
full-proposal convergence and the 21 Armijo candidates remain unchanged.

Stop for an inner guard or unconverged inner budget, then a fresh selection
guard, then fixed-and-fresh continuous convergence, then round-three budget.
The original-discrete fresh check remains separate. No budget or tolerance is
changed after viewing the result.

## Complete recorded result

| Ten viewed windows | Fixed union | Reselected union |
|---|---:|---:|
| Terminal fixed-objective convergence | 10/10 | 7/10 |
| Same-model fresh convergence | 0/10 | 0/10 |
| Original discrete fresh convergence | 0/10 | 0/10 |
| Applied corrections | 122 | 378 |
| Association rounds | 10 | 27 |
| Inner guard / inner budget / outer budget stops | 0 / 0 / 0 | 1 / 2 / 7 |
| Within-fit invalid / valid nonpassing trials | 0 / 0 | 138 / 0 |
| Accepted shortened steps | 0 | 11 |
| Accepted fixed-objective cost increases | 0 | 0 |

The extra **256 corrections** do not yield fresh convergence. All 378 accepted
steps lower their own fixed objective; only 11 are shorter than a full step.
Reselection changes objective composition: **16 of 27** equal-pose reselected
cost checks increase relative to the preceding fixed objective. These include
terminal diagnostics, without implying optimizer-step increases or physical
accuracy changes.

| Window | Corrections in rounds 1 / 2 / 3 | Terminal stop |
|---|---|---|
| Room / 120 | 10 / 10 / 17 | Outer budget |
| Room / 160 | 15 / 30 / — | Inner budget |
| Room / 200 | 6 / 10 / 5 | Outer budget |
| Room / 240 | 10 / 15 / 20 | Outer budget |
| Room / 279 | 8 / 30 / — | Inner budget |
| Desk2 / 120 | 10 / 16 / 9 | Outer budget |
| Desk2 / 160 | 14 / 23 / 11 | Outer budget |
| Desk2 / 200 | 18 / 22 / — | Fixed-source support / line search |
| Desk2 / 240 | 19 / 8 / 9 | Outer budget |
| Desk2 / 279 | 12 / 12 / 9 | Outer budget |

Room source 200 retains its repaired current support and completes all three
fixed fits, but still fails fresh stationarity. Room sources 160 and 279 exhaust
their second inner budgets. Desk2 source 200 stops on a second-round line search;
the other seven final fixed systems converge.

At the ten final freshly selected systems, maximum full translation proposals
range from **0.308 to 3.393 mm**, exceeding the unchanged **0.01 mm** threshold.
These are numerical optimization proposals, without a reference-pose error
interpretation. All original-discrete fresh support/condition checks and 0.03 m
RMS gates pass. Median fresh RMS improves **2.461792 → 2.434341 mm room** and
**2.826047 → 2.676903 mm desk2**. Lower residuals do not satisfy either fresh
convergence check or establish physical trajectory accuracy.

## Selected support still bounds the path

All **138 invalid candidates** occur in desk2 source 200's second round and
report `fixed source support`. Eleven accepted shortened steps progressively
reduce the multiplier, ending at **2^-20**. At the next proposal, **all 21
candidates**, from 1 through 2^-20, lose selected support and reject; no source
is silently dropped to accept that step.

The initial nine finite domain probes do not certify the whole subsequent
optimization path. Consistent incident-cell handling resolves the authored
identity disagreements, but does not extend a measured field across missing
support. These receipts identify the remaining domain failure without a
universal numerical-stability claim or a post-result relaxation of the guard.

## Source retention and metric changes

The initial population contains **80,428 source occurrences**. Final reselection
has **80,547**, retaining **63,984** initial identities (**79.5544%**), removing
**16,444** and adding **16,563**. The previous union's 100% retention relative
to the old boundary control applies to the first fixed fit; it does not extend
across these selection boundaries.

Across the 27 per-round transitions, removal/addition counts are **39,890 /
40,009**, and **164,274 shared normalizers change**. These cumulative counts
repeat observations across rounds, factors and windows. Receipts preserve shared
source-ID digests, exact shared point/normal equality, loss/addition counts and
metric-ratio ranges. Gradient-length metrics remain engineering weights without
calibrated uncertainty. No new independent measurements are acquired.

## Independent verification and reproduction

An isolated copy of the original outer verifier binds the fitted-center
independent audit. It reconstructs every inner trace, fixed metric, nonlinear
prior, candidate objective, matrix, spectral proposal and Rodrigues update.
Independent dictionary source sets verify transitions and normalizer ratios;
independent stop logic verifies bounds and round chaining. Complete selection
stages at each round's start and end agree across **336 raw-pair rows**, including
the first stages already retained in the fixed control.

There are **459 new-report system roles** and **142 rechecked control roles**.
The 122 first-round updates are repeated in both sets; stage rows also repeat
states at adjacent round boundaries. Original hash-qualified PNGs and acquisition
timestamps precede separate operational PCA and independent SVD geometry, with
axial-Z `/5000` conversion once and decoded observations shared. This remains
outcome-informed development on the same viewed indoor windows.

The analytic report independently checks **29 system roles**, seven scheduler
cases, three input guards and six outer-report corruptions. A 2 mm quantized
rendered motion fixture takes two actual rounds and two corrections to converge;
a separate nonzero-prior fixture also takes two rounds and preserves its prior.
Missing/plane data retain their guards. Unquantized and quantized identical views
each agree on **939 sources** and converge in one round without corrections.
The inherited inner controls retain 12 nonzero-prior gradient axes. Repeating
the analytic suite at the recorded-run start reproduces the standalone result
exactly. CI includes the new suite.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-reselected-center-refinement.py \
  --output artifacts/reselected-center-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-reselected-center-refinement.py \
  --output artifacts/reselected-center-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, the original hash-qualified inputs and new output
directories. The [analytic evidence](../assets/reselected-center-refinement-v1/analytic.json),
[summary](../assets/reselected-center-refinement-v1/summary.json),
[statistics](../assets/reselected-center-refinement-v1/statistics.json) and
[validation](../assets/reselected-center-refinement-v1/validation.json) retain
the complete first outcome in an 11-file evidence manifest.

Core checks pass **371 Rust tests and 49 scenarios with complete sensor-log
replays**. All 245 core-output hashes, 16 pinned native-source hashes and 631
previous evidence-manifest entries remain unchanged. No numeric real reference
label is parsed, new physical trajectory scored, tracker/prior/Schur state
committed or native/default setting changed. Vehicle integration and timing
remain unverified. Subjective maturity stays **about 20%** versus mature driving
OSS; **30% and 50% remain unmet**.
