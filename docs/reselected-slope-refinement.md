# Bounded continuous reselection: failed comparison

Three bounded selection-and-refit rounds do **not** improve convergence on
the ten viewed windows. Same-model fresh convergence stays **0/10**;
terminal fixed-objective convergence falls from **9/10 to 8/10**. Original
discrete fresh convergence also stays **0/10**, and both recordings' median
fresh residuals worsen. The original [single fixed fit](slope-admission-refinement.md)
remains unchanged; this research variant is not adopted.

The [preceding diagnosis](depth-objective-bridge.md) showed that selecting
sources and normalizers again changes stationarity even within the same
continuous model. This experiment implements that reselection between complete
local fits, retaining its failures rather than extending the budget after
viewing results.

| Ten viewed windows | Original fixed fit | Three-round experiment |
|---|---:|---:|
| Terminal fixed-objective convergence | 9 | 8 |
| Same-model fresh convergence | 0 | 0 |
| Original discrete fresh convergence | 0 | 0 |
| Applied corrections | 124 | 373 |
| Current-support stops | 1 | 1 |
| Inner / outer budget stops | 0 / 0 | 1 / 8 |
| Within-fit invalid / nonpassing / shortened trials | 0 / 0 / 0 | 0 / 0 / 0 |
| Within-fit accepted cost increases | 0 | 0 |

All 373 applied corrections are full steps and lower their own fixed objective.
The extra **249 corrections** do not yield fresh convergence. Reselection
changes the objective: **14 of 28** equal-pose reselected cost checks increase
relative to the preceding fixed objective. Those checks include terminal
diagnostics, rather than only boundaries followed by another fit. Their cost
differences are not optimizer-step increases or accuracy comparisons.

## Frozen policy and complete outcomes

The [design](../assets/reselected-slope-refinement-v1/design.json) and **130
source/context members** are frozen before recorded fitting. Use at most
**three rounds**, each with **30 corrections**, for at most **90 applied
corrections per window**. These are bounded offline experiments, without a
timing claim. Prepared one-sided C1 fields, geometric center selection,
nine probes, minimum 200 sources per factor, Huber 0.01 m loss, original
nonlinear prior, condition and increment guards, 21 Armijo candidates and
strict full-proposal convergence remain unchanged.

Within each round, retain every selected source, original point/normal value
and metric normalizer. Only between rounds, apply the same admission policy
at the preceding terminal pose, including its ordinary recomputed bilinear
center normalizers. The next fit uses exactly that reselected population.
The same incoming nonlinear prior appears once per objective, with its
original anchors/information and consumed-pair ownership; it is not accumulated
or remarginalized between rounds.

Stop first for an inner guard or unconverged inner budget, then for a fresh
selection guard, then for convergence of both the fixed and freshly selected
continuous systems, then at round three. There is no fallback to an earlier
state chosen by residual. The original discrete fresh check remains separate
and unchanged.

| Window | Corrections in rounds 1 / 2 / 3 | Terminal stop |
|---|---|---|
| Room / 120 | 7 / 14 / 12 | Outer budget |
| Room / 160 | 16 / 16 / 15 | Outer budget |
| Room / 200 | 0 / — / — | Initial current support |
| Room / 240 | 5 / 7 / 7 | Outer budget |
| Room / 279 | 13 / 8 / 30 | Inner budget |
| Desk2 / 120 | 10 / 5 / 19 | Outer budget |
| Desk2 / 160 | 19 / 17 / 16 | Outer budget |
| Desk2 / 200 | 27 / 9 / 5 | Outer budget |
| Desk2 / 240 | 19 / 11 / 24 | Outer budget |
| Desk2 / 279 | 8 / 24 / 10 | Outer budget |

All ten complete first-round reports match the archived original reports
exactly except the cap field, including all **124 first-round corrections**.
The control already terminates by 27 or an initial guard, so changing its
unused cap from 90 to 30 does not truncate any original update.

Room source 279 loses terminal fixed convergence in its third round. Room
source 200 retains its current-support failure. The other eight terminal fixed
systems converge, but their reselected proposals remain too large. Across the
nine supported final reselected systems, maximum full translation proposals
range from **0.099 to 1.527 mm**, above the unchanged **0.01 mm** limit.
Rotation uses the unchanged **1e−5 rad** limit. These are numerical corrections,
without reference-pose error scoring.

All ten original discrete fresh systems pass their support/condition checks
and 0.03 m RMS gates, while failing convergence. Room median fresh RMS worsens
**2.463832 → 2.464826 mm**; desk2 worsens **2.711780 → 2.808556 mm**. These
fitting residuals do not measure physical trajectory error. The frozen
numerical progression rule fails.

## Observations and weights really change

The initial population has **60,288 source occurrences**. Final reselection
has **60,429**, retaining **47,917** initial identities (**79.4802%**), losing
**12,371** and adding **12,512**. Similar population sizes do not imply the same
observations. Across all 28 per-round transitions, removal/addition counts are
24,659/24,800 and 156,521 shared normalizers change. These cumulative counts
repeat sources across rounds, factors and windows.

Receipts preserve source identities, shared point/normal equality, losses,
additions and shared-normalizer ratio ranges. Normalizers remain engineering
weights, without calibrated uncertainty. Source dropout occurs only at explicit
selection boundaries; unsupported selected sources still invalidate a candidate
within a fixed fit. No claim of 100% retention applies to this variant, and no
new independent measurements are acquired.

## Unresolved identity boundary controls

Analytic development exposed an existing initial-probe support disagreement
between operational and independent implementations for exactly identical
rendered views. It occurs with unquantized depths and with 2 mm quantization:

| Identical-view fixture | Operational sources | Independent sources | Only operational / only independent |
|---|---:|---:|---:|
| Unquantized | 963 | 974 | 4 / 15 |
| Quantized 2 mm | 964 | 974 | 3 / 13 |

The disagreeing sources project exactly onto integer image-grid coordinates
in the recorded control calculation. Prepared fields are checked separately;
the initial nine-probe admission sets still differ. Both full identity answers
remain in [analytic evidence](../assets/reselected-slope-refinement-v1/analytic.json),
and the auditor rejects their metadata mismatch. These two domain cases remain
**unresolved**, without a passed identity-convergence claim. No coordinate
snapping, source-set intersection, tolerance relaxation or change to the frozen
original selector hides the discrepancy.

The moving 2 mm-quantized fixture independently verifies two actual rounds
and two corrections, with same-model fresh convergence. A separate nonzero-prior
fixture also exercises two rounds and verifies unchanged prior ownership.
Missing/plane data keep their guards. Seven scheduler cases, three input guards
and six outer-report corruptions check bounds, stop precedence, fresh systems,
transitions and round chaining. The inherited inner suite retains 12 nonzero-prior
gradient axes. Its passing controls do not resolve the two identity boundaries.
Pre-freeze fixture-development failures are retained in the evidence's
`preflight` logs; they are not frozen recorded comparisons.

## Verification and reproduction

Every recorded source population agrees with the independent selector under
the original metadata checks. Full inner traces, fixed metric weights, priors,
matrix systems, spectral proposals, Rodrigues increments, all search receipts
and post-fit fresh systems are reconstructed. Independent dictionary identity
sets and weight-ratio aggregation verify selection transitions. Original PNG
digests, timestamps and axial-Z conversion `/5000` once precede separate PCA/SVD
geometry constructions; decoded observations remain shared.

There are **457 new-report system roles**, plus **144 rechecked control roles**;
the first-round states are repeated across both sets. The new analytic report
has 23 system roles, with no independent sensor-acquisition claim. All new
recorded fitting remains offline and label-free. Neither numerical progression
nor adoption is established; the unresolved analytic domains independently
preclude adoption.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-reselected-slope-refinement.py \
  --output artifacts/reselected-slope-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-reselected-slope-refinement.py \
  --output artifacts/reselected-slope-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, existing hash-qualified PNGs and new output directories.
The [summary](../assets/reselected-slope-refinement-v1/summary.json),
[statistics](../assets/reselected-slope-refinement-v1/statistics.json) and
[validation](../assets/reselected-slope-refinement-v1/validation.json) retain the
first complete failed comparison. CI includes the analytic controls. Core checks
pass **371 Rust tests and 49 scenarios with complete sensor-log replays**;
all 245 core-output and 16 pinned native-source hashes remain unchanged.

No numeric real reference label is parsed. No new continuous trajectory, physical
accuracy score, tracker/prior/Schur commit or native/default change occurs. Vehicle
integration and timing remain unverified. Subjective maturity stays **about 20%**
versus mature driving OSS; **30% and 50% remain unmet**.

A separately frozen [boundary-support variant](boundary-slope-refinement.md)
subsequently resolves the two authored identity admission disagreements using
conservative incident-cell support. This earlier three-round implementation,
its unresolved controls and its failed recorded result remain unchanged.
