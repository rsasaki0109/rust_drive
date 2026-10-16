# Extended fixed-source depth-slope correction budget

The extended local evaluation converges on **6/10** viewed windows, improving
from the [one-sided depth-slope control's 5/10](depth-slope-refinement.md).
Desk2 source 160 needs **39 corrections**, nine beyond the old cap. The six
initially supported windows now converge; four initial selected-support guards
remain. Original-discrete fresh convergence is still **0/10**, and the changed
window's fresh RMS worsens. The frozen progression rule therefore fails.

The entire original **97-correction trace prefix** and all ten complete
cap 30 reports are reproduced exactly. Sources, original metric normalizers,
prepared depth-slope fields, incoming prior, objective, candidate policy and
strict convergence thresholds stay fixed. The only algorithm changes are the
admitted correction caps and the new module's default cap.

| Ten viewed windows | Original cap 30 | Extended cap 90 |
|---|---:|---:|
| Own continuous-objective convergence | 5 | 6 |
| Original-discrete fresh convergence | 0 | 0 |
| Initial source / factor occurrences | 32,507 / 36 | 32,507 / 36 |
| Initial-current-support stops | 4 | 4 |
| Search / budget stops | 0 / 1 | 0 / 0 |
| Total applied corrections | 97 | 106 |
| Shortened / invalid / valid nonpassing trials | 0 / 0 /0 | 0 / 0 /0 |
| Accepted cost increases | 0 | 0 |

All **106 accepted corrections are full steps** and strictly lower the fixed
objective. The original numerical slack remains in the search; no observed
accepted step increases cost. Nine other terminal reports equal the control
after normalizing only their cap metadata. Existing early converged and
guarded states retain their actual applied counts.

## Additional computation and its limit

| Desk2 source 160 | Original cap 30 | Extended result |
|---|---:|---:|
| Applied corrections | 30 | 39 |
| Remaining maximum full translation norm (mm) | 0.054760 | 0.00743610 |
| Remaining maximum full rotation norm (rad) | 4.37849e−5 | 8.30489e−6 |
| Fixed-objective cost | 0.00614729221536 | 0.00614651435366 |
| Original-discrete fresh RMS (mm) | 2.036790 | 2.042797 |
| Termination | Budget | Converged |

The remaining full proposal across active variables meets the unchanged strict
**0.01 mm / 1e−5 rad** thresholds. These are correction norms, without physical
pose-error scoring. Smaller applied steps are not a convergence shortcut.
The changed window's fresh residual regression remains visible despite its
lower continuous-objective cost and newly converged full proposal.

| Requested checkpoint cap | Own convergence | Initial selected-support stops | Budget stops |
|---|---:|---:|---:|
| 30 | 5/10 | 4 | 1 |
| 60 | 6/10 | 4 | 0 |
| 90 | 6/10 | 4 | 0 |

Caps 60/90 carry the actual 39-correction terminal state forward for desk2 / 160;
no extra steps are applied after convergence. Its cap 30 report remains a
budget stop at the original pose. A deliberate substitution of the future
final pose into the cap 30 checkpoint is rejected independently. Proposal
guards and strict convergence precede the cap; a next candidate search follows
the cap, preserving the original checkpoint event ordering.

The four selected-support stops still lack sufficient current-frame
contributions in the fixed initial population. Original-discrete fresh
matching passes its support, conditioning and **0.03 m RMS** gates on all ten
states, while fresh full-proposal convergence fails everywhere. Room/desk2
median fresh RMS stays **2.463668 / 2.740464 mm** because the other nine windows
are unchanged. Extra computation supplies no new measurements or physical
accuracy evidence. No continuous real trajectory is refit or tracker state
committed, and the native default remains unchanged.

## Frozen continuation protocol

The [design](../assets/extended-slope-refinement-v1/design.json) and **119
source/context members** are archived before fitting. This is outcome-informed
development on the same viewed room/desk2 windows at source indices
**120/160/200/240/279**, not held-out validation.

The new module accepts fixed caps **10/20/30/60/90** and defaults to 90. Original
source selection, measured-depth fields, residual and nonlinear-prior
assembly, Huber 0.01 m loss, original GN solve, **21** Armijo candidates through
`2^-20`, numerical slack, current support, scaled SPD condition **1e6**, full
proposal bounds **0.10 m / 0.15 rad** and strict convergence functions are the
same original objects. Refiner and snapshot bodies differ only in admitted
caps/default; the checkpoint body is identical. There is no damping or new
regularization.

Select the source population once at the original initial poses and keep it
through the complete attempt. There is no restart, source reselection, metric
weight renewal or prior reset at 30 or 60. All **32,507 source occurrences / 36
factor occurrences**, exact original metadata and initial costs remain equal
to the control. Normalizers retain their existing engineering interpretation,
without calibrated uncertainty claims.

Exact old-prefix and complete cap 30 reconstruction assertions accompany every
recorded run. The independent oracle retains the same side eligibility,
reciprocal-mean slopes, polynomial interpolation, original matching normals,
prior/incidence matrices, spectral solve and Rodrigues search. Only the
admitted cap set changes in its report/checkpoint verifier bodies. Original
cap 30 controls are separately rechecked with their unmodified verifier.

## Independent evidence and reproduction

Inherited interpolation and eight side-availability fixtures pass. The actual
extended-cap rendered warm fit retains **988/988 sources** and converges in one
correction. Degenerate images retain guards; all **12 nonzero-prior
objective-gradient axes** pass, and four report corruptions are rejected.
Analytic verification covers **7 report / 6 checkpoint systems plus 1 separate
prior system**, with cap 30/60/90 checkpoint auditing.

Recorded verification covers **126 new-report / 117 control / 60 checkpoint
systems**, plus the future-state corruption rejection. These matrix roles
reuse measurements and terminal states; they are not independent new data.
See the [summary](../assets/extended-slope-refinement-v1/summary.json),
[statistics](../assets/extended-slope-refinement-v1/statistics.json) and
[validation](../assets/extended-slope-refinement-v1/validation.json).

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-extended-slope-refinement.py \
  --output artifacts/extended-slope-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-extended-slope-refinement.py \
  --output artifacts/extended-slope-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, new output directories and existing hash-qualified raw
PNGs. CI includes the extended analytic audit. Core regression passes **371
Rust tests and 49 scenarios with complete sensor-log replays**, retaining 245
core-output and 16 native-source digests. Timing and vehicle integration remain
unverified. Subjective maturity remains **about 20%** versus mature driving OSS;
**30% and 50% remain unmet**.
