# Thirty-correction local convergence diagnostic

This optional NumPy diagnostic asks whether the preceding
[alternating local refiner](alternating-depth-refinement.md) stopped too early.
It retains the original measured residual, reciprocal association, prior,
guards and strict convergence thresholds, and permits **three consecutive
ten-correction segments**. Its maximum is **thirty applied corrections**.
This is a local budget experiment on already viewed data, with no new tracker
integration or continuous trajectory scoring.

## Recorded outcome

| Five windows per recording | Room | Desk2 |
|---|---:|---:|
| Fresh convergence at 10 / 20 / 30 corrections | 0 / 0 / 0 | 0 / 0 / 0 |
| Thirty-correction budget stops | 5/5 | 5/5 |
| Guards across all checkpoints | 0 | 0 |
| Applied corrections | 150 | 150 |
| Fixed-population batches | 90 | 95 |
| Batches reaching fixed convergence | 81 | 73 |
| Median fresh RMS at 10 corrections | 2.4649 mm | 2.7658 mm |
| Median fresh RMS at 20 corrections | 2.4599 mm | 2.7848 mm |
| Median fresh RMS at 30 corrections | 2.4809 mm | 2.7687 mm |
| Remaining maximum-node translation proposal at 30 | 0.095–0.505 mm | 0.161–1.103 mm |

All ten windows exhaust thirty corrections without fresh convergence. The
first ten corrections completely reproduce every preceding control report.
The unchanged stopping threshold remains unmet even though every checkpoint's
fresh RMS is below 0.03 m. Six windows have a smaller remaining translation
proposal at thirty than at ten corrections; four have a larger proposal.
For example, room source 120 changes **0.445 → 0.092 → 0.168 mm** and desk2
source 200 changes **0.089 → 0.097 → 0.161 mm**. More corrections do not give
uniformly shrinking remaining proposals in this trial.

The last batches at the thirty checkpoints contain **288** raw-pair transitions.
Across those transitions, exact pair retention is **94.61% room / 94.25% desk2**,
with **2,774 / 4,120** retargeted source occurrences. All thirty checkpoint-last
batches lower their own frozen objective, while **15/30** have a higher total
objective after fresh association than before that batch. No whole-factor
admission gate changes. These aggregates reuse measurements across factors
and checkpoints; they are not independent observations or physical outliers.

The follow-up eligibility rule fails. Increasing the allowance from ten to
thirty corrections alone does not repair fresh convergence on these windows.
This does not prove a quantization floor, a limit cycle, physical drift causation,
or failure for every larger budget. No trajectory is refitted or newly scored.
The latest full optional reciprocal results stay **88/179 room / 75/179 desk2**
accurate, with failed complete physical protocols.

## Fixed protocol

The [design](../assets/budget-depth-refinement-v1/design.json) and source
archive are frozen before recorded fitting. Use the same saved source indices
**120, 160, 200, 240, 279** in each room and desk2 recording. The initial poses,
node timestamps, decoded axial depth and original incoming marginal prior
match the preceding experiment. Numeric real reference labels are not parsed.

Each segment calls the preceding unchanged solver. It fixes a reciprocal
population for at most three nonlinear corrections, then reselects pairs.
The first segment's complete report must **exactly equal** the archived
ten-correction control, including all poses, matrices, pixel hashes, search
receipts and stop decisions. Later segments start at the preceding returned
poses and use the same original incoming prior. Repetition never accumulates
another posterior prior or duplicates measurements inside an equation.

Only budget exhaustion permits another segment. Fresh convergence or any
guard ends the diagnostic immediately. A terminal endpoint is explicitly
carried forward for later requested checkpoints; it does not manufacture
updates. Checkpoints at **10, 20, 30** retain actual applied counts.

Fresh convergence still requires every non-origin node's full proposal to
fall strictly below **10 micrometres / 10 microradians**. The condition limit
**1e6**, per-node **0.10 m / 0.15 rad** bounds, 200-sample factor admission,
Huber weighting and bounded Armijo search are unchanged. A smaller line-search
multiplier cannot manufacture convergence. These thresholds are engineering
rules, not calibrated uncertainty or physical accuracy claims.

At each checkpoint, inspect the **last fixed batch** rather than all previous
batches. Record its initial/final pixel IDs through hashes and exact retention,
retargeting, source entry/exit and whole-factor admission counts. Evaluate its
old population at the initial and final poses, then its fresh population at
the same final poses. Reassociation cost must equal added-pair loss minus
removed-pair loss. The transported incoming prior cancels at equal final poses.
These overlapping occurrences do not label physically correct matches.

The predeclared follow-up eligibility rule requires **both recordings** to
converge every window, have no guards, pass every 0.03 m fresh RMS gate and
have final median fresh RMS no worse than their ten-correction control.
This diagnostic itself performs no full-trajectory evaluation. RMS under
changing populations does not establish better physical poses. Comparing
different correction allowances is not a timing or efficiency benchmark.

## Independent verification

The existing private verifier reconstructs SVD normals, reciprocal selection,
symmetric-normal matrix derivatives, prior coordinates, complete systems,
spectral solves, Rodrigues updates and Armijo costs for every segment.
A new outer verifier checks exact segment chaining, unchanged budgets,
honest counts and termination, and rejects unexplained early stops or work
after convergence/failure.

Endpoint proposals are independently solved again from raw depth. A separate
relative-camera selector and structured-array set oracle reconstruct all
checkpoint turnover counts and the cost partition. Extra checks of an endpoint
matrix repeat a reported system and are recorded separately; they are not
additional independent observations.

Rendered identity stops without applying a correction. The existing measured
warm start plus declared perturbation converges in **three corrections**,
without extending beyond the first segment. Missing measurements and a plane
stop through the original support/observability guards. **Thirteen** reported
rendered systems, **four** repeated endpoint checks and **four** pair transitions
are independently checked. Eleven report corruptions are rejected, including
stale later anchors, a coherent shortened budget report, false convergence,
wrong remaining proposals and incorrect turnover/cost accounting.

Separate outer-scheduler fixtures exercise thirty updates and second-segment
convergence/failure. They are bookkeeping controls, not image-derived matrix
evidence. Real multi-segment matrices are verified in the recorded trial.

The recorded verifier checks **670 reported systems**: 300 iteration systems,
185 fixed finals and 185 fresh finals. That includes **220** exactly reproduced
control systems and **450** systems from the additional segments. Thirty
endpoint proposal reconstructions repeat reported fresh systems and are
counted separately. All 288 checkpoint pair transitions are independently
checked. These counts describe validation work on reused measurements.

The mandatory core check passes **371 Rust tests and 49 scenarios with full
sensor-log replays**. All 245 deterministic core outputs, 390 preceding
evidence entries and 16 pinned native sources retain their hashes. The packet
contains 53 frozen source/context members; CI includes analytic and syntax checks.

## Reproduction and scope

Use the pinned optional NumPy/Pillow environment and original hash-pinned TUM
inputs. Output directories must be fresh:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-budget-depth-refinement.py \
  --output artifacts/budget-refinement-controls-replay
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-budget-depth-refinement.py \
  --output artifacts/budget-refinement-recorded-replay
```

The [report](../assets/budget-depth-refinement-v1/summary.json),
[validation](../assets/budget-depth-refinement-v1/validation.json) and
[manifest](../assets/budget-depth-refinement-v1/files.json) retain the local
pose/system certificates, input/source hashes and all checkpoint results.
Original images and physical reference files are not redistributed;
[TUM attribution and terms](tum-source-provenance.md).

This tool makes no tracker cache, clock, key, prior or Schur commit. It supplies
no driving controls, new physical environment, calibrated uncertainty or
real-time evidence. Local convergence alone cannot admit a vehicle pose.
The Rust default is unchanged. The subjective maturity assessment stays
**about 20%** compared with mature driving OSS; 30% and 50% remain unmet.
