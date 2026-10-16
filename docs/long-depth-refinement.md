# Thirty corrections with the same fixed observations

This optional NumPy diagnostic extends the
[sampling-domain local refiner](sampling-depth-refinement.md) from ten to
**thirty applied corrections**, holding one initial observation population,
its center normalizers and the original incoming prior throughout the solve.
It records exact **10, 20, 30** correction checkpoints. These are viewed local
optimization cases, without new continuous trajectory or physical scoring.

## Recorded outcome

| Five windows per recording | Room | Desk2 |
|---|---:|---:|
| Continuous convergence at 10 / 20 / 30 checkpoints | 0 / 0 / 0 | 0 / 2 / 2 |
| Complete cap-10 report equality | 5/5 | 5/5 |
| Final missing-current-support guards | 1 | 3 |
| Final line-search guards | 4 | 0 |
| Final budget exhaustion | 0 | 0 |
| Actual total applied corrections | 51 | 35 |
| Cap-10 control applied corrections | 34 | 20 |
| Original-discrete fresh convergence | 0/5 | 0/5 |

Desk2 source **160 converges at correction 16**, and source **200 at
correction 19**, under the unchanged full-proposal thresholds. These repair
two of the preceding five ten-update budget stops on their fixed continuous
objectives. The three room budget stops instead become line-search failures:
source 120 after **12**, source 160 after **18** and source 240 after **17**
corrections. Each of these three exhausted searches has eight validly sampled
candidates that fail the Armijo inequality. That observation does not prove
a pixel-knot or physical drift cause.

The preceding room source 279 still fails its search after four corrections.
Room source 200 and desk2 sources 120/240/279 retain their original missing
current-support stops. Thus **2/10** windows converge and **8/10** stop through
guards. No window requires correction 20: all 20/30 checkpoint outcomes and
actual counts agree, and no work continues after a terminal result. Thirty
is the permitted cap, not the number of applied updates in every window.

All ten complete cap-10 reconstructions exactly match the preceding control,
including every source weight, matrix, pose and search receipt. The new
attempts apply **86** total corrections versus **54** in the control, with
**49** shortened steps. The verifier reproduces **22** invalid candidates,
**157** valid nonpassing candidates and **86** accepted candidate steps.
These are local optimization work counts, not accepted tracker updates or
runtime comparisons.

The original-discrete fresh system remains unconverged on every returned
window, including the two continuous-objective successes. All original fresh
guards/RMS gates pass. Final original-discrete RMS medians are **2.4544 mm
room / 2.7405 mm desk2**, versus cap-10 **2.4539 / 2.7405 mm**. The progression
rule fails. No full trajectory is refitted or physically scored, and latest
optional reciprocal physical scores remain **88/179 room / 75/179 desk2**
accurate, with complete physical protocols unmet.

The independent audit checks **106 long-report systems**: 86 iteration
systems, ten continuous finals and ten original-discrete finals. Those include
replayed control prefixes. It separately rechecks **74 control systems** and
**60 checkpoint systems**. The checkpoint role contains repeated trace/final
states and carried-forward endpoints. These 240 matrix checks do not represent
240 independent measurements or entirely new equations.

## Frozen objective and checkpoint contract

The [design](../assets/long-depth-refinement-v1/design.json) and **72**
source/context members are frozen before recorded fitting. Use identical
initial poses, timestamps, original incoming priors and source indices
**120, 160, 200, 240, 279** per room and desk2 recording as the preceding
sampling-domain comparison. Numeric real reference labels are never parsed.

Select initial source IDs and center normalizers once. The nonlinear
inverse-depth residual, exact derivative, Huber weighting, prior transport,
factor ownership, support minimum **200**, scaled SPD condition limit **1e6**,
full proposal bounds **0.10 m / 0.15 rad** and eight Armijo multipliers
**1 through 1/128** are unchanged. The first optical origin remains fixed.
There is no damping, rank fill, new posterior prior or Schur elimination.

The original observations and weights are held even across the ten/twenty
checkpoints. Snapshot reconstruction always rebuilds the population from the
**original starting poses**, never from an intermediate pose. It requires
exact metadata equality before evaluating the returned fixed objective.
No source is dropped to reduce a candidate cost. Invalid sampling rejects
the candidate; exhausted search terminates the attempt.

Stop immediately on a proposal guard, continuous convergence or search failure.
Otherwise use the full thirty-update allowance. Continuous convergence still
requires each non-origin node's **full unscaled** proposal strictly below
**10 micrometres / 10 microradians**. Shortening a step cannot manufacture
convergence. The original-discrete freshly associated system and correction
remain separate outputs.

Checkpoint event ordering matters. At a requested cap, compute the full
proposal and check its guard/convergence before declaring budget exhaustion.
An attempted next search beyond that cap cannot retroactively change the
shorter attempt. For example, an exhausted search for update 11 means
**budget** for cap 10 and **guard** for cap 20. Earlier terminal outcomes are
explicitly carried forward with their actual correction count; they never
create fictitious updates.

The entire reconstructed cap-10 report must equal its archived control:
initial sources/normalizers/cost, all trace poses, systems, full proposals,
search receipts, final costs/systems/corrections and stop cause. This is
stronger than a score or final-pose comparison. Checkpoints retain enough
state to reconstruct that shorter report from the long trace.

The predeclared follow-up rule requires both cases to converge every continuous
window, improve original-discrete fresh-converged counts, have no local/fresh
guards, pass every original 0.03 m RMS gate and have original-discrete median
RMS no worse than cap 10. This local diagnostic performs no full trajectory
evaluation or default tracker integration. Different correction allowances
and verification work do not establish a timing improvement.

## Independent verification

Source proofs show that the solver differs from the frozen sampling-domain
implementation only in its bounded cap. Its independent fixed-objective
verifier likewise changes only cap checks; all selection, relative-camera
polynomial interpolation, SVD normals, prior coordinates, matrix construction,
spectral solves, Rodrigues increments and search verification remain intact.

Every long trace/final system is checked. Each checkpoint independently
reconstructs the original-population continuous system, its remaining proposal
and cost, and an original-discrete fresh system/proposal at the saved poses.
Control, long-report and checkpoint matrix roles are recorded separately.
Checkpoints can repeat systems from the trace or an earlier terminal endpoint;
they are not extra independent measurements.

Rendered measured-data warm initialization converges in one correction, and
its complete cap-10 snapshot equals the frozen original. Three additional
reported systems and six checkpoint systems are independently checked. Seven
new corruptions are rejected, including changed weights/caps, false convergence,
wrong endpoint counts/carry-forward, false budget termination and a wrong
checkpoint matrix.

The inherited sampling controls additionally check 15 rendered systems, one
nonzero-prior system, all twelve objective-gradient axes, twelve report
corruptions and three repeated rendered checks for source selection. They
retain identity/missing/planar guards, qualified interpolation controls and
the failed perturbed warm-start case. An unchanged center-only population
substitution is rejected. Isolated event-order fixtures check cap versus next
search failure, proposal guard and convergence; those fixtures are not
image-derived matrix evidence.

The mandatory core check passes **371 Rust tests and 49 scenarios with complete
sensor-log replays**. All 245 deterministic core outputs, 437 preceding
evidence entries and 16 pinned native sources retain their hashes. CI includes
the new analytic and syntax checks.

## Reproduction and evidence

Use the pinned optional NumPy/Pillow environment and original hash-pinned TUM
inputs. Output directories must be fresh:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-long-depth-refinement.py \
  --output artifacts/long-refinement-controls-replay
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-long-depth-refinement.py \
  --output artifacts/long-refinement-recorded-replay
```

The [report](../assets/long-depth-refinement-v1/summary.json),
[validation](../assets/long-depth-refinement-v1/validation.json) and
[manifest](../assets/long-depth-refinement-v1/files.json) retain complete long
and control traces, frozen weights, checkpoints and input/source hashes.
Commands return zero for verified implementation integrity even when the
quality progression condition fails. Original images and physical reference
files are not redistributed; [TUM attribution and terms](tum-source-provenance.md).

This tool commits no tracker clock, cache, key, prior or Schur state and supplies
no vehicle controls. Local convergence alone does not establish absolute pose
accuracy, visibility, physical outliers, uncertainty calibration, a new physical
environment or real-time behavior. The Rust default and subjective
**about-20%** maturity estimate remain unchanged; 30% and 50% remain unmet.
