# Directional probes at bounded-search failures

The read-only diagnostic finds a decreasing, validly sampled step beyond the
previous search bound at **all four** archived room failures. The three searches
whose original candidates were valid pass at **1/256** of their full proposal.
The sampling-domain control first becomes valid and passes at **1/1024**.
These results support testing a deeper bounded search. The optimizer has not
been continued, so the preceding **2/10** continuous-objective convergence and
**0/10** original-discrete fresh convergence remain the latest fit results.

## Recorded result

| Room source index | Prior applied corrections | Original 1/128 candidate | First positive Armijo pass | Paired objective change at that pass |
|---|---:|---|---:|---:|
| 120 | 12 | Valid, fails | 1/256 | −1.2534e−11 |
| 160 | 18 | Valid, fails | 1/256 | −5.6828e−10 |
| 240 | 17 | Valid, fails | 1/256 | −7.5676e−10 |
| 279, domain control | 4 | Invalid sampling | 1/1024 | −4.6814e−8 |

Each first pass also satisfies Armijo **without** the existing numerical slack.
Thus these particular passing decisions do not depend on accepting a cost
increase within that slack. Values are weighted objective changes, without
a calibrated physical error or covariance interpretation.

At the common diagnostic multiplier **1/4096**, neither signed endpoint
changes an interpolation cell in any of the four windows. Central secants
agree with the matrix's `g·d` to a maximum relative difference **3.71e−7**.
The baseline projections have no coordinates snapped to an integer pixel;
the smallest raw distance to the grid is **8.31e−6 pixels**. The data therefore
support a locally correct directional derivative at these saved poses.
Endpoint-cell changes occur at larger multipliers. Their loss partitions
are recorded, together with the original nonlinear prior contribution; a
cell-change count alone does not identify a physical correspondence error or
prove which term causes an exhausted search.

The sweep contains **168** signed probes: **158 valid / 10 invalid**. All
**32** original positive search candidates reproduce their recorded costs
and validity exactly. They are included in the 168, rather than additional
independent observations. Separate polynomial/relative-camera calculations
agree on every candidate domain and endpoint-cell count. Across valid probes,
maximum total-cost disagreement is **8.30e−16** and maximum paired-difference
disagreement **2.56e−16**. These are observed implementation differences,
not certified bounds on numerical error. The tiniest multipliers can amplify
rounding in secants; the complete signed sweep retains those values.

## Fixed experiment

The [protocol](../assets/depth-search-diagnostic-v1/design.json) selects the
three valid-candidate failures and the one sampling-domain failure from the
[thirty-correction result](long-depth-refinement.md). Its **75 source/context
members** are archived before any recorded sweep. This is post-hoc development
diagnosis of viewed cases, without a new held-out trial.

For each window, reconstruct the population from its **original initial poses**,
and check the exact source IDs and original frozen normalizers. Evaluate at
the archived final poses with the same incoming prior. Reconstruct the terminal
matrix independently, retain the unchanged guarded **full** Gauss-Newton
proposal, and replay the failed eight-candidate search.

Probe both signs of `2^-e` for **every exponent 0 through 20**. Positive
multipliers are possible steps along that proposal. Negative multipliers
supply the opposite finite difference; they are diagnostic samples rather
than accepted search steps. All non-origin poses receive the same left-root
increment convention as the solver, in metres and radians. The first optical
origin stays fixed. The experiment makes no tracker, timestamp, prior or
Schur-state commit and performs no iterative pose fitting.

Every original source must remain supported. Invalid sampling produces a
guard with **no objective or partial loss**, preserving the fixed population.
The diagnostic retains the original support, proposal and convergence guards.
It reports Armijo decisions with and without the existing
`1e-12 * max(1, abs(before_cost))` slack.

For precision, subtract Huber losses per source before summing with `math.fsum`.
Quadratic pairs use `(r1-r0)*(r1+r0)/2`; linear pairs subtract absolute residuals.
Cross-branch pairs use the original Huber losses. Add the separately evaluated
nonlinear prior difference. Full-objective subtraction is also retained.
Cell accounting groups sources by whether the two **endpoint** cells differ;
it does not count all cell crossings along the intervening curved pose path.

## Verification and reproduction

The operational calculation uses the existing corner-weight sampler and
root-frame transforms. The audit uses independently constructed SVD normals,
relative-camera transforms, polynomial inverse-depth sampling, incidence/prior
matrices, spectral solve, Rodrigues increments and homogeneous projection.
Raw PNG hashes, timestamps and axial-Z conversion (`/5000` exactly once) are
checked before either calculation. Pixel/vertex construction and sweep aggregation are
shared; this is not an independent implementation of the entire pipeline.
Prior populations, normalizers and measured observations remain the same.

Three authored image fixtures exercise a smooth cell, an exact grid knot
with opposing one-sided slopes, and lost sampling support. Their **126**
signed probes use injected directions, without image-derived solver proposals
or a physical accuracy claim. Four corruptions of costs, cell counts, invalid
costs and loss partitions are rejected. CI runs these fixtures without the
original recordings. Core verification passes **371 Rust tests and 49 scenario
runs with complete sensor-log replays**; 245 core outputs and all 16 pinned
native estimator sources retain their previous digests.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-search.py \
  --analytic-only --output artifacts/depth-search-diagnostic/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-search.py \
  --output artifacts/depth-search-diagnostic/recorded-repeat
bash scripts/check.sh
```

Use the project's pinned NumPy/Pillow environment. Output directories must be
new. Recorded reproduction needs the previously acquired, hash-qualified
room depth PNGs. The frozen [summary](../assets/depth-search-diagnostic-v1/summary.json),
[statistics](../assets/depth-search-diagnostic-v1/statistics.json),
[validation](../assets/depth-search-diagnostic-v1/validation.json), source
archive and manifest preserve the first result. No numeric reference labels,
new trajectory, automotive integration, timing claim or maturity increase
comes from these probes. The subjective mature-OSS comparison stays **about
20%**; the 30% and 50% goals remain unmet.
