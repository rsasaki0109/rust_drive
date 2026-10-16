# Cell-transition slopes at the saved stalls

The diagnostic identifies a local interpolation barrier at all **six saved
budget-stopped states**: room sources 120/160/240 from the deeper-search and
damped-direction methods. Along each final undamped GN proposal, the reported
objective slope changes from negative to positive across a small interval.
Exactly **one fixed source occurrence** changes endpoint interpolation cell,
accounting for at least **99.967%** of the slope change. Both methods encounter
the **same source pixel** in each window. This explains a concrete local
barrier to taking a larger step along those directions, without proving a
multidimensional minimum or physical correspondence error.

## Recorded result

Each anchor uses its last recorded GN multiplier as scale `s`. The table shows
the adjacent positive probe interval where the slope reverses. Slopes are
weighted objective derivatives per unit multiplier, without a calibrated
physical-error interpretation.

| Room source | Method | Interval / s | Slope before → after | Changed source pixel | Changed-cell share of slope jump |
|---|---|---|---|---:|---:|
| 120 | Deeper search | 1/8 → 1/4 | −5.2751e−8 → +1.9298e−8 | 197400 | 99.9981% |
| 120 | Damped | 1 → 2 | −3.7082e−8 → +1.8019e−8 | 197400 | 99.9965% |
| 160 | Deeper search | 1 → 2 | −2.4417e−7 → +6.9362e−8 | 261580 | 99.9670% |
| 160 | Damped | 1/4 → 1/2 | −5.1565e−7 → +7.5900e−8 | 261580 | 99.9975% |
| 240 | Deeper search | 1/2 → 1 | −1.3422e−7 → +2.4565e−8 | 258708 | 99.9947% |
| 240 | Damped | 1/4 → 1/2 | −1.9196e−7 → +3.1581e−8 | 258708 | 99.9998% |

IDs are original linear source-image pixel identities, not physical point
labels. Their target/source node pairs are **2/4**, **1/4**, **3/4** for sources
120, 160, 240 respectively. Exact identities and digests are retained for every
transition, including transitions outside the table. Both methods reuse the
same four viewed windows and original observations.

The two source-279 domain-control states instead lose fixed sampling support
at positive probe **s/2** and all larger positive probes. Their smaller positive
probes remain valid and descending. Invalid probes retain a sampling guard
and have no objective or slope.

Across eight anchors there are **152 probes**, comprising **140 valid / 12
invalid**, and **144 adjacent transitions** with **132 valid slope partitions**.
All independent comparisons resolve: no endpoint-cell disagreements occur.
Maximum implementation differences are **2.85e−12 pixels**, **1.02e−15 total
objective** and **3.83e−17 reported slope**. These are observed differences,
not certified numerical bounds. All sampled coordinates remain outside the
sampler's `1e−10`-pixel integer-snap band.

## Fixed read-only experiment

The [protocol](../assets/depth-cell-slopes-v1/design.json) and **86 source/context
members** are frozen before recorded probing. Reconstruct each population from
its **original initial poses**, then check the exact source IDs, frozen center
normalizers and original incoming prior. Evaluate at the archived final pose
from the [deeper-search](deep-search-depth-refinement.md) and
[damped-direction](damped-depth-refinement.md) reports. Reconstruct all **eight
existing terminal systems** independently; they are repeated states, without
new fitting equations or independent acquisitions.

For the unchanged guarded terminal GN direction, evaluate zero and both signs
of `s * 2^k` for every integer **k = −4 through 4**, giving nineteen positions.
Use the last trace entry's GN accepted multiplier as `s`, or `2^-20` if that
search had no passing candidate. No adaptive probes or optimizer continuation
follow the result. Source 279 supplies a sampling-domain control in each
method; the other six anchors are already viewed budget stops.

The original left-root path uses rotation `exp(alpha*omega)` and translation
`alpha*v`. At a nonzero multiplier its instantaneous left tangent is
`[v - alpha*(omega cross v), omega]`. The correction to translation matters
when both target and source rotate. Sum each clipped Huber residual times
its reported branch derivative along that tangent, plus the original nonlinear
prior derivative. Analytic controls separately verify the moving target/source
path against cost finite differences away from knots.

For each adjacent pair, partition the slope change into changed endpoint
cells, unchanged endpoint cells and the prior. Preserve exact changed-source
membership and verify that these terms sum to the total change. Per-source
paired Huber loss differences and a separate prior difference also provide
cost secants, reducing total-cost cancellation. Endpoint accounting does not
locate the earliest crossing, count all intervening crossings, or isolate a
unique continuous-time event.

The sampler rounds projections within `1e−10` of an integer. Its reported
derivative is the selected bilinear branch derivative, which need not equal
differentiating the literal rounded operation inside that band. Snap counts
are explicit. Independent cell choices differing within `3e−11` pixels of a
switch are retained as unresolved, with both answers excluded from slope-sign
conclusions. An injected analytic ambiguity exercises that branch even though
the recorded probes have no unresolved cases.

## Verification and interpretation

The operational computation uses existing root-frame transforms and explicit
corner weights. The audit uses relative-camera homogeneous projection,
polynomial sampling, independently constructed SVD normals and prior gradients,
and Rodrigues increments. Original raw PNG hashes, timestamps and axial Z
conversion (`/5000` once) precede both surface constructions. Original image
vertices, observations, path grid and partition aggregation remain shared;
this is not a completely independent pipeline.

Four authored fixtures exercise smooth sampling, a grid kink with opposing
one-sided slopes, lost support and a rotating target/source path. Their **76
probes** use injected directions and supply no guarded-solve or physical pose
accuracy claim. Three corruptions of slopes, invalid costs and cells are
rejected. A separate injected roundoff-sized switch is explicitly unresolved.
CI runs these controls without the recordings. Core verification passes **371
Rust tests and 49 scenario runs with complete sensor-log replays**; all 245
core outputs and 16 pinned native estimator sources retain their digests.

The local slope reversals and source-level attribution narrow the numerical
stall mechanism. They do **not** authorize dropping those pixels, treating
them as physical outliers, or replacing the convergence gate. A directional
minimum does not prove stationarity in all pose coordinates. A smoother
interpolation objective or an explicit treatment of active cell boundaries
would require a separate comparison of convergence, sampling availability
and physical pose accuracy. Neither is implemented by this diagnostic.

## Reproduce

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-cell-slopes.py \
  --analytic-only --output artifacts/depth-cell-slopes/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-cell-slopes.py \
  --output artifacts/depth-cell-slopes/recorded-repeat
bash scripts/check.sh
```

Use the pinned NumPy/Pillow environment, new output directories and previously
acquired hash-qualified room PNGs. Frozen
[summary](../assets/depth-cell-slopes-v1/summary.json),
[statistics](../assets/depth-cell-slopes-v1/statistics.json) and
[validation](../assets/depth-cell-slopes-v1/validation.json), source archive and
manifest preserve the first recorded result. There is no new optimizer result,
numeric real ground truth, trajectory fitting, physical score or tracker/prior/
Schur commit. Latest continuous convergence remains **2/10**, original-discrete
fresh convergence **0/10**, and subjective mature-OSS comparison **about 20%**.
The 30% and 50% goals remain unmet.
