# Consistent support near depth-cell boundaries

A conservative numerical support guard resolves the **two archived identity
admission disagreements** in the new research variant. Both implementations
now select the same **939 sources** in each authored fixture and independently
verify identity convergence without corrections.

On the ten viewed recorded windows, **all complete fit reports and checkpoint
sets exactly reproduce the original admission control**. The initial 60,288
source occurrences, their original normalizers, all 124 applied corrections
and **9/10 local convergence** are retained. Both original-discrete fresh and
same-model reselected convergence remain **0/10**. This fixes an observed
domain-consistency problem, without new physical pose-accuracy evidence or
native/default adoption.

## What the guard changes

The [earlier identity controls](reselected-slope-refinement.md#unresolved-identity-boundary-controls)
project exactly onto integer image coordinates. Tiny differences in the
operational and homogeneous projection calculations can choose different
adjacent cells. A cell with missing depth or slope support can then reject
one answer while the other answer remains supported. A C1 field across jointly
supported cells does not provide support across an unsupported neighbor.

The new policy forms a rectangle of **±1e−9 pixels** around each original
projected query. In each axis, take the floor of the lower and upper endpoint.
Require every cell at the four endpoint combinations to be supported. At an
integer coordinate this includes both incident cells; at an integer corner it
includes four. Away from grid boundaries, all four combinations name the same
original cell. Endpoint floors define the convention at the band's outer edge.

Each supported cell retains the original conditions: image bounds, four
measured depths strictly between 0.3 and 5 m, bounded depth spread, original
one-sided slope support at every vertex and finite slopes. A separately cached
Boolean cell map supplies this check. Missing depth remains missing.

After this domain check, call the original Hermite sampler on the **original
coordinates**. Retained residuals, target points and projective derivatives
are unchanged; no snapping, extrapolation, clipping or fallback is introduced
in the C1 sampler. The original bilinear geometric center selector and its
historical normalizers remain separate and unchanged.

The rectangle is an engineering guard, without a certified floating-point
error bound or probabilistic interpretation. Its outer edge is itself a
finite-precision classification. The controls resolve the two archived cases;
they do not establish universal numerical stability. Candidate support can
still change at unsupported geometry, and finite initial probes do not certify
the whole future optimization path.

## Explicit source loss in the authored cases

The guard is conservative and uses the measured cell map directly. It does not
intersect operational and audit answers at runtime or add missing observations.

| Identity fixture | Old operational / audit counts | New agreed count | Removed from operational / audit |
|---|---:|---:|---:|
| Unquantized | 963 / 974 | 939 | 24 / 35 |
| Quantized 2 mm | 964 / 974 | 939 | 25 / 35 |

The old disagreements are reproduced exactly, including their source IDs.
New source identities agree, and both zero-update fits plus 30/60/90 checkpoints
are independently verified. The corresponding historical sources and failed
evidence remain unchanged. Source retention is lower in these authored cases;
the 100% retention result below applies to the ten recorded windows only.

Curved-field controls check 18 projective derivative components, including
jointly supported grid knots, and bit-identical retained values versus the old
sampler. An unsupported neighboring cell rejects quarter/half-band queries on
either side of the boundary; a query safely outside the band uses ordinary
cell support. A missing diagonal corner requires all four incident cells,
and nonfinite/out-of-image queries reject. The additional
[band probes](../assets/boundary-slope-refinement-v1/band-probes.json) retain
their complete camera, query and support parameters.

## Recorded comparison and solver contract

The [design](../assets/boundary-slope-refinement-v1/design.json) and **135
source/context members** are frozen before fitting. This is outcome-informed
development on the same room/desk2 windows at source indices
120/160/200/240/279, without held-out data.

| Ten viewed windows | Original admission | Boundary support |
|---|---:|---:|
| Fixed continuous convergence | 9 | 9 |
| Original discrete fresh convergence | 0 | 0 |
| Same-model fresh convergence | 0 | 0 |
| Initial source / factor occurrences | 60,288 / 51 | 60,288 / 51 |
| Applied corrections | 124 | 124 |
| Invalid / nonpassing / shortened trials | 0 / 0 / 0 | 0 / 0 / 0 |
| Accepted cost increases | 0 | 0 |
| Initial-current-support stops | 1 | 1 |
| Search / budget stops | 0 / 0 | 0 / 0 |

All ten complete reports and their three checkpoints match the old control
exactly, including source/normalizer metadata, matrices, every correction,
trial receipt and terminal pose. Every source point, source normal and retained
engineering metric normalizer stays unchanged. There are no lost or newly
admitted recorded initial source occurrences. These are overlapping
factor/window counts, without new independent acquisitions.

Private unchanged objective/search and extended-driver functions bind the
new sampler and initial population. The original nine-probe body, bilinear
geometric seed, minimum 200 sources per factor, fixed population throughout
the 90-correction attempt, original nonlinear prior/ownership checks, Huber
loss, condition/increment guards, 21 Armijo candidates and strict full-proposal
convergence remain unchanged. Any lost selected support invalidates a candidate;
no source is dropped during fitting.

Room source 200 still has no admitted current factor. All nine other local
fits converge by 27 corrections. Original discrete fresh RMS medians remain
**2.463832 mm room / 2.711780 mm desk2**, with all 0.03 m residual gates passing
and no fresh convergence. A read-only same-model reselection also remains
0/10. These residuals do not measure physical pose error; the original
progression rule fails. The failed three-round experiment is not refitted or
promoted by this change.

## Independent checks and reproduction

The audit constructs cell support with separate sliding windows and uses
homogeneous projection. Independently prepared one-sided fields and polynomial
interpolation feed unchanged incidence/prior matrices, spectral solves,
Rodrigues increments, candidate-domain checks and checkpoints. Both operational
and audit cell maps agree at every prepared frame/level. Private binding proofs
keep the original control solver and audit isolated.

Recorded matrix roles are **144 new-report / 144 rechecked control / 60
checkpoint / 10 reselected systems**. These reuse observations and repeated
states. Analytic roles are 11 report systems, 18 checkpoint systems and one
separate prior system, with 12 nonzero-prior gradient axes. Four report
corruptions reject; missing/plane data retain their guards. The moved rendered
fit remains checked separately from the two identity cases.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-boundary-slope-refinement.py \
  --output artifacts/boundary-slope-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-boundary-slope-refinement.py \
  --output artifacts/boundary-slope-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, existing hash-qualified PNGs and new output directories.
The [analytic evidence](../assets/boundary-slope-refinement-v1/analytic.json),
[summary](../assets/boundary-slope-refinement-v1/summary.json),
[statistics](../assets/boundary-slope-refinement-v1/statistics.json) and
[validation](../assets/boundary-slope-refinement-v1/validation.json) retain the
first complete result. Original PNG digests, timestamps and axial-Z `/5000`
conversion precede both geometry constructions; decoded observations remain
shared. CI includes the new analytic suite.

Core checks pass **371 Rust tests and 49 scenarios with complete sensor-log
replays**. All 245 core-output and 16 pinned native-source hashes remain
unchanged. No numeric real reference label is parsed. No physical trajectory
score, new continuous trajectory, tracker/prior/Schur commit or native/default
change is introduced. Vehicle integration and timing remain unverified. Subjective
maturity stays **about 20%** versus mature driving OSS; **30% and 50% remain unmet**.
