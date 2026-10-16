# Depth-derived C1 interpolation and one-sided slopes

The final local variant retains **all 32,507 initial source occurrences** and
converges on **5/10** viewed windows, versus **4/10** with measured-normal
Hermite sampling. Its **97 applied corrections all use full steps** and
strictly lower their fixed objective. Original-discrete fresh convergence
remains **0/10**, and one observable window reaches the 30-correction cap.
The frozen progression rule fails; this remains an optional research path.

The first depth-difference version regresses to **3/10**. Its complete failure
is preserved before a separately frozen one-sided follow-up. The comparison is
outcome-informed development on the same ten windows, not held-out validation,
a new physical trajectory score or native/default adoption.

## Recorded comparisons

| Ten viewed windows | Local-normal control | Bilateral depth slopes | One-sided depth slopes |
|---|---:|---:|---:|
| Own continuous-objective convergence | 4 | 3 | 5 |
| Original-discrete fresh convergence | 0 | 0 | 0 |
| Initial source occurrences / factor occurrences | 32,507 / 36 | 32,507 / 36 | 32,507 / 36 |
| Initial-current-support stops | 4 | 4 | 4 |
| Search stops | 2 | 2 | 0 |
| Budget stops | 0 | 1 | 1 |
| Applied corrections | 86 | 86 | 97 |
| Shortened steps | 24 | 22 | 0 |
| Invalid / accepted / valid nonpassing candidates | 276 / 86 / 0 | 252 / 86 / 0 | 0 / 97 / 0 |
| Accepted cost increases | 0 | 0 | 0 |

The [local-normal control](local-normal-depth-refinement.md) supplies the
original poses, timestamps and incoming nonlinear priors. Both new runs retain
all **20 room / 16 desk2 factors**, their exact source metadata and original
bilinear-center metric normalizers. The one-sided follow-up also starts with
exactly equal initial costs to the bilateral version. No observations are
discarded during fitting to obtain these results.

| Window | Local-normal control | Bilateral slopes | One-sided slopes |
|---|---|---|---|
| Room / 120 | Converged 11 | Converged 13 | Converged 12 |
| Room / 160 | Converged 7 | Converged 10 | Converged 10 |
| Room / 200 | Initial support | Initial support | Initial support |
| Room / 240 | Converged 9 | Search after 9 | Converged 7 |
| Room / 279 | Search after 15 | Converged 12 | Converged 12 |
| Desk2 / 120 | Initial support | Initial support | Initial support |
| Desk2 / 160 | Search after 24 | Budget 30 | Budget 30 |
| Desk2 / 200 | Converged 20 | Search after 12 | Converged 26 |
| Desk2 / 240, 279 | Initial support | Initial support | Initial support |

The first depth-slope model newly converges on room / 279 but loses room / 240 and
desk2 / 200. Its two exhausted searches each have 21 unsupported candidates.
The follow-up extends the available field and restores those two windows.
It reruns from the same original initial states; the corrections are not
appended to the failed first fits. Each changed window diverges from its first
trace entry. Seven complete bilateral reports remain exactly equal, including
room 160/279 and the desk2 / 160 budget stop. Successes across variants are not
combined into a higher per-window headline.

| One-sided converged window | Remaining maximum full translation (mm) | Remaining maximum full rotation (rad) |
|---|---:|---:|
| Room / 120 | 0.00937634 | 3.04386e−6 |
| Room / 160 | 0.00435289 | 7.69060e−6 |
| Room / 240 | 0.00798907 | 7.41643e−6 |
| Room / 279 | 0.00683972 | 4.44492e−6 |
| Desk2 / 200 | 0.00959965 | 7.60234e−6 |

These full undamped proposals satisfy the unchanged strict **0.01 mm / 1e−5
rad** thresholds. Desk2 / 160 still has **0.054760 mm / 4.37849e−5 rad** remaining
after 30 full corrections. A budget stop does not grant convergence. At
caps 10/20/30, own convergence is **1/3/3** for bilateral slopes and **2/4/5**
for one-sided slopes. Actual terminal counts and checkpoint event ordering
are preserved.

The four initial-support stops persist because the selected fixed population
lacks sufficient current-frame contributions. Original-discrete fresh matching
still passes its support, conditioning and **0.03 m RMS** gates on all ten
states, but none has a sufficiently small fresh proposal.

Room median fresh RMS is **2.473233 mm** with local normals and **2.463668 mm**
in both depth-slope variants. Desk2 stays **2.740464 mm**. The follow-up's changed
room 120/240 residuals slightly worsen (**2.316526→2.316676 mm** and
**2.348367→2.354490 mm**), while desk2 / 200 improves
(**1.604468→1.600373 mm**). Equal medians do not establish per-window parity.
These are fitting residuals, without numeric real reference labels or physical
pose-error scoring. No continuous trajectory is refit or tracking state committed.

## Field model and frozen protocols

The [first design](../assets/depth-slope-refinement-v1/design.json) and
**110 source/context members** are archived before its fits. After preserving
that result, the [follow-up design](../assets/one-sided-depth-refinement-v1/design.json)
and **115 source/context members** are separately archived before the next
fits. Both use source indices **120/160/200/240/279** in the same two recordings.

For each image axis, consecutive inverse-axial-depth differences give secants
`a` and `b`. If both have the same nonzero sign, the vertex slope is their
harmonic mean `2*a*b/(a+b)`; otherwise it is zero. Values have units m⁻¹ and
image slopes m⁻¹/pixel. The bilateral field needs the measured center and all
four cardinal depth neighbors. Depths must lie strictly within **0.3..5 m**,
and each neighbor must differ from the center by strictly less than
`0.05+0.01*center_depth` metres. The two-pixel field border stays unsupported.

The one-sided extension uses the same neighbor eligibility. If both sides are
eligible, it keeps exactly the same harmonic slope. With only the low side,
use `rho_center-rho_low`; with only the high side, use
`rho_high-rho_center`. A vertex with neither eligible side on either axis is
unsupported. All choices are precomputed per vertex, then fixed throughout
fitting. They never depend on a candidate's residual or success.

Both fields feed the unchanged four-corner tensor bicubic Hermite sampler:
measured corner values, shared vertex slopes and zero mixed derivatives.
Adjacent jointly supported cells share values and both first derivatives,
making the field C1. The follow-up preserves every bilateral-valid vertex
slope and every bilateral-supported sample value/derivative exactly. New
support uses actual in-range measured neighbors on the eligible side.

Every interpolated cell still needs all four measured depths and the original
`max_depth-min_depth <=0.05+0.01*min_depth` gate. Missing corners remain missing;
there is no depth filling, integer snapping, clipping or bilinear fallback.
Interpolated inverse depth must be finite and positive, with finite derivatives.
One-dimensional slope limiting does not certify a two-dimensional range bound
or physical surface correctness. One-sided estimates have less neighborhood
support and may be less robust to noise.

Target sampling now uses depth differences directly. It no longer requires a
measured target plane normal or plane-offset gate. This changes the measurement
slopes and sampling domain. **Initial geometric matching and nine bilinear
admission probes still use their original measured normals**. Original depth,
vertices, normals, intrinsics, source identities and engineering metric
normalizers remain available unchanged. The source population and slope caches
remain fixed throughout the complete attempt.

AST/dependency proofs preserve objective assembly, incoming nonlinear prior,
Huber 0.01 m loss, GN solve, **21 Armijo multipliers** through `2^-20`, original
numerical slack, **30** correction cap, current support, scaled SPD condition
**1e6**, full-proposal bounds **0.10 m / 0.15 rad**, strict convergence and
checkpoint ordering. No damping, prior accumulation or tracker/Schur commit is
introduced. The solver alone is not the experimental difference.

## Independent evidence and reproduction

The operational field uses depth slices and the product-form harmonic mean.
The oracle stacks five samples and computes the reciprocal of their mean
reciprocals for same-sign secants. It independently checks side eligibility
and complete slope masks. Polynomial endpoint constraints reconstruct bicubic
values/derivatives separately from the operational Hermite basis. Separate
relative-camera residuals, SVD normals for original source selection,
prior/incidence matrices, spectral solves, Rodrigues increments and checkpoint
audits verify every report. Original PNG digests and axial-Z `/5000` conversion
precede both surface constructions; decoded depths/vertices are shared.

Analytic checks cover affine planes, six authored harmonic cases, **241
one-dimensional monotone samples**, integer interpolation, both C1 edge
derivatives, nonaffine XYZ finite differences, zero-normal target sampling,
missing corners/cardinal samples, an unused diagonal, depth jumps, absent-cache
failure and corrupt field masks/slopes. Eight additional side-availability
fixtures check domain inclusion and exact shared values, with **5,732 old /
5,796 new supported authored queries**. Newly admitted cells pass C1 and XYZ
finite differences. Original geometry and source normals remain unchanged.

Both rendered warm starts converge in one correction with **988/988 sources**.
Degenerate images retain guards; all **12 nonzero-prior gradient axes** pass,
and four report corruptions are rejected in each suite. Each suite checks
**7 report / 6 checkpoint systems plus 1 separate prior system**.

Recorded matrix roles are **106 new-report / 106 control / 60 checkpoint** in the
first trial and **117 new-report / 106 control / 60 checkpoint** in the follow-up.
These roles share observations and repeat terminal states; they are not
independent new measurements. The complete first summaries, sources and failed
outcomes remain separately frozen:
[first validation](../assets/depth-slope-refinement-v1/validation.json),
[follow-up validation](../assets/one-sided-depth-refinement-v1/validation.json),
[follow-up statistics](../assets/one-sided-depth-refinement-v1/statistics.json).

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-depth-slope-refinement.py \
  --output artifacts/depth-slope-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-depth-slope-refinement.py \
  --output artifacts/depth-slope-refinement/recorded-repeat
OPENBLAS_NUM_THREADS=1 python scripts/check-one-sided-depth-refinement.py \
  --output artifacts/one-sided-depth-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-one-sided-depth-refinement.py \
  --output artifacts/one-sided-depth-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, fresh output directories and existing hash-qualified
raw PNGs. CI includes both analytic suites. Core regression passes **371 Rust
tests and 49 scenarios with complete sensor-log replays**, preserving 245 core
output and 16 native-source digests. No new continuous real-trajectory accuracy,
vehicle integration or timing evidence is added. Subjective maturity remains
**about 20%** versus mature driving OSS; **30% and 50% remain unmet**.
