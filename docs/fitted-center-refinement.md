# Retaining legacy centers while admitting the fitted depth field

Local fixed-objective convergence improves from **9/10 to 10/10** on the same
ten viewed room/desk2 windows. Every original **60,288 source occurrence**, its
point, normal and engineering metric normalizer is retained exactly. The new
selector admits **20,140 additional occurrences** from those existing frames,
bringing the total to **80,428** across 57 factors.

Both original-discrete fresh and same-model reselected convergence remain
**0/10**, and desk2's median original-discrete fresh RMS worsens. The original
progression rule therefore fails. This is a local numerical result without
physical pose-accuracy evidence or native/default adoption.

## Center selection and the remaining support stop

The [boundary-support control](boundary-slope-refinement.md) still uses an
original bilinear center selector before probing the actual C1 depth field.
That center requires all four measured corner normals, although the fitted
depth field does not interpolate those normals. A missing unused corner
normal can exclude a source whose fitted-field residual is supported.

A read-only preflight inspected the room/desk2 source-200 windows before the
new source freeze. Its nine raw-pair rows are retained in the
[preflight log](../assets/fitted-center-refinement-v1/read-only-preflight.log).
For room source 200, the old center qualification prevents both relevant
factors from reaching the unchanged 200-source minimum:

| Room source-200 factor, target/source | 2/3 | 2/4, current | 3/4, current |
|---|---:|---:|---:|
| Raw reciprocal sources | 308 | 217 | 292 |
| Original bilinear corner support | 145 | 110 | 158 |
| Legacy center qualification | 141 | 107 | 155 |
| Fitted center qualification | 252 | 193 | 263 |
| Union before nine probes | 253 | 194 | 263 |
| Legacy after nine probes | 138 | 106 | 151 |
| Union after nine probes | 247 | 191 | 259 |
| Newly admitted factor population | 247 | 0 | 259 |

The new union restores an older factor and a current factor, totaling **506
source occurrences**. Pair 2/4 still fails at 191; no pooling or minimum-support
exception is introduced. Room source 200 now converges in six corrections.

## Frozen selection policy and metric ownership

Start from the same raw reciprocal matches and their measured source/nearest
target normal checks. Construct both center qualifications at the same initial
poses, before applying a whole-factor cutoff:

* The legacy branch retains the original bilinear depth/normal support,
  source-normal agreement above 0.8, distance bound and gradient-length metric.
* The added branch uses the unchanged boundary-guarded C1 depth field, the
  same source-normal agreement threshold and distance bound. It requires the
  measured depth and slope support used by that field, without filling or
  requiring unused target corner normals.

Take the union in original raw-source order. **Legacy qualification owns the
metric whenever both branches qualify**, even if the old whole factor was
below 200. A C1-only source receives the C1 gradient length at its initial
center. Then apply the exact original nine model-domain probes and the same
200-source factor threshold. Distances remain below 0.08 m at the fitted
level, or 0.12 m at the inherited coarse level.

Of the 20,140 newly admitted occurrences, **19,170 use C1-only metrics** and
**970 use legacy metrics** in previously excluded factors. The latter are new
admissions, without a new metric definition. All previously admitted source
values and metrics survive exactly. Source retention is **100%**; the expanded
population is **133.41%** of the old population. These overlapping factor/window
counts are not new independent measurements.

Within each fit the population and normalizers remain fixed. Private bindings
reuse the unchanged objective, nonlinear prior and ownership checks, Huber
loss, spectral condition/increment guards, 21 Armijo candidates, strict full
proposal convergence and 90-correction budget. Unsupported selected sources
invalidate an entire candidate. The C1 field, conservative boundary guard and
raw interpolation coordinates remain unchanged.

Adding existing-frame residuals changes the objective composition and its
balance with the unchanged prior. Gradient lengths are engineering metrics,
without calibrated uncertainty or an information-gain interpretation. Costs
across the two populations are not comparable accuracy scores.

## Recorded comparison

The [design](../assets/fitted-center-refinement-v1/design.json) and **139
source/context members** are frozen before recorded fitting. The same ten
windows, original initial poses, timestamps and incoming priors are used.
This is outcome-informed development on previously viewed data, without
held-out validation.

| Ten viewed windows | Boundary control | Center union |
|---|---:|---:|
| Fixed continuous convergence | 9/10 | 10/10 |
| Original discrete fresh convergence | 0/10 | 0/10 |
| Same-model fresh convergence | 0/10 | 0/10 |
| Initial source / factor occurrences | 60,288 / 51 | 80,428 / 57 |
| Retained original sources and metric values | 60,288 | 60,288 |
| Applied corrections | 124 | 122 |
| Invalid / nonpassing / shortened trials | 0 / 0 / 0 | 0 / 0 / 0 |
| Accepted cost increases | 0 | 0 |
| Guard / search / budget stops | 1 / 0 / 0 | 0 / 0 / 0 |

All 122 corrections accept a full step. Fewer corrections with a larger
population do not establish a runtime improvement; timing is not evaluated.
Per-window terminal results are:

| Dataset | Source index | Selected sources | Factors | Applied corrections |
|---|---:|---:|---:|---:|
| room | 120 | 8,364 | 8 | 10 |
| room | 160 | 8,881 | 6 | 15 |
| room | 200 | 506 | 2 | 6 |
| room | 240 | 11,589 | 6 | 10 |
| room | 279 | 4,119 | 3 | 8 |
| desk2 | 120 | 4,446 | 8 | 10 |
| desk2 | 160 | 12,886 | 6 | 14 |
| desk2 | 200 | 24,283 | 6 | 18 |
| desk2 | 240 | 2,138 | 6 | 19 |
| desk2 | 279 | 3,216 | 6 | 12 |

Every terminal full proposal has translation below 0.01 mm and rotation below
1e−5 rad for every free node. These are remaining optimization steps, without
a physical pose-error interpretation. All fits terminate by 19 corrections.
The 30/60/90 checkpoints carry those actual terminal states forward and give
10/10 convergence at every cap. No trace extends beyond 30, so the recorded
future-state substitution control is **not triggered** in this comparison.

All original-discrete fresh support, condition and 0.03 m residual gates pass.
Fresh RMS medians change from **2.463832 to 2.461792 mm room** and **2.711780
to 2.826047 mm desk2**. Neither fresh full-proposal convergence check passes
for any window, including reselection with the same C1 model and its ordinary
center normalizers. The desk2 regression and both 0/10 results preclude the
planned full-trajectory follow-up under the frozen progression rule.

## Independent checks and reproduction

An independent selector uses reciprocal matching, homogeneous projection,
separate bilinear/C1 polynomial samplers and source-ID unions to reconstruct
center qualification and metric ownership. It checks all **61 raw-pair stage
rows**, complete selected metadata and prepared fields. Existing independent
matrix, nonlinear-prior, objective, spectral-solve, Rodrigues-update and
checkpoint checks are reused through isolated bindings; the original control
solver and audit retain their original selection.

Recorded matrix roles are **142 new-report / 144 rechecked control / 60
checkpoint / 10 reselected systems**. These reuse observations and repeated
states. Authored controls check C1-only admission, legacy-only retention,
dual alignment rejection, distance rejection, missing depth and an unused
missing corner normal. The rendered moved fit retains 998 sources and converges
after one correction. Missing/plane data retain their guards, while unquantized
and 2 mm quantized identities converge without corrections. Both historical
939-source boundary identity resolutions remain independently checked.

The new analytic report checks 11 report systems, six checkpoint systems and
one separate prior system, including 12 nonzero-prior gradient axes. Four
corruptions reject: false convergence, source identities, metric digest and
final matrix. The recorded run repeats the analytic controls; its result
matches the final standalone analytic run exactly. CI includes this suite.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-fitted-center-refinement.py \
  --output artifacts/fitted-center-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-fitted-center-refinement.py \
  --output artifacts/fitted-center-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, the existing hash-qualified PNGs and new output
directories. The [analytic evidence](../assets/fitted-center-refinement-v1/analytic.json),
[summary](../assets/fitted-center-refinement-v1/summary.json),
[statistics](../assets/fitted-center-refinement-v1/statistics.json) and
[validation](../assets/fitted-center-refinement-v1/validation.json) retain the
first complete result in a 12-file evidence manifest. PNG digests and original
timestamps are checked before geometry construction; axial Z is converted
with `/5000` once, and decoded observations are shared.

Core checks pass **371 Rust tests and 49 scenarios with complete sensor-log
replays**. All 245 core-output hashes, 16 pinned native-source hashes and 619
previous evidence-manifest entries remain unchanged. No numeric real reference
label is parsed, physical trajectory scored, tracker/prior/Schur state committed
or native/default setting changed. Vehicle integration remains unverified.
Subjective maturity stays **about 20%** versus mature driving OSS;
**30% and 50% remain unmet**.
