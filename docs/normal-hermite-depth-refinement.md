# Measured-normal four-corner C1 inverse depth

The new local variant retains **all 32,507 initial source occurrences** and
improves its own continuous-objective convergence from **2/10 to 4/10** viewed
windows. Room gains three converged windows; desk2 loses one. Original-discrete
freshly associated convergence remains **0/10**, and room median fresh RMS
regresses. The frozen progression rule therefore fails. This optional research
path does not replace the native default or establish physical pose accuracy.

| Ten viewed windows | Bilinear control | 16-corner cubic | Cubic margin | Normal Hermite |
|---|---:|---:|---:|---:|
| Own continuous-objective convergence | 2 | 0 | 1 | 4 |
| Original-discrete fresh convergence | 0 | 0 | 0 | 0 |
| Initial source occurrences | 32,507 | 32,507 | 16,234 | 32,507 |
| Initial factor occurrences | 36 | 36 | 19 | 36 |
| Missing-current-support stops | 4 | 4 | 7 | 4 |
| Line-search stops | 1 | 6 | 2 | 2 |
| Budget stops at cap 30 | 3 | 0 | 0 | 0 |
| Applied corrections | 134 | 40 | 50 | 79 |

The earlier [cubic and margin protocols](cubic-depth-refinement.md) remain
unchanged. This is a separately frozen, outcome-informed development comparison
on the same ten windows, **not held-out validation**. Per-window successes from
different variants are not combined into a higher convergence count.

## Recorded outcomes

| Recording / source index | Bilinear control | Normal Hermite |
|---|---|---|
| Room / 120 | Budget 30 | Converged 11 |
| Room / 160 | Budget 30 | Converged 7 |
| Room / 200 | Missing current support | Missing current support |
| Room / 240 | Budget 30 | Converged 9 |
| Room / 279 | Search after 9 | Search after 8 |
| Desk2 / 120 | Missing current support | Missing current support |
| Desk2 / 160 | Converged 16 | Search after 24 |
| Desk2 / 200 | Converged 19 | Converged 20 |
| Desk2 / 240, 279 | Missing current support | Missing current support |

Every initial source metadata entry and original metric-normalizer digest
equals the bilinear control, including all **20 room / 16 desk2 factors**.
The remaining full proposal at each new converged state is:

| Window | Maximum translation norm (mm) | Maximum rotation norm (rad) |
|---|---:|---:|
| Room / 120 | 0.00692414 | 8.63940e−6 |
| Room / 160 | 0.00693017 | 4.09193e−6 |
| Room / 240 | 0.00776897 | 4.16560e−6 |
| Desk2 / 200 | 0.00909511 | 7.42542e−6 |

These satisfy the unchanged strict **0.01 mm / 1e−5 rad** thresholds on the full
undamped proposal. Small applied line-search steps do not grant convergence.
At caps 10/20/30, own convergence is **2/4/4**. Desk2 / 160 reaches a support-domain
search failure after 24 corrections; its cap 20 checkpoint honestly remains a
budget stop. Early terminal states carry their actual applied counts forward.

There are **198 invalid / 79 accepted / zero valid nonpassing** candidates and
**17 shortened steps**. Every accepted step strictly lowers its own fixed
objective; none relies on the existing numerical slack to increase cost. Both
terminal searches reject all **21 candidates** for fixed-source support loss.
Remaining full proposals there are **1.323947 mm / 0.00115514 rad** at room / 279
and **0.104421 mm / 8.92221e−5 rad** at desk2 / 160. C1 interpolation does not remove
domain boundaries.

All ten original-discrete fresh support/conditioning and **0.03 m RMS** gates
pass, but none has a sufficiently small fresh proposal. Room median fresh RMS
changes from **2.454214 to 2.473233 mm**; desk2 stays **2.740464 mm**. Residual
values from different interpolation objectives are not physical accuracy
scores. No numeric real reference labels are parsed or trajectory refit.

## Model and frozen protocol

The [design](../assets/normal-hermite-depth-refinement-v1/design.json) and
**100 source/context members** are archived before real local fitting. Use the
same original poses, timestamps, nonlinear incoming priors, and source indices
**120/160/200/240/279** in both recordings as the
[bilinear deeper-search control](deep-search-depth-refinement.md).

At each of four pixel corners, measured optical depth Z sets inverse depth
`rho=1/Z`. For the measured optical vertex p and fixed measured normal n, the
tangent plane supplies `rho_u=n_x/(fx*(n·p))` and
`rho_v=n_y/(fy*(n·p))`. Tensor bicubic Hermite interpolation uses these values,
first derivatives and a fixed **zero mixed derivative at every vertex**.
Adjacent jointly supported cells share all endpoint data, so their values and
both first derivatives agree. Exact projection derivatives form the original
root-frame incidence blocks.

Normals come from the existing **5×5 measured neighborhood**, with PCA in the
operational construction and SVD in the audit. Four-corner sampling therefore
does not mean only four raw measurements. Initial selection also retains the
original geometric matching and nine bilinear admission probes.

The sampler keeps original front/image bounds, four depths strictly between
**0.3 and 5 m**, finite corner normals with norm greater than 0.5, and the original
`max_depth-min_depth <=0.05+0.01*min_depth` continuity gate. The absolute
unit-normal plane offset must exceed **1e−6 m**. Interpolated inverse depth
must be finite and positive, with finite derivatives. There is no integer
snapping, clipping or fallback. Inconsistent measured normals can cause cubic
overshoot; positive finite values are not proof of correct surface shape.

Filter the original bilinear sampling-domain population once for new center
support; require at least 200 sources per factor. Preserve surviving source
identities, points, normals and original bilinear center normalizers exactly.
The actual recorded run loses no initial source. Freeze observations and
normalizers throughout fitting, rejecting any candidate that loses support.
These normalizers remain engineering metric scales, not calibrated uncertainty.

AST proofs retain unchanged objective assembly, nonlinear prior, Huber 0.01 m
loss, GN solver, **21** Armijo candidates through `2^-20`, numerical slack,
**30** correction cap, checkpoint ordering, conditioning/full-proposal guards
and strict convergence. No damping or tracker/Schur state commit occurs.
Interpolation, slopes and sampling domain change together; this comparison
does not isolate a benefit of C1 continuity alone.

## Independent evidence and reproduction

The sampler uses explicit Hermite bases. The oracle instead solves endpoint
value/derivative constraints for polynomial coefficients and constructs plane
covectors using homogeneous camera coordinates. Separate relative-camera
residuals, SVD normals, prior/incidence matrices, spectral solves, Rodrigues
increments, and candidate/checkpoint audits reconstruct the reports. Original
PNG hashes and axial-Z `/5000` conversion precede both surface constructions;
decoded depth vertices remain shared.

Analytic controls check plane reproduction, exact integer values, both edge
derivatives with inconsistent slopes, affine and nonaffine XYZ finite
differences, normal sign/scaling, all four missing corners, allowed missing
outer corners, invalid normals/near-zero plane offset, negative overshoot,
degenerate guards and all **12 nonzero-prior objective-gradient axes**. The
rendered warm start converges in one correction, retaining **988/988 sources**.
Four report corruptions are rejected. Analytic verification covers **7 report
systems, 6 checkpoint systems and 1 separate prior system**.

Recorded verification covers **99 new-report / 154 control / 60 checkpoint
systems**. These roles reuse observations and terminal states; they do not
constitute independent new measurements. The complete first summary, failed
searches, selection receipts and archived source members are retained.

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-normal-hermite-depth-refinement.py \
  --output artifacts/normal-hermite-depth-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-normal-hermite-depth-refinement.py \
  --output artifacts/normal-hermite-depth-refinement/recorded-repeat
bash scripts/check.sh
```

Use the pinned NumPy/Pillow environment, new output directories and existing
hash-qualified raw PNGs. See the [summary](../assets/normal-hermite-depth-refinement-v1/summary.json),
[statistics](../assets/normal-hermite-depth-refinement-v1/statistics.json) and
[validation](../assets/normal-hermite-depth-refinement-v1/validation.json).
CI includes the new analytic suite. Core regression passes **371 Rust tests
and 49 scenarios with complete sensor-log replays**, retaining **245 core-output
and 16 native-source digests**. Vehicle integration, continuous real-trajectory
accuracy and timing remain unverified. Subjective maturity stays **about 20%**
against mature driving OSS; **30% and 50% remain unmet**.
