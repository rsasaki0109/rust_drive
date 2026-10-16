# Local measured normal support for Hermite sampling

The new optional normal-support field passes its independent checks but leaves
local convergence at **4/10** and original-discrete fresh convergence at **0/10**.
It keeps **all 32,507 initial source occurrences**. Nine complete reports equal
the [original normal-Hermite control](normal-hermite-depth-refinement.md).
Only room source 279 changes: it proceeds from 8 to 15 corrections, then reaches
another sampling-domain boundary. Its remaining full proposal shrinks, while
its original-discrete fresh RMS regresses. The frozen progression rule fails.

## Model and qualification

The [design](../assets/local-normal-depth-refinement-v1/design.json) and
**105 source/context members** are archived before the ten new local fits.
This is an outcome-informed development experiment using previously viewed
room/desk2 windows at source indices **120/160/200/240/279**, not held-out
validation or a new physical trajectory evaluation.

Keep every valid original **5×5 measured normal**. At an absent interior normal,
try one **3×3 measured plane**, with all of the following conditions:

- All nine axial depths are strictly between **0.3 and 5 m**.
- Every sample differs from the center by strictly less than
  `0.05+0.01*center_depth` metres.
- The middle population-covariance eigenvalue exceeds **1e−8 m²**.
- The smallest eigenvalue is at most **0.1 times the middle eigenvalue**.

Orient the resulting normal toward positive dot product with the optical
vertex. Preserve the original two-pixel border. These are the existing normal
frontend's metric rank and plane-ratio thresholds, with a fully observed smaller
patch. Missing depths, nonplanarity and rank failures still reject; there is no
depth filling, smoothing or threshold retry.

Precompute a separate `hermite_normals` array per frame and pyramid level.
The original depth, vertices, normals and intrinsics remain unchanged. Only
target interpolation reads the additional field. Original geometric matching,
source normals, bilinear admission probes and metric normalizers retain their
original inputs. A missing prepared field rejects explicitly.

The same four-corner bicubic Hermite sampler, shared vertex slopes, zero mixed
derivatives and exact projective derivatives operate on the new field. It
remains C1 on jointly supported cells. Existing valid-normal corners retain
their slopes; the domain can expand where a missing normal gets a valid local
estimate. Smaller patches may be less robust to noise; neither the planarity
ratio nor C1 continuity certifies surface correctness.

The initial population and incoming prior remain fixed for the entire attempt.
All ten recorded source metadata and normalizer digests equal the control,
and initial costs are exactly equal. No new point observations, source weights
or raw factors come from normal completion. AST proofs preserve the complete
solver/objective/checkpoint bodies, **21** Armijo multipliers through `2^-20`,
original numerical slack, **30** correction cap, current support, conditioning,
full-proposal bounds and strict **1e−5 m / 1e−5 rad** convergence. No damping or
tracker/Schur commit occurs.

## Why the old searches stopped

The recorded evaluator separately replays all **42 archived candidates** at
the two original search stops. Separate projection/interpolation constructions
agree on lost source identities and support after completion. For each missing
corner, the report archives measured 3×3/5×5 depths and center-gated covariance
eigenvalues, checked by centered SVD.

| Old terminal window | Repeatedly blocking source / target corner | 5×5 plane ratio | 3×3 plane ratio | Old candidates supported after completion |
|---|---|---:|---:|---:|
| Room / 279 | Source 56388, target 2 / source 3, target pixel (59,78) | 0.243241 | 0 | 20/21 |
| Desk2 / 160 | Source 289452, target 3 / source 4, target pixel (168,464) | 0.112930 | 0.311565 | 0/21 |

These corners have measured center and neighborhood depths; the old normal
failure comes from the plane-ratio gate. The room 3×3 patch is flat, whereas
desk2's smaller patch still fails the unchanged 0.1 threshold. The full room
candidate has other lost support, so even the completed field supports only 20
of its 21 archived candidates. Candidate availability is not an applied update,
Armijo acceptance or proof of convergence. Target/source indices match the
saved factor metadata.

## Recorded outcome

| Ten windows | Original normal Hermite | Local normal completion |
|---|---:|---:|
| Own continuous-objective convergence | 4 | 4 |
| Original-discrete fresh convergence | 0 | 0 |
| Initial sources / factors | 32,507 / 36 | 32,507 / 36 |
| Missing-current-support / search / budget stops | 4 / 2 / 0 | 4 / 2 / 0 |
| Applied corrections | 79 | 86 |
| Shortened steps | 17 | 24 |
| Invalid / accepted / valid nonpassing candidates | 198 / 79 / 0 | 276 / 86 / 0 |
| Accepted cost increases | 0 | 0 |

Room 120/160/240 still converge after 11/7/9 corrections; desk2 / 200 still converges
after 20. Desk2 / 160 still stops after 24. All remaining initial support stops
remain. These **nine entire reports are exactly equal**, including traces,
poses, equations, costs and decisions. Room / 279 retains only the first **two**
old trace entries exactly, then follows the newly available sampling domain.

| Room / 279 | Control | New |
|---|---:|---:|
| Applied corrections | 8 | 15 |
| Remaining maximum full translation norm (mm) | 1.323947 | 0.589426 |
| Remaining maximum full rotation norm (rad) | 0.00115514 | 0.000486595 |
| Final fixed-objective cost | 0.00346083141005 | 0.00344139484817 |
| Original-discrete fresh RMS (mm) | 3.049857 | 3.091060 |

The remaining full proposal is still far above convergence thresholds. Both
new terminal searches have 21 invalid candidates. At caps 10/20/30, convergence
stays **2/4/4**. Room / 279's cap 10 state is a budget stop; the search failure
after 15 cannot be reported retroactively at cap 10.

A separately labeled [post-run read-only diagnosis](../assets/local-normal-depth-refinement-v1/post-run-support.json)
replays 42 terminal candidates after preserving the first-run summary. At the
smallest new room candidate, the previously repaired source 56388 remains
defined. Source 110504 (target 1 / source 3) now meets target corner (390,136), whose
3×3 plane ratio is **0.173288**, above 0.1; its 5×5 ratio is 0.107325. It has all
nine/25 depth samples, but neither neighborhood qualifies as planar. This
observed new barrier is retained. The diagnostic performs no continuation,
new fit, threshold adjustment or physical outlier labeling. Desk2's original
nonplanar corner remains its barrier.

All ten original-discrete fresh support/conditioning and 0.03 m RMS gates pass,
but fresh proposal convergence fails everywhere. Room/desk2 median fresh RMS
stays **2.473233 / 2.740464 mm**, because the other nine windows are unchanged.
The room / 279 residual regression remains visible despite the unchanged median.
There is no numeric real reference-label parsing, physical pose-error score or
new continuous trajectory fit. The native default is unchanged.

## Independent checks and reproduction

Operational completion uses offset covariance PCA. The oracle uses centered
full-nine-point SVD, preserving the original independent 5×5 normals. All
completed masks and normals are compared separately. The old polynomial
interpolation audit binds the new field privately; the original control audit
keeps its original field. Relative-camera residuals, incidence/prior matrices,
spectral solves, Rodrigues increments, all candidates and checkpoints remain
independently reconstructed.

Five analytic depth fixtures cover **15 pyramid layers**, including a valid
local plane inside a wider nonplanar patch. Further controls reject eight-point
support, missing center, roughness, rank-one geometry, an unprepared field and
a corrupted completion mask. Original depths, vertices, normals and borders
remain unchanged. Inherited interpolation checks and **12 nonzero-prior
objective-gradient axes** pass. Rendered fitting retains 988/988 sources and
converges in one correction; degenerate images retain guards. Four report
corruptions are rejected. Analytic matrix roles are **7 report / 6 checkpoint
systems plus 1 separate prior system**.

Recorded verification checks **106 new-report / 99 control / 60 checkpoint
systems**. These reuse observations and terminal states; they are not
independent new measurements. See the
[summary](../assets/local-normal-depth-refinement-v1/summary.json),
[statistics](../assets/local-normal-depth-refinement-v1/statistics.json) and
[validation](../assets/local-normal-depth-refinement-v1/validation.json).

```bash
source scripts/env.sh
OPENBLAS_NUM_THREADS=1 python scripts/check-local-normal-depth-refinement.py \
  --output artifacts/local-normal-depth-refinement/analytic-repeat
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-local-normal-depth-refinement.py \
  --output artifacts/local-normal-depth-refinement/recorded-repeat
bash scripts/check.sh
```

Use pinned NumPy/Pillow, new output directories and existing hash-qualified raw
PNGs. CI includes the new analytic suite. Core checks pass **371 Rust tests
and 49 scenarios with complete sensor-log replays**, retaining 245 core-output
and 16 native-source digests. No vehicle integration or timing evidence is
added. Subjective maturity remains **about 20%** versus mature driving OSS;
**30% and 50% remain unmet**.
