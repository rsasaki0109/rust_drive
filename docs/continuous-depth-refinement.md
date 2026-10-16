# Continuous inverse-depth local refinement

This optional NumPy experiment replaces nearest-pixel residual evaluation
inside a saved local fit with a continuous target-depth surface. It freezes
the initial **source observation IDs and metric normalizers** for the whole
attempt, then uses bilinear inverse-depth interpolation as poses move. It
does not replace the tracker or its original reciprocal factor.

## Recorded outcome

| Five saved windows per recording | Room | Desk2 |
|---|---:|---:|
| Continuous-objective convergence | 0/5 | 0/5 |
| Original discrete fresh convergence | 0/5 | 0/5 |
| Line-search guard stops | 4 | 5 |
| Missing measured-support stops | 1 | 0 |
| Applied corrections | 0 | 1 |
| Invalid candidate trials | 32 | 46 |
| Valid nonpassing candidate costs | 0 | 0 |
| Original discrete RMS at most 0.03 m | 5/5 | 5/5 |

The one applied correction, desk2 source 279, uses **1/64** of its full
proposal. Its next search exhausts all eight candidates. Nine windows apply
no correction; room source 200 cannot admit a new measured edge after initial
four-corner qualification. No window consumes its ten-update allowance or
converges. Thus continuous interpolation with this strict frozen observation
domain regresses local availability relative to the ten-correction alternating
control, which had no guard failures.

After this failed trial, a separately identified
[post-hoc domain replay](../assets/continuous-depth-refinement-v1/domain-diagnostic.py)
checks all **79** recorded candidate poses without fitting. Operational and
independent selectors agree on the exact surviving source IDs and separately
count interpolation-domain versus geometry/alignment losses. All **78**
invalid candidates lose fixed source support; the sole valid candidate is
the applied 1/64 step.

At the final 1/128 candidate of each exhausted search, only **1–4 source
occurrences** are lost, representing **0.0116–0.1258%** of that window's
initial factor sample occurrences. Room source 240 loses one of **8,628**.
Some failures come from interpolation qualification, others from geometry or
normal/gradient alignment. This directly exposes how the strict whole-domain
rule blocks these candidate searches. It neither labels those samples as
physical outliers nor explains the preceding tracker's absolute drift.
The [domain-loss report](../assets/continuous-depth-refinement-v1/domain-loss.json)
records all factors and both loss groups. It is post-hoc evidence, with its
own source digest, rather than part of the pre-fit 60-member source archive.

The experiment independently verifies **21 newly reported systems**: one
applied iteration, ten continuous final systems and ten original-discrete
final systems. **Twenty** archived control systems, at the original starts
and alternating returned poses, are separately reconstructed. These are
checks on reused measurements, not independent observations.

The predeclared follow-up condition fails. The latest full optional reciprocal
trajectory scores remain **88/179 room / 75/179 desk2** accurate; no trajectory
is refitted or newly scored here. The original-discrete RMS medians at the
new returned poses are **2.4838 mm room / 2.7405 mm desk2**, versus control
**2.4649 / 2.7658 mm**. Most returned poses are unchanged starts. Different
correspondence populations and a different continuous objective prevent these
local quantities from establishing physical improvement.

## Fixed objective and domain

The [design](../assets/continuous-depth-refinement-v1/design.json) and **60**
source/context members are frozen before recorded fitting. Use exactly the
same initial poses, original incoming priors and source indices **120, 160,
200, 240, 279** per viewed room/desk2 recording as the preceding
[alternating comparison](alternating-depth-refinement.md). Numeric real
reference labels are never parsed.

Start from the original reciprocal source pixel IDs, then qualify four target
depth corners, geometry and source-normal/target-gradient alignment. Admit
at least 200 qualified samples per raw factor. At the starting poses, store
each retained source point and the positive normalizer
`s_i = norm(gradient F_i)`.

For a source point transformed into target optical coordinates `q`, define
`F_i(q) = rho(u(q), v(q)) - 1/q_z`, where `rho` is bilinear inverse target
axial depth. The residual is `r_i = F_i / s_i`. Its initial normalizer remains
fixed for the **complete attempt**, so the Huber-plus-original-prior objective
is the same function at every valid pose. The residual has engineering metre
units; it is not calibrated covariance or an exact Euclidean surface distance.

Interpolation cells may change. Values are continuous across qualified cell
boundaries; derivatives can have knots. At an exact integer pixel, the declared
right-cell derivative is used. Initial source points and normalizers never
change, and no new posterior prior is accumulated. This differs from the
[earlier subpixel tracker](subpixel-keyframe-depth.md), which reselects support
and recomputes its metric normalization at each linearization.

Every initially retained source must remain inside the qualified interpolation
and geometry domain at a candidate pose. An invalid source rejects the entire
candidate: it cannot silently disappear, reduce a factor count or become a
zero loss. The initial source-normal and depth qualification are geometric
engineering checks, not guarantees of physical visibility or correct matches.

The exact inverse-depth projective derivative supplies opposite source/target
blocks under root-frame left pose increments. The first optical origin stays
fixed. Existing nonlinear prior transport, unique raw-factor ownership,
scaled positive-definite solve and **1e6** condition limit are preserved,
without damping or rank fill. Full proposals retain **0.10 m / 0.15 rad**
per-node bounds.

Try at most eight Armijo multipliers **1 through 1/128**. Record a domain
failure with no candidate cost, then try the next multiplier. No valid passing
candidate is a line-search guard failure. Apply at most ten corrections.
Continuous convergence uses every node's **full unscaled** proposal below
**10 micrometres / 10 microradians**; shortening a multiplier cannot create
convergence.

Return a separate freshly selected **original-discrete** system and proposal.
Convergence of the new objective is not convergence of that original system.
The predeclared follow-up rule requires both cases to converge every continuous
window, improve original-discrete fresh-converged counts, have no local/fresh
guards, pass the original 0.03 m RMS gates and have original-discrete median RMS
no worse than control. The new objective's costs/RMS are not compared directly
with costs/RMS from another residual definition.

## Independent verification and limits

The private checker reconstructs SVD normals, relative-camera reciprocal source
IDs, polynomial interpolation, frozen normalizers, incidence-block matrices,
quaternion prior coordinates, spectral solves, Rodrigues updates and complete
candidate searches. Normalizer hashes bind the recorded values, while a
separate numerical reconstruction prevents a self-consistent altered weight
from passing. Twelve finite-difference objective axes include a nonzero
transported prior and both moved target/source nodes.

Rendered measured-data warm initialization converges after **one** correction,
with position/rotation errors approximately **0.0195 mm / 0.0000741 rad**,
inside 4 mm / 4 mrad bounds. The original discrete fresh proposal is still
unconverged. Adding a declared **2 cm and rotation perturbation** to that same
warm start instead ends in line-search failure after **two** corrections.
This failure is retained; the easier warm-start success is not general recovery
evidence. Truth generates and scores images, and never initializes the fitter.

Identity applies no correction. Missing data and a plane stop through existing
support/observability guards. Thirteen reported rendered systems and one
additional prior system are independently checked. Inherited scalar samples,
three gradient axes, pixel-knot continuity and corner rejection controls pass.
An explicitly injected direction/gradient exercises eight exhausted search
trials, including three domain failures; it is not an image-derived proposal.
Twelve corrupted reports are rejected, including an invalid trial presented
as zero cost and a coherently changed normalizer.

The mandatory core check passes **371 Rust tests and 49 scenarios with full
sensor-log replays**. All 245 deterministic core outputs, 402 preceding
evidence entries and 16 pinned native sources retain their hashes. CI includes
the new analytic and syntax controls.

## Reproduction and evidence

Use the pinned optional NumPy/Pillow environment and original hash-pinned TUM
inputs. Output directories must be fresh:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-continuous-depth-refinement.py \
  --output artifacts/continuous-refinement-controls-replay
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-continuous-depth-refinement.py \
  --output artifacts/continuous-refinement-recorded-replay
```

The separately sourced post-hoc diagnostic replays the frozen packet:

```bash
OPENBLAS_NUM_THREADS=1 python assets/continuous-depth-refinement-v1/domain-diagnostic.py \
  --root . --summary assets/continuous-depth-refinement-v1/summary.json \
  --output artifacts/continuous-domain-loss-replay.json
```

The [report](../assets/continuous-depth-refinement-v1/summary.json),
[validation](../assets/continuous-depth-refinement-v1/validation.json) and
[manifest](../assets/continuous-depth-refinement-v1/files.json) retain every
pose, system, frozen normalizer and attempted search. Commands return zero for
integrity success even when all convergence/availability outcomes fail.
Original images/reference files are not redistributed;
[TUM attribution and terms](tum-source-provenance.md).

The tool commits no tracker cache, clock, key, prior or Schur state and supplies
no vehicle controls. It establishes neither a new physical environment nor
uncertainty calibration or real-time behavior. The Rust default and subjective
**about-20%** maturity estimate remain unchanged; 30% and 50% remain unmet.
