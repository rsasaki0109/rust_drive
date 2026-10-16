# Initial margin for continuous depth refinement

This optional NumPy comparison qualifies the initial source observations at
a nine-point image stencil before calling the unchanged
[continuous local solver](continuous-depth-refinement.md). It asks whether
avoiding initially fragile image/qualification boundaries reduces frozen-domain
search failures. It does not relax the candidate-domain or convergence rules.

## Recorded outcome

| Five saved windows per recording | Room | Desk2 |
|---|---:|---:|
| Margin / preceding continuous convergence | 0/5 / 0/5 | 0/5 / 0/5 |
| Original-discrete fresh convergence | 0/5 | 0/5 |
| Margin / control line-search guard stops | 1 / 4 | 2 / 5 |
| Margin / control missing-support stops | 4 / 1 | 3 / 0 |
| Applied tentative corrections, margin / control | 2 / 0 | 3 / 1 |
| Admitted initial source occurrences, margin / control | 787 / 23,515 | 5,732 / 36,971 |
| Admitted initial factor occurrences, margin / control | 3 / 23 | 7 / 28 |
| Invalid new candidate trials | 19 | 25 |
| New budget exhaustion | 0 | 0 |

The stencil and 200-sample factor admission together retain only **6,519 of
60,486 source occurrences (10.78%)**. That includes discarding whole factors
with fewer than 200 survivors; it is not solely a per-sample stencil rejection
rate. Counts overlap across factors and saved windows, so they are not new
independent measurements or physical outlier counts.

Seven windows have no admitted raw factor after the new initial selection,
and cannot attempt an update. Room source 240 applies two corrections and
desk2 source 160 applies three, before their searches fail. Desk2 source 200
retains six factors and 5,503 sources but fails its first search. Every window
still terminates through a guard. Fewer line-search failures come with more
missing-support failures; the comparison does not repair local convergence
or availability. Five tentative updates are not accepted tracker updates.

Original-discrete fresh guard failures remain zero, and all final original
RMS values pass 0.03 m. Their medians are unchanged from the continuous control
at **2.4838 mm room / 2.7405 mm desk2**. These local quantities do not measure
absolute pose accuracy. The predeclared follow-up condition fails; no full
trajectory or tracker integration follows. Latest full optional reciprocal
trajectory scores remain **88/179 room / 75/179 desk2** accurate, with complete
physical protocols unmet.

The independent recorded audit checks **25 new reported systems**: five
iteration systems, ten continuous finals and ten original-discrete fresh
finals. All **21** preceding continuous control systems are independently
rechecked with their original support. This is validation work on reused
observations, not 46 independent physical measurements.

## Predeclared selection and scope

The [design](../assets/margin-depth-refinement-v1/design.json) and **64**
source/context members are frozen before recorded fitting. Use the same
initial poses, original incoming priors, node timestamps and saved source
indices **120, 160, 200, 240, 279** per room and desk2 recording as the preceding
continuous experiment. These are already viewed development windows. Numeric
real reference labels never enter the trial.

For each source initially retained by the preceding continuous selector,
transform its measured point into target optical coordinates. Hold optical
axial Z fixed, and probe image displacements `(du,dv)` from `{-1,0,1}²` in
the current pyramid layer's pixels. Require all nine queries to meet the same
four-corner depth/normal bounds, depth continuity, finite positive gradient,
3D distance and source-normal/gradient alignment checks. Retain only sources
passing all probes, then require at least **200** surviving observations
per admitted raw factor.

This finite stencil is an engineering heuristic. It does not certify the
entire region between probes, any change in axial depth or rotation, or
physical visibility. Every original candidate-domain guard still runs during
optimization. The probe points are qualification queries, not extra sensor
measurements and not additional normal-system factors.

Each surviving source carries its **original center normalizer**. Probe
gradients do not become new weights. Freeze those observations and normalizers
for the complete attempt. The inverse-depth residual, exact derivative, Huber
objective, nonlinear prior chart, unique factor ownership, SPD condition limit
**1e6**, full **0.10 m / 0.15 rad** proposal limits and eight Armijo multipliers
**1 through 1/128** are unchanged. There is no damping or rank filling.

At most ten corrections may be applied. Every initially selected source must
remain qualified at every candidate pose; it cannot silently disappear to
lower cost. Continuous convergence requires each non-origin node's **full**
proposal below **10 micrometres / 10 microradians**. Original-discrete fresh
convergence is reconstructed separately and is not interchangeable with the
continuous objective's convergence.

The predeclared follow-up eligibility rule requires both cases to converge
all continuous windows, improve original-discrete fresh-converged counts, have
no local or original-discrete guards, pass the original 0.03 m RMS gate at all
windows and have original-discrete median RMS no worse than control. This
local experiment does not evaluate a full trajectory. Different initial
source populations prevent direct inference from continuous cost/RMS changes;
original-discrete population changes also prevent RMS alone from proving
better physical poses. There is no runtime comparison.

## Independent verification

A source comparison proves that the solver's entire `refine` function body
is identical to its frozen predecessor. Objective, equation, residual,
metadata and search helpers are the same function objects. Only initial
population selection changes.

The independent selector uses relative-camera homogeneous image projection,
inverse-intrinsics rays, polynomial inverse-depth interpolation and array
intersections over the stencil. SVD normals are reconstructed independently.
A private unchanged continuous verifier receives only this initial-selection
replacement; operational module globals and archived source files are not
modified. It checks every source identity, center normalizer, prior/system,
spectral solve, pose increment, candidate search and stop decision. The
complete preceding continuous control reports are separately rechecked with
their original selector.

A hand-counted fixture has three center-qualified samples: one clear interior
sample, one at the image boundary and one next to a missing depth corner. The
nine-probe selector independently retains exactly **one**. Real rendered warm
initialization reduces source occurrences from **1,050 to 988**, proving that
the new policy changes measured support rather than just report metadata.
Substitution of the old center-only population is rejected.

The inherited controls under the new selection check **15** reported rendered
systems and one extra nonzero-prior system, with all **12** objective-gradient
axes and **12** corrupted reports. Another three reported-system checks repeat
the rendered warm case for selection/subset verification. The warm start
converges in **one correction**, with errors approximately **0.0198 mm /
0.0000275 rad**, within 4 mm / 4 mrad. Its original-discrete fresh system is
unconverged. The declared 2 cm/rotation perturbation instead fails its search
after **four corrections**; it is not successful initialization recovery.

Identity applies no correction. Missing data and a plane preserve original
support/observability failures. Corner, knot and scalar interpolation controls
also pass. An explicitly injected direction/gradient isolates exhausted-search
receipts, with two invalid candidates; that fixture is not an image-derived
normal-system proposal.

The mandatory core check passes **371 Rust tests and 49 scenarios with complete
sensor-log replays**. All 245 deterministic core outputs, 415 preceding
evidence entries and 16 pinned native sources retain their hashes. CI includes
the new analytic and syntax controls.

## Reproduction and evidence

Use the pinned optional NumPy/Pillow environment and original hash-pinned TUM
inputs. Output directories must be fresh:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-margin-depth-refinement.py \
  --output artifacts/margin-refinement-controls-replay
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-margin-depth-refinement.py \
  --output artifacts/margin-refinement-recorded-replay
```

The [report](../assets/margin-depth-refinement-v1/summary.json),
[validation](../assets/margin-depth-refinement-v1/validation.json) and
[manifest](../assets/margin-depth-refinement-v1/files.json) retain complete
new/control poses, source identities, normalizers, equations and searches.
Commands return zero for verified implementation integrity, including failed
local quality outcomes. Original images and physical reference files are
not redistributed; [TUM attribution and terms](tum-source-provenance.md).

No tracker clock, key, cache, prior or Schur state is committed. The original
Rust default and driving pipeline stay unchanged. No new physical domain,
trajectory accuracy, calibrated uncertainty or real-time behavior is claimed.
Subjective maturity remains **about 20%** compared with mature driving OSS;
30% and 50% remain unmet.
