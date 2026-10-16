# Separating initial matching from the sampling domain

This optional NumPy experiment distinguishes the geometry used to select
initial matches from the data support needed to evaluate their fixed
inverse-depth objective. It jointly changes the initial margin stencil and
the candidate-domain policy. It does not establish physically correct matches,
tracker admission or continuous trajectory accuracy.

## Recorded outcome

| Five windows per recording | Room | Desk2 |
|---|---:|---:|
| New / margin / center-only continuous convergence | 0 / 0 / 0 | 0 / 0 / 0 |
| New / margin / center-only local guard stops | 2 / 5 / 5 | 3 / 5 / 5 |
| New missing-current-support stops | 1 | 3 |
| New line-search stops | 1 | 0 |
| New ten-correction budget stops | 3 | 2 |
| New / margin / center-only tentative corrections | 34 / 2 / 0 | 20 / 3 / 1 |
| New admitted initial source occurrences | 10,156 | 22,351 |
| Margin admitted initial source occurrences | 787 | 5,732 |
| Center-only admitted initial source occurrences | 23,515 | 36,971 |
| New original-discrete fresh convergence | 0/5 | 0/5 |

The new selection retains **32,507** admitted source occurrences, versus
**6,519** with the preceding all-qualification margin and **60,486** with
center-only selection: **53.74%** of center-only occurrences remain. It admits
36 factor occurrences, versus 10 and 51 respectively. Stencil selection and
whole-factor minimum-200 admission both affect these counts. Measurements
overlap across factors/windows and are not independent physical observations.

Five windows reach the ten-correction allowance without a local guard;
none reaches continuous convergence. Room source 279 applies four corrections
before line-search exhaustion. Room source 200 and desk2 sources 120/240/279
lack an admitted edge from the newest acquisition. Desk2 source 120 still has
1,244 retained older-view source occurrences; old support cannot substitute
for a measured edge from the current frame.

The total **54** tentative corrections include **23** shortened steps. The
auditor reproduces **22** invalid candidate trials, **42** valid candidates
that fail Armijo, and **54** passing candidates. Those counts describe search
work, not speed or accepted tracker updates. Local guard stops fall from
10/10 in either predecessor to 5/10 here, but convergence remains **0/10** and
the progression rule fails. Optimizing more windows does not establish their
physical accuracy.

All ten returned original-discrete systems pass their 0.03 m RMS gate and
have no fresh guard, while all remain unconverged. Median original-discrete
RMS is **2.4539 mm room / 2.7405 mm desk2**, compared with margin control
**2.4838 / 2.7405 mm**. This is not a new absolute pose score. No trajectory is
refitted or physically scored; latest full optional reciprocal results stay
**88/179 room / 75/179 desk2** accurate, with complete physical protocols unmet.

The independent recorded checker verifies **74 new reported systems**:
54 iteration systems, ten continuous finals and ten original-discrete fresh
finals. It separately rechecks **25 margin-control** and **21 center-only
control** systems. These 120 checks reuse measured data and do not count as
120 independent observations.

## Fixed protocol

The [design](../assets/sampling-depth-refinement-v1/design.json) and **68**
source/context members are frozen before recorded fitting. Use the same
initial poses, original incoming priors, timestamps and source indices
**120, 160, 200, 240, 279** per viewed room and desk2 recording as the
[initial-margin comparison](margin-depth-refinement.md) and its
[center-only continuous predecessor](continuous-depth-refinement.md).
Numeric real reference labels are never parsed.

Initial matching retains the original reciprocal source IDs and continuous
center qualifier, including **0.08 m** source/target distance and **0.8**
source-normal/gradient alignment gates at scale 1. Source center normalizers
are unchanged. Before freezing observations, require sampling support at nine
image displacements `(du,dv)` in `{-1,0,1}²`, holding target optical axial Z
fixed. At those displaced probes, check the sampling support rather than
repeating source-normal/distance agreement. A factor needs at least **200**
surviving sources.

Sampling support still requires positive optical Z, the original image bounds,
four depth corners in **0.3–5 m**, their depth continuity, valid corner normals
and a finite positive inverse-depth gradient norm. Depth is neither filled
nor extrapolated. The target corner normals remain part of data qualification;
this is not purely an algebraic domain. Nine finite probes do not certify
intermediate coordinates or later depth/rotation changes.

Freeze all admitted source IDs and their original center metric normalizers.
During optimization, every source must retain sampling support. A missing or
out-of-image sample rejects the entire candidate without a cost; it cannot
silently disappear to improve the objective. Source-normal/distance agreement
is an initial matching criterion in this variant and is not repeated inside
the fixed objective. Thus a defined residual can cease to satisfy the older
geometric matching gates. That is an explicit policy change, not evidence
that such a correspondence is physically correct or safe for a live tracker.

The residual remains bilinear target inverse axial depth minus inverse query
Z, divided by the frozen initial gradient norm. This has engineering metre
units, without calibrated noise/covariance meaning. The complete nonlinear
Huber-plus-original-prior objective, exact projective derivative, ownership,
scaled SPD solve and **1e6** condition limit remain unchanged. There is no
damping or rank fill. Full per-node proposals remain bounded at **0.10 m /
0.15 rad**. At most eight Armijo multipliers **1 through 1/128** are tried,
and at most **ten** corrections may be applied.

Continuous convergence still uses each node's full unscaled proposal below
**10 micrometres / 10 microradians**. Shortening a multiplier cannot manufacture
convergence. The final original-discrete fresh system and correction are
reconstructed separately. Convergence of this fixed objective is not convergence
of that original system.

Both archived predecessors are rechecked from raw measurements, without
fitting new control poses. They share initial poses and priors but have
different selected populations and candidate domains. The joint change cannot
attribute an effect uniquely to either the stencil or the repeated geometric
gates. Continuous costs/RMS are not directly comparable across populations,
and original-discrete population changes also prevent RMS from establishing
absolute physical pose improvement. This is not a timing benchmark.

The predeclared follow-up rule requires both cases to converge every continuous
window, improve original-discrete fresh-converged counts, have no local/fresh
guards, pass all original 0.03 m RMS gates and have original-discrete median RMS
no worse than the preceding margin control. This local comparison performs no
new full-trajectory evaluation or default integration.

## Independent verification

Source comparisons prove that the bodies of `evaluate`, `equation`, `energy`,
`search` and `refine` equal the original continuous implementation. Only their
initial-population and candidate-value policies change. Metadata and the
original solve/prior dependencies remain shared and unchanged.

Two separate private verifier instances preserve the old initial geometric
selector while replacing candidate sampling and stencil selection in the
new auditor. Relative-camera homogeneous projection, inverse-intrinsics rays,
polynomial interpolation and array intersections independently reconstruct
the sampling policy; SVD reconstructs measured normals. Every source ID,
frozen normalizer, prior/matrix, spectral solve, Rodrigues update, candidate
cost and termination is checked. Each control is rechecked with its original
source selection and domain policy. Frozen operational source files and module
globals are not modified by the new verifier.

Hand-counted sampling retains one of three center-supported probes, rejecting
an image boundary and an adjacent missing depth corner. Separate manual
fixtures distinguish a defined interpolated residual from a failed source-normal
agreement gate or a **0.09 m** distance disagreement. Those fixtures test the
policy boundary; they are not evidence of physically valid correspondences
or accepted image-derived motion.

Inherited controls under this new policy check **15 reported rendered systems**
and one extra nonzero-prior system, with all **12 objective-gradient axes**
and **12 corrupted reports**. Another three system checks repeat the rendered
warm case to verify stencil/source selection. Substitution of the center-only
population is also rejected. Rendered warm initialization retains **988 of
1,050** center-qualified source occurrences, converges after one correction
and remains within 4 mm / 4 mrad. Its original-discrete fresh system stays
unconverged. The declared 2 cm/rotation perturbation instead fails its search
after four corrections; robust initialization recovery remains unverified.

Identity applies no correction; missing and planar measurements preserve support
and observability failures. Scalar/corner/pixel-knot interpolation controls pass.
An injected direction/gradient isolates exhausted-search receipts; its two
invalid candidates are not an image-derived normal-system proposal.

The mandatory core check passes **371 Rust tests and 49 scenarios with complete
sensor-log replays**. All 245 deterministic core outputs, 426 preceding
evidence entries and 16 pinned native sources retain their hashes. CI includes
the new analytic and syntax controls.

## Reproduction and evidence

Use the pinned optional NumPy/Pillow environment and original hash-pinned TUM
inputs. Output directories must be fresh:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-sampling-depth-refinement.py \
  --output artifacts/sampling-refinement-controls-replay
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-sampling-depth-refinement.py \
  --output artifacts/sampling-refinement-recorded-replay
```

The [report](../assets/sampling-depth-refinement-v1/summary.json),
[validation](../assets/sampling-depth-refinement-v1/validation.json) and
[manifest](../assets/sampling-depth-refinement-v1/files.json) retain the complete
new/control poses, matrices, selected sources, weights and search receipts.
Commands return zero for integrity success, including failed convergence.
Original images and physical reference files are not redistributed;
[TUM attribution and terms](tum-source-provenance.md).

The tool commits no tracker clock, cache, key, prior or Schur state and supplies
no vehicle controls. It establishes no new physical domain, uncertainty
calibration or real-time behavior. The Rust default and subjective
**about-20%** maturity estimate remain unchanged; 30% and 50% remain unmet.
