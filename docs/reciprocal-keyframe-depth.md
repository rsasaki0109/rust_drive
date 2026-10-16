# Reciprocal projective pixel association

This optional NumPy experiment adds a return projection to the
[selective-level depth tracker](selective-keyframe-depth.md). A forward match
is retained only if its target vertex projects back to the original sampled
source pixel. The symmetric measured-normal residual and derivative, Huber
weights, PCA normals, selective schedule, prior and all admission/commit
guards are unchanged. The Rust driving pipeline and default estimator remain
unchanged.

## Recorded outcome

| Method | Room accepted / accurate / 179 | Desk2 accepted / accurate / 179 | Terminal loss |
|---|---:|---:|---|
| Original Rust fused depth, preceding evidence | 179 / 64 | 179 / 161 | Neither |
| Selective-level control | 179 / 77 | 179 / 76 | Neither |
| Reciprocal projected pixels | 176 / 88 | 179 / 75 | Neither |

The outcome is a tradeoff. Room gains eleven accurate updates despite three
condition rejections at source indices 252–254. Desk2 loses one accurate
update. Room's maximum position/rotation errors change from
**0.293017 m / 0.186621 rad** to **0.280169 m / 0.197894 rad**; desk2 changes
from **0.187061 m / 0.205988 rad** to **0.211982 m / 0.233135 rad**.
Both complete physical-accuracy protocols fail. There are no resets or expired
acquisitions, and the three rejected room updates remain in its denominator.

The [matched comparison](../assets/reciprocal-keyframe-depth-v1/comparison-with-parent.json)
retains full summaries and statistics over timestamps accepted by both methods.
The room gain remains over the 176 common accepted timestamps: 77 control
updates versus 88 reciprocal updates satisfy the accuracy gates. Rejected
updates still count against the full 179-update protocol.
The [initialization receipt](../assets/reciprocal-keyframe-depth-v1/initialization-parity.json)
confirms identical initialization and records the first changed acquisition.
All input inventories, manifests and camera interpretation match the selective
control. The Rust default and subjective maturity estimate of **about 20%**
remain unchanged.

All accepted fits reach the finest-level five-iteration cap. The
[convergence record](../assets/reciprocal-keyframe-depth-v1/convergence.json)
reports median maximum-node proposed translation steps at returned poses of
about 0.304 mm in room and 0.508 mm in desk2; maxima are 21.85 mm and 6.75 mm.
These local quantities do not establish convergence or physical accuracy.

## Diagnostic and fixed association rule

The [diagnostic](../assets/reciprocal-keyframe-depth-v1/association-diagnostic.json)
first exercises the preceding selective tracker on a rendered plane and three
spheres at 160×120 with independent Gaussian axial-depth noise of sigma 2 mm,
seed 891021. Both stationary and moving fifteen-frame profiles over 0.42 s
accept all 14 updates. Maximum position/rotation errors are **0.868 mm / 0.000774 rad** and
**1.389 mm / 0.001039 rad**, respectively. Truth generates the test images and
scores the result; it never supplies operational poses. This short fixed
noise/geometry fixture is not a sensor-noise calibration or a causal
explanation of drift in real recordings.

The real-image diagnostic fixes source indices **120, 160, 200, 240, 279** in
each preceding recording. It verifies frozen ledgers and source-image hashes,
reconstructs SVD normals and every original finest-level returned-pose system
at those ten saved poses, including prior transport. It then evaluates the
reciprocal filter at the same poses, without fitting or numeric real reference
labels.

| Candidate occurrences before pair admission, five acquisitions each | Room | Desk2 |
|---|---:|---:|
| Original forward candidates | 67,179 | 80,044 |
| Return-consistent candidates | 56,664 | 65,207 |
| Removed | 15.65% | 18.54% |
| Same-pose finest iteration guards satisfied | 5/5 | 5/5 |

Counts include all geometric candidates across overlapping raw pairs before
the 200-sample pair minimum. They are not independent observations or known
outlier labels. Changed populations can lower RMS without improving physical
accuracy. One admissible linearization does not guarantee successful refitting.
Both recordings are already viewed development data.

The [design](../assets/reciprocal-keyframe-depth-v1/design.json) fixes this rule
before new recorded fitting. Original source strides remain 1/2/4 at levels
4/2/1. First apply every original source-normal, target-normal, forward depth,
image-bound, 3D-distance and normal-dot gate. Transform each selected target
vertex through the target optical pose into the source optical frame. Reject
return axial Z at or below 0.3 m before division. Project using the source
layer's intrinsics and round each coordinate to nearest integer, ties to even.
Require exact equality to both original sampled source coordinates; there is
no tolerance radius or target-grid substitution.

The comparison adds no samples and does not smooth or fill depth. A target
pixel can return to at most one sampled source pixel. Reciprocal projected
pixels do not prove physical visibility or true correspondence. Finer-level
source sampling remains asymmetric; this is not bidirectional factor summation.
The residual Jacobian differentiates fixed selected pairs, not discrete pixel
assignment or the return predicate.

Each admitted pair still needs at least 200 geometric samples. Its Huber
cutoff is 0.01 m, and its count/RMS cover all retained reciprocal candidates.
Selective coarse guards, strict finest support/condition/increment admission,
returned-pose raw RMS at most 0.03 m, displacement at most 0.50 m and the
0.20 s accepted-age expiry remain unchanged. Five nodes, 24 pose variables,
18 prior coordinates and at most 22 trace events remain the structural limits.

## Independent verification

The operational matcher carries sampled `(u,v)` coordinates through its
forward filters and transforms matched root vertices back with the source
rigid pose. The separate oracle carries flattened pixel IDs and forms a
relative homogeneous transform, its inverse and the two intrinsic matrices.
It rebuilds the reciprocal mask and symmetric differential with matrix
products; it calls neither the operational mask nor residual routine.
Independent SVD normal reconstruction and the existing state/Schur audit
remain active.

Three rendered fixed-pose layers have forward/reciprocal counts
**731/729**, **980/976** and **1078/1078** at levels 4/2/1. Both implementations
agree on counts, H/g and RMS. With unstrided coarse pixels, swapping endpoints
preserves count, squared residual and H, and reverses g. Exact identity
preserves the original support and Huber system at all three levels.
Controlled injected layers exercise an out-of-image return, a 64-to-one
collision reduced to one reciprocal source, and return depth below 0.3 m.
These injected layers test selection rather than claiming physical images.

Both inherited fifteen-frame analytic branches pass 4 mm / 4 mrad bounds:
**452** iteration/returned-pose systems and nine retained marginal generations
are checked. Maximum position errors are about 0.059 mm and 0.058 mm. All nine
inherited report corruptions are rejected; planar degeneracy, absent fresh
support, terminal expiry and late-elimination atomicity checks remain active.
The source checker proves that the engine differs from the frozen selective
control only by its pair import.

Recorded estimation completes for both cases before either independent audit;
each audit uses a private module and callbacks. Both must finish before
numeric physical-reference scoring. All 179 updates remain in each denominator,
with identity initialization outside it, no resets, physical gates of 0.10 m /
0.10 rad and reference interpolation brackets at most 20 ms. RGB and numeric
reference poses do not enter the estimator.

The recorded audit verifies **8,103** reported systems: **7,427** applied
iteration systems, **47** skipped coarse systems, **355** returned-pose systems
and **274** elimination-input systems, plus all 274 Schur-result priors. Room
consumes 451 unique raw factors across 135 marginal generations and skips
45 coarse condition failures. Desk2 consumes 503 across 139 and skips two
coarse increment failures. All three room fit rejections are independently
reproduced; neither tracker expires.

The core check passes **371** Rust tests and **49** scenario executions with
full sensor-log replays. All **245** deterministic outputs and **301** previous
evidence files retain their hashes. CI adds the new optional association
analytic and syntax checks.

## Reproduction and limits

Use the pinned optional environment in `scripts/requirements-visual.txt` and
the existing hash-pinned TUM inputs:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-associations.py \
  --output artifacts/depth-associations-replay
OPENBLAS_NUM_THREADS=1 python scripts/check-reciprocal-keyframes.py \
  --output artifacts/reciprocal-keyframes-analytic
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-reciprocal-keyframes.py \
  --output artifacts/reciprocal-keyframes-recorded
# Expected exit 1: complete physical accuracy remains unmet.
```

Output directories must be fresh. The packet retains source/context hashes,
sensor-only ledgers and compact physical scores. Merging compact fields into
matching ledger rows and serializing `{summary, frames}` with two-space JSON
indentation and a final newline reconstructs the original scored reports
byte for byte. Original images and reference files are not redistributed.
[TUM attribution and license](tum-source-provenance.md).

The [packet manifest](../assets/reciprocal-keyframe-depth-v1/files.json) records
file sizes and SHA-256 digests. The
[validation receipt](../assets/reciprocal-keyframe-depth-v1/validation.json)
distinguishes mathematical/state integrity from physical accuracy, and the
[source comparison](../assets/reciprocal-keyframe-depth-v1/previous-rust-source-parity.json)
checks the 16 preceding pinned sources including the Rust estimator.

Reciprocity can remove useful measurements and change observability. It does
not correct depth/intrinsic bias or frozen-prior approximations, establish
calibrated uncertainty, or validate new environments, long-duration mapping,
real-time performance or vehicle integration.
