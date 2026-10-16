# Finest-level depth-fit continuation

The [reciprocal tracker](reciprocal-keyframe-depth.md) reaches its finest-level
five-update cap on every accepted recorded fit. This diagnostic asks whether
ten more local updates suffice to satisfy its existing convergence thresholds.
**None of the ten saved fits converges.** The tracker and its 10/7/5 iteration
caps remain unchanged. This is an optional NumPy diagnostic, with no new
trajectory accuracy result or Rust driving change.

## Fixed local protocol

Use source indices **120, 160, 200, 240, 279** from each of the two already
viewed recordings. These are the same indices as the preceding association
diagnostic. Verify the archived sensor-only ledgers, input freeze and source
image hashes. Reconstruct every saved finest-level final system with independent
SVD normals and the original incoming marginal prior before continuing.

Begin with copies of all accepted window poses at that acquisition, including
the fixed first optical pose. Preserve the incoming prior and re-associate
measured pixels after each correction. Stop when every non-origin node's
proposed translation is strictly below **10 micrometres** and rotation strictly
below **10 microradians**, an original support/condition/increment guard fails,
or **ten additional corrections** have been applied. Recompute and check the
final system without applying an eleventh correction.

The original PCA normals, reciprocal selection, symmetric residual and Huber
weights are unchanged. The diagnostic changes no cache, accepted clock, keys
or prior, performs no elimination and does not fit a new trajectory. It parses
no numeric real reference poses. A failed guard remains distinct from
convergence; passing the scalar 0.03 m final RMS gate alone is not admission.

The initial exploratory execution preceded the certificate checker. The
[design](../assets/depth-convergence-v1/design.json) and complete source archive
freeze the subsequent verified repeat with the same indices and budget.
This is viewed development evidence, not a preregistered held-out experiment.

## Result

| Five saved fits per recording | Room | Desk2 |
|---|---:|---:|
| Additional applied updates | 50 | 50 |
| Converged fits | 0/5 | 0/5 |
| Guard failures | 0 | 0 |
| Final RMS lower than starting RMS | 2/5 | 2/5 |
| RMS non-increasing throughout | 0/5 | 0/5 |
| Largest final node translation from saved poses | 2.913 mm | 2.844 mm |

The final maximum-node proposed translation norms range from **0.042–1.198 mm**
in room and **0.089–1.054 mm** in desk2. All ten samples retain final raw RMS
below 0.03 m, but none reaches the strict joint translation/rotation stop rule.
Support counts change in all ten cases. Raw RMS uses changing populations and
omits the marginal prior's cost, so these RMS comparisons do not establish
descent of a shared objective. Small local corrections do not establish small
absolute pose error.

This result does not prove that more than ten updates could never converge,
or that a full trajectory with a different budget would have the same physical
accuracy. It supplies no justification for increasing the operational cap.
The preceding room **88/179** and desk2 **75/179** accuracy scores remain the
latest reciprocal trajectory results; both complete protocols fail.

## Independent checks and retained evidence

The continuation uses the frozen operational Cholesky solver and direct left
increments. The separate verifier reconstructs raw pixel selection and residuals
with the existing pixel-ID/camera-matrix oracle, rebuilds normals with SVD,
uses spectral solves, and applies its separate Rodrigues formula. Original
depth decimation, vertices, intrinsics and the already validated oracle remain
shared; this does not constitute an independently developed sensor frontend.

All **110** continuation systems are independently checked: 100 applied
iteration systems and ten fresh returned-pose systems. The ten archived starting
systems are also reconstructed before continuation, separately from that count.
The certificate retains all H/g, counts, RMS, proposals, conditions, before/after
poses and incoming priors. The verifier checks per-step metrics, exact pose
chaining, the independent ten-update bound and the reason for termination.

Analytic controls verify measured identity stops before a needless correction
and measured missing depth stops before any correction. Injected algebra then
isolates budget, zero diagonal, poor condition, oversized increment, support
failure after two updates and high final RMS bookkeeping. These injected
systems are not claimed as physical image evidence. **30** analytic metric
systems pass. Six corrupted reports are rejected: extra updates, unexplained
early termination, false convergence, a coarse step, false residual metrics,
and a scaled final system. Input poses and node roots remain unchanged.

The mandatory core check passes **371** Rust tests and **49** scenarios with
complete sensor-log replays. All **245** deterministic outputs, **324** previous
evidence files and the 16 pinned preceding native sources retain their hashes.
CI includes the new analytic controls and Python syntax checks.

Use the existing optional environment and original hash-pinned TUM inputs:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-depth-convergence.py \
  --output artifacts/depth-convergence-controls-replay
OPENBLAS_NUM_THREADS=1 python scripts/diagnose-depth-convergence.py \
  --output artifacts/depth-convergence-recorded-replay
```

Both commands require fresh output directories and return zero when their
integrity checks pass; they do not assert physical trajectory accuracy or that
all continuations converge. The [summary and traces](../assets/depth-convergence-v1/summary.json),
[validation receipt](../assets/depth-convergence-v1/validation.json) and
[SHA-256 manifest](../assets/depth-convergence-v1/files.json) preserve the result.
Original images are not redistributed; [TUM attribution](tum-source-provenance.md).
The subjective maturity estimate against mature driving OSS stays **about 20%**.
