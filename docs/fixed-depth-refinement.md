# Refinement with fixed measured pixel pairs

The optional local solver freezes reciprocal pixel correspondences once,
then repeatedly updates poses while rebuilding the residuals, rotating-normal
Jacobian, Huber weights and transported prior. All **10** saved real-image
windows converge on that fixed population in **1–5** corrections. After
selecting correspondences again, **0/10** satisfy the same convergence rule.

The [predeclared progression rule](../assets/fixed-depth-refinement-v1/design.json)
therefore fails. This solver is retained as an independently checked local
research tool. It is not integrated into the tracker or Rust driving pipeline,
and no new full trajectory or physical accuracy is claimed. The preceding
reciprocal scores remain **88/179 room** and **75/179 desk2** accurate, with
both complete protocols unmet. The subjective maturity estimate stays
**about 20%** against mature driving OSS.

## Same starting poses, two objectives

The preceding [correspondence diagnostic](depth-correspondence.md) found 49/100
saved continuation updates whose reassociated cost exceeded their initial
cost, despite descent on their fixed population. This experiment tests whether
fully refining a frozen population also stabilizes the freshly associated
system.

Use the same source indices **120, 160, 200, 240, 279** in each already viewed
recording. Begin at the saved accepted poses before the preceding continuation,
with exactly the original incoming marginal prior. The control is the
archived ten-update continuation that selects pixels afresh each iteration.
Both methods therefore start from the same measured-data estimate, without
reference poses or a different initialization.

| Five saved windows per recording | Room | Desk2 |
|---|---:|---:|
| Fixed-population convergence | 5/5 | 5/5 |
| Fresh-population convergence after refinement | 0/5 | 0/5 |
| Control fresh-population convergence | 0/5 | 0/5 |
| New applied corrections, total | 11 | 13 |
| Control applied corrections, total | 50 | 50 |
| Fixed/fresh support or solve guard failures | 0 / 0 | 0 / 0 |
| Fresh RMS at most 0.03 m | 5/5 | 5/5 |

The largest proposed node translation after re-association ranges from
**0.159–1.534 mm** in room and **0.168–1.567 mm** in desk2. Fixed-population
proposals satisfy the original strict 0.01 mm / 0.01 mrad bounds. The final
fresh RMS medians are **2.460 mm** room and **2.731 mm** desk2; the ten-update
controls give **2.464 mm** and **2.768 mm**. Changing sample populations prevent
treating those RMS differences as physical-accuracy gains.

Fewer local corrections do not establish faster complete tracking or real-time
performance. Fixed convergence does not certify convergence after reassociation,
tracker admission or correct absolute poses. This limited result does not
prove that an outer loop which refreshes correspondences could never help.

## Solver contract

Select reciprocal measurements only once at the supplied starting poses.
Keep each admitted raw pair's exact measured points/normals and pixel hash;
its minimum support remains **200 samples**. At every inner pose, transform
those measurements again and rebuild the symmetric normalized-normal residual
and its rotation derivative. Recompute Huber weights with cutoff **0.01 m**
and transport the original marginal factor through its nonlinear chart.
Each equation counts every retained raw factor once; repeated optimization
does not accumulate duplicate measurements or commit a new prior.

Use the existing scaled Cholesky solve, positive definiteness and condition
limit **1e6**. Full proposals retain **0.10 m / 0.15 rad** node bounds. There
is no damping, pseudoinverse or rank fill. Try the existing eight Armijo
multipliers **1 through 1/128** against the same frozen Huber-plus-prior cost.

Apply at most **ten corrections**. Stop before applying a correction when
every non-origin node's full proposal is strictly below **10 micrometres /
10 microradians**. An inadmissible solve or exhausted search remains a failure,
separate from convergence. Shortening an update cannot manufacture convergence.
The first optical pose stays fixed; the current acquisition has no incoming
prior. The window remains at most five nodes and 24 pose coordinates.

Finally return two separate equations and remaining proposals: one using the
original frozen population, another using fresh reciprocal associations.
The solver returns copied local poses and a diagnostic certificate. It
changes no node roots, cache, clock, keys or prior, and performs no Schur
elimination. Its result is not a complete tracker admission: displacement,
clock and commit policy remain outside this local tool.

## Independent checks

The verifier reconstructs the initial pixel population with separately
computed patch-SVD normals and relative-camera projection. Each frozen
linearization uses an independent normalized-normal matrix derivative,
incidence matrices and quaternion prior coordinates. Spectral solves and a
separate Rodrigues formula check the full proposal, every attempted multiplier
and after poses. A target-camera objective checks the root-camera operational
cost. No operational residual, normal, H/g or Cholesky routine supplies the
expected result.

The real-image comparison checks **44** newly reported systems: **24** applied
iteration systems, ten fixed final systems and ten fresh final systems.
It separately reconstructs **20** archived control systems at the starting
and control-final poses. These counts are mathematical checks, not independent
new measurements. Source images, manifests, incoming priors and archived
initial/control poses remain hash-bound to the previous packet.

Analytic controls check ten reported systems, a measured-data warm start with
final position error **0.0261 mm**, and a perturbed start requiring two
frozen relinearizations. The complete objective, including a nonzero prior,
passes finite differences on **12** pose axes after movement. Missing fresh
support and planar rank deficiency stop without corrections. Nine corrupted
certificates are rejected, covering the frozen population, inner/fresh systems,
multiplier, pose, both convergence flags, cost and iteration count. Original
input roots remain unchanged. Truth generates and scores the rendered fixture;
it does not initialize the operational fit.

The mandatory core check passes **371** Rust tests and **49** scenarios with
complete sensor-log replays. All **245** deterministic core outputs, **368**
previous evidence files and 16 pinned preceding native sources retain their
hashes. CI includes the new analytic controls and syntax checks.

## Evaluation decision and reproduction

Before the new image fitting, the design and archive freeze **45** source/context
members. Advancing to full trajectories requires both cases to improve their
fresh-converged counts, have no fixed/fresh guard failures, pass all fresh RMS
gates and have fresh median RMS no worse than the control. Both fresh-converged
counts stay at zero, so no full-trajectory trial or default change follows.
No numeric real reference labels enter this local comparison.

Use the pinned optional NumPy/Pillow environment and original hash-pinned TUM
inputs. Output directories must be fresh:

```bash
OPENBLAS_NUM_THREADS=1 python scripts/check-fixed-depth-refinement.py \
  --output artifacts/fixed-refinement-controls-replay
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-fixed-depth-refinement.py \
  --output artifacts/fixed-refinement-recorded-replay
```

The commands return zero for integrity success, including when the design's
progression condition is false. The [report](../assets/fixed-depth-refinement-v1/summary.json),
[validation receipt](../assets/fixed-depth-refinement-v1/validation.json) and
[manifest](../assets/fixed-depth-refinement-v1/files.json) retain the full new
pose/system/search certificates, control results and source/input hashes.
Original images and physical reference files are not redistributed;
[TUM attribution and terms](tum-source-provenance.md).
