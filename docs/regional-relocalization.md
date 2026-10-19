# Stopped regional map relocalization

An optional measured-map recovery policy can start a new localization epoch after
lost wheel/gyro motion. Its default is disabled: the motion-continuity guard keeps
braking. This experimental policy supports supplied fixed routes and surveyed
maps with ordinary synchronous body-XY LiDAR. An explicit
[navigation-preservation option](navigation-relocalization.md) can retain a live
mission and recheck its active route. Other navigation combinations, lane changes,
mapped signals/signs/intersections and XYZ/multi-height configurations are
rejected so a reset cannot silently discard ongoing mission/control state.

## Operational contract

Set `relocalization: {}` beside `localization_map` in a scenario or sensor-log
pipeline header to request defaults. Configuration validates the search bounds,
shared comparison budget, confirmation count and finite attempt/retry limits.
The simulator's dropout schedule remains outside that operational header.

While motion is invalid, the previous pose/covariance and accepted GNSS clocks
remain frozen and braking stays at -6 m/s² with zero steering. Recovery first
requires three new synchronous wheel/gyro samples with absolute speed at most
0.05 m/s and yaw rate at most 0.01 rad/s; sample gaps over 75 ms break the chain.
It then requires a current scan acquired after the loss. Delayed scans cannot
reconstruct their acquisition pose across that missing-motion interval.

The last covered pose supplies only a bounded search center: ±4 m in each ENU
axis and ±0.3 rad yaw by default. An evenly distributed subset of at most 128 raw
scan points is fixed before scoring, using indices `i*N/min(N,128)`. All selected
scan/map pairs vote for translation cells at five yaw angles; the top 25 cells
per angle seed at most 125 bounded ICP refinements. Complete coarse voting and
all selected refinements share one 20-million comparison budget. An exact
nearest-neighbor k-d tree prunes only half-spaces beyond the current distance
bound and preserves equal-distance index ties. Every visited map point charges
the same budget. Exhaustion cannot promote the processed prefix.
Existing unique/trimmed correspondences,
convergence, residual, geometry and covariance limits still apply.

A distinct fit over 0.25 m or 0.05 rad away with RMS at most
`1.1*max(best_rms,0.02m)` rejects the search as ambiguous. This compares the finite
selected hypotheses and establishes no global map uniqueness. Three distinct
current scan acquisitions must propose positions within 0.15 m and yaw within
0.02 rad, with at least 0.5 s between attempts and at most 0.75 s between confirmations. The
maximum is eight attempts. Missing/moving/repeated wheel observations, bad scans,
conflicting proposals and attempt exhaustion keep braking.

The final measured fit and its full 3×3 covariance initialize a fresh pipeline
with the same supported route and calibrations. Covariance floors preserve
correlations by congruence scaling. `localization_epoch` increments; track IDs
must be interpreted within that epoch. Filter histories, occupancy, tracks,
predictions, planner and controller are newly constructed. The confirming frame
brakes. GNSS/LiDAR acquisitions at or before the reset cannot fuse again or renew
freshness; subsequent fresh measurements must satisfy the normal health gates.
The new filter has no accepted GNSS fix, so the default policy also requires a
fresh post-reset GNSS acquisition. A separate opt-in
[bounded map-support window](relocalization-map-only.md) can support that new
epoch for at most five seconds before GNSS returns.

The optional `relocalization` diagnostic records attempts, confirmation status,
last gap/reset and the finite seed/candidate evidence on search frames.

## Frozen evaluation

The 28 inputs are frozen before first physical execution. Both the reference 2D
and native RNE dynamic plants use authored straight/curved roads with the
original 120 m route, 3.5 m half-width, 6 m/s cruise speed, 45 s deadline and
existing GNSS delays.
Twelve off-road circular landmarks supply a map sampled before simulation,
without runtime truth entering the driver. Seeds 1/7/42 provide 12 recovery cases;
four healthy and four otherwise identical policy-disabled controls accompany
periodic-map, no-overlap, missing-motion and delayed-scan controls in both plants.
The periodic-map control explicitly permits 100 million comparisons so it must
exhibit an actual distinct-fit ambiguity rather than merely exhaust a budget.

Gates require an actual stop in [10.5,11.2) s, recovery during [11,15] s, original
goal/deadline, at least 1 m swept clearance, and valid-epoch RMSE≤0.2 m,
maximum position error≤0.4 m and yaw error≤0.1 rad. Negative controls must stay
stopped after 12 s without a new epoch. All invalid-epoch pose errors and general
CLI accuracy failures remain in full-interval scoring.

A separate raw-point correlation pass reconstructs every selected seed and vote.
Exhaustive nearest correspondences and complex-number rigid fits reconstruct
the bounded stopping steps from the best and ambiguity-witness seeds. A joint
normal-matrix inversion separately checks covariance. Other candidates
are retained and exactly replayed, without claiming an independent complete
ICP search implementation. Independent raw wheel clocks locate motion loss;
a chronological full-matrix filter checks every valid prefix/post-reset epoch.
Physical truth enters only synthesis and scoring, including swept clearance.

## Recorded outcomes

The final revision B passes **28/28** frozen cases: **12 recovery cases** stop,
initialize a measured new epoch and reach the original goal; **four healthy
controls** reach their goals; **12 disabled/negative controls** remain stopped.
Recovery occurs at **12.10 s** in every recovery case. Across recorded valid epochs,
worst RMSE is **0.099631 m**, maximum position error **0.250918 m** and
maximum yaw error **0.094553 rad**. Independent state/covariance and physical
checks plus exact replay cover **19,496 control ticks**.

The general CLI still records **24 failed accuracy summaries** because it
scores poses during the invalid epoch. Those original errors and exit codes
remain visible, including the recovered-goal cases. Only valid epochs qualify
pose accuracy; the missing-motion interval is never filled with invented motion.

The complete first revision A is also retained: all 12 recovery cases exhausted
the same 20-million comparison budget and stayed stopped; the other 16 cases
passed. Revision B replaces regional refinement's nearest-neighbor work with an
exact k-d tree. A 500-query exhaustive comparison checks equal-distance ties,
empty search radii and budget refusal; one saved failing scan now completes all
125 hypotheses in 5,057,956 comparisons. The **same 28 scenario inputs, search
bounds, hypotheses, comparison budgets and physical acceptance gates** are rerun.
All failed physical outcomes are retained. Earlier synthetic uniform-grid failures
and the pre-physical correlation repair are separately recorded.

The first revision B checker records **27/28** classifications. In one native
curved-road case, it incorrectly adds a small **unexecuted next step** as a
convergence requirement. The declared existing ICP rule uses the **last executed
step**. That step is 0.000073570 m / 0.000036193 rad; reconstruction from the
selected raw seed agrees with the published pose within 2.9e-14 m. The checker
now independently reconstructs bounded ICP from the best and ambiguity-witness
seeds and verifies that actual stopping rule, final residuals and joint
covariance. No tolerance is raised. The original failed classification and all
first raw bytes/CLI summaries remain; re-auditing repeats no physical run and
changes no driving algorithm or frozen physical gate. Unit controls reject a
forged pose and a fit that cannot stop within its iteration budget.

The final exact `bash scripts/check.sh` passes **718 Rust tests**, **57 Python
tests** and **49 standard drive/replay pairs**. All **245 standard files** and
all **470 previous recordings' replay outputs** retain exact bytes. Native
release formatting, strict all-target Clippy and **63 tests** pass. Final
runtime hashes match the physical candidates; locks, toolchains and engine pin
remain unchanged. Source snapshots, all first outcomes, failed CLI summaries,
raw decoded/stored hashes and both algorithm revisions are in the
[public result packet](../assets/regional-relocalization-results.json).

Raw streams are retained as lossless XZ. Six large older research JSON files
were separately losslessly encoded to fit the local workspace; the receipt
retains their original paths, decoded hashes and unchanged first outcomes.
The comparative estimate is **about 27%**, subjective against Autoware, Apollo
and openpilot; the 30% waypoint remains open.

## Reproduction and limits

```sh
source scripts/env.sh
bash scripts/check.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 -m pip install -r scripts/requirements-relocalization.txt
python3 scripts/check_relocalization.py --output artifacts/regional-relocalization
```

Use a new output directory. `--freeze-only` retains inputs without executing;
`--smoke --plant reference-2d` or `--plant rne-dynamic` selects 10 cases per plant.
NumPy is an offline qualification dependency and adds no driver/runtime dependency.

These are authored planar Linux x86_64 simulation results. Search/confirmation
can fail outside the bounded region, with aliases or insufficient measured
geometry. Stopped wheel observations and the supplied map remain assumptions;
real-time latency, automotive uncertainty, global initialization, delayed
odometry, general live mission-state recovery and real-vehicle safety remain unqualified.
The 30% maturity waypoint remains unmet.
