# Confirmed intermediate lateral paths

Optional `refine_lateral_candidates: true` adds six intermediate proposals to
the existing center/left/right lattice. It requires explicit
`lateral_candidate_margin_m` calibration and stays off by default. The bounded
candidate set addresses corridors where a full side offset passes too close to
a flanking obstacle while a smaller offset still clears the center obstacle.

## Planning and observation contract

Let `L = min(3.5, max(0, half_width - vehicle_radius - margin))`, in meters.
The original targets are `0, +L, -L`; additional targets are
`±0.875L, ±0.75L, ±0.5L`. Targets with magnitude at most 0.1 m are omitted.
At most nine proposals enter the existing geometry, containment, curvature,
acceleration, braking and synchronized collision checks.

An added proposal must clear the complete nominal planning path and its hold.
Its forecast-disc clearance is at least the larger of 1 m and the existing
time/motion reserve. Forecast knots, interior radius peaks and acceleration
between path knots remain covered. A rejected addition cannot create a partial
stopping approach or arm the obstruction-stop duty. The original three choices
retain their stopping and collision policies.

A complete added path receives preference when the best original proposal is
blocked or fails the 1 m forecast check. Existing settled-hold and terminal-goal
priorities remain ahead of that preference; ordinary switching costs still
apply. The selected added target must remain eligible through at least three
distinct accepted LiDAR acquisition stamps spanning at least 0.2 seconds, with
no acquisition gap over 0.35 seconds. Control ticks using a cached scan cannot
increase the count. A changed proposal, interrupted feasibility, unhealthy
pipeline, missing/invalid/regressing scene context or route reset clears support.
Each selected path still passes its current complete checks after confirmation.

Local chassis tracking retains the intermediate maneuver's completion anchor
through a temporary center braking prefix. Residual tracking recovery uses the
existing wheelbase and declared lateral-acceleration bounds. Scene confirmation
uses accepted measured scan clocks; obstacle labels and physical evaluation
geometry remain outside the driver. Standalone planner callers supply this
context through `observe_refinement_scene`; without it only the original choices
can be selected.

## Fixed evaluation and retained failures

The new paired corpus has eight configurations per policy: reference and native
RNE plants, straight and curved routes, seeds 7/42. Three static circles bound a
central obstacle and both side corridors. Each enabled/disabled pair changes
only the policy flag and diagnostic name. All conditions keep the 45-second
goal deadline, 1 m swept physical clearance, 0.1 m road reserve, 0.3 m position
RMSE, 0.6 m maximum position error and 0.1 rad maximum body-heading error.

The regression corpus enables refinement on the 28 preceding standalone
enclosure conditions with an existing road-width calibration, including all
eight original incomplete missions and their 65-second deadlines. The other
24 preceding conditions lack this required calibration; they remain covered by
disabled-policy replay rather than being counted as enabled physical tests.
Strict failed gates remain separate from the comparison's no-new-failure result.

| Fixed final measurement | Result |
|---|---:|
| Enabled new corridor conditions meeting every gate | 8/8 |
| Paired three-target controls meeting every gate | 0/8 |
| Supported parent comparisons with no new failed gates | 28/28 |
| Supported parent conditions meeting every strict gate | 20/28 |
| Original obstacle missions reaching their goal | 0/8 |
| Minimum enabled / control swept clearance | 1.200527 / 0.753040 m |
| Minimum enabled actual road reserve | 0.657880 m |
| Maximum enabled position RMSE / position error | 0.057053 / 0.171514 m |
| Maximum enabled body-heading error | 0.017219 rad |
| Latest enabled goal completion | 27.35 s |

All eight controls retain their physical-clearance failures. The 28 supported
parent conditions retain eight source/goal failures and three forecast-coverage
failures, with overlaps. This subset's 20/28 result does not replace the preceding
full 20/52 standalone qualification. All 28,887 final physical/control ticks
remain scored. The three complete runtime cohorts execute the same 44 frozen
input configurations, totaling 132 correlated physical executions; re-auditing
the first cohort adds no executions. All eight disabled controls retain all five
output streams exactly across the three runtime variants.

The first sampler attempted all 52 parents; validation refused the 24 unsupported
inputs before any physical execution. The first physical implementation also
applied the larger reserve to legacy stopping choices and caused six new failed
regression gates. Restoring legacy stopping behavior left three new failures:
two forecast-coverage regressions and one incomplete physical side pass. Those
runs selected an intermediate target for only one, two or three control ticks.
Fresh-acquisition confirmation addresses these observed transient selections.
The frozen inputs, original failed gates, raw recordings and both failed runtime
variants remain retained. No physical or forecast threshold was relaxed.

Independent Python evaluation checks full-rate physical motion, localization,
road containment and actual goal stopping. Forecast trajectories receive
midpoint/Lipschitz certificates every ten control ticks; all intermediate
selections must cover the complete planning horizon. Existing acquired-disc,
tracking and forecast-coverage oracles remain required on regression conditions.
Rust controls check cached scans, interrupted support, route reset, both lateral
directions, impossible corridors and complete sensor-only replay with faults.
The final corpus has 2,827 independent trajectory certificates, including 84
selected intermediate paths. A separate recording audit verifies necessary
healthy three-acquisition/0.2-second context for all 862 intermediate control
ticks; proposal eligibility continuity is covered by the Rust controls.

The final unchanged-source `bash scripts/check.sh` passes 871 Rust tests,
165 Python tests and 49 standard drive/replay cases. All 245 standard outputs
retain exact bytes. Native formatting, 65 tests and strict release all-target
Clippy pass. All 376 preceding sensor logs, totaling 226,292 control ticks,
retain exact replay output bytes. All 660 archives from the three physical
cohorts have encoded and decoded SHA-256 verified.

Earlier required invocations remain recorded: a successful 869-test command
whose source snapshot changed, a compilation failure during a concurrent edit,
and an invocation stopped after a CI directory correction. The final command
uses unchanged operational sources. The native CI steps now use the checkout's
explicit `rust_drive` directory, including the preceding delayed-heading step.
Workflow parsing and local commands pass; remote CI for this branch remains
unverified.

## Reproduce and boundaries

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_refined_lateral.py
# Use a fresh output directory.
python3 scripts/check_refined_lateral.py --output artifacts/refined-lateral
bash scripts/check.sh
```

The [result packet](../assets/refined-lateral-results.json) records complete
outcomes, frozen input bytes, source hashes, failed variants and archive receipts.
These are authored planar simulation conditions with circular object proxies,
declared road widths and synthetic sensor calibration. This bounded lattice
does not implement continuous lateral optimization or calibrate forecast
uncertainty. The preceding eight incomplete obstacle missions remain open.
Default adoption, measured automotive sensing and vehicle operation remain
unqualified. Comparative engineering maturity stays **about 29%**; the 30%
waypoint remains unmet.
