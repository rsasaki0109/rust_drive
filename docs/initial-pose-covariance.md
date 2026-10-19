# Explicit initial pose covariance and local side-path tracking

The driving pipeline accepts an optional initial covariance for its supplied
pose calibration. It also publishes the complete posterior matrix when that
option is present, including after acquisition-time GNSS and delayed-map
replay. This supports checking an explicitly declared startup model; it does
not estimate initial heading uncertainty from sensors.

## Contract

`PipelineConfig.initial_pose_covariance` is a symmetric positive definite
3×3 matrix for `(x, y, yaw)` in world ENU. Position entries use m²,
position/heading cross terms use m·rad, and heading variance uses rad². All
entries, the initial pose and the position-variance trace must be finite;
yaw variance must be at most π². Invalid input returns an error before driving,
without replacing or clamping the caller's matrix. Correlated priors are
supported. `Ekf::with_pose_covariance` exposes the same validation independently.

When the option is absent or JSON `null`, initialization retains the original
`diag(1, 1, 0.03)` and omits the initial-prior field. The full posterior is
omitted when none of this prior, the optional
[GNSS covariance shape](gnss-covariance.md) or
[recovery policy](gnss-recovery-guard.md) is supplied. With an explicit prior,
every `PipelineOutput.pose_covariance` contains all nine posterior entries;
`position_variance` remains the x/y trace. Existing health thresholds, GNSS NIS
gating, process noise and observation covariance are retained. The declared
prior is recorded in the sensor-log header and the posterior participates in
strict replay and finite-value validation.

`Scenario.initial_heading_offset_rad` is a separate, simulation-only physical
spawn perturbation, bounded to ±0.2 rad. It changes the physical pose in both
plants, while the pipeline keeps its original supplied pose. The perturbation,
actor state and sensor-injection schedules are excluded from `PipelineConfig`.
The covariance is never calculated from the run's truth or error labels.

## Fixed research calibration

This protocol declares a uniform initial heading error over ±3 degrees before
running physical trials, giving yaw variance `(3*pi/180)^2/3` =
0.0009138522593601257 rad². Position variance stays 1 m² on each axis and initial
cross terms are zero. This is an authored calibration assumption, not measured
vehicle calibration. The seven native seed-7 launch-heading failures in the
[previous mid-edge protocol](mid-edge-routing.md) motivated the comparison.
The original default covariance and failed recordings remain available.

Every original world, map, actor, speed/clearance rule and deadline is retained.
The additional whole-episode quality gates are position RMSE ≤0.3 m,
maximum position error ≤0.6 m and maximum yaw error ≤0.1 rad. The delayed-GNSS
composition additionally retains its original 0.2/0.4 m position gates.

The initial cohort has 53 episodes per plant: eight existing mid-edge worlds
at seeds 1/7/42, same-edge/obstacle/OSM worlds with ±2-degree physical heading
offsets at seeds 101/202/303, three temporal-fusion worlds at seeds 1/7/42,
and two ±0.12-rad miscalibration controls. The latter deliberately violate the
calibration assumption and must fail the yaw gate. Their expected rejection
is separate from a successful driving-quality result.

## Tracking repair and retained failures

The first covariance-only trial passes 53/53 expectation checks in the
reference plant and 51/53 in native dynamics. Native obstacle seed 303 at
−2 degrees reaches the goal with only 0.541761 m clearance, failing the
unchanged 1 m gate; +2 degrees remains stopped and misses its 65 s deadline.
Paired controls with only the prior removed also miss the deadline. Those
controls are viewed diagnosis, not independent confirmation.

The short local-route pursuit preview can follow the repeatedly reanchored
heading prefix while losing the remaining lateral error. A first geometric
preview floor repairs one native case, but the other still fails clearance
at 0.957655 m. Reusing the existing standard road preview for native side
paths repairs both. A newly frozen cohort then adds seeds 404/505/606 at
±2 degrees: the full native 71/71 expectations pass, while reference 69/71
retains two obstacle stops at −2 degrees, seeds 505/606.

The final repair applies that same side-path policy to both plants. For local
routes with `abs(lateral_target) > 0.1 m`, pursuit uses the existing
`clamp(3 + 0.45*speed, 3, 8)` m preview, bounded below by wheelbase and twice
the chassis reference offset. Local centerline turns retain their original
short preview. This adds no fitted coefficient and changes no planner/world
or acceptance threshold. Nonfinite lateral targets brake before preview
selection. Independent circle-intersection tests cover both steering signs
and both vehicle reference geometries. Physical regression tests retain all
four original native controls and both newly discovered reference stops.

A second cohort, fixed before executing this final repair, adds seeds
707/808/909 at ±2.5 degrees in the same three unchanged worlds. Its 36 physical
episodes across both plants are new confirmation; all earlier outcomes are
viewed regressions. First trials, both rejected preview variants, both cohorts,
producer sources/binaries and lossless recordings are retained locally under
`artifacts/initial-pose-covariance-v1/` with SHA-256 receipts.

The shared-preview comparison passes 177/178 expectations, retaining reference
seed 606 at 0.785318 m clearance. Its added Rust regression also fails the
required check; this failure is retained. The next two probes shorten the
reanchoring span: a one-wheelbase floor repairs the reference cases but fails
native seed 303 at 0.757017 m, and adding the existing quintic lateral-acceleration
bound still fails both native signs (0.781971 / 0.762309 m). Disk exhaustion
also interrupts part of the first probe's replay/storage; those environmental
failures are recorded separately from physical clearance failures.

The adopted policy therefore limits anchored recovery to the point/rear-axle
reference. Position and heading corrections fade over the original maneuver's
remaining arc, with a wheelbase floor and the existing quintic second-derivative
bound for the configured lateral acceleration. This prevents every replanning
tick from postponing completion by another full transition. Independent planner
endpoint tests cover both signs before and after the original anchor. The
pipeline preserves this policy over route handover. Native chassis/course
tracking keeps the preceding joined prefix and the independently repaired road
preview; the regressing chassis recovery variants are not adopted.

Before executing this selective policy, an additional seed 1001 at ±2.75 degrees
in the three worlds was fixed, adding 12 previously unconsumed physical episodes.
The complete final protocol contains **190 episodes: 186 positive quality cases
and four deliberately miscalibrated negative controls**. The earlier 178 cases
are now viewed regressions. Full outcomes, including every rejected stage,
appear in the linked measurements. This split qualifies the tested simulation
reference geometries; it is not a universal controller-feasibility result.

## Independent evaluation and reproduction

Python reconstructs every pose, covariance entry and GNSS freshness/count
from delivered observations using a joint matrix update, rather than the
producer's sequential correction. Delayed GNSS and map cases use an unbounded
chronological event oracle. Every posterior must be finite, symmetric and
positive definite and agree within 1e−8. Physical route/speed checks, swept
clearance, complete truth-error scoring and strict sensor replay apply to
every episode. Truth is used only for physical synthesis and evaluation.

Closed-form correlated-Gaussian, rotated-ENU, fractional delayed-replay and
corruption controls check cross covariance, including corruption that leaves
the position trace unchanged. Reported NEES/ellipsoid coverage is descriptive
for correlated authored simulation ticks under the supplied calibration;
it is not an independent consistency test or real-world uncertainty result.

```sh
source scripts/env.sh
cargo build --workspace --release --locked
python3 scripts/check_initial_pose.py --extended --output artifacts/prior-reference
# Build the separately pinned native integration first (see its README).
python3 scripts/check_initial_pose.py --extended --plant rne-dynamic \
  --output artifacts/prior-native
```

Use new output directories. `--extended` includes all 95 specifications per
plant; without it the original 53 are used. `--smoke` retains seed-7 worlds,
seed-303 physical offsets, both seed-505/606 reference regressions and the two
miscalibration controls. CI records the smoke protocol in both plants; its
other platforms remain pending until their jobs actually run.

[Published measurements and provenance](../assets/initial-pose-covariance-results.json)
record complete outcomes, including failed development trials. This remains
a simulation prototype with external initial pose/calibration. Automatic
initialization, measured calibration, sustained automotive localization and
calibrated forecast/planning uncertainty remain open.
