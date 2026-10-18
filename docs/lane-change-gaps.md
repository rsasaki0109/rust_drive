# Observed gaps before one lane change

The optional `--wait-for-gap` mode holds the vehicle on the source lane before
an authored lane change. It uses the normal LiDAR clustering, tracking and
current-time forecasts; simulator actor positions and motion schedules never
enter the driving pipeline. The resolved route is fixed before departure.

## Run and reproduce

```sh
source scripts/env.sh
cargo build --release --locked --bin rustdriving
python3 scripts/lane_change_gap_demo.py --output artifacts/gap-examples --seed 7
```

The script refuses an existing output directory. It retains the exact map plan,
scenario, physical run, sensor log and replay for three cases:

| Case | Prescribed target-lane actor | Expected behavior |
| --- | --- | --- |
| `rear` | Starts 40 m behind at 3 m/s | Wait on source; change after it passes and opens a gap |
| `release` | Stops 28 m ahead, moves at 4 m/s after 15 s | Wait; then change and reach the goal |
| `blocked` | Remains stopped 28 m ahead | Remain on source through the 35 s deadline |

Goal cases retain a 120 s deadline. All use 3.5 m lanes, a 1.25 m ego circle,
1.1 m actor circle, a 2 m/s cruise cap and an independently evaluated 1 m swept
clearance floor. The rear and front actors follow straight world paths instead
of following the ego's lane-change curve. Their schedules are evaluation inputs,
not behavior supplied to the driver.

For a plan without traffic, the CLI invocation is:

```sh
target/release/rustdriving lane-route \
  --map maps/lanes/two-lane-change.json --start source --goal target-exit \
  --change-side left --change-start 25 --change-length 30 --wait-for-gap \
  --scenario-output artifacts/gap-scenario.json > artifacts/gap-plan.json
```

Without `--wait-for-gap`, the existing geometry-only behavior is unchanged.
The JSON report adds `lane_change_gate`; scenario export places it in
`lane_change`. Configuration requires the fixed local geometry mode, a bounded
straight target lane, an aligned source prefix and a monotonic connector, space
before/after the change, cruise at most 4 m/s, and no live navigation or other
mapped controls. The entry must leave at least `vehicle.radius + 5 m` of prefix.

## Admission rule and state

The target-lane axis is the coordinate frame for the gate. Its forecast window
is 6 s. A forward-travel interval covers ego displacement from zero to
`(max(cruise, current speed) + 0.5 m/s) * time`. It expands behind by combined
circle radii, 6 m and 2 s of the fastest predicted forward actor speed; expansion
ahead uses combined radii, 6 m and 2 s of the ego speed bound. The target band
expands laterally by the actor radius and 0.5 m. A forecast intersecting this
moving interval blocks admission. Linear segment clipping checks between forecast
samples, including lateral crossings; malformed or incomplete 6 s forecasts block.
This is a bounded admission hypothesis, not a forecast of the entire maneuver.

`Waiting` supplies a temporary route prefix ending before entry. The unchanged
planner/controller brakes toward it; an explicit low-speed brake prevents hold
creep. `Ready` requires at least three distinct accepted LiDAR acquisitions
spanning 1 s, healthy localization/sensing and source-route alignment. Scan age
must be at most 150 ms, and acquisition gaps over 200 ms restart confirmation.
Cached observations cannot advance it; fault recovery requires acquisitions
after the latest unusable control-time epoch. A new conflict revokes `Ready`.
An approach speed cap reduces speed near the entry while retaining a 0.6 m/s
creep floor; it does not guarantee a stop for arbitrary late threats.

Crossing entry with permission changes state to `Changing`. It retains the full
route; new traffic continues through the existing collision-aware planner and
sensor-health braking. It does not reintroduce a prefix behind the vehicle or
generate a return-to-source maneuver. `Completed` requires the estimated rear to
clear the connector and its circle to fit inside the target lane. State,
blocking track IDs, current forecast front/rear bumper gaps and clear-acquisition
origin are included in telemetry and exact sensor-only replay.

## Evidence and limits

Tests compare continuous clipping with a dense independent time sweep, including
crossings whose sampled endpoints are outside the band, rotated axes, rear
closure and front departure. Acquisition, fault, geometry and exact-replay tests
exercise the shared pipeline. Three reference cases run with seeds 1, 7 and 42:
all require a physical source-lane stop, 1 m swept clearance, no collisions or
road violations, and exact replay. Successful cases also require a measured
block followed by completion and a physical front gap during the change.

The gate relies on the existing finite-range synthetic LiDAR and tracker. It
does not establish free space behind occlusions, detect every road user, bound
unobserved acceleration, or guarantee gap availability after the 6 s window.
Track expiry retains the existing 0.6 s policy. There is no negotiated priority,
abort trajectory, live closure response during the change or automotive sensor
validation. The tested outcomes apply to these authored reference worlds.

`ObjectSpec.world_motion` is simulator-only `{position, velocity}` in world ENU
meters and m/s. It is mutually exclusive with route motion and following policy;
the legacy `s`, `lateral`, `speed` and `lateral_speed` values must be zero. Position
is bounded to ±1000000 m and speed to 15 m/s. Existing activation/motion-start/
motion-end timing applies, and position continues beyond the ego route without
an endpoint clamp. Legacy scenarios and schema-1 logs omit the optional fields.
