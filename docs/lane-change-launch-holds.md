# Hold an early mission retry when its observed gap closes

The opt-in [resume policy](lane-change-resumes.md) now watches traffic during
the initial part of its fixed connector. A new gap conflict or unusable sensing
can select a source-lane stop while calibrated stopping room remains. The driver
holds braking, confirms a new measured stop and observes a new clear interval
before continuing **the same connector**. It does not create another lane change.

```sh
source scripts/env.sh
cargo build --release --locked --bin rustdriving
python3 scripts/lane_change_launch_demo.py --output artifacts/launch-examples --seed 7
python3 scripts/check_lane_change_launches.py --input artifacts/launch-examples \
  --output artifacts/launch-audit.json
```

The script refuses an existing output directory. It retains map exports,
scenarios, full-rate physical frames, sensor logs and exact replays. Lanes remain
3.5 m wide, the ego circle 1.25 m, actor circles 1.1 m, cruise 2 m/s, deadline
120 s and minimum swept clearance **1 m**. All actors use simulator-only world
motion; their positions, activation times and intentions never enter the driver.

The first target obstacle appears at (52, 3.5) at 20 s, causing the early return,
and begins moving at 4 m/s at 40 s. Each new case adds a second observed actor:

| Case | Additional world event at 49 s | Required outcome |
| --- | --- | --- |
| `rear` | Actor starts at (30, 3.5), moving forward at 6 m/s | Stop inside source, reconfirm and reach the original goal |
| `rear-slow` | Actor starts at (35, 3.5), moving forward at 4 m/s | Hold longer, then continue the same crossing to the goal |
| `launch-blocked` | Actor stays at (54, 3.5) | Hold inside source for the rest of the episode |
| `launch-recovery` | Same as `rear`; LiDAR reports failure during [50, 51) s | Reacquire stop and gap evidence after recovery, then reach the goal |

## Stopping room and commitment

The planner's fixed resume route is clipped against the source rectangle eroded
by `vehicle.radius + 0.25 m`. Its first boundary intersection is the maximum
planning-prefix endpoint; the extra 0.25 m is an empirical tracking reserve.
The current estimated circle plus 0.05 m must also fit source.

For projected resume-route progress `s`, define:

```text
v = max(abs(estimated_speed), abs(fresh_measured_speed)) + 0.1 m/s
b = configured_max_deceleration / 2
d = 0.3 s * v + v² / (2*b) + 0.1 m
planning_endpoint = s + d + 1 m
```

The extra meter accounts for the existing planner stopping one meter before its
route endpoint. The whole endpoint must fit before the eroded-source boundary.
Otherwise the crossing is committed and this early hold is no longer available.
The ordinary collision-aware planner and sensor-health brake continue operating.
An apparent later backward estimate cannot undo this commitment.

Before commitment, both lanes retain the existing six-second continuous gap
policy, with 6 m plus two-second headway and both radii. Target occupancy keeps
its 0.5 m lateral expansion; source uses its physical band. Unhealthy evidence,
freshness loss, acquisition gaps over 0.2 s or either lane being blocked can
select the temporary prefix. Source/target permissions and widths remain those
of the original authored configuration.

During the hold, the command requests at least `b` braking, including while
moving. It does not alternate acceleration and braking as forecast confidence
changes. The normal planner still checks the prefix, and health faults still
use emergency braking. Route-dependent planner state resets at prefix selection
and restoration; controller steering history remains intact.

## Measured stop and release

Three distinct fresh wheel/gyro acquisitions must confirm speed at most
0.05 m/s and yaw rate at most 0.01 rad/s, with estimated speed at most 0.1 m/s
and source-circle containment. Motion observations must be no older than 0.05 s.
Every unusable or non-stationary observation resets stop confirmation and the
clear interval. A confirmed stop starts a new acquisition epoch.

Release needs at least three distinct scans with distinct accompanying motion
samples spanning one second after the stop and latest unusable observation.
Scans must be no older than 0.15 s, with acquisition gaps at most 0.2 s. Both
lanes must be clear. Cached data and delayed pre-epoch scans cannot release the
brake. A sensing fault during the hold requires another measured stop.

The calibrated full-route planner must preview the remaining crossing and rear
clearance. All preview segments **through the connector endpoint plus the ego
radius and 0.5 m** must remain at least 1 m plus 0.3 m tracking reserve from every
observed forecast segment, after subtracting both radii. Beyond that bounded
crossing, normal time-aligned collision checks apply. This spatial reservation
is limited to the remaining crossing; initial mission retry admission retains
its separate full-preview clearance check.

`return_state.resume_state.launch_hold` records `Braking`, `Holding`, `Confirming`
and `Released`, the fixed prefix endpoint, source limit, braking calibration,
blockers, stop/fault/clear/release times and crossing-preview clearance. It is
absent when no hold has occurred. The original resume request and destination
remain fixed. Only one early hold/release is supported; later conflicts use the
ordinary planner without another return or retry.

## Evidence and limitations

The two retained parent-binary rear-approach runs passed physical acceptance,
with minimum clearances 1.149005 m and 1.131011 m, but never held a measured
stop before reaccelerating. Their complete logs, original executable and verified
replays remain retained. The added hold is a new behavior requirement; those
parent runs are not relabeled as physical failures.

Four cases run with seeds 1, 7 and 42. They require source containment during
braking and hold, no hold acceleration/creep, exactly the same connector, the
expected goal/hold outcome, no collision/road violation, 1 m swept clearance and
exact replay. The independent checker reconstructs the quintic and source
boundary, verifies the stopping budget, recalculates full-rate physical
separation and inspects the acquisitions and clearance behind release.

The reserves and braking response are reference-plant assumptions, not proven
error bounds. Late appearances after commitment, repeated hold/release,
mid-change return, occlusion, unknown acceleration, real steering measurement,
camera control and general mission recovery remain unsupported. This adds no
CARLA, RNE or real-vehicle validation.
