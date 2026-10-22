# Resume after an early lane-change return

`--resume-after-return` opts into **one** retry of the original mission after
an early return and measured stop. It builds a new forward connector from the
current estimated stationary pose to the original target lane beyond the first
change. The remaining authored route and destination stay the same.

```sh
source scripts/env.sh
cargo build --release --locked --bin rustdriving
python3 scripts/lane_change_resume_demo.py --output artifacts/resume-examples --seed 7
python3 scripts/check_lane_change_resumes.py --input artifacts/resume-examples \
  --output artifacts/resume-audit.json
```

The script refuses an existing output directory and retains scenario exports,
full-rate physical frames, sensor logs and exact replays. These are authored
reference worlds, using the same 3.5 m lanes, 1.25 m ego circle, 1.1 m obstacles,
2 m/s cruise and **1 m swept-clearance floor** as the early-return examples.
The 120 s mission deadline matches the normal lane-change mission; the vehicle
first returns to source and stops before considering a retry.

| Case | World event after the target obstacle appears at 20 s | Required outcome |
| --- | --- | --- |
| `release` | Obstacle begins moving at 4 m/s at 40 s | Observe release, retry once and reach the original destination |
| `blocked` | Obstacle stays at (52, 3.5) | Hold the return stop throughout the episode |
| `reblocked` | Obstacle moves; another appears at (66, 3.5) at 54 s | Retry once, then brake for the new observation |

Actor coordinates, schedules and intentions are simulator/evaluator inputs.
The driver receives synthetic measured scans, wheel speed/gyro and GNSS, with
no actor truth or schedules in its operational header.

## Export and configuration

```sh
target/release/rustdriving lane-route \
  --map maps/lanes/two-lane-change.json --start source --goal target-exit \
  --change-side left --change-start 25 --change-length 30 \
  --wait-for-gap --return-on-conflict --resume-after-return \
  --resume-distance 30 --scenario-output artifacts/resume-scenario.json \
  > artifacts/resume-plan.json
```

Resume requires early-return mode, which already requires forward and reverse
map permission. `--resume-distance` is the maximum connector length: default
30 m, finite range 10..60 m. It requires the resume flag. JSON stores the optional
`lane_change.return_on_conflict.resume_after_return` object with
`max_change_distance_m`. Missing configuration preserves the prior return-and-hold
behavior and serialized outputs. These controls still exclude live navigation
updates, other mapped traffic controls and rear-axle-offset mode.

## Observation and route contract

The existing return policy first earns `Aborted` with three distinct fresh
measured stops near the return endpoint. Resume starts a **new observation epoch**
at that stop. It requires healthy sensing, wheel/gyro observations no older than
0.05 s, measured speed at most 0.05 m/s, yaw rate at most 0.01 rad/s and estimated
speed at most 0.1 m/s. The circle plus 0.05 m must fit source, and heading must be
within 0.15 rad of its forward axis. Braking remains applied during this hold.

Both lanes must pass the existing six-second continuous forecast gap policy:
6 m clearance plus two-second headway and both radii. Target occupancy retains
its 0.5 m lateral expansion; source uses its physical band as in early return.
At least three distinct scans and distinct accompanying motion acquisitions
must span one second **after** the stop and latest unusable observation. Scans
must be no older than 0.15 s, with acquisition gaps at most 0.2 s. Conflicts,
unhealthy or non-stationary evidence and missing acquisitions reset confirmation;
cached or delayed pre-epoch samples cannot authorize departure.

`routing::lane_resume::plan` constructs a quintic from the stopped estimated
position and heading, assuming zero initial curvature, to zero lateral
derivatives on target. The stationary gyro check does not measure tire steering;
this assumption is limited to the reference plant. A Bernstein derivative-hull
bound covers continuous curvature, and the circle/reserve control hull must fit
the unchanged source/target union. Samples are at most 0.25 longitudinal meters.
The join must lie beyond the original change, within the shared straight section,
with forward stopping margin. The original straight target suffix is validated,
and its remaining points and destination are appended without the old connector.

Search tries lengths from 10 m in 1 m steps, including a fractional maximum.
The existing calibrated planner must preview the complete crossing and rear
clearance. All preview segments must stay spatially separated from every forecast
segment by **1 m plus 0.3 m tracking reserve**, after subtracting both radii.
No feasible candidate means continued braking at the return stop.

On acceptance, `Aborted` becomes `Resuming`. Route-dependent planner state resets;
controller steering history remains intact. The fixed connector and original
tail stay active after `Completed`, which requires the rear to clear the connector
and the circle to fit target. New traffic and sensor faults use the normal
planner and emergency brake. There is no second return/retry loop. Telemetry
retains stop/clear/start acquisition times, blockers, request, curvature bound,
connector length and preview clearance in `return_state.resume_state`.

During the early part of this connector, an [observed-gap hold](lane-change-launch-holds.md)
can pause while a calibrated stop still fits inside source. It confirms a new
measured stop and fresh post-stop gap before restoring the same fixed connector.
It adds no second lane-change request or return/retry loop.

## Evidence and limits

Three cases run with seeds 1, 7 and 42, requiring a physical source stop, no hold
creep, exactly one retry when expected, the original goal or a measured final
stop, no collision/road violation, 1 m swept clearance and exact replay. Separate
checks reconstruct the quintic in a power basis, densely check curvature,
recalculate swept separation and physical containment, and inspect the sensor
acquisitions behind departure. Fault, cached/delayed scans, source rear traffic,
insufficient room, bad heading and malformed calibration have rejection tests.

This is a bounded simulation policy. It does not establish general mission
recovery, mid-change return, dynamic lane-map updates, occlusion handling,
unknown acceleration coverage, camera control, CARLA/RNE behavior or real-vehicle
validity. Existing failed external perception/localization evaluations remain
unchanged.
