# Early return from an authored lane change

`--return-on-conflict` adds a measured early-return policy to the observed-gap
gate. A new target-lane conflict can select a curve back to the source centerline
and stop there. It applies while the vehicle circle still fits inside source,
with fresh motion and confirmed source observations. It does not return from
the middle of a change. An additional opt-in [resume policy](lane-change-resumes.md)
can retry the mission once after the measured return stop.

## Reproduce

```sh
source scripts/env.sh
cargo build --release --locked --bin rustdriving
python3 scripts/lane_change_return_demo.py --output artifacts/return-examples --seed 7
```

The script refuses an existing directory and retains plans, scenarios, physical
runs, full-rate truth, sensor logs and exact replay. All examples retain 3.5 m
lanes, a 1.25 m ego radius, 1.1 m obstacle radii, 2 m/s cruise, a 70 s stop deadline
and an independent 1 m swept-clearance floor.

| Case | Prescribed new observation | Expected outcome |
| --- | --- | --- |
| `early` | Target obstacle at (52, 3.5) activates at 20 s | Return to source and stop |
| `source-blocked` | Target obstacle and source obstacle at (40, 0) activate at 20 s | Reject return; brake on the current route |
| `late` | Target obstacle activates at 24 s after more lateral movement | Reject early return; use existing braking |

These are prescribed appearances in the reference world. Their positions and
schedules only synthesize sensors and score outcomes; they are absent from the
operational header and do not model real vehicle intentions. Export a normal
goal scenario with the optional policy using:

```sh
target/release/rustdriving lane-route \
  --map maps/lanes/two-lane-change.json --start source --goal target-exit \
  --change-side left --change-start 25 --change-length 30 \
  --wait-for-gap --return-on-conflict --return-distance 30 \
  --scenario-output artifacts/return-scenario.json > artifacts/return-plan.json
```

The return flag requires gap mode and explicit map permission for the reverse
boundary crossing. `--return-distance` is the **maximum** forward return length,
default 30 m, range 10–60 m. The header's `return_on_conflict` adds source endpoints,
width and `max_return_distance_m`; validation checks adjacency, travel direction,
shared extent and stopping room. Rear-axle-offset calibration is excluded.
A return leaves the original destination unmet. The demonstration explicitly
expects a stop; ordinary export continues to expect the original goal.

## Admission, geometry and control

The gate monitors target conflicts after `Changing`. Source admission uses the
same six-second longitudinal gap/headway interval with the **physical** source
band inflated by the observed object radius. At least three distinct accepted
scans must span 1 s, with age at most 150 ms and gaps at most 200 ms; faults require
later acquisitions. Target admission retains its existing 0.5 m lateral expansion.
A neighboring proxy outside source does not erase source evidence solely because
of that target margin. The separate path preview below enforces clearance.

A return needs healthy sensing/localization, a new scan and measured forward
speed at least 0.3 m/s from odometry at most 50 ms old. The estimated circle plus
0.05 m must still fit inside source. Otherwise the original route and existing
collision-aware braking remain active.

`routing::lane_return::plan` constructs a forward quintic Bezier curve matching
estimated position, heading and measured curvature `yaw_rate / speed`. Final
position lies on source with zero lateral slope and curvature. A Bernstein
derivative-hull bound checks continuous curvature against
`tan(max_steer) / wheelbase / 1.005`; sampling is at most 0.25 longitudinal meters.
The circle/reserve envelope must fit the supplied physical lane bands.

Search considers lengths in 1 m steps from 10 m to the maximum, including a
fractional maximum. It selects the first steerable candidate whose existing
planner preview reaches a stopped endpoint. That preview must stay spatially
separated from **all segments** of the bounded forecast by at least 1 m plus a
0.3 m tracking reserve after subtracting both radii. This conservative check
does not assume intersecting paths occur at different times, but remains limited
to the forecast. It checks crossings between both sets of sampled endpoints.

`Aborting` follows the fixed selected curve. Only route-dependent planner state
resets; controller steering history remains intact. Newly observed traffic and
sensor faults continue through normal collision checks and emergency braking.
`Aborted` needs three distinct fresh measured stops near the endpoint, low
estimated speed and source-circle containment. Braking then holds the stop;
the original change is not retried unless the separate resume policy is enabled.
Telemetry retains the request, curvature
bound, preview clearance, source blockers and decision. Sensor replay rebuilds
the curve and compares every output.

## Physical evaluation and retained failures

Only this opt-in mode scores the declared source/target lane union plus the
original remaining route. Circles may straddle the shared boundary while staying
inside the unchanged outer edges. Near section ends, capsule containment is
conservative. The controller's path and status labels do not define the road.
Other modes retain their preceding road checks.

The first fixed 30 m return stopped on source but passed within **0.707009 m** of
the obstacle and failed the unchanged 1 m gate. Its complete log, original
executable and verified 1401-tick replay are retained. Shortest-feasible search
and explicit preview clearance repair that failure. A subsequent seed-1 run
safely stopped but missed the required return: target expansion classified a
neighboring proxy as source occupancy. That missed outcome, executable and replay
also remain retained. Physical/replay success alone does not prove a return.

Tests use an independent power-basis quintic solver, dense curvature checks,
physical surface sampling, segment crossings, fault epochs, cached motion,
source conflicts, late poses, malformed calibration and reverse permissions.
Three cases run with seeds 1, 7 and 42. Return requires actual lateral movement,
source containment, a measured stop and physical hold; rejection cases must
record their reason. Every case requires 1 m swept clearance, no collision/road
violation and exact replay. Occlusion, unknown acceleration, forecast expiry,
mid-change return, stopping-place selection and general mission recovery remain open.
No camera, CARLA, RNE or real-vehicle validation is established.
