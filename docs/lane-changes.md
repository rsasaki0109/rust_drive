# One authored lane change

`lane-route` can generate one map-permitted lane change from its start lane,
then follow directed successors from the neighboring lane to the goal. The
generated geometry runs through the existing sensor-driven reference simulator
and sensor-only replay. An optional [observed-gap gate](lane-change-gaps.md) now
holds the vehicle on its source lane until measured traffic clears its admission
conditions. Geometry remains fixed before departure; merge negotiation, abort
trajectories and search for arbitrary sequences of changes remain future work.

## Run

```sh
source scripts/env.sh
mkdir -p artifacts/lane-change
cargo run --release --locked --bin rustdriving -- lane-route \
  --map maps/lanes/two-lane-merge.json \
  --start east-right-in --goal east-exit \
  --change-side left --change-start 5 --change-length 30 \
  --cruise-speed 4 --scenario-output artifacts/lane-change/scenario.json \
  > artifacts/lane-change/plan.json
cargo run --release --locked --bin rustdriving -- run \
  --scenario artifacts/lane-change/scenario.json --seed 7 \
  --output artifacts/lane-change/run
cargo run --release --locked --bin rustdriving -- replay \
  --log artifacts/lane-change/run/sensors.jsonl \
  --output artifacts/lane-change/replay
```

All three `--change-*` options are required together. `left` and `right` are
relative to travel direction. Distances are longitudinal meters from the start
of the source lane, not seconds or arc length along the curved connector. The
example begins changing at 5 m and reaches the target centerline at 35 m of
the shared 40 m section. It then follows `east-left-merge` and `east-exit`.
The fixture forbids the reverse change; asking for it fails before file creation.

Omit `--scenario-output` to obtain only the JSON plan. Export keeps the existing
120 s default deadline, 3.5 m lane widths and vehicle calibration; the requested
4 m/s cruise is capped at the itinerary's lowest limit, 3 m/s. Existing output
files, including input aliases, are never overwritten.

## Geometry and validation

The independent routing API is `LaneNetwork::lane_change_route`, with a
`LaneChangeRequest { side, start_m, length_m }` and a circular planar
`VehicleConfig`. It requires the existing straight, same-direction, coextensive
neighbor contract. Source and target must be open, the requested direction must
be explicitly permitted, and the goal must be reachable from the target using
only explicit successors. Intermediate and goal closures also apply. Selecting
a neighbor does not relax any following-route restriction.

The lateral blend is `f(u) = 10u³ - 15u⁴ + 6u⁵`, for `0 ≤ u ≤ 1`. Position,
tangent and curvature match the straight source and target at the continuous
curve's endpoints. Output is a polyline sampled at no more than 0.25 longitudinal
meters, with at most 4096 transition segments. The prefix stays on the source
centerline, the suffix on the target centerline; no physical lane is widened.
The transition's conservative corridor stays inside the combined lane bands
under the map geometry tolerance. The narrowest itinerary half-width must exceed
the vehicle radius by at least 0.05 m.

A continuous curvature bound uses `max f' = 15/8` and
`max |f''| = 10/√3`, including the accepted micrometer neighbor drift. It must
fit `tan(max_steer) / wheelbase / 1.005`, matching the existing local corner
fitter's steering-radius reserve. Short or out-of-section changes, nonfinite
values and invalid vehicle calibration fail before a plan is returned. This
geometric bound does not establish dynamic tracking for arbitrary maps/speeds.
Scenario export also runs the existing complete pipeline validation and corner
fitting; the planner subsequently creates its time-indexed trajectory using
the existing acceleration, lateral acceleration and obstacle constraints.

The report adds `lane_change` metadata: source/target IDs, request, vehicle
calibration, continuous curvature bound, steering bound and corridor margin.
`plan.lane_ids` lists the partially traversed source and target followed by whole
successor lanes. `plan.distance_m` measures the actual sampled route. Width and
speed remain conservative minima over the entire itinerary. The exported road
network is one resolved edge; original map search and permissions are not rerun
by sensor replay. Retain the map and plan report as selection provenance.

## Evidence and remaining work

Routing tests use independent de Casteljau Bezier evaluation and dense arc-length
and curvature calculations, both directions, rotated roads, unequal widths,
map-tolerance drift and exact section endpoints. Negative checks cover forbidden
directions, closures, unreachable goals, curvature, envelope and allocation limits.
CLI acceptance exercises left, mirrored right and northbound changes with seeds
1, 7 and 42. It checks goal completion, collisions, corridor violations, actual
entry of the vehicle circle into the target lane before its end, and exact
sensor-only recomputation. Independent lane-band checks inspect the retained
truth samples; the existing simulator evaluates physical failures every tick.

These geometry examples are empty authored reference worlds. Separate
[observed-gap examples](lane-change-gaps.md) add tracked front/rear traffic and
waiting/ready/changing/completed state. Abort trajectories, live closure handling
during a change, repeated changes, camera integration, CARLA and real-vehicle
validation remain unimplemented.
