# Authored lane maps and following routes

The optional `rustdriving-routing::lanes` module represents directed lane
sections, explicit successor connections, left/right neighbors and directional
map permissions for lane changes. It computes a shortest **following** route
between whole start and goal lanes while excluding requested lane closures.
The CLI can export that itinerary as a runnable scenario for the existing
sensor-driven reference simulator and sensor-only replay.

This is an authored-map feature. OSM lane import, moving lane-change trajectories,
traffic-gap acceptance, merge priority and general lane-level navigation during
driving are future work. The demonstrated merge follows a supplied centerline
into a shared successor in an empty world; it does not negotiate with traffic.

## Run

From the repository root, with the pinned toolchain enabled:

```sh
source scripts/env.sh
mkdir -p artifacts/lanes
cargo run --release --locked --bin rustdriving -- lane-route \
  --map maps/lanes/two-lane-merge.json \
  --start east-left-in --goal east-exit \
  --scenario-output artifacts/lanes/merge.json > artifacts/lane-plan.json
cargo run --release --locked --bin rustdriving -- run \
  --scenario artifacts/lanes/merge.json --seed 7 --output artifacts/lanes/run
cargo run --release --locked --bin rustdriving -- replay \
  --log artifacts/lanes/run/sensors.jsonl --output artifacts/lanes/replay
```

`lane-route` prints a JSON report. Without `--scenario-output` it only computes
the route. Start at `east-right-in` for the straight branch; add
`--closed-lane east-right-merge` to select the explicit bypass. Merely naming
the neighboring `east-left-in` as a goal does not invent a lane-change connector.
Scenario export accepts optional `--cruise-speed` (default 2 m/s) and `--duration`
(default 120 s). It validates the complete request before creating a new file
and refuses to overwrite an existing output, including an input path or link.

## Contract

The input has `schema_version: 1` and a `lanes` array. Each lane contains:

- `id`: a unique identifier, at most 128 UTF-8 bytes, without outer whitespace or control characters.
- `centerline`: world ENU `{x, y}` points in meters, ordered in the travel direction.
- `width_m`: full lane width, 0.5–12 m; constant within this lane section.
- `speed_limit_m_s`: authored limit, 0.1–20 m/s.
- `successors`: permitted longitudinal next-lane IDs; missing or empty means terminal.
- Optional `left` and `right`: `{lane_id, change_allowed}` relative to travel direction.

Successors must share the exact endpoint coordinates. Connections are explicit:
equal coordinates alone never create permission. Neighbor references must be
reciprocal on opposite sides; their `change_allowed` values may differ. The
initial neighbor model accepts only straight, parallel, coextensive sections
sharing a boundary at half the sum of their widths, within 1 micrometer of
geometric tolerance. Opposing traffic, reversed sides, overlapping lanes,
unmatched extents and curved neighbor sections are rejected. Curved following
sections without neighbor declarations remain usable.

Limits are 4096 lanes, 2048 points per lane, 65536 points per map, 32 successors
per lane and 32768 total successor references. Coordinates are finite and within
±1000000 m; each section is at least 1 mm long. CLI input is bounded to 16 MiB.
Unknown JSON fields, unsupported versions, missing references, duplicate IDs or
closures, invalid numbers and unreachable requests fail explicitly.

`LaneNetwork::following_route` minimizes the sum of full selected-lane lengths,
including start and goal. Equal-cost choices use sorted IDs, independently of
input order. The same lane as start and goal means traversing that whole lane.
`lane_change_target` only reports declared map permission after closure checks;
it does not check traffic or authorize an actual maneuver.

The plan exports lane IDs, distance, resolved geometry, the narrowest width and
lowest speed limit, plus a road-network chain for existing consumers. Scenario
cruise is conservatively capped by the lowest selected-lane limit for the entire
trip. Piecewise speed limits and widths are not yet used by the planner. Export
enables the existing calibrated local corner fillets and rejects corners that
cannot fit the unchanged corridor and default vehicle. This does not guarantee
tracking feasibility for arbitrary maps.

The generated scenario contains only the selected chain. Sensor replay recomputes
driving on its resolved geometry; it does not rerun the original lane search or
apply live lane closures. Retain the input lane map and CLI report to reproduce
the pre-departure selection. Existing road maps and log schema 1 are unchanged.

## Validation and retained limitation

Routing tests compare search against exhaustive simple paths on a cyclic lane
graph, test deterministic ties, directed permissions, closures, reversed travel
direction, malformed topology and resource limits. Public CLI tests exercise
straight, merging and closure-detour routes with sensor-only recomputation, and
reject unknown fields, oversized input, unreachable neighbors and infeasible
vehicle corners before scenario export.

The first exporter omitted local corner fitting. Its left-merge seed-1 run
stopped at about 19.185 m, failed the unchanged 120 s goal deadline and produced
a replayable 2401-tick log. The corrected exporter uses the existing local
geometry mode; lane width, vehicle, requested speed and deadline retain their
original values. This failure is evidence that legal graph connectivity alone
does not establish a driveable trajectory. No real-vehicle or traffic-interaction
capability is established by these authored examples.
