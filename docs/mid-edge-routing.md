# Directed road origins and destinations inside edges

Navigation now accepts either the existing node-ID string or an explicit
directed edge and its arc distance in meters:

```json
{"start":{"edge":"approach","s_m":12.5},
 "goal":{"edge":"east-exit","s_m":25.25},"closed_edges":[]}
```

The original map stays intact. `s_m` follows its canonical ENU centerline from
the edge's `from` node and must be finite and strictly inside that edge. Use a
node ID at a junction. There is no nearest-road selection, reverse driving or
implicit direction inference. An endpoint on a closed edge is unreachable;
out-of-range values and stationary requests fail explicitly. The existing
one-micrometer minimum route-segment spacing also applies to clipped geometry.

The independent routing crate exports `RouteEndpoint` and `EdgePosition`,
`shortest_route_between` and `shortest_route_between_with_speed_zones`.
Node-only requests delegate to the original algorithms, preserving their tie
choices and JSON. Partial requests retain the origin's incoming directed edge
at the first junction, and check restrictions before entering a destination's
partial edge. Both `no` and `only` rules apply. A destination behind the origin
on the same directed edge requires a legal real cycle; the edge can occur twice.

The returned `RoutePlan.traversals` records each original edge's traversed arc
interval. It is omitted for legacy requests. `node_ids` contains only nodes
actually reached, with the origin node included for a node request; it can be
empty for a same-edge partial route. Geometry preserves interior map knots and
uses the narrowest traversed width. Whole-edge speed caps are clipped and rebased
onto this route, with the existing 128-interval / 1500 m policy budget unchanged.

Both simulation adapters initialize at the requested clipped route start.
Sensor-only logs retain the original map and endpoint request; replay recomputes
the route. Closure-driven handover still requires a reachable stop before the
divergence and three healthy measured standstill samples. Its stopping boundary,
common prefix, active speed policy and independent closure evaluation now use
the traversed distances. Map speeds and geometry change together at handover.
These are original-request routes with stopped handover, without moving rerouting
or map matching of an arbitrary live vehicle position.

## Run

```sh
source scripts/env.sh
cargo build --release --locked
target/release/rustdriving road-route \
  --map maps/osm/german-road-network.json \
  --start-edge osm-way-25216931-0-forward --start-s 5.25 \
  --goal-edge osm-way-25216931-1-forward --goal-s 42.04066978032147 \
  --scenario-output artifacts/mid-edge-example.json \
  --cruise-speed 2 --duration 180
target/release/rustdriving run \
  --scenario artifacts/mid-edge-example.json --output artifacts/mid-edge-example
target/release/rustdriving replay \
  --log artifacts/mid-edge-example/sensors.jsonl --output artifacts/mid-edge-example/replay
```

`road-route` prints geometry, traversals and speed intervals as JSON. Node flags
`--start NODE` / `--goal NODE` can be mixed with edge endpoints. Repeatable
`--closed-edge ID` applies directed closures. All validation precedes scenario
creation, and an existing output or link is never overwritten. Scenario export
retains the simulation's existing 20–1000 m / width / duration bounds; the
standalone routing API also supports shorter valid routes.

## Fixed evaluation and retained failure

[Recorded results](../assets/mid-edge-routing-results.json) retain eight fixed
worlds across seeds 1, 7 and 42 in both reference and native dynamic RNE plants:
a same-edge route, its sensed obstacle composition, different partial edges,
a prohibited first turn, a live detour, blocked-route reopening, a closed
destination and the original attributed OSM road geometry. Inputs, deadlines,
clearance and quality gates were fixed before the first physical trial. No
producer, world or numerical gate changed after those trials.
The OSM case retains the [original source and ODbL attribution](../maps/osm/SOURCE.md);
its 3 m half-width remains explicit simulation calibration.

All 48 episodes pass the route/physical contract with exact raw sensor replay:
42 reach their requested destinations and six remain stopped before the closed
destination edge. There are 60,674 full physical ticks. Independently swept
minimum clearance is 1.153489 m against the unchanged 1 m floor; maximum mapped
speed excess is 0.086412 m/s against 0.15 m/s. All six reopenings retain at least
5.30 seconds of continuous actual standstill before reopening. The unchanged
goal deadlines are met; the slowest goal completes at 94.1 seconds.

**Complete qualification passes only 41/48 episodes.** Seven native seed-7
episodes fail the fixed whole-episode maximum-yaw-error gate: 0.109343 rad
versus 0.1 rad during launch. Position RMSE (maximum 0.066637 m) and maximum
position error (0.185828 m) pass their 0.3 / 0.6 m gates. The OSM native seed-7
episode passes, as do all reference and other native seeds. No failed episode
was removed or counted as a complete pass. The first auditor stopped at this
quality failure; an additive reporting change finishes its remaining physical
checks and retains every metric, reproducing all 48 original pass/fail outcomes.
The original auditor and first reports remain in the local evidence packet.

An independent exhaustive path enumerator checks 768 requests over every
closure subset of a cyclic graph, with unrestricted, `no` and `only` turns.
All 178 reachable routes and 590 explicit unreachable refusals agree with Rust,
including clipped geometry, visited nodes, width and speed intervals.
Closed-form and corruption controls check the evaluator itself.

Eight parent-version controls reject the new endpoint input. When only the
recorded header's map is manually cut into equivalent endpoint nodes, that
same parent reproduces every original sensor input and expected pipeline
output exactly in eight native seed-7 recordings. This also reproduces the
seven launch-heading errors: the new routing API does not repair or introduce
that estimator behavior. It removes the need for callers to rewrite map topology.

```sh
python3 scripts/test_mid_edge.py
python3 scripts/check_mid_edge.py --routing-corpus-only --output artifacts/mid-edge-corpus
python3 scripts/check_mid_edge.py --seeds 1 7 42 --output artifacts/mid-edge-reference
# Requires the separately pinned native RNE build; returns 1 for the seven
# retained heading-quality failures while preserving complete raw results.
python3 scripts/check_mid_edge.py --plant rne-dynamic --seeds 1 7 42 \
  --output artifacts/mid-edge-native
```

Use fresh output directories. Recordings are losslessly gzip-archived with
compressed and uncompressed SHA-256 receipts. Linux simulation validation does
not establish real-road navigation, calibrated heading uncertainty, oriented-body
planning or field performance.

The final `bash scripts/check.sh` passes 665 Rust tests, 39 Python tests and
49 standard drive/replay pairs. All 245 standard outputs remain byte-identical;
102 previous sensor-fusion recordings replay exactly. Native format, Clippy
and all 59 integration tests pass, including the existing explicit rejection
of the two short-range follower clearance failures.
