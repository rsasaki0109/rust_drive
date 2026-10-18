# Fixed-route speed intervals

Optional surveyed `speed_zones` allow a fixed route to slow before a mapped
interval and regain cruise after passing it. Previously a lower map speed
could only reduce cruise for the whole mission. This feature accepts known
map configuration; it does not recognize signs or infer legal speed limits.

```json
"speed_zones": [
  {"start_s_m": 30.0, "end_s_m": 60.0, "speed_m_s": 2.0}
]
```

Distances are arc lengths in meters from the original resolved route origin;
speeds are meters per second. The optional list belongs to both `Scenario`
and the recorded `PipelineConfig`. Empty lists are omitted when serialized,
so existing absent-field scenarios and sensor replay retain their behavior.
Each list has at most 128 entries. Each finite interval must satisfy
`0 <= start < end <= full route length <= 1500 m`, and its speed must lie in
`[0.1, 8] m/s`. Invalid configuration is rejected before driving. Overlapping
intervals apply their lowest speed; a zone cannot raise the cruise target.

The lane-route exporter can also generate these intervals directly from a
selected whole-lane itinerary with `--local-speed-limits`.
[Map compilation and validation](lane-speed-zones.md).

## Planning contract

The planner projects candidate positions onto the supplied route. It expands
each interval by the ego circular radius in route distance, covering the
front on entry and rear on release. A geometric segment that intersects an
expanded interval caps both endpoints. Thus a centimeter-long interval
between the 81 geometry samples remains visible. This is conservative at
sample boundaries and uses the existing circular, planar vehicle model.

The existing backward reachable-speed pass anticipates these caps with
20% nominal braking headroom; the forward pass retains calibrated acceleration
and braking limits. Arrival times still integrate constant acceleration.
Obstruction-truncated profiles receive the same caps before being retimed and
collision checked. Goal stops and their stationary holds remain constraints.

An already active first-interval cap above which the vehicle is measured to
be moving permits bounded braking recovery from that measured speed. It
cannot instantaneously correct an invalid initial speed. Unreachable future
caps reject the candidate and can cause emergency braking. Neither this
profile nor its recovery rule guarantees physical compliance from arbitrary
initial conditions or with an uncalibrated vehicle/controller.

Intervals currently require a fixed route. A statically resolved navigation
request is supported, including a northbound route. Live navigation updates
and lane-change alternate routes are rejected with nonempty intervals;
their different route origins would require explicit remapping. Regulatory
stopping prefixes retain the original route origin and configured full
intervals. A zone beyond a temporary stop prefix remains valid after release.
The stop-sign combination is a physical regression fixture.

## Independent acceptance

The simulator keeps every 0.05-second physical frame when intervals are
configured. Its separate evaluator clips swept route progress against each
expanded interval and evaluates interpolated physical speed throughout the
overlap. A zone wholly between two frames is still scored. An integration
interval counts at most once when limits overlap. Any excess above the fixed
**0.15 m/s** evaluation tolerance adds a failure and makes the run fail.
The tolerance is an acceptance measurement, not a planner speed allowance.

The Python evaluator reconstructs route progress independently from recorded
truth positions and speeds. It requires every physical frame and rejects
unexercised intervals. The six acceptance fixtures additionally require goal
completion, simulator clearance/rule gates, exact sensor-only replay, and
reacceleration to at least 4 m/s in a fixed post-zone probe. Ground truth
enters scoring only; the driver receives surveyed intervals and sensors.

```sh
source scripts/env.sh
cargo build --release --locked
python3 scripts/test_speed_zones.py
python3 scripts/check_speed_zones.py --output artifacts/speed-zones --seeds 1 7 42
```

Use a fresh output directory for each run. To score a single complete recording:

```sh
python3 scripts/check_speed_zones.py --run artifacts/speed-zones/single/seed-7/run/run.json
```

The first frozen reference trial covers single, sequential, overlapping and
centimeter-long intervals, a stop sign, and a rotated northbound route on
seeds 1/7/42, plus three empty-list controls. All 21 current episodes pass.
The 18 configured-zone episodes retain 12,138 physical frames; their maximum
excess is 0.07804 m/s, and all recover above 6.02 m/s in the post-zone probe.
The three controls match the parent commit's five outputs byte for byte.
The parent parser rejects the new field, including an empty list; those
rejections are preserved. With the unsupported field removed, the six parent
seed-7 runs violate the same surveyed limits when scored externally. That
comparison deliberately adds a map policy and does not claim identical input
schemas. [Validation summary](../assets/route-speed-zones-results.json).

This trial uses authored, empty, straight reference roads with synthetic
sensors. Unit tests also cover obstruction retiming and measured initial
overspeed; they do not qualify those combinations in a physical plant.
Curved-road compliance, arbitrary route projection ambiguities, lateral
departure, live remapping, native RNE dynamics, external sign data and real
vehicles remain unqualified for this feature. Existing short-range follower
failures remain recorded and are not repaired by speed intervals.
