# Mapped road speeds during stopped route handover

An authored road edge can now include an optional surveyed speed limit:

```json
{"id":"detour", "from":"fork", "to":"merge",
 "points":[{"x":40,"y":0},{"x":160,"y":0}],
 "half_width":5.5, "speed_limit_m_s":2.0}
```

`RoadEdge.speed_limit_m_s` uses meters per second and must be finite and in
`[0.1, 20]`. The bounded planning consumer caps values above 8 m/s at 8 m/s.
An absent field supplies no mapped cap; configured cruise, curvature and
vehicle limits still apply. Existing absent-field serialization is retained.
OSM import and the option-free lane exporter continue to omit this field;
this change does not infer or import road-sign speed values.

`RoadNetwork::shortest_route_with_speed_zones` preserves the existing directed,
distance-based search and turn-rule handling. It compiles the selected edges
onto exact concatenated route knot indices. Adjacent equal caps merge;
uncapped edges remain gaps. More than 128 resulting intervals or a capped
route longer than 1500 m is rejected explicitly. Routing cost remains geometric
distance. A profile-budget failure does not silently remove a limit or select
a different optimization objective.

The navigator stores geometry and its speed policy as one active or pending
plan. A known closure still requires a reachable stop before the divergence,
healthy localization/alignment and three fresh measured standstill samples.
The old policy remains active while braking or holding. A successful stopped
handover replaces geometry and speed intervals together, retaining sensor,
localization and tracking state. No-route and malformed-update holds retain
the old policy until a valid replacement is accepted.

An unreachable route latches at least 0.5 m/s² of requested braking only after
fresh healthy odometry and estimated speed both confirm at most 0.05 m/s,
within 3.5 m of the stopping endpoint. This preserves its original approach
and prevents noisy low-speed corrections from restarting a completed stop.
The latch continues through a pending reopening until the route is actually
accepted. Normal following or fault recovery clears it; ordinary handover
braking does not acquire it. Existing health braking remains in force.

The hold repair exposed an existing sensitivity in obstacle passing: a
completed lateral shift could return toward the center before the slower
centerline object was passed. The planner now retains its passing preference
until the entire inflated object is behind the ego vehicle. Faster
departing objects can still release that preference after the shift, and
collision feasibility always takes precedence. This uses sensed predictions
and route progress, without simulator object identities or physical truth.

The simulator records the full original map in sensor-only configuration
whenever any edge has a speed limit, including static pre-departure detours.
Replay derives both initial and replacement profiles from that map. Manually
supplied `speed_zones` remain incompatible with navigation, avoiding ambiguous
policy sources. The existing restrictions on other mapped traffic controls
and lane-change gates remain in force for this navigation path.

This supports fixed map speed values during closure-driven routing. Closure
snapshots still contain edge IDs, timestamps and revisions; they cannot update
the speed values themselves. Moving handover and live speed-value updates remain
outside this feature. The subsequent [explicit mid-edge routing](mid-edge-routing.md)
clips these same caps and stopped-handover boundaries to directed partial requests.

```sh
source scripts/env.sh
cargo build --release --locked
python3 scripts/test_navigation_speeds.py
python3 scripts/check_navigation_speeds.py \
  --output artifacts/navigation-speeds --seeds 1 7 42
```

Use a fresh output directory. Seven frozen authored worlds cover a detour,
no-route holding followed by reopening, a northbound map, a centimeter-long
low-speed edge, permanent blockage, an uncapped detour that must discard the
old route's restriction, and a static detour. They retain the original graph
geometry and use empty worlds to isolate speed-policy behavior. New speed
trials have explicit 140 s deadlines, or 35 s for the blocked case. The
preceding traffic/deadline fixtures are unchanged.

The Python auditor independently searches the original unrestricted authored
graphs, reconstructs each selected policy from canonical node endpoints and
edge geometry, and scores every physical integration interval. It checks the
old policy through a switch and the new policy from that instant, without
mixing progress coordinates. The excess tolerance remains 0.15 m/s. Goal
cases must exercise all final-route intervals and regain at least 3.5 m/s
in a fixed post-restriction probe; the uncapped case probes the detour itself.
Reopening additionally requires two seconds of physical standstill before
15 s. Physical and estimated switch speeds must be at most 0.05 m/s, and
every run must replay exactly. The 1 m circular-clearance floor is retained,
but empty-world success does not qualify traffic interaction.

The first frozen trial exposed failed continuous standstill in the reopening
case on seeds 1 and 7. Brief 0.1 m/s creep pulses also exist in the unprofiled
baseline, whose coarse recordings can miss them. The original source, all
35 initial episodes and the two rejected audits remain preserved. The repair
adds measured-stop brake retention and reruns the same worlds with unchanged
deadlines, closure times and numerical gates. An intermediate broad hold
passed these empty worlds but failed the preceding traffic clearance test;
its source, recordings and failed check remain retained. Narrowing or latching
the hold alone also failed that test. The final repair retains braking through
route acceptance and prevents premature center return around slower obstacles.
The preceding traffic test keeps its unchanged greater-than-1 m clearance gate.

Option-free controls use the same worlds with edge limit fields removed and
seed 7. Current and parent recordings are compared, with intentional hold
behavior differences retained. The same hold fix also affects existing
live-navigation regression outputs; passing-side retention can also change
other obstacle trajectories. Every changed standard recording is reported and
retained alongside the original. This is authored reference-plant validation;
native RNE, cross-platform CI and real-vehicle qualification of this combination
remain unverified.
Earlier short-range follower failures remain preserved.

The [recorded result](../assets/navigation-speeds-results.json) contains 21
speed-map runs, seven current option-free controls, and six additional
speed-map/obstacle composition runs, all with exact sensor replay. The six
compositions copy only the fixed obstacle from the preceding handover and
reopening fixtures into the corresponding speed worlds, retaining the same
map, closure schedule, deadline and 1 m floor. Their minimum independently
swept clearance is 1.162 m. Across all 27 capped runs, maximum speed excess
is 0.090 m/s, below the unchanged 0.15 m/s tolerance; every reopening holds
physically still for at least 3.65 s before the reopening snapshot.

`bash scripts/check.sh` passes 629 Rust tests, 25 Python tests and all 49
standard drives with exact replay. Together, the final current acceptance
contains 83 distinct episodes. The passing/hold fixes intentionally change
58 of 245 standard outputs across 12 scenarios; all original outputs and
new recordings are retained, and every standard acceptance gate still passes.
Five of seven option-free controls remain byte-identical to the parent;
the blocked and reopening controls change due to the repaired stop hold.
Changing only the original recorded detour limit from 2 to 4 m/s makes replay
reject at 11.4 s, exactly when that route is accepted. These reference results
do not increase the approximately 20% subjective overall maturity estimate.
