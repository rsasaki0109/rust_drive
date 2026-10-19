# Mapped speeds through a fixed lane change

`lane-route --local-speed-limits` now supports one fixed, map-permitted lane
change, including optional observed-gap admission with `--wait-for-gap`.
It uses the source lane limit before the connector, the lower source/target
limit throughout the connector, then the target and selected successor limits.
Requested cruise remains a ceiling. Known map speeds are configuration;
neither sign recognition nor traffic intent is inferred.

```sh
source scripts/env.sh
cargo build --release --locked
mkdir -p artifacts/change-speeds
target/release/rustdriving lane-route \
  --map maps/lanes/change-speeds-target-slow.json \
  --start source --goal target-exit \
  --change-side left --change-start 25 --change-length 30 \
  --local-speed-limits --wait-for-gap --cruise-speed 4 --duration 120 \
  --scenario-output artifacts/change-speeds/scenario.json \
  > artifacts/change-speeds/plan.json
target/release/rustdriving run --scenario artifacts/change-speeds/scenario.json \
  --seed 7 --output artifacts/change-speeds/run
target/release/rustdriving replay --log artifacts/change-speeds/run/sensors.jsonl \
  --output artifacts/change-speeds/replay
```

This command exports an empty authored world. The independent acceptance
command below additionally installs a prescribed traffic actor in one case.

## Route and admission contract

`LaneNetwork::lane_change_route_with_speed_zones` returns the existing change
plan plus its effective speed intervals. The existing geometry builder records
three private knot indices: source-prefix end, connector end and target-section
end. The compiler reads their actual accumulated polyline lengths. Source
longitudinal distances cannot substitute for these curved-route distances:
the 30 m connector between lanes 3.5 m apart adds roughly 0.29 m of arc length.

An empty source prefix or target suffix emits no empty interval. Remaining
whole successor lanes use their exact route knot boundaries. Equal adjacent
effective limits merge, speeds above the consumer ceiling become 8 m/s, and
routes over 1500 m or more than 128 intervals fail explicitly. Existing map
permission, curvature, width and vehicle constraints remain in force. Without
the option, legacy reports and mission-wide minimum-speed exports are retained.

Optional admission retains its source-lane stopping prefix while traffic is
observed to block entry. That prefix keeps the full route origin and speed
policy. After fresh-clear dwell, the same fixed connector is released.
Admission continues to use the conservative configured-cruise gap bound;
a lower local speed does not reduce that bound. The existing 4 m/s cruise
ceiling for gated changes remains. Sensor-only replay records and recomputes
both admission and speed configuration.

`--return-on-conflict` can also compile an explicit cap for the alternate
return-and-stop route. [Return contract and independent acceptance](lane-return-speeds.md).
`--resume-after-return` and live navigation remain incompatible with nonempty
speed intervals; they need separate tail remapping.

## Independent reference acceptance

```sh
python3 scripts/test_lane_change_speeds.py
python3 scripts/check_lane_change_speeds.py \
  --output artifacts/lane-change-speeds --seeds 1 7 42
```

Use a fresh output directory. Six maps exercise lower target/source limits,
right and northbound changes, a connector ending at the shared section boundary,
and a centimeter-long low-speed successor. The first four use observed-gap
admission. In the lower-target case, a world-ENU actor remains still until
12 s and then drives forward at 4 m/s. Only its synthetic sensor observations
reach the driver; its schedule stays in simulation/evaluation.

The evaluator reconstructs the connector using independent de Casteljau blend
evaluation, original map endpoints and selected successor geometry. It checks
the compiled policy on the union of map/profile boundaries, scores full-tick
physical speeds with the unchanged 0.15 m/s excess tolerance, reconstructs
swept circular clearance with a fixed 1 m floor, and verifies exact sensor
replay. A fixed post-restriction probe must reach at least 3.5 m/s. The traffic
case also requires source-circle containment until 12 s, an observed blocked
stationary wait, and no physical source departure before 13 s.

The first auditor incorrectly read PascalCase admission states as lowercase
and rejected completed maneuvers. Its source, reports and all original runs are
preserved. Corrected enum decoding re-scores the same recordings; numeric gates,
worlds and driver code are unchanged. [Final measurements and retained audit
error](../assets/lane-change-speeds-results.json).

This is a bounded authored reference-plant trial. Variable topology, arbitrary
sequences of changes, mission-retry speed remapping, general traffic
interaction, native RNE qualification and real vehicles remain outside it.
Earlier short-range follower failures remain preserved and are not repaired.
