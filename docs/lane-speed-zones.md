# Local speeds from a selected lane itinerary

`lane-route --local-speed-limits` compiles authored lane speed limits into the
fixed-route speed intervals used by the driving pipeline. A low-speed section
then affects its own route interval and braking approach, while later faster
sections can regain the requested cruise. Whole-lane following routes and one
fixed lane-change connector are supported. [Connector rules and observed-gap
admission](lane-change-speeds.md). Live remapping remains unsupported.

```sh
source scripts/env.sh
cargo build --release --locked
mkdir -p artifacts/lane-speed-example
target/release/rustdriving lane-route \
  --map maps/lanes/speed-zones.json --start approach --goal exit \
  --local-speed-limits --cruise-speed 6 --duration 120 \
  --scenario-output artifacts/lane-speed-example/scenario.json \
  > artifacts/lane-speed-example/plan.json
target/release/rustdriving run \
  --scenario artifacts/lane-speed-example/scenario.json --seed 7 \
  --output artifacts/lane-speed-example/run
target/release/rustdriving replay \
  --log artifacts/lane-speed-example/run/sensors.jsonl \
  --output artifacts/lane-speed-example/replay
```

The example selects three successive sections: 0–30 m at 6 m/s, 30–60 m at
2 m/s, then 60–120 m at 6 m/s. Cruise remains 6 m/s; the planner anticipates
the lower middle limit and releases it after the circular rear clears it.
The [fixed-route speed contract](route-speed-zones.md) defines braking,
footprint padding, overlapping limits, initial overspeed recovery and physical
evaluation. This option does not detect signs or obtain legal limits online.

## Compilation and replay

`LaneNetwork::following_route_with_speed_zones` retains the ordinary directed
whole-lane search, tie breaking, closures and geometry. It returns a
`LaneSpeedRoutePlan` with the ordinary `plan` and effective `speed_zones`.
Intervals index the concatenated route's cumulative lengths at each original
lane boundary, including interior polyline points. This prevents independent
per-lane rounding sums from leaving an unbounded endpoint. Equal neighboring
effective limits merge. Limits above 8 m/s become 8 m/s, the current consumer's
ceiling; the original lane map and the plan's minimum map limit remain intact.

The optional compiler rejects routes longer than 1500 m or more than 128
distinct adjacent speed intervals after merging. It never truncates a policy.
The exported scenario preserves requested cruise in 0.1–8 m/s and records
the compiled intervals in the sensor-only header. Width remains the narrowest
selected lane width for the entire route. The original lane map and CLI
report should be retained: replay recomputes driving from the resolved
geometry and intervals, rather than rerunning the original lane search.

Without `--scenario-output`, the option previews the same compiled intervals
in the JSON report. With an output file, all validation occurs before file
creation; existing outputs and input maps are never overwritten. Combining
the option with an alternate-route return/retry request fails before output creation. Without
the new option, existing reports and exports retain their mission-wide
minimum speed and omit the new report property.

## Independent reference evaluation

```sh
python3 scripts/test_lane_speed_zones.py
python3 scripts/check_lane_speed_zones.py --output artifacts/lane-speed-zones \
  --seeds 1 7 42
```

Use a fresh output directory. The evaluator invokes the public exporter,
drives each generated scenario and verifies exact sensor replay. It rereads
the original lane map, checks the frozen selected lane IDs, reconstructs
polyline lengths independently, rejects closed/disconnected itineraries and
compares effective caps across the union of map/compiler boundaries. It then
scores every physical tick against the reconstructed map policy. A centimeter
interval cannot disappear into a larger fast interval or an evaluation gap.

The same fixed 0.15 m/s physical excess tolerance applies. The probe after the
last restrictive section starts 8 m after its expanded end and finishes 10 m
before the route endpoint; current local-policy runs must regain at least
4 m/s there. Six authored maps cover the middle low limit, successive limits,
a centimeter interval, a northbound route, map limits above the consumer
ceiling, and a closure-selected shallow curved detour. Each runs seeds 1/7/42.
[Recorded outcomes and validation](../assets/lane-speed-zones-results.json).

The trial also compares option-free export and driving with the preceding
commit on seed 7. It preserves the old parser's rejection of the new option.
Existing explicit-speed-zone fixtures and standard scenario bytes are
separate regressions, not replaced by this trial.

These are authored empty roads in the reference bicycle plant with synthetic
sensors. This connects supplied map policy to driving; it does not qualify
general curved roads, variable width, interactive traffic, native RNE dynamics,
perception-derived limits or real vehicles. Fixed lane-change speed mapping has
its own [separate reference trial](lane-change-speeds.md).
No existing short-range follower failure is repaired by this addition.
