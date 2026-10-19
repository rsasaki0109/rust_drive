# Speed limits during an early lane-change return

`lane-route --local-speed-limits --wait-for-gap --return-on-conflict` now
exports both the original route intervals and a surveyed speed cap for the
alternate return-and-stop path. The return cap is the minimum of the original
source and target lane limits, bounded by the consumer's 8 m/s ceiling.

```sh
target/release/rustdriving lane-route \
  --map maps/lanes/return-speeds-left.json --start source --goal target-exit \
  --change-side left --change-start 25 --change-length 30 \
  --wait-for-gap --return-on-conflict --local-speed-limits \
  --cruise-speed 4 --duration 70 --scenario-output artifacts/return/scenario.json
```

The exporter produces an empty world. The independent acceptance command below
adds the prescribed late target-lane conflict. Map permission must explicitly
allow crossing back; existing observed source clearance, fresh scans, motion,
geometry, steering, trajectory and 1.3 m preview-clearance gates remain in force.

`LaneChangeReturnConfig.speed_limit_m_s` is optional, finite, and in `[0.1, 8]`.
When original `speed_zones` coexist with an early return, this alternate cap is
required. It covers the entire new route from zero to its actual arc length,
including the initial join and the final stop. The return preview and the
route-dependent planner reset use the same cap. Original route intervals
continue to apply before the switch. They cannot be reused with the alternate
origin. Requested cruise and measured initial motion remain unchanged.

The lower cap remains active after the vehicle is wholly back in the source
lane. This deliberately conservative return ends at a stop, with no subsequent
source-lane acceleration or mission retry. An already excessive measured speed
uses the planner's bounded recovery; arbitrary switching conditions cannot
guarantee immediate physical compliance. `--resume-after-return` with local
speed limits, live speed-map handover and arbitrary alternate-route intervals
remain unsupported. Absent optional fields preserve existing serialization.

The simulator scores original intervals through the switching instant, then
scores that instant and subsequent physical integration against the explicit
return cap. It does not apply the new cap backward to the preceding interval.
The unchanged excess tolerance is 0.15 m/s. Truth positions and actor schedules
enter physical scoring only; the driver receives map geometry/caps and sensors.

```sh
python3 scripts/test_lane_return_speeds.py
python3 scripts/check_lane_return_speeds.py \
  --output artifacts/lane-return-speeds --seeds 1 7 42
```

Use a fresh output directory. Six fixed worlds exercise left, right and
northbound returns, a lower target-lane limit, a blocked source and a conflict
too late for source containment. Four require a measured return followed by
at least two seconds of physical source-contained standstill, within 0.25 m
of its centerline. The other two must reject the return for the prescribed
observed reason. Every case must physically begin the change, retain 1 m swept
circular clearance, obey the original and alternate speed policies and replay
exactly from sensor inputs.

The auditor independently reconstructs initial connector arc lengths and map
limits, verifies both directional crossing permissions, and scores all physical
frames. An untraveled original tail is allowed because the return changes the
destination; a crossed short interval cannot be skipped. Synthetic negative
controls cover missing post-return limits and the policy switching instant.

The first frozen seeds 1/7/42 trial passes all 18 local-policy episodes:
12 completed returns and six required rejections. It retains 25,218 physical
frames, with maximum speed excess 0.04480 m/s and minimum swept clearance
4.25925 m. All returns begin at the prescribed 20 s conflict and retain at
least 43.8 s of final physical standstill. Six option-free current runs match
the parent's six runs byte for byte. Changing only the recorded return cap
from 2 to 4 m/s causes sensor replay to reject at the first return tick,
20.000 s; the unchanged pre-return prefix still matches.
[Measured results and validation](../assets/lane-return-speeds-results.json).

This is authored reference-plant validation. Native RNE dynamics, arbitrary
traffic and real vehicles remain unqualified for the combined feature.
Earlier short-range follower failures remain preserved.
