# Object motion across localization corrections

The pipeline tracks LiDAR objects and retains braking-history samples in a
continuous wheel/gyro frame. GNSS and registered-map corrections change the
transform from that frame into the current localized world. They do not enter
object velocity or create new observations.

Previously each acquisition used its independently corrected EKF world pose.
A heading correction of 3.5 degrees can shift a return 40 m away by about 2.4 m.
Comparing that return with the uncorrected old track can manufacture object
motion, break association and distort collision forecasts. The existing distance
gate and global assignment cannot resolve this coordinate inconsistency.

Only the accepted EKF **prediction** increment advances the continuous ego pose.
The same increment retains the configured ordinary or chassis/axle motion model.
GNSS and optional fixed-map updates then determine the current continuous-to-world
rigid transform. Acquisition-pose history, detection association and predictor
history remain in continuous coordinates. The pipeline transforms track positions
and forecast points into world coordinates and rotates velocity vectors, without
adding a localization translation or its apparent velocity. All planning and
permission checks still receive world-coordinate observations.

A delayed scan interpolates continuous ego motion at its original acquisition
stamp, then uses the current world transform for publication and its occupancy
ray origin. Missing, duplicate or rejected scans cannot renew `last_seen` or the
accepted LiDAR clock. Cached published coordinates can change with localization,
even while the underlying continuous observation is unchanged. The map retains
its historical log-odds cells; this change does not rebuild those cells.

New pipeline outputs include optional `tracking_frame: {position, yaw}`, a pose
mapping continuous coordinates into world coordinates. It is recomputed from
observations during replay. Older schema-1 logs without this field remain
readable, but their expected outputs require the original algorithm executable
for exact reproduction. New expectations do not migrate old evidence.

## Checks

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-pipeline --test tracker_frame
bash scripts/check.sh
python3 scripts/check_tracker_intervals.py \
  --log artifacts/check/*/sensors.jsonl --output artifacts/tracker-frame-audit.json
```

Independent sensor fixtures specify a straight wheel/gyro trajectory and known
body-point circles, with alternating position-fix errors. Three retained old
failures cover static-target identity/velocity, delayed moving returns and
forecasts, and cached observations during absent/rejected scans. Further checks
exercise genuine turns through the yaw wrap, chassis lateral displacement and
sustained observed braking. The transform is checked against an independently
integrated sensor trajectory; exact replay also reconstructs it.

The arithmetic auditors invert the logged rigid transform before checking
per-track acquisition intervals and braking history. Their tolerances accommodate
floating-point coordinate round trips. They do not establish identity accuracy or
independent physical motion accuracy.

The intersection evidence checker also reconstructs the existing post-fault
acquisition barrier and the maximum 0.2 s gap between clear scans. A delayed scan
acquired before the latest unhealthy decision cannot start the clear timer.
Six retained old/current delayed-case checks exposed and corrected an omission
in the auditor; the operational intersection policy was unchanged. Forged
coordinate, velocity, cached-position and forecast outputs remain rejected by
the independent arithmetic checks.

## Measured reference comparison

The required workspace check passed 559 tests and 49 seed-7 scenario runs with
exact sensor-only replay. Another 12 runs cover seeds 1 and 42 for blockage,
opposing/successive crossings, occlusion, cut-in and reactive fleet traffic;
12 matching original-executable runs also replay exactly. Independent coordinate,
filter-interval, motion-forecast and physical checks passed for all 61 current
episodes, covering 46,593 control steps. Coordinate reconstruction from wheel/gyro
inputs remained within the declared 1e-7 tolerance; interval-correction errors
were zero. These authored simulations do not measure external tracking accuracy.

For three static circles blocking the road, minimum physical clearance was:

| Seed | Original executable | Continuous tracking frame |
| --- | ---: | ---: |
| 1 | 3.579863 m | 3.643893 m |
| 7 | 3.532493 m | 3.664867 m |
| 42 | 3.592950 m | 3.646238 m |

All three stopped with zero collisions/road violations and retained the 3 m
regression floor. At seed 7, the maximum reported speed of these static objects
dropped from 3.210897 to 0.072474 m/s. The remaining value includes simulated
measurement noise and shape-fit variation.

Clearance did not improve in every scene. The largest reduction among the 61
comparisons was **0.288103 m**, in opposing crossings at seed 1:
1.388267 to 1.100164 m, retaining its existing 0.7 m floor. No scenario geometry,
vehicle dimensions, acceptance threshold or operational intersection policy was
relaxed. All 61 compared current runs had zero collisions and road violations.

The local `artifacts/tracker-frame-v1` packet retains original source/executables,
three runtime regression failures, the original auditor failures, six forged-output
rejections and both generations of run/replay evidence. New large recordings are
losslessly compressed with decompressed/stored SHA-256 indexes. The 245 historical
live outputs were restored byte-for-byte. The additional diagnostic and behavior
changes affect 183 of the 245 current-baseline output files; new output generation
does not overwrite the original expectations or establish backward exact replay.

This remains a simulation research prototype. Wheel/gyro drift, observation
uncertainty, shape/occlusion effects and distance-only correspondence remain
limitations. There is no external automotive tracking benchmark, real-vehicle
validation or real-time guarantee supplied by this repair.
