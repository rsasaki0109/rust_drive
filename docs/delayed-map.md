# Acquisition-time map fusion

Set `localization_map.allow_delayed_scans: true` together with
`gnss_latency_compensation: true` to register delayed LiDAR scans against the
supplied fixed XY map. The default remains synchronous matching. This option
adds actual acquisition-time map correction to the shared reference/native
pipeline, rather than transforming an old measured pose into a current one.

The filter retains accepted GNSS and map measurements alongside wheel/gyro
intervals. Each kind has a 64-event limit within a 0.5-second history. A map
scan must be at most **0.2 seconds old**, preserving the existing map freshness
bound. GNSS rewind remains bounded to 0.5 seconds. Delayed odometry, clock-offset
estimation, per-ray deskew and global relocalization remain unsupported.

Registration is seeded by the reconstructed acquisition pose, including
equal-time GNSS but excluding any corrections acquired later. A fitted map
pose/covariance is inserted at the same acquisition clock; subsequent motion,
GNSS and map updates are recalculated. Equal-time order is motion, GNSS, map.
Fractional acquisition times split the original motion interval. Both declared
rear-axle and native chassis-reference prediction models are supported.

Every previously accepted measurement must still pass its unchanged EKF gate.
A map transaction that would invalidate an accepted GNSS/map correction is
refused atomically: current pose, covariance, history and GNSS evidence remain
unchanged. The map diagnostic reports rejection; it cannot cover a GNSS outage.
Fresh genuine GNSS may still support driving. A conflicting GNSS transaction
uses the existing invalid-GNSS braking latch and requires a covered post-fault
acquisition. Rejected observations are never revived by a later rewind.

GNSS counts, observed/accepted stamps and NIS continue to describe **original
delivery processing**. Replaying an old accepted fix may change its historical
innovation, but cannot manufacture another delivered observation or rewrite
that original NIS. Additional zero-omitted diagnostics report retained GNSS
events and subsequent GNSS corrections replayed after an insertion. Map
diagnostics record the acquisition prior and delivery age in this explicit mode.

Map freshness remains the original scan stamp. Startup still requires genuine
GNSS; a fresh map may cover at most ten seconds since that actual fix. Missing,
expired, future, repeated, uncovered or pre-fault scans cannot renew permission.
An explicit LiDAR failure brakes immediately. Recovery requires a scan acquired
after the fault epoch, even when a queued old scan has a valid age. Absolute
corrections remain outside the continuous wheel/gyro tracking frame.

Registration correspondences are fitted once per original observation. Replay
uses that admitted fitted pose and covariance, with conservative covariance
floors; it does not refit old correspondences. Conditional measurement reuse,
synthetic sensor noise and authored offline map correctness remain assumptions.
This is planar simulation research without real vehicle or real-time validation.

```sh
source scripts/env.sh
bash scripts/check.sh
python3 scripts/check_delayed_map.py --output artifacts/delayed-map \
  --seeds 1 7 42 --controls

# Optional native adapter with its separately pinned engine/toolchain:
cargo +1.95.0 build --manifest-path integrations/rne/Cargo.toml --release --locked
python3 scripts/check_delayed_map.py --plant rne-dynamic \
  --binary integrations/rne/target/release/rustdriving-rne \
  --output artifacts/delayed-map-native --seeds 1 7 42 --controls
```

Use fresh output directories. Four worlds were frozen before physical results:
100 ms map / 400 ms GNSS delivery, 150 ms map / on-time GNSS delivery, a
transient LiDAR failure with map-only recovery, and an expired 300 ms map
control. They retain existing authored map points and object geometry, 25-second
goal / eight-second fault deadlines and the 1 m clearance floor. Eighteen
enabled goal episodes must meet whole-episode 0.2 m RMSE, 0.4 m maximum position
error and 0.1 rad yaw gates. The uninterrupted cases must drive at least 10 m
above 2 m/s during GNSS acquisition loss [3,8). The transient case must actually
stop below 0.05 m/s during [4.8,5.5), recover using new map acquisitions before
GNSS returns, and reach the goal. Six expired-map fault episodes must retain
braking at the original 0.75-second accepted-GNSS age bound and stop physically. Six paired controls disable
only delayed map admission and must fail the moving-outage gate by stopping;
their whole-episode errors and deadline results remain reported.

The Python auditor reconstructs the complete observation history without
producer checkpoints. Independent joint-matrix Joseph updates and pivoted
Gauss-Jordan inversion verify acquisition priors, current pose/covariance,
original GNSS NIS/count ownership, replay counters and outage permission.
Registration outputs are its map measurements; exact raw sensor replay checks
the fitter separately. This is an independent temporal-fusion calculation,
not a second scan-registration implementation. Truth is read only for physical
error, stopping, recovery motion and swept-clearance scoring.
[Recorded results and provenance](../assets/delayed-map-results.json).

All first frozen trials pass their respective gates: 18 enabled goal episodes,
six expired-map stop controls and six synchronous-only comparison controls.
The goal episodes have maximum RMSE 0.026149 m, maximum position error
0.182380 m and latest completion 20.3 s. All six transient-failure episodes
physically stop and regain at least 2 m/s with map support before GNSS returns.
Minimum independently swept clearance across all 30 episodes is above 2.419 m.
The full 10,564-tick audit reconstructs 2,511 subsequent map corrections and
383 subsequent GNSS corrections, with maximum pose/prior differences below
1.5e-14. No producer, world or gate repair was needed after the first trials.

All synchronous-only controls eventually reach the goal after genuine GNSS
returns; their failure concerns the frozen continuous-motion gate during the
outage. Disabling only delayed map admission in a recorded configuration makes
raw replay reject at the first delayed map delivery, 0.1 s, after removing the
new enrichment fields from expected outputs to avoid a presence-only mismatch.
The 72 preceding feature logs still reproduce, and all 245 standard outputs
remain byte-identical. Required checks pass 654 workspace Rust tests, 34 Python
tests and 49 standard run/replay pairs; the separate native suite passes 58 tests.
Comparative maturity remains about 21%, with the 30% waypoint unmet.
