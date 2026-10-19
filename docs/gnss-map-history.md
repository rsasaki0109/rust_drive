# Delayed GNSS with synchronous map corrections

`gnss_latency_compensation: true` can now be combined with `localization_map`.
Previously this configuration was rejected because rewinding GNSS would erase
intervening map updates. The shared filter now retains accepted map poses and
their full measurement covariances alongside measured wheel/gyro intervals.
GNSS is corrected at acquisition time, then both kinds of event are replayed
to the current clock. Equal-time GNSS precedes the map update. Fractional
motion intervals and both declared chassis models are supported.

History remains bounded to 0.5 seconds, 64 motion intervals and 64 accepted
map measurements. No simulator truth, actor identity or dropout schedule enters
the filter. The authored offline map is unchanged from the preceding map
fixtures; this is not a measured automotive survey.

Every replayed map correction must pass its ordinary joint XY/body-yaw EKF
gate again. If a delayed GNSS fix would invalidate an originally accepted map
measurement, the entire GNSS transaction is refused: current pose, covariance,
history and accepted/observed timestamps remain unchanged. The pipeline records
`history_conflict_fixes`, brakes through its existing invalid-GNSS latch, and
requires a covered acquisition newer than that fault to recover. It never
silently drops an accepted map update or revives a rejected one.

Map registration is performed once on the original synchronous scan. Replay
uses that fitted pose and covariance; it does not rerun correspondences after
GNSS changes an earlier estimate. Conditional measurement reuse and the fixed
covariance floors remain research assumptions. An inconsistent transaction
stops conservatively rather than attempting global relocalization.

Neither map replay nor GNSS delivery renews acquisition freshness or increments
original observation counts. Map support still needs a genuine initial GNSS
fix, a fresh accepted synchronous scan, and no more than the configured ten
seconds since that actual fix. Existing GNSS innovation holds, fault epochs,
odometry/LiDAR freshness and covariance guards still apply. Absolute map/GNSS
corrections do not enter the continuous wheel/gyro tracking history.

Additional optional diagnostics count retained map corrections, successfully
replayed map corrections and refused conflicting fixes. Zero-valued new fields
are omitted for compatibility with preceding map-free delayed-GNSS logs.
The mode remains opt-in. Delayed map scans, delayed odometry, clock-offset
estimation, global localization and real vehicle integration are unsupported.

```sh
source scripts/env.sh
bash scripts/check.sh
python3 scripts/check_gnss_map_history.py \
  --output artifacts/gnss-map-history --seeds 1 7 42

# Optional adapter with its separately pinned engine and Rust toolchain:
cargo +1.95.0 build --manifest-path integrations/rne/Cargo.toml --release --locked
python3 scripts/check_gnss_map_history.py --plant rne-dynamic \
  --binary integrations/rne/target/release/rustdriving-rne \
  --output artifacts/gnss-map-history-native --seeds 1 7 42
```

Use fresh output directories. Five scenarios preserve the preceding authored
worlds, map points, deadlines and 1 m clearance floor, adding only the enabled
fusion mode and 400 ms GNSS delivery delay. Across seeds 1/7/42 and both plants,
six recovery episodes must drive at least 10 m during the five-second GNSS
outage, remain above 2 m/s, regain actual GNSS and reach the goal within the
original 25 seconds. Whole recovery accuracy gates are 0.2 m RMSE, 0.4 m maximum
position error and 0.1 rad yaw error. Twenty-four fault controls require a
physical stop for excessive outage, lost LiDAR, an unrelated map or a disabled
map. Fault accuracy is gated while accepted map matches are fresh; whole
fault-episode errors remain reported without a general accuracy claim.

The separate Python auditor rebuilds the complete delivered past, using joint
matrix Joseph updates and pivoted Gauss-Jordan inversion instead of producer
history snapshots and Cholesky. It verifies each pose, speed, covariance trace,
GNSS innovation, accepted timestamp/count, replay count and outage permission.
Its map measurements are the recorded registration outputs; it is an independent
temporal-fusion oracle, not a second scan-registration implementation. Full raw
sensor replay separately checks registration and downstream outputs. Physical
truth is read only to score error, driven distance and swept clearance.

The first audit wrongly counted normal goal-approach stops after the recovery
outage as stops inside that outage. Its failed results and original auditor
are retained. The corrected audit restricts that one check to the fixed [3,8)
acquisition outage, preserving producer code, recordings and all frozen gates.
[Recorded qualification and provenance](../assets/gnss-map-history-results.json).

All 30 episodes pass exact replay and the repaired independent audit, covering
7,488 physical/control ticks, 747 delayed corrections and 2,268 map-event
replays. The six moving recovery episodes have maximum RMSE 0.005513 m and
maximum position error 0.018920 m; each drives more than 14.84 m during the
outage. Minimum independently swept clearance across all episodes exceeds
2.449 m. Prolonged loss brakes at 12.85 s, LiDAR loss at 6.15 s, and unavailable
map support at 3.6 s, preserving the original acquisition-age limits. Disabling
compensation in an otherwise unchanged recorded input makes replay reject at
the first delayed fix, 0.4 s, even after removing the added latency diagnostics
from expected outputs. The 42 preceding delayed-GNSS logs also reproduce.

This closes a concrete simulation integration gap. Comparative maturity stays
about **21%**: automotive input validation, calibrated uncertainty, sustained
localization and broader operational evidence remain open, as does the 30%
waypoint.
