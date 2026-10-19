# Acquisition-time GNSS correction

`PipelineConfig.gnss_latency_compensation: true` enables a bounded fixed-lag
correction in the operational driving pipeline. A delayed GNSS position is
gated and corrected at its original acquisition time, followed by replay of
the wheel/gyro prediction intervals up to the current control time. Both the
reference and native RNE adapters use this shared implementation.

Native dynamic mode also records the engine's declared chassis/rear-axle
reference and enables its existing local geometry/course-based steering model.
Absent supplied motion limits, it uses the adapter's research calibration:
2.0 m/s² acceleration and 2.5 m/s² braking/lateral authority, further bounded
by supplied friction. This is declared native-plant geometry, not measured
vehicle calibration. Native comparisons include this calibrated enabled mode;
reference comparisons isolate GNSS acquisition-time correction.

`GnssHistory` in the independent localization crate retains at most 0.5 seconds
and 64 prediction intervals. Each interval stores its prior state/covariance,
accepted measured motion and chassis offset. An acquisition inside an interval
splits that interval. The original joint innovation threshold of 36 and Joseph
correction remain unchanged. The optional chassis prediction uses its existing
midpoint integration and measured yaw rate. Replay recalculates the subsequent
pose and covariance; it does not shift a raw GNSS position by simulator motion.

Only strictly newer acquisition stamps can correct the filter. Invalid or
future stamps, duplicate/reordered observations, unavailable history and
clock gaps cannot fabricate motion coverage. An unavailable interval activates
the existing invalid-GNSS braking latch; recovery requires an accepted acquisition
newer than that fault's delivery epoch. Accepted freshness remains the original
GNSS acquisition time, with the existing 0.75-second health limit. Internal
replay never increments observation counts or refreshes the acquisition clock.

The pipeline publishes optional `gnss_latency` diagnostics: compensated fix
count, unavailable-history count, current interval count and last accepted
delay. With the option disabled, neither the configuration field nor these
diagnostics is serialized. Enabled zero-delay operation preserves every
preceding output bit except the added diagnostics.

The mode requires the pipeline's validated wheel/gyro samples and common
monotonic clock. It replays the motion intervals actually used by the pipeline;
it does not implement delayed-odometry fusion, clock-offset estimation or
per-point LiDAR deskew. Synchronous accepted fixed-map pose/covariance events
can now enter the same bounded history. Conflicts refuse the whole GNSS
transaction and brake without renewing either observation clock.
[Combined map-event contract](gnss-map-history.md). Low-level users must use
the history wrapper for map corrections, or reset after other external updates.

The simulator's optional `sensor_timing.gnss_delay_ticks` delays only delivery,
keeping raw GNSS values, variance and acquisition stamps intact. It is
independent of LiDAR failure queues. A tick is 0.05 s, with transport delays
up to 20 ticks admitted for positive and negative experiments; the correction
limit remains 0.5 s. Transport schedules, bias/dropout labels and evaluation
truth never enter pipeline configuration. All affected physical ticks are
recorded for independent scoring.

## Reproduction and evidence

```sh
source scripts/env.sh
bash scripts/check.sh
python3 scripts/check_gnss_latency.py \
  --output artifacts/gnss-latency --seeds 1 7 42

# Optional native adapter, with its documented pinned engine checkout:
cargo +1.95.0 build --manifest-path integrations/rne/Cargo.toml --locked --release
python3 scripts/check_gnss_latency.py --plant rne-dynamic \
  --binary integrations/rne/target/release/rustdriving-rne \
  --output artifacts/gnss-latency-native --seeds 1 7 42
```

Use fresh output directories. Five worlds were frozen before physical results:
straight, curved, faster curved, GNSS dropout and GNSS outliers. They keep a
45-second deadline, 1 m clearance floor, 6/8 m/s cruise and 200/400 ms delivery
delays. The fixed accuracy gates are RMSE at most 0.2 m and maximum error at
most 0.4 m. Fault worlds additionally require a physical stop and healthy
recovery to the goal. Each control disables only compensation in the same
world; the required paired RMSE reduction is at least 60%. Two subsequent
composition worlds were frozen separately before their first trials: a 4 m/s
curved-road obstacle pass with a 45-second deadline, and the original reactive
lead-stop/resume world retaining its 65-second deadline and stop outcome. Both
use 400 ms GNSS delay and the same 0.2/0.4 m accuracy and 1 m clearance gates.
Lead following must physically stop below 0.05 m/s during 14–24 s and regain
at least 2 m/s after 25 s. These cases are included in the commands above.

The Python auditor rebuilds the complete delivered past in acquisition order
without importing producer history snapshots. It uses an explicit joint 2D
inverse and Joseph matrix update rather than the producer's whitening and
sequential scalar update. It checks every pose, speed, covariance trace,
innovation and acquisition/count diagnostic, and separately reconstructs
physical errors and swept clearance. Controls are reconstructed at delivery
time. Closed-form, motion and report-corruption tests check the auditor.

The [recorded result](../assets/gnss-latency-results.json) retains all 42 positive
episodes and 18 uncompensated negative controls. All positive episodes pass
physical, accuracy, independent reconstruction and exact replay checks.
Maximum positive RMSE is below 0.089 m; maximum position error is below 0.253 m.
Across 18 paired comparisons, RMSE improves by at least 92.7%. All 18 uncompensated
controls fail the preceding 1 m maximum-error gate, with their original
failures preserved. The parent binary exactly reproduces every control output.

The first three native obstacle passes failed the unchanged 0.4 m accuracy
gate, reaching 0.647–0.686 m error despite collision-free physical passes.
Their recordings remain rejected. Applying the declared chassis reference,
compatible local geometry and explicit motion calibration together repairs
those cases to at most 0.253 m, retaining at least 1.227 m independently swept
clearance across both plants. Existing configuration guards remain in force;
rejected intermediate calibration configurations are retained separately.
All 21 affected native episodes are rerun with the same worlds and gates.

Disabling compensation in a recorded configuration makes replay reject at the
first delayed acquisition, 0.4 s. The negative input removes only the added
latency diagnostic fields from expected outputs to avoid a presence-only
mismatch; all raw observations and expected poses/covariances stay unchanged.

These are authored sensor-only reference/RNE driving measurements. They close
a demonstrated temporal-fusion error in the integrated loop. Real automotive
timing distributions, independent covariance calibration, external correction
replay and hardware performance remain unqualified.
