# Track velocity after missed detections

The LiDAR alpha-beta tracker now corrects velocity using the time between the
two observations of **that track**. Empty scans and measurements of other
objects do not shorten this interval or renew its `last_seen` stamp.

Previously, prediction used the track's observation interval, but velocity
correction divided its residual by the interval between consecutive scans.
For example, detections at 0 and 0.5 s separated by four empty 10 Hz scans
produced five times the intended velocity correction. The same error occurred
when other objects remained visible between those detections.

The position gain remains 0.75 and the velocity gain 0.12. The velocity
denominator retains its 0.01 s floor; inferred speed remains capped at 15 m/s.
Tracks still expire when an accepted scan is at least 0.6 s after their last
observation. A reappearing object after expiry gets a new identity and zero
initial velocity. There is no prediction-only renewal during a long wait.
Association, circle fitting and the prediction model are unchanged.

The public `Tracker::update` contract requires successful scans in strictly
increasing acquisition-time order. The driving pipeline validates this order
and transforms points at acquisition time before calling the tracker. Delayed
delivery does not change the measured interval. Forecasts subsequently advance
the resulting track from its acquisition time to the control clock.

## Reproduce the arithmetic audit

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-perception --test tracker_intervals
cargo test --locked -p rustdriving-pipeline --test tracker_reacquisition
bash scripts/check.sh
python3 scripts/check_tracker_intervals.py \
  --log artifacts/check/*/sensors.jsonl --output artifacts/tracker-interval-audit.json
```

The auditor streams complete schema-1 logs and checks each correction using
consecutive positions and original observation timestamps, independently of the
reported new velocity. It also checks that cached tracks remain unchanged and
new identities have zero velocity. It refuses an existing output. This checks
filter timing, without independently estimating cluster centers or qualifying
object identities or physical velocity accuracy.

Regression checks compare identical object measurements with and without
intervening empty scans or other detections. Another scan updates two tracks
whose observation intervals differ. A 200-second empty wait checks expiry.
The pipeline check uses known circular point sets, a delayed reacquisition,
hand-calculated track/forecast values and exact replay.

Old source, executable and failed regression results remain in the local
evidence packet. Historical recordings remain readable, but recordings affected
by the old correction can mismatch when replayed by the updated algorithm.
Use the original binary to reproduce those expectations; regenerating current
expectations does not migrate or repair their historical outputs.

This repair supplies no occlusion-aware free-space evidence, uncertainty model,
global assignment or independent automotive tracking benchmark. The existing
0.6 s expiry, greedy association and low-speed forecast deadband retain their
limitations. It establishes no real-vehicle or real-time capability.
