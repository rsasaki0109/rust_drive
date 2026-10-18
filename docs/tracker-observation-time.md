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
Circle fitting and the prediction model are unchanged by the timing repair.
The subsequent [global association repair](tracker-global-assignment.md) separately
replaces greedy matching while retaining this per-track time contract.

The public tracker accepts successful scans in strictly increasing
acquisition-time order. The driving pipeline validates this order
and transforms body points into continuous acquisition-time coordinates before
calling the tracker. Delayed
delivery does not change the measured interval. Forecasts subsequently advance
the resulting track from its acquisition time to the control clock.

## Rejected acquisitions

`Tracker::try_update(&detections, acquisition_stamp)` returns
`Result<Vec<Track>, TrackerUpdateError>`. It rejects nonfinite or negative times,
duplicate/older scans, nonfinite detection centers and nonfinite or negative
radii. Errors identify a timestamp failure or the first invalid detection index.
Validation covers the whole batch before expiry or matching. A rejected batch
leaves every track, the accepted scan time and identity allocation unchanged;
repaired measurements can retry the same acquisition time. Identifier exhaustion
also rejects the complete staged update without wrapping or partially committing.

Successful empty scans advance the accepted **scan** time while retaining each
track's original observation time until it is seen again or expires. This scan
clock rejects reordered acquisitions; velocity correction continues to use the
separate per-track interval. Rejected scans advance neither clock.

The existing `update` signature remains available. On rejection it returns the
unchanged cached tracks. Callers needing to distinguish successful acquisition
from rejection must use `try_update`; an unchanged track vector is not evidence
of a successful scan.

The driving pipeline uses the checked API. A tracker rejection activates
`InvalidLidar` and emergency braking, preserving its continuous-frame tracks,
occupancy grid and last accepted LiDAR time. A localization correction can still
reproject cached world-coordinate tracks without changing their observation times;
see the [tracking frame contract](tracker-localization-frame.md).
Recovery needs a new valid scan acquired after
the latest fault control time. Other pipeline state, including localization,
continues its ordinary frame processing. This is not a transaction across the
entire pipeline.

Standalone callers supply the acquisition clock. The tracker cannot infer a
delivery time or identify a finite forward jump as a future observation; the
pipeline separately checks scan time against its control clock. The checked API
does not impose detection-count limits or establish bounded processing latency.

## Reproduce the arithmetic audit

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-perception --test tracker_intervals
cargo test --locked -p rustdriving-perception --test tracker_inputs
cargo test --locked -p rustdriving-pipeline --test tracker_reacquisition
bash scripts/check.sh
python3 scripts/check_tracker_intervals.py \
  --log artifacts/check/*/sensors.jsonl --output artifacts/tracker-interval-audit.json
```

The auditor streams complete schema-1 logs and checks each correction using
consecutive positions and original observation timestamps, independently of the
reported new velocity. It uses the optional logged `tracking_frame` transform to
compare values in continuous coordinates; older logs without it retain world-frame
arithmetic. It also checks that cached observations remain unchanged in their
tracking frame and new identities have zero velocity. It refuses an existing output. This checks
filter timing, without independently estimating cluster centers or qualifying
object identities or physical velocity accuracy.

Regression checks compare identical object measurements with and without
intervening empty scans or other detections. Another scan updates two tracks
whose observation intervals differ. A 200-second empty wait checks expiry.
The pipeline check uses known circular point sets, a delayed reacquisition,
hand-calculated track/forecast values and exact replay.

Input rejection checks retain four original failures: duplicate/reversed
corrections, loss of tracks on a NaN clock, partial invalid batches and reordered
data after a successful empty scan. They also interleave 500 rejected scans with
100 accepted scans and compare the entire result with processing only accepted
measurements. A dependency-failure unit check deliberately advances the private
tracker's scan clock ahead of the enclosing pipeline, then verifies braking,
unchanged tracks/grid/accepted-LiDAR time and post-fault recovery. This injected
internal clock disagreement is not a measured live-sensor failure or a replayable
external input sequence. Normal pipeline inputs already had clock validation.

Old source, executable and failed regression results remain in the local
evidence packet. Historical recordings remain readable, but recordings affected
by the old correction can mismatch when replayed by the updated algorithm.
Use the original binary to reproduce those expectations; regenerating current
expectations does not migrate or repair their historical outputs.

This repair supplies no occlusion-aware free-space evidence, uncertainty model,
global assignment or independent automotive tracking benchmark. The existing
0.6 s expiry, distance-only association and low-speed forecast deadband retain their
limitations. It establishes no real-vehicle or real-time capability.
