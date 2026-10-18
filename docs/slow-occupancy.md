# Occupancy while slow motion is unconfirmed

The experimental slow-motion predictor previously froze every forecast position
when its observation history did not confirm motion. A moving 0.5 m/s track
could lose confirmation on an intermediate turn, stop, backward position
increment, missing sample or reordered acquisition. That withdrawal correctly
removed confirmation, but a single static forecast omitted continued motion.
The preceding observation-window change reduced the mirrored crossing's seed-42
physical clearance from 1.276578 m to 1.179197 m. Both exceeded the unchanged
1 m authored acceptance floor; the loss still warranted a forecast correction.

## Implemented behavior

Only `ObservedBraking::with_slow_motion()` and `confirm_slow_motion: true` enable
the new occupancy. The default remains off. At current measured speeds from
0.1 through less than 0.7 m/s without confirmed history, let `p` be the acquired
position, `v` measured velocity, and `elapsed = acquisition_age + future_offset`.
The center is `p + v * elapsed / 2`; the additional radius is
`|v| * elapsed / 2`. Adding the measured physical radius covers a body remaining
at `p`, continuing at `v`, or occupying any position between these hypotheses.
The triangle inequality establishes this geometric coverage. No simulator truth,
actor labels, schedules, goal residence or future observations enter it.

The forecast still has 41 knots at 0.2 s spacing, an eight-second horizon, and
acquisition-anchored timestamps. Cached calls add no confirming acquisitions.
Contradictory histories retain the stricter observation-window vetoes. An
unconfirmed envelope never enables observed braking. Confirmed slow motion and
faster tracks keep their existing motion/braking hypotheses. Measured speed
below 0.1 m/s retains the established static deadband.

`Prediction.additional_radius_m` is an optional JSON sequence, omitted when
empty. Empty means zero extra radius; otherwise it must have one finite,
nonnegative value per position, with finite total occupied radii. Values
interpolate linearly and hold their final value after the horizon, matching the
position contract. Existing JSON without the field remains readable. Rust
struct-literal callers now supply an empty vector for the previous footprint.
Malformed radius sequences fail closed at planning and behavior gates.

The trajectory collision sweep splits at every intervening forecast knot,
including a stationary ego hold, and uses the maximum occupied endpoint radius
on each subinterval. This conservatively covers any intervening linear radius
change. Existing directional reserve and acceleration/chord inflation remain.
Intersection sweeps and lane-change entry use the same interval radii. Spatial
return/resume/launch previews subtract the maximum radius for each object
segment. Frame changes rotate/translate centers while retaining scalar radii.

## Measured comparison

The parent is local commit `8b14368`. World, route, vehicle, initial conditions,
sensor schedules, seeds, 1 m clearance floors, deadlines and goal holds are
unchanged. Additional episodes reuse the preceding experiment's scenario input
bytes: six slow/static cases at seeds 1/7/42, three ordinary regressions at
1/42, five deadline variants, and two known short-range failures. The sixth
deadline variant is part of the 49 required standard runs. The standard mode
remains the default. Machine-readable final counts and source/evidence hashes
are in [the result receipt](slow-occupancy-results.json); retained local inputs,
parent binary, failed parent regressions, logs and compressed current evidence
are under `artifacts/slow-occupancy-v1`.

| Condition | Parent minimum clearance | Current minimum clearance |
| --- | ---: | ---: |
| slow crossing, seed 42 | 1.256404 m | 1.311320 m |
| mirrored slow crossing, seed 42 | 1.179197 m | 1.257435 m |
| mirrored slow crossing, seed 7 | 1.231514 m | 1.399029 m |
| near-deadband crossing, seed 42 | 1.316521 m | 1.314783 m |

The smallest clearance change is -0.001738 m in the near-deadband seed-42 case;
no universal monotonic improvement is claimed. All six existing deadline
conditions still arrive by 65 s and sustain the eight-second terminal hold.
The short-range seeds 1/42 retain clearances 0.296276/0.303729 m and fail their
original acceptance conditions; they remain visible negative evidence.

Geometric tests sample stopped, moving and intermediate positions at/between
knots across rotated directions, speeds and acquisition ages. Other regressions
exercise growing and interior-peaked radii during a hold, malformed sequences,
intersection/lane entry, return preview and exact sensor replay. Python audits
independently reconstruct occupied centers/radii and reject missing, shrunken,
negative, nonfinite and misaligned envelopes. These authored arithmetic tests
are distinct from the physical simulation episodes and external datasets.

## Limits

This is a bounded geometric hypothesis for measured direction and speed, not a
calibrated covariance, probability, reachability proof or guarantee against
unobserved acceleration/turns. At eight seconds the endpoint occupancy is held;
the method does not extend physical coverage indefinitely. Confirmed motion
can still later stop, and the sub-0.1 m/s deadband remains. More conservative
occupancy can delay progress. The evaluation is limited to authored reference
simulations; no KITTI, CARLA, native RNE, real-vehicle, cross-platform or real-time
qualification was performed here. Broader external forecast-quality evidence is
still required before enabling slow motion by default.
