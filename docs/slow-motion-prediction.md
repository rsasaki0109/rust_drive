# Forecasting observed slow motion

The shared pipeline offers experimental extrapolation of sustained observed motion
below 0.7 m/s. Set `confirm_slow_motion: true` in pipeline configuration or a
simulation scenario, or construct `ObservedBraking::with_slow_motion()` directly.
The default is **off**, pending broader forecast and adoption qualification. The
original experiment missed the existing 65-second reactive-follower deadline;
the later [terminal-settling correction](terminal-settling.md) repairs the tested
deadline cases without changing the predictor. Directional crossing
reserve is strengthened in the default planner independently of this opt-in.
The established speed deadband freezes every forecast point at the last measured
position, even after a long sequence of consistent moving observations. A slow
crossing could therefore enter the vehicle's route without appearing in its
future swept occupancy.

`ObservedBraking` uses its existing continuous-frame history, bounded to 0.8 s
and 32 observations per present identity. At speeds from 0.1 through less than
0.7 m/s, it requires observations near 0.2, 0.4 and 0.6 s before the latest
acquisition, each within 25 ms. Each earlier velocity must have speed at least
0.1 m/s and direction cosine at least 0.95 relative to the latest velocity.
Each chronological position interval must show forward progress of at least
0.1 m/s and displacement direction cosine at least 0.9. A 1e-9 positional/time
tolerance handles floating-point comparisons.

The [complete observation-window check](prediction-observation-window.md) also
examines every intervening acquisition. A measured stop or changed velocity
direction withdraws confirmation even when all four spaced anchors agree.
Adjacent position increments must retain displacement-direction cosine at least
0.9; their roundoff tolerance is 1e-9 m. A zero increment is allowed, while the
spaced intervals still require the minimum 0.1 m/s forward progress. Only
distinct acquisition timestamps contribute evidence.

Confirmed motion uses the measured velocity, beginning at acquisition age plus
future offset. It retains the 41 samples, 0.2 s spacing and eight-second horizon.
Confirmed slow motion may also use the existing sustained-braking hypothesis:
three aligned intervals must each show at least 0.3 m/s² deceleration, half the
weakest is used for at most one second from acquisition, and fresh reacceleration
or observation age over 0.15 s withdraws braking. Every intervening velocity
must also agree with the current direction and contain no measured speed increase
beyond 1e-9 m/s. This avoids a discontinuity
when a braking track crosses below 0.7 m/s. Missing motion support,
contradictory positions, changed directions, an observed stop or an absent identity
withdraw confirmation; duplicate control calls cannot supply observations. This
introduces roughly 0.6 s of required evidence after measured motion is established.
The default `ObservedBraking` and stateless `ConstantVelocity` retain their
original deadband. The replay header records the explicit opt-in; omitting it
selects the established policy, and changing it invalidates affected expectations.

## Unconfirmed occupancy

With the experimental option enabled, a current speed from 0.1 to less than
0.7 m/s that lacks confirmation now retains both a stopped and a continuing
hypothesis. The per-knot center lies halfway from the acquired position toward
constant-velocity continuation; an additional radius of half that displacement
covers both endpoints and intermediate positions. Acquisition age is included.
An actual zero/sub-0.1 m/s observation retains the static deadband. Confirmation
still requires every observation-window check above; unsupported histories do
not authorize the braking hypothesis. See [the occupancy contract and measured
comparison](slow-occupancy.md). The default remains off.

## Crossing reserve

Prediction alone eliminated contacts in the new 0.5 m/s crossing, but an intermediate
implementation still had only 0.800501 m physical clearance at seed 1, failing its
fixed 1 m acceptance floor. That failed source and executable remain retained.

The circular sweep now also retains an empirical direction-based reserve. Let
`v_perp` be forecast velocity perpendicular to the candidate tangent and let
`q = |v_perp| / |v|`, with zero for static forecasts. Its margin is:

```text
0.30 + 0.06 * min(future_seconds, 5)
     + max(0.40 * min(|v_perp|, 1), 0.70 * q)   [meters]
```

Perpendicular motion receives at least 1 m at time zero, including slow motion.
The previous transverse-speed term is retained so almost-parallel fast motion
cannot narrow its existing envelope. Static/parallel motion retains its old
reserve. Stationary path segments use the estimated heading; unavailable headings
conservatively use `q = 1` for moving forecasts. Initial points, full candidate
sweeps, retimed stops and terminal holds use the same rule. This is an empirical
simulation allowance, not calibrated covariance or a guaranteed physical clearance.

## Independent checks

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-prediction --test slow_motion
cargo test --locked -p rustdriving-pipeline --test slow_motion
cargo test --locked -p rustdriving-sim --test slow_crossing
bash scripts/check.sh
target/release/rustdriving run --scenario scenarios/slow-crossing.json \
  --seed 7 --output artifacts/slow-crossing-run
```

Analytic fixtures check 0.2/0.5/0.69 m/s motion, rotated directions, acquisition
age, cached calls, inconsistent positions/velocities, missing anchors, observation
gaps, identity loss, reordered samples and stopping. A raw delayed circular scan
fixture verifies acquisition stamps, measured velocity, forecast displacement and
exact replay. The physical regression requires genuine goal completion, no contacts
or road violations, at least 1 m swept clearance and actual use of a slow forecast
at seeds 1/7/42. Actor truth appears only in sensing synthesis and independent
physical scoring.

The arithmetic auditor reconstructs motion support from chronological measured
intervals and the recorded policy. New support and the margin change can invalidate exact expected outputs
from older algorithms. Original executables and recordings remain the way to
reproduce those expectations.

## Reference measurements

These measurements describe `d50f351` before the terminal-settling correction.
Its failed recordings and executables remain unchanged.

The required check passed **568 tests and 49 default scenario/replay pairs**.
The full current comparison covers 77 reference episodes: **74 physical passes
and three retained deadline failures**. All 77 replay exactly and pass independent
coordinate, filter-interval and forecast arithmetic audits, covering 58,704 control
steps. The 26 original comparison runs remain frozen; experiment configuration
adds only the explicit forecast-policy flag to the otherwise unchanged worlds.

Five slow-crossing variants at seeds 1/7/42 cover 0.25/0.5/0.65 m/s, opposite-side
motion and delayed starts. All 15 experimental runs complete with zero contacts,
zero road violations and at least **1.227 m** swept clearance against the fixed
1 m floor. Three experimental static controls have zero confirmed-slow forecasts.
For the public 0.5 m/s crossing:

| Seed | Original clearance | Experimental clearance | Original / experimental contact steps |
| --- | ---: | ---: | ---: |
| 1 | 0.112318 m | 1.259600 m | 0 / 0 |
| 7 | -0.486477 m | 1.258900 m | 104 / 0 |
| 42 | 0.220335 m | 1.335643 m | 0 / 0 |

The default directional reserve also improves opposing-crossing clearance at
seed 1 from 1.100164 to 1.313319 m and at seed 42 from 1.079032 to 1.315262 m.
At the required seed 7 it improves from 1.058300 to 1.374229 m.
Among the 74 passing comparisons, none has lower clearance than its original
parent result. This is a measured property of these fixtures, not a general bound.

The follower-deadline adoption check retains the original road, 65-second limit,
eight-second physical residence and 1 m clearance floor:

| Seed | Default prediction | Experimental slow prediction |
| --- | --- | --- |
| 1 | Pass at 63.85 s | Fail: residence incomplete at 65 s |
| 42 | Existing failure at 65 s | Fail at 65 s |

All three failed current runs have zero contacts/road violations and at least
2.998 m clearance. They remain **failed completion results**. The seed-1
regression blocks automatic adoption of slow prediction; default mode preserves
the original passing seed-1 outcome. This is distinct from the existing seed-42
default failure.

Local `artifacts/slow-motion-v1` retains original and intermediate failed sources,
executables, physical recordings, mode comparisons, five replay/corruption checks,
and lossless recording indexes. The 245 historical live outputs were restored
byte-for-byte; 29 of 245 parent-baseline outputs change under the current default.
Debug symbols were disabled for local dev/test builds to bound generated storage;
the pinned compiler, lockfile and release settings were retained. No failed result
is relabeled as a physical pass.

## Adoption limit

A slow follower can remain on course toward the stopped vehicle under a single
constant-velocity hypothesis, even when the simulated actor will subsequently
stop. Continuing the existing bounded braking model into confirmed slow motion
reduces that discontinuity, but does not establish its eventual stopping intent.
The original repeated stops/restarts exceeded the retained 65-second deadline
despite collision-free physical clearance. Terminal settling repairs the tested
completion regressions by maintaining an already reached stopping place;
it does not infer the follower's intent or relax a rejected hold. The original
failed runs and experimental regression are preserved. Automatic default adoption
remains withheld: these authored fixtures do not qualify forecast error across
general traffic. The experiment does not supply interaction-aware following or
general terminal negotiation.

There is no semantic pedestrian detection or intent model. Motion below 0.1 m/s,
unobserved starts/turns, shape/occlusion errors, insufficient timing support and
uncalibrated motion uncertainty remain limitations. No external automotive accuracy,
real-vehicle, RNE or real-time qualification is supplied by this extension.
