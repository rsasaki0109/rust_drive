# Rear headway for expanding predicted occupancy

The experimental slow-motion envelope covers an acquired stopped position and
constant-velocity continuation, using a midpoint center and a growing circular
radius. The preceding implementation used the expanded radius for geometric
sweeps but derived rear headway only from the center's velocity. An envelope
could consequently admit a rear gap that its contained continuation rejected.

## Implemented correction

For each prediction interval, project center motion onto the target lane's
forward axis. Add the positive change of additional occupied radius divided by
the interval duration. The maximum nonnegative sum supplies the existing
rear-approach headway. A contracting radius never reduces the center-speed
allowance. Backward motion offsets growth when the forward occupied edge remains
stationary. Nonfinite or over-30 m/s approach bounds retain fail-closed behavior.

The helper retains the 6 s forecast sweep, 6 m minimum gap, 2 s headway,
per-interval maximum occupied radius, lateral expansion and one-second clear
acquisition requirement. The same correction reaches the entry gate and optional
return/resume/launch checks through their existing shared helper. No new policy,
actor intent, truth input, geometry, deadline, stopping hold or acceptance floor
is supplied to the driver. Experimental slow-motion prediction remains off by
default. Empty additional radii preserve the original headway calculation.

A simple counterexample places ego at longitudinal 20 m and a radius-1 m object
at 7.25 m in the target lane. Ego radius is 1.25 m. A 0.6 m/s continuation closes
the 12.75 m center gap enough to block within six seconds: required center
separation is `1.25 + 1 + 6 + 0.6 * (6 + 2) = 13.05 m`. The enclosing envelope
moves its center at 0.3 m/s and grows by 0.3 m/s. Center-only headway accounted
for just 12.45 m, so the old gate admitted it. The corrected bound includes
both terms. The original failing test executable, source and log are retained.

Rust regressions also check 36 contained blocked continuations across rotated
and translated axes, preserve a clear receding envelope, and verify that new
rear occupancy revokes `Ready`, clears its timer and requires new clear scans.
These are authored geometric/state tests, separate from physical simulation.

## Independent observation and replay diagnostics

Twelve predeclared raw-circle inputs exercise the actual sensing, tracking,
prediction, behavior and replay pipeline. Ego's reported pose remains at
(20, 0) m. Each input has 81 control ticks at 20 Hz, 32 circular returns at 10 Hz,
body radius 0.7 m, forward motion 0.5 m/s, rear initial x of 6.5/7/7.25 m, and
alternating lateral offsets 0/3/5/8 cm. Identical inputs and unchanged world
tracks/forecasts are verified between saved parent and candidate binaries.
These authored inputs hold odometry fixed despite commanded acceleration;
they are not physical driving episodes or external sensor recordings.

The independent Python audit projects the recorded measured track velocity and
samples its constant-motion continuation every 5 ms through the six-second
admission window. A strict interior conflict that is demonstrably contained in
the recorded circular envelope must appear as a blocking track, with state
`Waiting` and revoked clear time. This verifies sufficient conflict evidence;
it does not certify absence of a conflict or reproduce the Rust segment clipper.

The parent misses 149 contained conflicts across the nine jittered inputs; the
candidate misses zero across 324 contained witnesses. The three no-jitter
controls are unchanged. Both versions reproduce all their own 972 ticks exactly
on the release CLI. The parent accepts the three unchanged candidate controls
and rejects the nine changed candidate logs at their actual behavior mismatch,
without writing a success report. Independent arithmetic-audit regressions run
in `scripts/check.sh` and the existing CI matrix; no hosted CI run is claimed.

## Physical comparison and reproducibility

Six scenarios are frozen before paired evaluation, at seeds 1/7/42: the existing
rear, release and blocked lane-gap examples, plus slow rear actors beginning at
x = -12/-10/-8 m with speed 0.5 m/s. The latter explicitly enable experimental
slow prediction. Geometry, 1.1 m actor radius, ego calibration, sensor schedule,
6 m/2 s admission rules, 1 m physical clearance floor, 120 s goal deadlines and
35 s stop deadline are retained from the established examples. Input hashes,
all paired outcomes and independent audits are recorded in
[the result receipt](lane-occupancy-headway-results.json). Every lane-change
world retains every physical tick, allowing an independent synchronized swept
clearance recomputation. Truth enters simulation/scoring only.

The local packet is `artifacts/lane-occupancy-headway-v1`. It retains the parent
`03783c6` binary and original source, failing regression, candidate binary,
frozen worlds, raw input authoring source/lock, exact CLI replay reports,
independent audits and compressed recordings with original/stored SHA-256.
The 245 older live standard outputs are restored byte-for-byte with separate
backup inodes after the required check. The result receipt distinguishes newly
executed episodes from historical evidence; previous insufficient-range failures
remain preserved and are not reclassified.

## Limits

The correction bounds approach for the supplied piecewise-linear occupancy;
it is not a prediction of unobserved acceleration, intent, reliable sensing,
free space behind occlusion or the entire maneuver. A larger uncertainty may
still reduce progress. The dense witness audit proves only sampled positive
conflicts, and its arithmetic fixtures add no automotive accuracy evidence.
The scope remains authored reference simulation and diagnostic raw points.
External automotive data, native RNE, real-vehicle, real-time and cross-platform
qualification were not performed in this change. The about-20% subjective OSS
maturity estimate and unmet 30%/50% waypoints are unchanged.
