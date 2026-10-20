# Settling a collision stop without repeated restarts

Optional `settle_obstruction_stops: true` keeps a completed obstruction stop
until a complete nominal candidate clears the ordinary collision sweep. It is
off by default, recorded in the sensor-only header, and independent of the
experimental `forecast_envelope` policy. Neither forecast positions/radii nor
the existing collision margins and eight-second hold checks are reduced.

The preceding [envelope experiment](forecast-envelope.md) exposes the trigger:
after an empty candidate set brakes the vehicle, a slower measured state makes a
retimed partial approach feasible again. Its speed reward requests acceleration;
the next changed arrival time can reject that candidate and brake again. The
reference blocked-road seed-42 run still moves at 0.4 m/s at its deadline.

## State and release contract

A planner emergency caused by an observed candidate collision records a pending
collision brake. The moving approach retains the previous behavior. Once a
subsequent valid estimate is at most 0.2 m/s, a blocked candidate must brake
immediately to rest and include the complete eight-second stationary hold.
Estimated pose, wheel-derived speed and predicted occupancy are the only inputs;
physical actor labels, simulator schedules and acceptance truth do not enter
this decision.

The stop uses the calibrated braking authority and preserves its measured
initial speed. At extremely small positive speeds, a minimum 1e-7 m segment
keeps braking and the full hold representable. A conflicting hold remains
infeasible and commands the existing emergency response. Invalid inputs cannot
arm a new collision brake or withdraw an existing one.

A **complete nominal path** that passes the unchanged sweep can release the
brake, including a safe lateral escape. A retimed stop before the same
obstruction cannot release it. During settling, complete clear paths precede
partial blocked paths in candidate selection; the original geometric, steering,
speed, road-containment and collision checks remain mandatory. Ordinary candidate
costs are unchanged before settling. Goal settling and regulatory stop prefixes
retain their existing rules.

While the stop remains active, the pipeline retains at least -0.5 m/s² requested
braking so low-speed feedback noise cannot restart the vehicle. Unhealthy sensing
still commands -6 m/s². Route-dependent maneuver resets preserve pending/active
obstruction braking and re-evaluate it against the new route. A new localization
epoch creates fresh planner state; preserving this hold across measured
relocalization is outside this increment.

Release uses the ordinary fresh sensor/forecast contract, without an additional
clear-scan dwell. The existing endpoint-hold convention after eight forecast
seconds is unchanged. This is simulation stop settling, without calibrated
uncertainty, actor intent or a guarantee against arbitrary future motion.

## Fixed regression protocol

The cohort freezes **56 episodes** before the first new physical trial. Forty
reuse the preceding five authored worlds, seeds 7/42 and both reference/native
plants, with the new stop policy enabled. They retain both the ordinary and
experimental envelope forecast modes. Their forty parent recordings remain
unchanged controls, with all source, localization and coverage failures retained.

Sixteen new paired episodes add two straight blocked-road follow-ups: the actor
starts moving laterally at 2 m/s at time 12 s; the second additionally withholds
GNSS acquisitions from 12 to 15 s. Both use the same envelope budgets, geometry,
sensing, 65 s goal deadline and full-rate recording. Enabled/disabled pairs differ
only in `settle_obstruction_stops`.

The fixed physical floor is 1 m with no contacts/road/rule violations. Actual
speed must stay at most 0.05 m/s over 14–20 s in permanently blocked inputs and
10–11.5 s before the moving release. The GNSS-loss case also requires actual
standstill from 13.5 through 14.95 s. A clear-path restart must occur by 22 s and
complete the goal within the unchanged deadline. At least ten genuine held
control ticks exercise each enabled targeted case. Source acceptance and every
preceding 0.3 m RMSE / 0.6 m position / 0.1 rad yaw / 99% forecast-coverage gate
remain separately scored.

The independent contract auditor folds healthy emergency observations, the
measured low-speed threshold and complete versus partial route progress. No
producer latch or phase diagnostic seeds its state. Partial stopped candidates
must remain monotonic in speed, keep the eight-second tail, and retain braking.
Every admitted trajectory in these straight targeted inputs receives a separate
collision certificate: it integrates the constant-acceleration ego path, splits
at forecast knots, and recursively bounds clearance using relative speed and
radius-growth Lipschitz constants. A contact witness or an unresolved bound
fails verification. This avoids depending on the Rust temporal-chord method.

The regression criterion requires no new failed acceptance gate relative to
preceding worlds, plus the new hold/release requirements. It does **not** turn
old incomplete goals, heading errors or uncalibrated coverage into complete
qualification. Disabled controls retain their measured violations. These are
viewed authored regressions, without held-out or automotive qualification.

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-planning --test obstruction_stops
cargo test --locked -p rustdriving-pipeline --test obstruction_stops
python3 scripts/test_obstruction_stops.py
# Requires built root/native release executables; fresh output directory.
python3 scripts/check_obstruction_stops.py --check-policy-regression \
  --output artifacts/obstruction-stops
bash scripts/check.sh
```

`--check-policy-regression` selects the explicitly separate policy regression
exit condition. Omitting it requires complete qualification, including the
preceding forecast/accuracy gates. Both forms write all pass/fail outcomes and
losslessly verified raw XZ streams with encoded/decoded SHA-256 receipts.

## Recorded results

The [complete result asset](../assets/obstruction-stop-results.json) retains the
frozen 56 inputs, all row-level failures, source/binary hashes and archive
receipts. No new physical trial or acceptance threshold was discarded or
adjusted after viewing these outcomes.

| Check | Recorded result |
|---|---|
| Enabled policy regression | 48/48 pass |
| Permanent envelope-obstruction hold | 4/4 at actual 0 m/s throughout 14–20 s |
| Enabled moving-obstruction / GNSS-fault goals | 8/8, finishing in 26.95–32.2 s |
| Disabled controls failing the no-restart contract | 8/8; actual hold-window speeds reach 0.6–0.7 m/s |
| Original forty source acceptance outcomes | 36/40 pass, up from 35/40 |
| Complete forecast/accuracy/mission qualification | 0/56 pass; inherited failures remain scored |
| Physical separation | No contacts or road/rule violations; minimum swept clearance 1.175659 m |

All enabled release cases remain at actual 0 m/s over 10–11.5 s. Healthy cases
request motion at 12.6 s after the actor moves; GNSS-loss cases hold actual
0 m/s over 13.5–14.95 s and request motion when GNSS recovers at 15 s. The
independent auditor certifies 671,536 clearance interval bounds across the
targeted enabled and disabled trajectories. The disabled controls retain 712
no-restart violation ticks rather than counting as positive policy passes.

Every new physical recording replays exactly: 56 episodes and 42,447 control
ticks. All 280 stored streams pass encoded and decoded hash verification.
Independent full-rate circular road checks find zero violations and a minimum
0.575006 m corridor margin. The preceding forty forecast recordings also retain
identical replay outputs over 32,948 ticks with the new policy omitted.

The exact required `bash scripts/check.sh` passes 790 Rust tests, 106 Python
tests and 49 standard drive/replay pairs; all 245 standard output files remain
byte-identical. The native adapter additionally passes 63 tests, formatting,
release build and strict Clippy. These are local Linux results.

The four preceding envelope missions still miss their unchanged deadlines;
native launch-heading accuracy and uncalibrated coverage also remain failed.
The new stop policy stays off by default, independently of the envelope option.
Comparative engineering maturity remains **about 29%**; automotive qualification
and the 30% waypoint remain open.
