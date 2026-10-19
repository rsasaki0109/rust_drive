# Speed limits after an early-return mission retry

`lane-route --local-speed-limits --wait-for-gap --return-on-conflict
--resume-after-return` now supports one bounded retry after a measured source
stop. It uses the same original surveyed map and observed-clearance gates.
It does not create a live speed-map feed or an arbitrary new itinerary.

```sh
target/release/rustdriving lane-route \
  --map maps/lanes/resume-speeds-thin-tail.json --start source --goal target-exit \
  --change-side left --change-start 25 --change-length 30 \
  --wait-for-gap --return-on-conflict --resume-after-return --local-speed-limits \
  --cruise-speed 4 --duration 120 --scenario-output artifacts/retry/scenario.json
```

The exporter writes an empty world. The acceptance command below adds the
prescribed conflicts. Source and target permissions, widths, steering bounds,
scan/motion freshness and preview-clearance gates remain unchanged. Local
speed limits do not reduce the conservative configured-cruise traffic gap.

The original fixed-route intervals apply before a return. The explicit
`LaneChangeReturnConfig.speed_limit_m_s` caps the entire return-and-stop route.
For the retry, that same cap applies over the entire new connector. The
unchanged original target tail is then clipped at its actual join and moved
to the new arc origin: `new_s = connector_length + original_s - original_join_s`.
An interval that crosses the join is clipped; one wholly behind it is omitted.
Overlapping and centimeter-long tail intervals remain effective. Equal
adjacent caps may merge. Invalid geometry metadata and more than 128 resulting
intervals reject the candidate. No retry starts without a feasible candidate.

`LaneResumePlan::speed_zones` supplies this conversion. The retry preview,
active planner, launch-hold prefixes and subsequent release previews all use
the same rebased policy. Prefixes preserve the full retry origin. Resume
geometry and speed caps remain fixed during those launch holds; release does
not discard or recompute them from a different origin. Requested cruise is
restored after lower limits and circular rear clearance have passed.

Sensor replay retains the original intervals and alternate connector cap;
it recomputes the retry from measured inputs. No actor intentions or physical
truth are added to driving configuration. The physical evaluator scores the
preceding interval using its old policy, scores the switching instant using
the new one, then continues in the new arc frame. It never interpolates route
progress between different origins. Excess over the unchanged 0.15 m/s
tolerance fails physical acceptance.

```sh
python3 scripts/test_lane_resume_speeds.py
python3 scripts/check_lane_resume_speeds.py \
  --output artifacts/lane-resume-speeds --seeds 1 7 42
```

Use a fresh directory. Six fixed reference worlds cover left, right and
northbound retries, a 1 cm low-speed successor, a permanently blocked target,
and a new conflict during launch. Every local-policy case must return and
physically hold inside the source for at least one second. Four must finish
the original mission and reach at least 3.5 m/s in a fixed post-limit probe.
The remaining cases require a continued source stop or launch hold. Every
case retains the 1 m swept circular-clearance floor and exact sensor replay.

The Python scorer reconstructs the new Bernstein connector independently,
computes its arc length, clips original-map boundaries and evaluates every
physical integration interval. Goal runs must exercise all rebased intervals.
Arithmetic negative controls demonstrate that reusing old distances misses
the new tail's physical speed violation. Separate option-free compatibility
controls use the same maps with empty worlds and the legacy mission-wide
cruise minimum; they do not claim equivalent traffic timing under local caps.

The first frozen seeds 1/7/42 trial passes all 18 local-policy episodes,
including 12 completed missions, three blocked-target stops and three
persistent launch holds. It retains 35,024 physical frames. Maximum speed
excess is 0.09003 m/s, minimum swept clearance is 7.52868 m, and every completed
mission reaches at least 4.01478 m/s in the post-limit probe. Source return
standstill lasts at least 22.35 s. Six empty-world option-free runs match their
six parent runs byte for byte. Altering only the original 1 cm tail limit
from 1.5 to 4 m/s rejects replay at 58.100 s, after the retry has begun.
[Measurements and validation](../assets/lane-resume-speeds-results.json).

This is bounded authored reference-plant validation. Repeated arbitrary lane
changes, live speed-map updates, general traffic interaction, native RNE
qualification of this combination and real vehicles remain outside it.
Existing short-range follower failures remain preserved.
