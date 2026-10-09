# Mapped priority crossings

The shared Rust driver now yields before a known conflict rectangle when observed traffic can enter it. Priority occupancy comes from LiDAR-derived tracks and predictions, not simulator poses, actor schedules or an infrastructure "clear" flag. This first implementation targets an authored fixed forward route, not general right-of-way negotiation.

![Actual native RNE priority crossing rendered in 3D](../assets/intersection-demo.gif)

The GIF displays recorded native vehicle and actor positions from the preceding priority-crossing implementation, before forecasts were rebased to the current control time. It retains its original trace and fingerprints; rerunning the scenario with revised prediction arithmetic can produce different telemetry. The original suburban scenery, perpendicular crossing street and triangular yield sign are decorative and never enter LiDAR, collision acceptance or the driving policy. Moving traffic headings come from consecutive recorded positions. [GIF provenance](../assets/intersection-demo.json) and [baseline physical acceptance evidence](../assets/intersection-results.json) identify that recording. Existing opening, follower, fleet and stop-sign GIFs remain separate historical recordings.

## Run and reproduce

```sh
cargo run --release --locked --bin rustdrive -- run \
  --scenario scenarios/intersection-crossing.json --seed 7 --output artifacts/intersection
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/intersection/sensors.jsonl --output artifacts/intersection/replay

# Requires the pinned RNE checkout and Rust 1.95.0; no GPU.
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --plant dynamic --scenario scenarios/intersection-crossing.json --seed 7 \
  --output artifacts/rne-intersection

# Activate the documented Pillow environment; Blender is optional.
python scripts/render_demo_3d.py artifacts/rne-intersection/run.json \
  --output artifacts/intersection/demo.gif --samples 16 --threads 4
python scripts/render_demo_3d.py artifacts/rne-intersection/run.json \
  --preview-time 11 --output artifacts/intersection/preview.gif \
  --samples 16 --threads 4 --scene-output artifacts/intersection/scene.blend

bash scripts/check-hazards.sh --output /tmp/rustdrive-intersections
```

## Driver contract

`PipelineConfig.yield_intersections` contains `YieldIntersection { stop_line, conflict_bounds, exit_s_m }`. The stop line and exit use arc length on the configured fixed route; `conflict_bounds` is a world-ENU axis-aligned rectangle with finite `min`/`max` coordinates. Map entries supply known geometry only. Live navigation handover is rejected when these controls are configured, rather than silently retaining stale arc lengths. This initial domain uses straight forward routes.

For each unpassed intersection, every forecast segment in the next eight seconds, measured from the current control time, is swept against a conservatively inflated axis-aligned conflict rectangle. Predictions advance acquisition-time tracks through their observation age before evaluating future zone occupancy. Each rectangle face moves outward by the track radius plus 0.5 m; the square corners deliberately overapproximate a circular sweep. An intersecting prediction blocks entry and selects the existing stop-line route prefix. A stationary tracked occupant also blocks. This mechanism can yield before an actor reaches the physical crossing; it is a conservative observed-motion forecast, not an actor-intention model.

Permission requires healthy sensing, accepted LiDAR age at most 0.15 s, estimated heading within 0.2 rad of the route and circular corridor containment. One continuous second of clear evidence must include distinct accepted scan acquisitions; repeated or old timestamps cannot refresh the permission timer. Unhealthy sensing clears the dwell. A low-speed brake cap retains the stopped position while constrained. A current healthy, aligned estimated front crossing with fresh-scan permission commits entry; the intersection becomes passed after the healthy aligned estimated rear clears `exit_s_m`. States are `Waiting`, `Proceeding` and `Passed`, with a separate commitment flag. Passed states retain their final diagnostics rather than re-evaluating traffic behind ego. The full collision planner remains active inside and beyond the crossing.

An approach-speed envelope also applies while a zone is uncommitted and `Proceeding`, before the front crosses the original stop line. `Waiting` instead retains its bounded stop-line route prefix; applying another cruise cap there would unnecessarily slow the existing stopping profile. Let `b` be half the planner's calibrated maximum deceleration, `d = max(line - estimated_front - 2 m, 0)`, and response allowance `r = 0.25 s`. With default planner braking of 2.5 m/s², `b` is 1.25 m/s²; the emergency controller command is a separate limit. The planner's candidate cruise is bounded by `max(0.5 m/s, sqrt((b*r)^2 + 2*b*d) - b*r)` and by configured cruise speed, taking the closest restriction across zones. This leaves estimated stopping headroom while permission can still be revoked. The 0.5 m/s floor permits crossing a freshly cleared line; after commitment, that zone no longer caps approach speed. The normal planner cruise is restored after each plan. This bounds proposed cruise, not instantaneous physical speed, and feasible acceleration/braking profiles remain independently checked. It does not guarantee either the nominal two-meter reserve or the one-meter physical gate for arbitrarily late or unseen threats.

Stop signs and signals can impose a shorter independent route constraint. A released yield constraint cannot override a remaining stop-sign hold, a red signal or an obstacle. The simulated priority actors continue their configured motion; they do not negotiate with ego or acquire a semantic right-of-way state.

## Independent physical acceptance

The simulator evaluator samples actual circular vehicle/object positions at 20 Hz using true circle-to-rectangle distance, including rounded-corner geometry. Each occupancy interval starts at the first inside sample and ends at the first outside sample; an open interval stays occupied through the final episode timestamp. These are sampled rule intervals, with no interpolation of zone entry/exit between frames. The evaluator does not reuse the driver's conservative inflated-rectangle forecast test. Priority traffic and ego must remain separated by at least **two seconds** in time when passing the same conflict zone. Physical overlap, premature passage and initial occupancy are evaluated independently of pipeline state. This sampled rule scoring complements the separate continuous circular collision sweeps and corridor checks; collision-free passage alone cannot prove yielding.

The independent Python checker reconstructs occupancy from complete 20 Hz physical frames, compares rule summaries, checks priority separation and retains the fixed **1 m** clearance floor. It also checks actual stopping/waiting where required, complete sensor-only recomputation and fault recovery. The first priority-crossing revision passed **228 positive episodes** (111 reference / 117 native), retaining the preceding 198-run baseline and adding **30 intersection episodes** across seeds 1, 7 and 42. Both known short-range physical failures remained explicitly rejected outside the positive count. [Baseline results and source fingerprints](../assets/intersection-results.json).

Across all 30 intersection episodes, the smallest sampled physical gap is **4.50 s**, the smallest physical front-to-line waiting margin is **1.926332 m**, and the shortest continuous near-line hold is **2.40 s**. Gates remain 2 s priority separation and 1 m clearance/waiting margin. These are measurements for the authored layouts and speeds, not a general traffic-safety guarantee.

| Native RNE fixture, seed 7 | Duration / replay ticks | Physical priority gap | Longest continuous yield hold |
|---|---|---|---|
| Crossing | 33.20 s / 665 | 4.55 s | 4.60 s |
| Successive crossings | 38.15 s / 764 | 4.55 s | 9.60 s |
| Blocked intersection | 40.00 s / 801 | No ego entry | 30.20 s |
| GNSS recovery | 33.80 s / 677 | 5.05 s | 5.10 s |
| Stop-sign combination | 35.15 s / 704 | 4.85 s | 2.55 s |

The GNSS fixture revokes uncommitted permission during [10,14) while traffic is already blocking. A separate sensor-only integration test injects acquisition failure during an active clear timer and requires new confirmation after recovery. The Rust physical evaluator rejects a control-ignoring backend. An altered priority-occupancy trace is also rejected by Python even when driver outputs are untouched and a forged zero-violation summary is supplied.

| Fixture family | Exercised behavior |
|---|---|
| `intersection-crossing` | Stop for observed approaching priority traffic, then release and reach the goal |
| `intersection-successive` | Keep the constraint while a second actor can enter the same zone |
| `intersection-blocked` | Hold without entering a permanently occupied zone |
| `intersection-gnss-recovery` | Brake and reset clear evidence during the injected [10,14) localization fault; recover from new healthy observations |
| `intersection-stop-sign` | Satisfy the independent continuous stop requirement and priority constraint before entry |

That first revision passed local formatting, warnings-denied workspace/native Clippy, locked builds, **160 workspace tests**, **16 native integration tests** and **41 reference scenario/replay pairs**, plus the **228 positive episodes** reported above. Its native seed-7 GIF contains **112 encoded/audited scene states**, **960 × 640** pixels and **12.50 s playback** including its final pause. Its input bytes match the validated episode; trace, renderer and GIF fingerprints are recorded in the compact evidence. The editable scene reopens on CPU without external image references. The GIF remains this preceding verified recording. Publication CI is separate from local evidence. [Detailed validation](validation.md).

## Acquisition timing and varied authored roads

LiDAR delivery can be delayed without replacing acquisition timestamps. The driver transforms body-frame returns and occupancy rays through its bounded acquisition-time EKF pose history. It continues checking scan age at delivery; priority permission retains the 0.15 s bound and its distinct-acquisition clear timer. Motion prediction now separately advances tracks to the current control clock. Later GNSS corrections do not smooth past poses; delayed odometry/GNSS fusion, per-point deskew and uncertainty propagation are still absent. [Exact history bounds, forecast origin, injection configuration and replay commands](sensor-replay.md).

| Added fixture | Authored calibration |
|---|---|
| `intersection-fast-wide` | 130 m road, 6 m/s cruise, 2.5 m half-width, 10 m-long conflict rectangle; cross traffic at 4 m/s |
| `intersection-slow-narrow` | 80 m road, 3 m/s cruise, 1.8 m half-width, 6 m-long conflict rectangle; cross traffic at 3 m/s |
| `intersection-two-zones` | 140 m road, 4 m/s cruise, two separately controlled rectangles; second actor reaches its crossing center at 31 s |
| `intersection-delay-two` | 10 Hz delivered scans, 100 ms simulated delivery delay |
| `intersection-lidar-recovery` | 10 Hz delivered scans, 50 ms delay, explicit acquisition failure during [14.05,14.30) |
| `intersection-cadence-five-hz` | 5 Hz delivered observations, no delivery delay; native/reference acquisition remains 10 Hz |

The recovery window flushes queued scans and resets permission; recovery requires a new acquisition and clear confirmation. Timing/failure schedules remain simulator-only and are excluded from the replay configuration. Unit/integration tests exercise a stationary world circle during translating/turning ego motion with 100 ms delayed body-frame scans, exact sensor-only replay, uncovered/expired acquisitions, and shortest-yaw history interpolation. Supported injection ranges are broader than these particular physical calibrations.

The preceding acquisition-time reprojection sweep passed **264 positive runs** (129 reference / 135 native), including **66 intersection runs**, of which **18** exercise timing injection. All logs fully replayed. Local formatting, warnings-denied workspace/native Clippy, locked builds, **171 workspace tests**, **16 native tests** and **47 reference run/replay pairs** passed. Across those 66 intersection runs, the smallest sampled physical priority gap was **4.45 s**, the smallest waiting margin was **1.900082 m**, and the shortest near-line hold was **2.40 s**. Original 2 s / 1 m gates were retained. These are historical measurements for the acquisition-time reprojection revision. [Baseline compact evidence and fingerprints](../assets/sensor-timing-results.json).

| Native RNE fixture, seed 7 | Duration / replay ticks | Physical priority gap |
|---|---|---|
| Fast / wide | 31.60 s / 633 | 4.55 s |
| Slow / narrow | 34.75 s / 696 | 5.70 s |
| Two zones | 51.40 s / 1029 | 4.80 s |
| 100 ms delivery | 33.30 s / 667 | 4.65 s |
| 50 ms delivery + acquisition recovery | 34.05 s / 682 | 5.40 s |
| 5 Hz observations | 33.35 s / 668 | 4.65 s |

That native recovery case has five explicit failure ticks, discards one pending scan and resets an active clear dwell before resuming. The independent timing checker rejects changed acquisition timestamps, hidden failure reports and a removed scheduled observation even when driver outputs are left untouched. At that preceding revision, the existing GIF trace was byte-identical to the corresponding validated episode; that correspondence does not extend to revised forecast arithmetic. The recovery recording also rendered a CPU preview and exported an editable scene.

```sh
cargo run --release --locked --bin rustdrive -- run \
  --scenario scenarios/intersection-fast-wide.json --seed 7 \
  --output artifacts/intersection-fast
cargo run --release --locked --bin rustdrive -- run \
  --scenario scenarios/intersection-cadence-five-hz.json --seed 7 \
  --output artifacts/intersection-cadence
```

## Repaired late conflict

![Actual native RNE late second-crossing run with current-time forecasts and an approach envelope](../assets/late-crossing-demo.gif)

This current native seed-7 recording uses the unchanged original late-crossing scenario. The CPU-rendered GIF contains **201 encoded/audited scene states**, **960 × 640** pixels and **21.40 s playback** including its final pause. Recorded ego and priority traffic positions drive the display; the generated two-crossing streets and signs remain decorative. The editable scene reopens on CPU with **551 objects** and no external image references. [Trace, renderer and GIF provenance](../assets/late-crossing-demo.json) identifies the 59.90 s / 1199-tick input recording. It is separate from the historical GIF above.

`intersection-late-conflict` preserves a second crossing whose actor reaches its center at 35 s, later than the 31 s positive calibration. At the preceding acquisition-time reprojection revision, the reference seed-7 run revoked uncommitted permission too late to retain the required one-meter physical front-to-line waiting margin: the measured minimum was **0.267444 m**. The CLI's collision/road/zone-separation summary passed and all **1098** sensor ticks replayed, but the independent waiting-margin checker rejected the episode. It was excluded from that revision's positive acceptance alongside the two prior native short-range follower failures. Its actor schedule, road/control geometry and one-meter gate remain the regression target for the current-time forecasts and approach envelope. Collision-free deterministic replay alone cannot establish rule acceptance.

The revised driver passes this unchanged fixture in both plants across seeds 1, 7 and 42. A second fixture, `intersection-late-delayed`, retains the same actors, road/control geometry and gates while adding 100 ms LiDAR delivery delay. All twelve targeted physical runs and complete sensor replays pass. The smallest waiting margins are **1.939764 m** for the original fixture and **1.955410 m** for the delayed fixture; the smallest sampled priority gaps are **9.30 s** and **9.35 s**, respectively. The shortest continuous near-line holds are **3.60 s** and **3.45 s**. Both retain the original one-meter waiting/clearance floor and two-second zone separation, without earlier actor scheduling or relocated stop lines.

| Native RNE, seed 7 | Duration / replay ticks | Physical waiting margin | Physical priority gap |
|---|---|---|---|
| Original late crossing | 59.90 s / 1199 | 2.003782 m | 9.30 s |
| Same crossing with 100 ms delivery | 59.95 s / 1200 | 1.993557 m | 9.35 s |

The independent checker reconstructs current-time forecasts from acquisition tracks and braking history, reconstructs the `Proceeding` approach bound with a separate braking-budget calculation, and checks reachable trajectory speeds. A changed `Proceeding` trajectory exceeding the bound is rejected. The original mixed stop-sign/priority and two-zone waiting tests also pass with their unchanged hold gates: `Waiting` keeps its existing stopping prefix, without the additional approach cruise cap. Final-source formatting, warnings-denied workspace/native Clippy, locked builds, **181 workspace tests**, **16 native tests** and **49 reference run/replay pairs** pass.

The final complete sweep passes **276 positive episodes** (**135 reference / 141 native**), retaining the preceding 264 and adding the twelve original/delayed late-crossing runs. Every log fully replays, totaling **228,961 sensor ticks**. All **78 intersection episodes**, including **24 timing-injection episodes**, retain the same two-second priority gap and one-meter waiting/clearance floors. Across these 78 runs, the smallest physical gap is **8.95 s**, the smallest waiting margin is **1.900082 m**, and the shortest continuous near-line hold is **2.20 s**. Both prior native short-range follower failures remain independently rejected outside the positive count. The late-crossing fixture is now part of positive acceptance. [Final compact evidence, source/checker fingerprints and GIF trace verification](../assets/prediction-epoch-results.json).

Current-time forecasting also exposed a GNSS-burst steering regression during development: native seed 1 exceeded its unchanged 0.5 m localization-error gate at **0.623638 m**. The final sustained-innovation-rejection hold brakes earlier on two new rejected fixes, without changing the NIS 36 or 0.75 s accepted-age limits. Final native burst maximum errors across seeds 1, 7 and 42 are **0.184327 / 0.222971 / 0.412726 m**; isolated GNSS-spike cases do not activate the added hold. The same compact evidence retains the candidate failure and verified final sensor-only checks. This is bounded authored fault handling, not a general localization-integrity guarantee.

```sh
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --plant dynamic --scenario scenarios/intersection-late-conflict.json --seed 7 \
  --output artifacts/rne-late-crossing
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/rne-late-crossing/sensors.jsonl \
  --output artifacts/rne-late-crossing/replay
python scripts/render_demo_3d.py artifacts/rne-late-crossing/run.json \
  --output artifacts/rne-late-crossing/demo.gif --samples 16 --threads 3
python scripts/render_demo_3d.py artifacts/rne-late-crossing/run.json \
  --preview-time 34.5 --output artifacts/rne-late-crossing/preview.gif \
  --samples 16 --threads 3 --scene-output artifacts/rne-late-crossing/scene.blend
```

## Remaining work

There is no semantic vehicle/pedestrian classification, camera sign detector, signal-camera recognition, all-way stop arrival ordering, protected-turn logic, lane topology, roundabout policy or interactive/multimodal forecasting. Other actors do not comply with mapped controls. A hidden/unobserved actor or incorrect prediction can invalidate the policy's assumptions; the authored tests do not establish general right-of-way safety. Conflict zones and entry/exit geometry are supplied maps, with no external map importer or moving-route remapping.

Physical driving and sensor acquisition remain planar with circular footprints. Decorative cars and street meshes do not establish rectangular collision, road contact or full 3D perception. Resource latency, comfort and real-road performance are unverified. The subjective maturity estimate is about **8%**, reflecting the new observed-motion priority-yield behavior rather than the GIF or test count; this extension alone does not satisfy the broader 10% capability gate. [Maturity criteria](maturity.md) and [capability boundaries](capabilities.md).
