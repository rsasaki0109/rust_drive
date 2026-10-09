# Mapped priority crossings

The shared Rust driver now yields before a known conflict rectangle when observed traffic can enter it. Priority occupancy comes from LiDAR-derived tracks and predictions, not simulator poses, actor schedules or an infrastructure "clear" flag. This first implementation targets an authored fixed forward route, not general right-of-way negotiation.

![Actual native RNE priority crossing rendered in 3D](../assets/intersection-demo.gif)

The GIF displays recorded native vehicle and actor positions with original suburban scenery, a perpendicular crossing street and a triangular yield sign. Moving traffic headings come from consecutive recorded positions. All added road/sign meshes are decorative and never enter LiDAR, collision acceptance or the driving policy. [GIF provenance](../assets/intersection-demo.json) and [compact physical acceptance evidence](../assets/intersection-results.json) record the trace and renderer fingerprints. Existing opening, follower, fleet and stop-sign GIFs remain separate recordings.

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

For each unpassed intersection, every forecast segment in the next eight seconds is swept against a conservatively inflated axis-aligned conflict rectangle. Each rectangle face moves outward by the track radius plus 0.5 m; the square corners deliberately overapproximate a circular sweep. An intersecting prediction blocks entry and selects the existing stop-line route prefix. A stationary tracked occupant also blocks. This mechanism can yield before an actor reaches the physical crossing; it is a conservative observed-motion forecast, not an actor-intention model.

Permission requires healthy sensing, accepted LiDAR age at most 0.15 s, estimated heading within 0.2 rad of the route and circular corridor containment. One continuous second of clear evidence must include distinct accepted scan acquisitions; repeated or old timestamps cannot refresh the permission timer. Unhealthy sensing clears the dwell. A low-speed brake cap retains the stopped position while constrained. A current healthy, aligned estimated front crossing with fresh-scan permission commits entry; the intersection becomes passed after the healthy aligned estimated rear clears `exit_s_m`. States are `Waiting`, `Proceeding` and `Passed`, with a separate commitment flag. Passed states retain their final diagnostics rather than re-evaluating traffic behind ego. The full collision planner remains active inside and beyond the crossing.

Stop signs and signals can impose a shorter independent route constraint. A released yield constraint cannot override a remaining stop-sign hold, a red signal or an obstacle. The simulated priority actors continue their configured motion; they do not negotiate with ego or acquire a semantic right-of-way state.

## Independent physical acceptance

The simulator evaluator samples actual circular vehicle/object positions at 20 Hz using true circle-to-rectangle distance, including rounded-corner geometry. Each occupancy interval starts at the first inside sample and ends at the first outside sample; an open interval stays occupied through the final episode timestamp. These are sampled rule intervals, with no interpolation of zone entry/exit between frames. The evaluator does not reuse the driver's conservative inflated-rectangle forecast test. Priority traffic and ego must remain separated by at least **two seconds** in time when passing the same conflict zone. Physical overlap, premature passage and initial occupancy are evaluated independently of pipeline state. This sampled rule scoring complements the separate continuous circular collision sweeps and corridor checks; collision-free passage alone cannot prove yielding.

The independent Python checker reconstructs occupancy from complete 20 Hz physical frames, compares rule summaries, checks priority separation and retains the fixed **1 m** clearance floor. It also checks actual stopping/waiting where required, complete sensor-only recomputation and fault recovery. The full sweep passes **228 positive episodes** (111 reference / 117 native), retaining the preceding 198-run baseline and adding **30 intersection episodes** across seeds 1, 7 and 42. Both known short-range physical failures remain explicitly rejected outside the positive count. [Compact results and source fingerprints](../assets/intersection-results.json).

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

Local formatting, warnings-denied workspace/native Clippy, locked builds, **160 workspace tests**, **16 native integration tests** and **41 reference scenario/replay pairs** pass. The complete seeded reference/native suite and final 3D render are still being verified; their totals are not asserted here.

## Remaining work

There is no semantic vehicle/pedestrian classification, camera sign detector, signal-camera recognition, all-way stop arrival ordering, protected-turn logic, lane topology, roundabout policy or interactive/multimodal forecasting. Other actors do not comply with mapped controls. A hidden/unobserved actor or incorrect prediction can invalidate the policy's assumptions; the authored tests do not establish general right-of-way safety. Conflict zones and entry/exit geometry are supplied maps, with no external map importer or moving-route remapping.

Physical driving and sensor acquisition remain planar with circular footprints. Decorative cars and street meshes do not establish rectangular collision, road contact or full 3D perception. Resource latency, comfort and real-road performance are unverified. The subjective maturity estimate is about **8%**, reflecting the new observed-motion priority-yield behavior rather than the GIF or test count; this extension alone does not satisfy the broader 10% capability gate. [Maturity criteria](maturity.md) and [capability boundaries](capabilities.md).
