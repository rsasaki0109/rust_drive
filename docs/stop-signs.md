# Mapped stop signs

Fixed-route stop signs now stop the shared Rust pipeline, hold continuously for two seconds and release the route constraint. Both the reference bicycle and native RNE dynamic plant execute the same behavior. This is an authored planar road rule, not camera sign recognition or intersection right-of-way reasoning.

![Actual native RNE stop, two-second hold and restart rendered in 3D](../assets/stop-sign-demo.gif)

The GIF renders a real seed-7 native episode with original suburban scenery and an octagonal sign. Blender Cycles runs on CPU at 3× playback speed. Sign/scenery meshes are display-only; map stop-line geometry configures the driver. Physical driving and LiDAR remain planar. Full trace/renderer hashes, scene-transform audit and physical summary are in [GIF provenance](../assets/stop-sign-demo.json). [Compact seeded acceptance evidence](../assets/stop-sign-results.json).

## Run and reproduce

```sh
cargo run --release --locked --bin rustdrive -- run \
  --scenario scenarios/stop-sign-single.json --seed 7 --output artifacts/stop-signs
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/stop-signs/sensors.jsonl --output artifacts/stop-signs/replay

# Requires the pinned RNE checkout and Rust 1.95.0; no GPU.
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --plant dynamic --scenario scenarios/stop-sign-single.json --seed 7 \
  --output artifacts/rne-stop-signs

# Activate the documented Pillow environment; Blender is optional.
python scripts/render_demo_3d.py artifacts/rne-stop-signs/run.json \
  --output artifacts/stop-signs/demo.gif --samples 16 --threads 4
python scripts/render_demo_3d.py artifacts/rne-stop-signs/run.json \
  --preview-time 10.5 --output artifacts/stop-signs/preview.gif \
  --samples 16 --threads 4 --scene-output artifacts/stop-signs/scene.blend

bash scripts/check-hazards.sh --output /tmp/rustdrive-stop-signs
```

## Driver contract

`PipelineConfig.stop_signs` contains known `StopLine { id, route_s_m }` map entries. Positions use arc length on the fixed configured route. Signal and sign IDs/positions must be unique across both kinds; at most 64 combined controls are accepted. Stop signs must be at least eight meters apart and fit the existing stop-line route bounds. Live route handover with controls is rejected rather than silently retaining incorrect arc lengths.

States are `Approaching`, `Holding`, `Released`, `Passed`. The timer runs only when all sensor-health checks pass, estimated and accepted wheel speeds are at most 0.05 m/s in magnitude, accepted odometry is at most 0.05 s old, estimated circular-front margin is 0.5–3.5 m, yaw is within 0.2 rad of the route and the circular vehicle fits the corridor. Missing motion, rolling, excessive distance, heading/containment failures or unhealthy sensing reset the timer. Reaching two continuous seconds releases the prefix; only healthy, aligned estimated crossing marks the sign passed. A stop far ahead of the sign never satisfies it. Stops are discharged once per configured forward route traversal.

The nearest remaining sign and nonpermissive signal choose the shortest temporary route prefix; the existing collision-aware reachable-profile planner still runs. Map, estimator and tracks remain intact. While an unreleased sign is near and estimated speed is at most 0.2 m/s, requested acceleration is capped at −0.5 m/s² to retain brake pressure and prevent noisy feedback from repeatedly creeping during the hold. This is a simulation control policy, not a measured actuator-pressure interface or optimized comfort controller. A released sign cannot override a red signal or obstacle constraint.

## Independent physical acceptance

The simulator rule evaluator reads actual circular-front progress and actual speed, never driver stop status. Before crossing each line it requires two continuous seconds within 0–3.5 m at speed magnitude at most 0.1 m/s. A rolling, distant or interrupted stop fails; a backend that ignores all commands and accelerates through the sign is rejected by the full simulation. These physical thresholds are independent of the stricter observed-speed policy above.

The Python checker independently reconstructs the same physical holds/crossings from every 20 Hz truth frame, compares summaries, requires driver release after a healthy measured hold and enforces a fixed **1 m minimum unreleased front-line margin**. It checks complete sensor-only replay and timer reset/recovery in the injected-GNSS case. Manually changing a physical frame to cross early, while retaining driver diagnostics, is rejected. No acceptance gate is relaxed from the preceding suite.

Five fixtures × two plants × seeds 1/7/42 produce **30 positive stop-sign episodes**, added to the preceding 168. All **198 positive runs** pass; two known short-range follower failures remain explicitly rejected and excluded. Minimum observed stop-sign margin is **1.931879 m**; minimum continuous physical hold before crossing is **2.05 s**. Empty-world clearance value 1000 is a sentinel, not a measured distance to real obstacles.

| Fixture | Native seed-7 result | Physical behavior |
|---|---|---|
| `stop-sign-single` | Goal at 30.55 s / 612 ticks | 2.10 s standstill; front crosses at 13.30 s |
| `stop-sign-two` | Goal at 42.15 s / 844 ticks | Holds 2.10 / 2.05 s; crossings 13.30 / 28.65 s |
| `stop-sign-signal` | Goal at 41.90 s / 839 ticks | Stops at sign, independently holds red light until green at 32 s |
| `stop-sign-obstacle` | Stopped at 40 s / 801 ticks | Sign discharged; sensed blockage still stops ego before the obstacle |
| `stop-sign-gnss-recovery` | Goal at 34.70 s / 695 ticks | Rejected GNSS bias on [10,14); timer reset, new healthy hold, crossing at 17.45 s |

## Remaining work

No camera sign detector, all-way stop arrival-order reasoning, priority/yield policy, general intersection negotiation, pedestrian priority, lane changes, external sign-map importer or sign compliance by simulator traffic actors is implemented. The circular-front approximation does not establish rectangular-body or road-contact safety. A two-second hold is an explicit research fixture policy, not a statement of universally applicable traffic law. Native episodes retain conservative emergency planner ticks; comfort, latency and real-road performance are unverified. [Capability boundaries](capabilities.md) and [maturity milestones](maturity.md).
