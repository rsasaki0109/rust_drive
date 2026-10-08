# Development guide

## Toolchain and commands

Rust 1.90.0 is pinned, edition 2024. Use `cargo build --workspace --locked`, `cargo test --workspace --locked`, `cargo fmt --all --check`, and `cargo clippy --workspace --all-targets --locked -- -D warnings`. `bash scripts/check.sh` additionally builds release binaries and runs eleven reference scenarios and verifies their sensor logs. Optional friction fixtures run through the RNE hazard suite. Keep `Cargo.lock` under version control and use `--locked` in CI and installation. Four build jobs are a suitable default for the cloud machine; `RUSTDRIVE_BUILD_JOBS` overrides setup parallelism.

The checkout already provides task isolation in Codex cloud. Use the existing `/workspace/rust_drive` checkout; do not create additional Git worktrees unless explicitly requested.

`bash scripts/setup.sh` installs pinned Rust if needed, fetches locked dependencies, builds release, and runs tests. It does not require root or apt. The cloud installation lives under `/workspace/.rustdrive-tools`; on other hosts, an existing Rust installation is reused or a local user tool directory is chosen. `source scripts/env.sh` activates a local installation when Cargo is absent from PATH. A C linker is needed for Rust's default native linking (for example the usual Linux build-essential packages or the platform's developer tools).

Internet destinations for a first install are `sh.rustup.rs`, `static.rust-lang.org`, `index.crates.io`, and `static.crates.io`; GitHub is needed for checkout. Optional visualization package installation uses `pypi.org` and `files.pythonhosted.org`. There are no required application secrets. Once installed and fetched, `cargo test --workspace --locked --offline` exercises the default stack without network access.

## Files

- `crates/*/src`: working algorithms and the simulator.
- `crates/sim/tests/end_to_end.rs`: complete-loop tests, seed reproducibility and fault/stop criteria.
- `scenarios/*.json`: supplied acceptance scenarios.
- `artifacts/`: ignored generated logs and development demos.
- `assets/demo.gif`, `.png`, `.json`: intentionally committed README media and provenance.
- `scripts/render_demo.py`: renderer of saved simulation traces, not a second driving implementation.

## Write a scenario

```json
{
  "name": "My obstacle scenario",
  "duration": 40,
  "road_length": 150,
  "half_width": 5.5,
  "curve_amplitude": 2,
  "expected": "goal",
  "objects": [
    {"s": 55, "lateral": 0, "radius": 1.0},
    {"s": 90, "lateral": 0, "radius": 1.0, "speed": 2.0}
  ]
}
```

Road centerline is `(x, amplitude*sin(x/28))`; `road_length` sets x extent, not total curved arc length. Objects use route arc length `s`, lateral offset, circular radius, and optional route-speed, lateral-speed, and activation time. Objects exist from `active_from` (default 0); motion starts at `max(active_from, moving_from)`, with `moving_from` defaulting to 0. This permits a stationary object to be present before a later crossing, instead of making it appear at motion onset. Route position clamps at the end. `lidar_dropout` and `gnss_dropout` stop new measurements at the selected simulated time. These are scenario/environment settings, not information passed to the planner.

`expected` can be:

- `goal`: arrive within 2 m arc length of the route end and reduce speed below 0.2 m/s within the duration.
- `stop`: stop below 0.2 m/s after making at least 10 m progress without reaching the goal.
- `fault`: activate freshness braking and stop below 0.2 m/s without reaching the goal.

Every outcome also requires zero colliding evaluation steps, zero road-boundary violations, and localization maximum error ≤1 m. Scenario geometry and numerical values are validated; unknown JSON fields are rejected. The parser caps duration and road length, but this is a developer tool, not an untrusted network service. Inspect failure logs rather than changing acceptance criteria simply to make a scenario pass.

## Trace and rendering

Each recorded 10 Hz frame includes simulator truth (evaluation/display only), the EKF estimate, current LiDAR points, tracks, forecasts, planned trajectory, applied command, driving mode, progress and true obstacle clearance. Collision statistics include relative swept checks at the 20 Hz integration rate. `summary.json` also records seed and observed localization error. Frames can be visualized offline with:

```sh
python3 scripts/render_demo.py artifacts/demo/run.json --output artifacts/my-demo.gif
```

The renderer rejects a failing run as a success demo. Simulations can still emit failing telemetry; inspect JSON to diagnose the issue. Sensor-only `sensors.jsonl` additionally records every 20 Hz input and pipeline output plus calibrated configuration. Run `cargo run --release --locked --bin rustdrive -- replay --log artifacts/demo/sensors.jsonl --output artifacts/replay`. This recomputes the algorithm outputs, not simulator truth. An unsuccessful short mission can still have a correctly reproducible sensor log; use the physical acceptance report separately. See [sensor replay](sensor-replay.md).

## Replace an algorithm

`Perception`, `Predictor`, `Planner`, and `Controller` traits are defined in `rustdrive-core`. Build alternatives as real libraries and wire them into `rustdrive-pipeline`, so all backends and replay exercise the same implementation. Keep world/body transforms, SI units, timestamp/freshness handling, calibration and failure behavior explicit. Add a regression scenario plus independent acceptance metrics rather than a mock-only interface test. For an AI method, document model provenance and inference preprocessing, and retain a classical baseline for diagnosis.

## External simulators

No CARLA adapter is shipped. The default 2D simulator fulfills the initial reproducible end-to-end loop. The optional [RNE integration](../integrations/rne/README.md) provides CPU-only native kinematic/dynamic vehicle models and Rapier LiDAR acquisition, using the same pipeline and log contract. Its manifest is excluded from the default workspace; use Rust 1.95.0 and its own lockfile. Run `bash scripts/setup-rne.sh` once, then `bash scripts/rne-demo.sh dynamic` with the Pillow venv active. CI pins the engine revision and tests the integration on Linux.

A later gate is a CARLA synchronous fixed-step bridge with timestamped sensors, explicit Unreal/world-frame conversion, collision callbacks, route fixtures and latency checks. Do not use CARLA actor transforms as operational localization or obstacle ground truth as operational perception. See [roadmap](roadmap.md).

## Hazard and friction fixtures

`bash scripts/check-hazards.sh` builds both release binaries, runs all 48 reference/RNE runs across seeds 1/7/42 and verifies complete replay. Results include actual acceleration checks for friction fixtures and a source fingerprint; the command returns failure if either physical acceptance or replay fails. It uses standard-library Python only. See [current routing and regression results](routing.md).

Optional scenario `dynamics` supplies `friction_coefficient` (0.1–1.2) and `steering_lag_s` (0–1 s). These fields require RNE `--plant dynamic`; other plants reject them. They are fixed known calibration, not online estimation. Existing scenarios without the fields keep nominal behavior. `PipelineConfig.motion_limits` contains optional conservative `max_deceleration_m_s2` and `max_lateral_acceleration_m_s2`, which are validated and recorded in replay headers.

## Road maps and clearance constraints

Optional `navigation` provides a directed map, start/goal IDs and known closed edge IDs; it replaces sine-road geometry with the selected route. The three `route-*.json` fixtures share one map and demonstrate destination changes and pre-departure detours. Objects use arc length on the selected route. Mapped goals additionally require the truth center within 2 m of the destination. Optional `min_clearance_m` adds an independent swept-circle acceptance floor, without changing planner inputs. The regression suite also enforces fixed per-fixture floors and expected mapped edge sequences. See [map schema and complete evidence](routing.md).

`bash scripts/rne-demo.sh dynamic assets/rne-demo.gif scenarios/route-detour.json` regenerates the opening README GIF after activating a Pillow environment. Without the third argument, the script retains its original mission default.
