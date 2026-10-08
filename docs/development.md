# Development guide

## Toolchain and commands

Rust 1.90.0 is pinned, edition 2024. Use `cargo build --workspace --locked`, `cargo test --workspace --locked`, `cargo fmt --all --check`, and `cargo clippy --workspace --all-targets --locked -- -D warnings`. `bash scripts/check.sh` additionally builds release binaries and runs every supplied scenario. Keep `Cargo.lock` under version control and use `--locked` in CI and installation. Four build jobs are a suitable default for the cloud machine; `RUSTDRIVE_BUILD_JOBS` overrides setup parallelism.

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

Road centerline is `(x, amplitude*sin(x/28))`; `road_length` sets x extent, not total curved arc length. Objects use route arc length `s`, lateral offset, circular radius, and optional route-speed, lateral-speed, and activation time. Motion starts at `active_from` (default 0). Route position clamps at the end. `lidar_dropout` and `gnss_dropout` stop new measurements at the selected simulated time. These are scenario/environment settings, not information passed to the planner.

`expected` can be:

- `goal`: arrive within 2 m arc length of the route end and reduce speed below 0.2 m/s within the duration.
- `stop`: stop below 0.2 m/s after making at least 10 m progress without reaching the goal.
- `fault`: activate freshness braking and stop below 0.2 m/s without reaching the goal.

Every outcome also requires zero colliding integration steps, zero road-boundary violations, and localization maximum error ≤1 m. Scenario geometry and numerical values are validated; unknown JSON fields are rejected. The parser caps duration and road length, but this is a developer tool, not an untrusted network service. Inspect failure logs rather than changing acceptance criteria simply to make a scenario pass.

## Trace and rendering

Each recorded 10 Hz frame includes simulator truth (evaluation/display only), the EKF estimate, current LiDAR points, tracks, forecasts, planned trajectory, applied command, driving mode, progress and true obstacle clearance. Collision statistics include relative swept checks at the 20 Hz integration rate. `summary.json` also records seed and observed localization error. Frames can be visualized offline with:

```sh
python3 scripts/render_demo.py artifacts/demo/run.json --output artifacts/my-demo.gif
```

The renderer rejects a failing run as a success demo. Simulations can still emit failing telemetry; inspect JSON to diagnose the issue. Seeded rerunning is tested; arbitrary recorded sensor streams are not yet accepted as pipeline replay input. A stable sensor-log replay interface is a roadmap item.

## Replace an algorithm

`Perception`, `Predictor`, `Planner`, and `Controller` traits are defined in `rustdrive-core`. Build alternatives as real libraries and wire them into simulator orchestration. Keep world/body transforms, SI units, timestamp/freshness handling, calibration and failure behavior explicit. Add a regression scenario plus independent acceptance metrics rather than a mock-only interface test. For an AI method, document model provenance and inference preprocessing, and retain a classical baseline for diagnosis.

## External simulators

No CARLA adapter is shipped. The default 2D simulator fulfills the initial reproducible end-to-end loop. The next gate is a CARLA synchronous fixed-step bridge with timestamped sensors, explicit Unreal/world-frame conversion, collision callbacks, route fixtures and latency checks. Do not use CARLA actor transforms as operational localization or obstacle ground truth as operational perception. See [roadmap](roadmap.md).
