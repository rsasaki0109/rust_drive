# RustDriving

![Recorded city pursuit among ordinary traffic, hillside streets, a breakaway guardrail and the station loop](assets/urban-hills-chase-demo.gif)

[🎮 Play City Playground — 自分で運転・ドリフト・カーチェイス](https://rsasaki0109.github.io/rust_driving/)

<table>
  <tr>
    <td width="33%"><img src="assets/city-crossing-demo.gif" width="260" alt="Recorded daytime intersections"><br>Daytime intersections</td>
    <td width="33%"><img src="assets/harbor-pursuit-demo.gif" width="260" alt="Recorded harbor pursuit"><br>Harbor pursuit</td>
    <td width="33%"><img src="assets/forest-trail-demo.gif" width="260" alt="Recorded forest shortcuts"><br>Forest shortcuts</td>
  </tr>
  <tr>
    <td width="33%"><img src="assets/plaza-drift-demo.gif" width="260" alt="Recorded dusk drift"><br>Dusk drift</td>
    <td width="33%"><img src="assets/station-loop-demo.gif" width="260" alt="Recorded station curves"><br>Station curves</td>
    <td width="33%"><img src="assets/stacked-roads-demo.gif" width="260" alt="Recorded elevated / underground"><br>Elevated / underground</td>
  </tr>
</table>

<details>
<summary>Autonomous-driving simulations</summary>

<table>
  <tr>
    <td width="33%"><img src="assets/japan-city-demo.gif" width="260" alt="Recorded japanese city"><br>Japanese city</td>
    <td width="33%"><img src="assets/prediction-3d-demo.gif" width="260" alt="Recorded braking and recovery"><br>Braking and recovery</td>
    <td width="33%"><img src="assets/stop-sign-demo.gif" width="260" alt="Recorded stop sign"><br>Stop sign</td>
  </tr>
  <tr>
    <td width="33%"><img src="assets/intersection-demo.gif" width="260" alt="Recorded priority crossing"><br>Priority crossing</td>
    <td width="33%"><img src="assets/vru-demo.gif" width="260" alt="Recorded pedestrians and cyclists"><br>Pedestrians and cyclists</td>
    <td width="33%"><img src="assets/lidar-3d-demo.gif" width="260" alt="Recorded inclined 3d lidar"><br>Inclined 3D LiDAR</td>
  </tr>
</table>

</details>

Autonomous driving research in Rust, with recorded simulations and an optional
[manual-driving browser playground](docs/browser-playground.md).

## Quick start

Requires [Rust 1.90.0](https://rustup.rs/) (pinned in this repository).

```sh
cargo run --release --locked --bin rustdriving -- run \
  --scenario scenarios/mission.json --seed 7 --output artifacts/demo

cargo run --release --locked --bin rustdriving -- replay \
  --log artifacts/demo/sensors.jsonl --output artifacts/replay
```

## Docs

- [Architecture](docs/architecture.md) · [Capabilities](docs/capabilities.md)
- [Native RNE](integrations/rne/README.md) · [Hero reproduction](docs/browser-playground.md#hero-recording)
- [Development](docs/development.md) · [Validation](docs/validation.md) · [Contributing](CONTRIBUTING.md)
- [Maturity and roadmap](docs/maturity.md)

[Apache-2.0](LICENSE) · [Third-party licenses](THIRD_PARTY.md) · [OSM data: ODbL 1.0](maps/osm/SOURCE.md)
