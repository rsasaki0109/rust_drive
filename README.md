# RustDriving

![Recorded Japanese urban driving simulation](assets/japan-city-demo.gif)

<table>
  <tr>
    <td width="33%"><img src="assets/rne-3d-demo.gif" width="260" alt="Recorded closure detour simulation"></td>
    <td width="33%"><img src="assets/traffic-fleet-demo.gif" width="260" alt="Recorded traffic queue simulation"></td>
    <td width="33%"><img src="assets/prediction-3d-demo.gif" width="260" alt="Recorded braking and recovery simulation"></td>
  </tr>
  <tr>
    <td width="33%"><img src="assets/stop-sign-demo.gif" width="260" alt="Recorded stop sign simulation"></td>
    <td width="33%"><img src="assets/intersection-demo.gif" width="260" alt="Recorded priority crossing simulation"></td>
    <td width="33%"><img src="assets/late-crossing-demo.gif" width="260" alt="Recorded late cross traffic simulation"></td>
  </tr>
  <tr>
    <td width="33%"><img src="assets/native-scene-demo.gif" width="260" alt="Recorded physical cuboid stop simulation"></td>
    <td width="33%"><img src="assets/multi-height-demo.gif" width="260" alt="Recorded optional multi-height LiDAR simulation"></td>
    <td width="33%"><img src="assets/lidar-3d-demo.gif" width="260" alt="Recorded optional inclined 3D LiDAR simulation"></td>
  </tr>
  <tr>
    <td width="33%"><img src="assets/ground-demo.gif" width="260" alt="Recorded optional measured-ground filtering simulation"></td>
    <td width="33%"><img src="assets/vru-demo.gif" width="260" alt="Recorded pedestrian and cyclist simulation"></td>
    <td width="33%"><img src="assets/hazard-demo.gif" width="260" alt="Recorded occluded crossing simulation"></td>
  </tr>
</table>

Autonomous driving research in Rust. Recorded simulations, including optional
sensing modes.

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
- [Native RNE](integrations/rne/README.md) · [Demo reproduction](docs/japanese-city-demo.md#reproduction)
- [Development](docs/development.md) · [Validation](docs/validation.md) · [Contributing](CONTRIBUTING.md)
- [Maturity and roadmap](docs/maturity.md)

[Apache-2.0](LICENSE) · [Third-party licenses](THIRD_PARTY.md) · [OSM data: ODbL 1.0](maps/osm/SOURCE.md)
