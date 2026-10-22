# RustDriving

![Recorded Japanese urban driving simulation](assets/japan-city-demo.gif)

![Recorded closure detour simulation](assets/rne-3d-demo.gif)

![Recorded traffic queue simulation](assets/traffic-fleet-demo.gif)

![Recorded braking and recovery simulation](assets/prediction-3d-demo.gif)

![Recorded stop sign simulation](assets/stop-sign-demo.gif)

![Recorded priority crossing simulation](assets/intersection-demo.gif)

![Recorded late cross traffic simulation](assets/late-crossing-demo.gif)

![Recorded physical cuboid stop simulation](assets/native-scene-demo.gif)

![Recorded optional multi-height LiDAR simulation](assets/multi-height-demo.gif)

![Recorded optional inclined 3D LiDAR simulation](assets/lidar-3d-demo.gif)

![Recorded optional measured-ground filtering simulation](assets/ground-demo.gif)

![Recorded pedestrian and cyclist simulation](assets/vru-demo.gif)

![Recorded occluded crossing simulation](assets/hazard-demo.gif)

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
