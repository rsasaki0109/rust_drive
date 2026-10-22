# RustDriving

**Autonomous driving research in Rust.**

[![Recorded left-hand RNE driving through Japanese urban intersections with signals, roadworks and diverse road users](assets/japan-city-demo.gif)](docs/japanese-city-demo.md)

Recorded simulations. Click a demo for checks and reproduction.
The gallery includes optional sensing modes.

<table>
  <tr>
    <td width="33%" align="center">
      <a href="docs/3d-demo.md">
        <img src="assets/rne-3d-demo.gif" width="280" alt="Recorded RNE map-closure detour with planned trajectories">
        <br><strong>Closure detour</strong>
      </a>
    </td>
    <td width="33%" align="center">
      <a href="docs/vehicle-fleet.md">
        <img src="assets/traffic-fleet-demo.gif" width="280" alt="Three reactive traffic vehicles form a queue ahead of ego">
        <br><strong>Traffic queue</strong>
      </a>
    </td>
    <td width="33%" align="center">
      <a href="docs/3d-demo.md#reproduce-the-follower-gif">
        <img src="assets/prediction-3d-demo.gif" width="280" alt="Recorded GNSS-fault stop, follower braking and recovery">
        <br><strong>Braking and recovery</strong>
      </a>
    </td>
  </tr>
  <tr>
    <td width="33%" align="center">
      <a href="docs/stop-signs.md">
        <img src="assets/stop-sign-demo.gif" width="280" alt="Ego holds at a mapped stop sign before restarting">
        <br><strong>Stop signs</strong>
      </a>
    </td>
    <td width="33%" align="center">
      <a href="docs/intersections.md">
        <img src="assets/intersection-demo.gif" width="280" alt="Recorded RNE ego yields to priority cross traffic">
        <br><strong>Priority crossings</strong>
      </a>
    </td>
    <td width="33%" align="center">
      <a href="docs/intersections.md#repaired-late-conflict">
        <img src="assets/late-crossing-demo.gif" width="280" alt="Recorded native episode with late cross traffic at two junctions">
        <br><strong>Late cross traffic</strong>
      </a>
    </td>
  </tr>
  <tr>
    <td width="33%" align="center">
      <a href="docs/native-scenes.md">
        <img src="assets/native-scene-demo.gif" width="280" alt="Native LiDAR sensing stops ego before a physical cuboid">
        <br><strong>Physical cuboid stop</strong>
      </a>
    </td>
    <td width="33%" align="center">
      <a href="docs/multi-height-lidar.md">
        <img src="assets/multi-height-demo.gif" width="280" alt="Optional multi-height LiDAR detects a low physical slab">
        <br><strong>Multi-height LiDAR</strong>
      </a>
    </td>
    <td width="33%" align="center">
      <a href="docs/lidar-3d.md">
        <img src="assets/lidar-3d-demo.gif" width="280" alt="Inclined native XYZ LiDAR detects an elevated beam">
        <br><strong>Inclined 3D LiDAR</strong>
      </a>
    </td>
  </tr>
  <tr>
    <td width="33%" align="center">
      <a href="docs/ground-lidar.md">
        <img src="assets/ground-demo.gif" width="280" alt="Measured-ground processing follows a moving lead into a stop">
        <br><strong>Measured-ground filtering</strong>
      </a>
    </td>
    <td width="33%" align="center">
      <a href="docs/road-users.md">
        <img src="assets/vru-demo.gif" width="280" alt="Recorded native road users include pedestrians and a cyclist">
        <br><strong>Road users</strong>
      </a>
    </td>
    <td width="33%" align="center">
      <a href="docs/hazard-validation.md">
        <img src="assets/hazard-demo.gif" width="280" alt="Sensor and planning diagnostics during an occluded crossing">
        <br><strong>Occluded crossing</strong>
      </a>
    </td>
  </tr>
</table>

Sensor-driven perception, localization, prediction, planning and control for
planar simulation. [Capabilities and limits](docs/capabilities.md).

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
