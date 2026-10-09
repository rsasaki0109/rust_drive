# Native 3D query scenes

RustDrive can opt into static, upright cuboids in the actual CPU-only RNE/Rapier query world. Their dimensions, height and yaw affect native LiDAR returns and a separate physical clearance guard. A ground-level barrier blocks the driving sensor; an otherwise identical raised barrier clears that sensor and the recorded ego capsule. This extends the simulator geometry beyond the preceding circular actors. Ego motion still uses the native **planar Ackermann plant**.

This feature does not add 3D perception. The driving pipeline receives the existing body-frame planar scan at **0.6 m** height. Additional horizontal scans at **0.15 m** and **3.7 m** are recorded only for independent validation; they never enter localization, tracking, planning or control. Scene labels, cuboid geometry and simulator poses remain simulator/evaluation inputs. Roadside buildings, trees, pavements and the detailed vehicle display model remain cosmetic.

## Run, replay and check

Use the development branch, activate the local tools and fetch the pinned engine as described in the [README](../README.md#cpu-only-robot-native-engine-demo). No new Rust dependencies or engine revision are required.

```sh
source scripts/env.sh
bash scripts/setup-rne.sh
cargo +1.95.0 run --release --locked \
  --manifest-path integrations/rne/Cargo.toml -- \
  --scenario scenarios/native-scene-ground-stop.json \
  --scene scenes/ground-barrier.json --plant dynamic --seed 7 \
  --output artifacts/native-ground
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/native-ground/sensors.jsonl \
  --output artifacts/native-ground/replay
bash scripts/check-native-scenes.sh
```

The native CLI writes the usual `run.json`, `summary.json` and sensor-only `sensors.jsonl`, plus `scene.json` when `--scene` is supplied. The sidecar includes normalized geometry, native substep positions, body poses, scan acquisitions, firing-ordinal ranges and physical acceptance results. Scene guard failures also fail the run summary and CLI exit status. Reusing an output directory without `--scene` removes its stale `scene.json`.

Sensor replay recomputes the driving pipeline from the logged observations. It neither reads the scene sidecar nor regenerates a physical world. A verified replay establishes repeatable pipeline computation; scene acceptance is checked separately.

The independent checker defaults to seeds **1, 7 and 42**, both native **kinematic and dynamic** plants, and four fixtures:

| Scene | Driving fixture | Expected result |
| --- | --- | --- |
| `ground-barrier` | `native-scene-ground-stop` | Stop before a physical barrier |
| `raised-barrier` | `native-scene-raised-goal` | Reach the goal below the raised cuboid |
| `rotated-barrier` | `native-scene-ground-stop` | Stop before a yaw-rotated physical barrier |
| `blind-low-slab` | `native-scene-blind-low` | Required rejection: the operational scan misses a low obstacle |

The complete local matrix passed: **18 positive episodes and six required low-slab rejections**, with 32 rejected evidence mutations. The independent checks reconstructed 705,080 returned rays across 6,824 acquisitions and checked 136,084 native motion samples. The maximum range residual was 0.038302 m, below the 0.06 m tolerance. Ground/rotated cases retained at least 7.116577 m capsule clearance; raised cases retained 1.15 m. [Recorded compact results](../assets/native-scene-results.json) preserve the source and input hashes. The generated `artifacts/native-scenes/report.json` also records completion, full ray and capsule checks, sensor replay, speed profiles and forecasts. [Validation history](validation.md) keeps these authored fixtures separate from broader capability claims.

```sh
# Faster local subset; this does not establish the full matrix:
python3 scripts/check-native-scenes.py --plants dynamic --seeds 7 \
  --output artifacts/native-scenes-subset
```

## Scene coordinates and acceptance

Scene inputs use meters in **ENU**: `center_m = [east, north, up]`, `half_extents_m` specifies positive local-axis half dimensions, and `yaw_rad` rotates about up within `[-pi, pi]`. Schema version 1 accepts one to 128 uniquely identified cuboids, finite coordinates within 2000 m, and no unknown fields.

```json
{
  "schema_version": 1,
  "name": "Ground-level physical road barrier",
  "static_cuboids": [
    {
      "id": "barrier",
      "center_m": [35, 0, 1],
      "half_extents_m": [0.5, 3, 1],
      "yaw_rad": 0
    }
  ]
}
```

The barrier is one meter deep, six meters wide and two meters high. Its raised counterpart keeps those dimensions and changes its center height to 4.5 m, giving a bottom height of 3.5 m. The independent slab-intersection oracle checks all 720 ray ordinals at each recorded height against cuboids and existing actor capsules, including misses and nearest-hit occlusion. It allows 0.06 m range error for the configured 0.008 m native range-noise sigma. This validates these authored ray fixtures; it does not calibrate a physical sensor.

The ego query collider is the existing upright capsule: its axis spans **0.1–1.1 m**, expanded by the recorded circular radius, typically **1.25 m**. Its vertical extent therefore reaches below the displayed ground. It is not a calibrated car body. Clearance is evaluated against the actual cuboid dimensions and yaw, independently of the decorative hatchback mesh.

The guard checks every recorded native integration substep and its intervening center chord. It conservatively expands horizontal separation by `12 m/s × dt / 2`, after verifying each 5 ms planar displacement stays within that bound, then checks capsule-to-box surface distance against a fixed **1 m** floor. The independent oracle uses rectangle edges and corners rather than the adapter's interval minimizer. Mutation checks reject removed colliders, altered height, changed ray returns and falsified clearance results. The pinned integrator applies one fixed midpoint velocity for each translation substep; this is a guard for that discrete simulation, not a bound on continuous physical chassis or tire motion. These are offline conservative acceptance checks; they do not introduce contact response or an operational obstacle-avoidance fallback.

The low-slab rejection is deliberately retained. A low obstacle can intersect the capsule while lying below the 0.6 m driving scan; the 0.15 m diagnostic scan sees it, but that information remains outside driving inputs. A planar perception pipeline cannot claim safety for arbitrary 3D geometry from these fixtures.

## Render the actual physical geometry

![Actual native ground-barrier stop, with the physical cuboid rendered from audited scene evidence](../assets/native-scene-demo.gif)

The published dynamic-plant recording uses seed 7 and runs for **35 seconds / 701 sensor ticks**. Ego stops before the barrier with **7.391012 m** minimum capsule-guard clearance. The full recording is rendered as **118 audited states / 118 GIF frames**, 960 × 640 pixels, using eight Cycles CPU samples and three threads. The 1.67 MB GIF plays at 3× speed with a final pause; all 351 recorded display poses and each rendered cuboid's corners are checked. [GIF provenance](../assets/native-scene-demo.json).

Install Blender and activate the Pillow environment described in [3D replay setup](3d-demo.md#reproduce-the-opening-gif). Rendering is opt-in: `--native-scene` takes the **generated evidence sidecar**, not the original scene fixture.

```sh
python3 scripts/render_demo_3d.py artifacts/native-ground/run.json \
  --native-scene artifacts/native-ground/scene.json \
  --output artifacts/native-ground/demo.gif --samples 8 --threads 3
# A single CPU-rendered PNG and editable snapshot:
python3 scripts/render_demo_3d.py artifacts/native-ground/run.json \
  --native-scene artifacts/native-ground/scene.json \
  --preview-time 12 --output artifacts/native-ground/preview.gif \
  --scene-output artifacts/native-ground/scene.blend --samples 4 --threads 2
```

The orange cuboid mesh uses the exact accepted center, full dimensions and yaw, including its actual height. The renderer requires successful run and scene summaries, matching backend/scenario/seed, and matching timestamps/body poses for every displayed trace frame. It audits the Blender mesh's eight actual world-space corners to within **10⁻⁴ m** of the normalized physical geometry at every rendered state. The native-scene input SHA-256, normalized geometry, summary, seed and audited state counts are added to GIF provenance. Matching body poses and timeline tie the sidecar to the displayed run; its seed must also match the run summary.

Omitting `--native-scene` preserves the existing cosmetic replay path and does not silently add physical geometry to historical GIFs. Rendering never changes sensing, acceptance or driving commands. The editable `.blend` is a snapshot of the final rendered state, not a sensor simulator or a baked animation.

## Implemented and remaining work

Implemented: optional native static cuboid query geometry, height-specific native ray evidence, simulator-side conservative capsule acceptance, an independent geometric checker, sensor-only replay, and audited visualization of those same cuboids.

Not implemented: volumetric operational perception, 3D object tracking, a calibrated rectangular vehicle body, tire/road contact response, suspension, elevation changes, general mesh scenes, cameras, semantic recognition, real-vehicle safety validation or real-time guarantees. The next perception step must use measured multi-height returns with an explicit sensor contract and independent blind-obstacle acceptance, rather than feeding scene labels or simulator geometry to the driver.
