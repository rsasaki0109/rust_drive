# Inclined native 3D LiDAR

The optional native `--lidar-3d` mode acquires actual inclined XYZ ray returns from the CPU-only RNE/Rapier query world. Its 720 azimuth columns and 16 elevation rings span **−15° to +15°**. This measures geometry between the preceding three horizontal planes, including an authored barrier at 1.4–1.6 m height that the 0.15, 0.6 and 3.7 m planes miss.

Driving still uses a **measured-height gate followed by planar projection, clustering, tracking and planning**. Ego motion remains the native planar Ackermann plant. This is not volumetric object perception, object-height inference, 3D mapping, suspension or contact response.

**The current scenes have no physical road/ground collider.** The rendered road and suburban scenery remain cosmetic. Ground points at Z=0 would fall inside the configured collision-height interval and be projected as occupied obstacles: there is no ground segmentation or road extraction. These authored flat-road query fixtures do not establish that the pipeline can consume general real-world road point clouds.

## Run and replay

Use the development branch and the existing [CPU-only RNE setup](../README.md#cpu-only-robot-native-engine-demo).

```sh
source scripts/env.sh
bash scripts/setup-rne.sh
cargo +1.95.0 run --release --locked \
  --manifest-path integrations/rne/Cargo.toml -- \
  --scenario scenarios/native-scene-mid-stop.json \
  --scene scenes/midbeam-barrier.json --lidar-3d \
  --plant dynamic --seed 7 --output artifacts/lidar-3d-mid
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/lidar-3d-mid/sensors.jsonl \
  --output artifacts/lidar-3d-mid/replay
python3 scripts/check-lidar-3d.py --compact \
  --output artifacts/lidar-3d-scenes
```

`--lidar-3d` requires `--scene` and is mutually exclusive with `--multi-height`. The adapter writes the usual run, summary and sensor log, plus `scene.json` physical-scene evidence. Sensor replay reconstructs projection and all subsequent driving outputs from the raw XYZ returns and logged calibration. It does not read simulator scene dimensions or inject obstacle labels.

The compact checker mode verifies each case, archives its full evidence and removes that case's expanded outputs to limit disk usage. A complete report, rather than the configured case list, establishes the observed pass/fail results.

## Calibrated beam and coordinate contract

The optional pipeline configuration field `lidar3d` contains:

```json
{
  "azimuth_columns": 720,
  "elevation_rings": 16,
  "min_elevation_rad": -0.2617993877991494,
  "max_elevation_rad": 0.2617993877991494,
  "mount_height_m": 0.6,
  "min_range_m": 0.2,
  "max_range_m": 45.0,
  "collision_bottom_m": -1.15,
  "collision_top_m": 2.35
}
```

`SensorFrame.lidar3d` is an acquisition stamp and a list of `returns`, each containing `ray_index` and `point: {x, y, z}`. XY are body forward/left; Z is up above the calibrated road datum and includes the 0.6 m mount height. Z varies with the actual measured range and inclined beam. It is not a horizontal-plane label or an inferred object height. The native adapter uses yaw-only orientation over a flat datum, rather than arbitrary pitched/rolled sensor extrinsics.

The firing ordinal is `column × elevation_rings + ring`. Azimuth is `−π + 2π × column / (azimuth_columns − 1)` and elevation interpolates uniformly between the configured endpoints. Both azimuth endpoints are included. The body beam direction is `(cos(e) cos(a), −cos(e) sin(a), sin(e))`; measured Z is `mount_height + range × sin(e)`. The 16 native rings are separated by 2°, with −1°/+1° adjacent to horizontal and no exact 0° ring.

Each acquisition is one instantaneous full sweep of **11,520 firing ordinals**, with first returns only. Missing ordinals mean no return; an adapter acquisition error must set `lidar_failed` instead of supplying an empty healthy cloud. Native radial range noise has a configured 0.008 m standard deviation, without angular jitter. The simulator does not advance within the sweep. There is no per-return motion timestamp or scan-motion deskew.

The generic shared calibration bounds columns to 2–2048, rings to 2–64 and their product to 20,000. Elevation endpoints must lie within ±60°, have a span of at least 10⁻⁴ rad, and all calibration values must be finite. Mount and collision heights are bounded to −5–10 m, with at least 0.1 m collision span. The range interval is bounded and must be at least 0.1 m wide. The current native CLI uses the fixed configuration above.

## Return validation and planar projection

Every return must have a unique in-range ordinal and finite XYZ coordinates. The measured range is the norm of `(x, y, z − mount_height)` and must lie in the calibrated interval, with a 10⁻⁷ m numerical tolerance. Each normalized direction component must agree with its ordinal's calibrated beam within 10⁻⁴. At most the configured ray count, and never more than 20,000 returns, may be supplied. All returns are validated before height selection, including returns later excluded from planning.

The height gate selects the actual measured Z coordinate inclusively within the configured collision interval. With the current virtual capsule's 0.1–1.1 m axis and 1.25 m radius, that interval is **−1.15–2.35 m**. The capsule extends below the displayed ground and is not a calibrated physical car body. [Native collider and acceptance boundaries](native-scenes.md#scene-coordinates-and-acceptance).

Selected returns are sorted by firing ordinal. The first measured point in each **5 cm** body-XY voxel becomes a point in the ordinary planar `LidarScan`. That cloud uses the acquisition-time EKF pose and the existing 2D perception, tracker, predictor, occupancy grid, circular collision envelopes and planner. Scene geometry and truth poses are independent evaluation inputs, not driving inputs.

Ordinary, multi-height and inclined-3D configurations/inputs are exclusive. Future, negative or non-finite acquisition stamps and unavailable acquisition poses are invalid; duplicate or older stamps cannot refresh freshness. The existing pose history covers at most 0.35 s without extrapolating an unavailable pose. Raw clouds are delayed, dropped or delivered atomically by the simulation sensor adapter.

As in multi-height mode, an acquisition, malformed-input or pose-coverage failure latches braking at the current control time. Missing, duplicate and delayed pre-fault clouds cannot clear it. Recovery requires a complete valid covered acquisition stamped strictly after both the fault epoch and the last accepted scan. Optional `lidar3d` fields are omitted when absent, preserving the preceding log paths.

## Independent physical checks and FOV limits

The complete local matrix passed: **36 positive episodes and 12 required rejections**, across both native plants and seeds 1, 7 and 42. Positive scenes cover ground, rotated, raised, low, sub-low and midbeam obstacles. The retained failures are six prior multi-height midbeam episodes and six close upper-capsule episodes outside initial inclined-scan coverage. [Compact result asset](../assets/lidar-3d-results.json), the complete report and [validation history](validation.md) record the actual results and exact source/input hashes.

The sweep replayed **28,861 sensor ticks**, checked 288,178 native motion samples and reconstructed 151,418,880 inclined firing ordinals, including misses, across 13,144 XYZ acquisitions. It verified 2,233,363 genuine returns, with maximum range residual 0.037587 m below the 0.06 m physical-ray tolerance. Midbeam positives retained at least 6.219278 m capsule-guard clearance; sub-low positives retained at least 4.727081 m. The overall positive minimum was 1.15 m in the raised-obstacle fixtures.

The independent XYZ oracle rejected 112 mutations. Rust sensor replay rejected 96 of 98 wire-mutation variants; two omissions of excluded overhead returns left driving computations unchanged and were still rejected by the physical measurement oracle. Reproducible driving outputs alone do not establish that all raw measurements match the authored physical scene.

The checker uses an independent 3D slab-ray oracle against the authored cuboids. It checks nearest hits and misses for all calibrated ordinals, measured ranges, XYZ/range reconstruction, raw sensor-log correspondence, physical capsule clearance and sensor-only replay. It reconstructs expected height-selected 5 cm voxel counts; shared projection tests and replay exercise the operational projection, whose internal cloud is not directly exported for this checker. The new matrix contains static cuboids only and does not establish inclined-3D sensing or prediction of moving traffic.

Native `scene.json` records `operating_mode: "lidar3d"`, the calibration, and `cloud_3d` acquisition evidence with 11,520 nullable ranges and actual typed XYZ returns. The preceding horizontal channels remain diagnostic comparison evidence rather than driving inputs in this mode. This checker inspects their metadata and hit counts; it does not independently re-raycast those horizontal diagnostics. Their previous acceptance evidence and the unchanged legacy-mode comparison remain separate.

Elevation FOV and discrete rings still leave blind regions. The close upper obstacle at x=5 m, with bottom height 2.2 m, lies above the initial +15° coverage from a 0.6 m mount. In dynamic seed 7, the first physical guard overlap occurs at **1.97 s** and the first XYZ return at **3.6 s**, with zero returns before that overlap. Some rays observe it later, after it has already been passed; whole-run absence is not claimed. Late detection does not undo the earlier physical failure.

Detecting the midbeam and sub-low fixtures does not establish continuous height coverage or general obstacle safety. Occlusion, range limits, discrete angular sampling and the absence of ground segmentation remain material limits. No real-sensor calibration, general terrain handling, object classification, volumetric tracking or real-time performance is demonstrated.

## Audited 3D replay

![Actual inclined-LiDAR RNE run stopping before a barrier between the legacy horizontal scan planes](../assets/lidar-3d-demo.gif)

The published dynamic-plant recording uses seed 7 and retains **35 seconds / 701 sensor ticks**. It contains **22,690 genuine XYZ returns** across 351 acquisitions. The first inclined return arrives at **4.9 s**; all three legacy horizontal diagnostic channels have zero hits in this episode. Ego stops with **6.770538 m** minimum physical capsule-guard clearance and zero overlaps. Twenty emergency ticks and a maximum of three planar tracks remain visible in the telemetry: one physical cuboid is not necessarily one segmented 3D object. The fixture expects a stop, rather than goal arrival through the barrier.

The GIF contains **118 audited states / 118 encoded frames**, at 960 × 640 pixels and 3× playback with a final pause. Eight Cycles CPU samples and three threads produce a 1.81 MB asset. All 351 recorded display poses, 118 physical-cuboid mesh states and the recorded XYZ/range/ordinal consistency are checked. [GIF provenance](../assets/lidar-3d-demo.json) includes the exact native calibration, raw-cloud counts, projection rule and input/renderer SHA-256 values.

Install Blender and activate the Pillow environment from [3D replay setup](3d-demo.md#reproduce-the-closure-detour-gif), then render the accepted recording:

```sh
python3 scripts/render_demo_3d.py artifacts/lidar-3d-mid/run.json \
  --native-scene artifacts/lidar-3d-mid/scene.json \
  --output artifacts/lidar-3d-mid/demo.gif --samples 8 --threads 3
```

The orange cuboid uses its actual center at 1.5 m and actual 0.2 m vertical thickness, without visual exaggeration or invented supports. The renderer audits the recorded inclined cloud's calibration, ordinal/XYZ/range consistency, body poses and actual world-space cuboid mesh corners. It renders vehicle and geometry replay; it does not depict synthesized sensor rays as measurements. Provenance records the accepted trace and scene hashes, calibrated XYZ acquisition counts, planar projection rule, instantaneous scanning and no deskew. The independent physical ray checker remains separate from these visualization consistency checks.
