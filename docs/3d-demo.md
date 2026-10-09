# 3D replay of actual RNE driving

The README opening GIF records a left-side urban drive with two actual infrastructure signals and nineteen road users: passenger cars, trucks, seven pedestrians (including an elder with a cane, a child and a parent with a stroller), four cyclists and a leashed dog. Fixed SI car/truck dimensions replace the old radius-scaled vehicle meshes. A fixed-scale orthographic camera follows the recording against original city scenery. [Road-user checks, meshes and commands](road-users.md). Prior three-actor, ground/body, closure-detour and follower GIFs remain historical recordings. Blender Cycles runs on CPU with no GPU, display server, ROS or CARLA requirement. Simulation and sensor replay remain renderer-independent.

![Actual RNE closure-detour run rendered in 3D](../assets/rne-3d-demo.gif)

Blue is ego, amber is a recorded obstacle or reactive follower, teal is the actual planned trajectory and purple is a tracked motion forecast. The red line is a **map closure overlay**, not a physical barrier. Cosmetic vehicle meshes, road markings, illumination and the camera are visualization only; they do not enter sensors or collision acceptance.

## Models and suburban scenery

The original procedural assets in [`blender_assets.py`](../scripts/blender_assets.py) create hatchbacks, sedans, cargo vans and pickups with shaped body panels, sloped glazing, pillars, mirrors, door handles, bumpers, a grille, head/tail lights and five-spoke alloy wheels. Vans add cargo panels and rear-door seams; pickups add an open bed, rails and a tailgate. Ego has a decorative roof sensor housing; this does not calibrate the actual LiDAR mount. Wheel rotation follows recorded travel distance. Current vehicle meshes use independent SI presets rather than collision-radius scaling. Mirrors/sensor bounds are audited separately from body size. These presets are original cosmetic dimensions rather than calibrated production vehicles; older GIFs retain their historical scaling. Static circular obstacles use reflective barrel meshes.

The suburban test-road scene includes continuous raised pavement, curbs, faceted street trees, streetlights and small campus buildings with windows, sills and entrances. Placement uses the authored road corridors and a fixed scenery seed of 1729. Pavement is omitted around adjoining corridors to avoid overlapping junction surfaces. Perspective ego/traffic cameras remain available. The opening urban recording uses a fixed-scale orthographic street camera; recorded scale and pose states are checked. These assets are original geometry and materials, with no external model or texture downloads.

Scenery is **display-only**: buildings, trees and street furniture are not added to RNE's sensor or collision world. Driving results, circular acceptance, road widths and actual recorded actor positions are unchanged. The separate opt-in [native cuboid scenes](native-scenes.md) render actual Rapier query geometry when `--native-scene` supplies its accepted evidence sidecar; the renderer audits those meshes against the physical boxes.

## Edit a Blender scene

`--traffic-models sedan van pickup` assigns those display types in stable actor appearance order, cycling if more actors exist. The default is `hatchback`; ego remains a blue hatchback. `--camera traffic` frames ego and active reactive vehicles together; the default `ego` camera remains available. Neither option changes simulation inputs. [A verified three-vehicle queue and complete regeneration commands](vehicle-fleet.md).

Export a scene snapshot from a recorded run, then open the `.blend` file in Blender to edit vehicle meshes, materials, scenery or camera placement:

```sh
python3 scripts/render_demo_3d.py artifacts/rne-3d/run.json \
  --preview-time 7 --output artifacts/3d/suburban-preview.gif \
  --scene-output artifacts/3d/suburban-scene.blend
blender artifacts/3d/suburban-scene.blend
```

The preview is a PNG. `--scene-output` saves editable objects at the last rendered state; it is a snapshot, not a baked animation or simulator project. Export is optional and does not affect Rust dependencies. Full GIF regeneration continues to use the recorded timeline. CI exports a CPU-rendered native scene as an artifact.

## Reproduce the closure-detour GIF

Install Blender and activate the Pillow environment from the README. Blender **4.3.2** with Cycles CPU is verified locally. Rendering uses 16 samples per pixel and four CPU threads by default. No OpenImageDenoise or graphics driver is required.

```sh
bash scripts/setup-rne.sh
bash scripts/rne-3d-demo.sh
# Intentional historical closure-detour asset regeneration:
bash scripts/rne-3d-demo.sh assets/rne-3d-demo.gif
```

The helper executes `route-handover-fast`, dynamic plant, seed 7, against the pinned RNE engine. It independently verifies the complete sensor log before invoking the renderer. Output defaults to `artifacts/rne-3d/demo.gif`, with a PNG poster and JSON provenance. Required Rust toolchains, engine revision and driving algorithms are unchanged.

Render an existing successful RNE trace directly:

```sh
python3 scripts/render_demo_3d.py \
  artifacts/rne-3d/run.json --output assets/rne-3d-demo.gif
# A quick single-frame preview writes a PNG and does not produce a GIF:
python3 scripts/render_demo_3d.py \
  artifacts/rne-3d/run.json --preview-time 22 --output artifacts/3d/preview.gif
```

The wrapper also accepts renderer options after its output/scenario/trace-directory arguments:

```sh
bash scripts/rne-3d-demo.sh artifacts/3d/smoke.gif \
  scenarios/route-handover-fast.json artifacts/3d/smoke \
  --preview-time 22 --samples 4 --threads 2
```

## Reproduce the follower GIF

The second GIF retains the short follower fixture's original **65-second deadline**, **eight-second residence** and **1 m clearance floor**. Native seed 1 finishes at 64.25 seconds. Its measured improvement and remaining repeated stops are in [observed braking](observed-braking.md).

```sh
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --scenario scenarios/traffic-follower-deadline.json --plant dynamic --seed 1 \
  --output artifacts/follower-3d
cargo run --release --locked --bin rustdriving -- replay \
  --log artifacts/follower-3d/sensors.jsonl --output artifacts/follower-3d/replay
python3 scripts/render_demo_3d.py \
  artifacts/follower-3d/run.json --output assets/prediction-3d-demo.gif
```

## Scene verification and provenance

The Blender worker applies each sampled ego position/yaw and each active object position from the recorded truth. It exports an audit from the **actual scene transforms after application**, not a copy of the requested poses. The packager checks object identity/count, ego yaw within 10⁻⁵ rad and planar positions within 10⁻⁴ m to accommodate Blender's float32 transforms. Playback samples approximately every 0.3 simulation seconds at ten GIF frames per second, followed by a final pause. The GIF uses a shared 192-color palette, no dithering and a mild 3×3 median filter confined to the 3D viewport to reduce rendering noise and download size. HUD text and the recorded timeline are preserved. Identical encoded frames may merge; total duration and resolution are checked.

Provenance records the actual RNE backend, pinned engine revision, complete scenario/summary, input trace SHA-256, renderer command, Blender version, sample count, number of verified scene states and encoded GIF frames. It also records the asset style, scenery seed/counts and a SHA-256 of the worker, asset generator and packager sources. Every render uses a fresh temporary frame directory; stale frames cannot fill gaps in a new capture. Failing, non-RNE or unsupported-schema runs are rejected before rendering. GIF bytes can vary with Blender, fonts and sampling versions.

The RNE CI job renders real native mission and three-vehicle fleet frames with Cycles CPU after physical scenario/replay checks. Rendering remains outside Cargo's dependencies and the required Rust-only workflow. Full GIFs are generated and inspected locally; CI's 3D check is a single-frame smoke test.

## Limits

This is **3D visualization of a planar driving simulation**, not full 3D driving physics, an RNE renderer capture, camera perception, road elevation/suspension/contact-response validation or a photorealistic sensor feed. The road and vehicle meshes do not replace the independent circular collision evaluator. RNE still integrates ego natively and uses Rapier LiDAR; the reactive follower still uses the shared one-dimensional traffic integrator. Rendering never influences a driving command.

The diagnostic top-down renderer and its historical recordings remain available through `scripts/render_demo.py`. Neither rendering style establishes real-vehicle safety, traffic-rule compliance or real-time performance.

The opening and follower assets retain their captures from `5df62d1`, including their historical renderer source hashes. The fleet addition uses the updated worker with vehicle selection and camera fitting; regenerating older captures with that worker can change their visual bytes.
