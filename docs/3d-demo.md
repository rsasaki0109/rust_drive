# 3D replay of actual RNE driving

The README opening GIF and follower GIF now show a perspective 3D scene rendered from successful RNE dynamic recordings. Blender Cycles runs on CPU, with no GPU, display server, ROS or CARLA requirement. Simulation and sensor replay remain renderer-independent.

![Actual RNE closure-detour run rendered in 3D](../assets/rne-3d-demo.gif)

Blue is ego, amber is a recorded obstacle or reactive follower, teal is the actual planned trajectory and purple is a tracked motion forecast. The red line is a **map closure overlay**, not a physical barrier. Cosmetic vehicle meshes, road markings, illumination and the camera are visualization only; they do not enter sensors or collision acceptance.

## Reproduce the opening GIF

Install Blender and activate the Pillow environment from the README. Blender **4.3.2** with Cycles CPU is verified locally. Rendering uses 16 samples per pixel and four CPU threads by default. No OpenImageDenoise or graphics driver is required.

```sh
bash scripts/setup-rne.sh
bash scripts/rne-3d-demo.sh
# Intentional README asset regeneration:
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
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/follower-3d/sensors.jsonl --output artifacts/follower-3d/replay
python3 scripts/render_demo_3d.py \
  artifacts/follower-3d/run.json --output assets/prediction-3d-demo.gif
```

## Scene verification and provenance

The Blender worker applies each sampled ego position/yaw and each active object position from the recorded truth. It exports an audit from the **actual scene transforms after application**, not a copy of the requested poses. The packager checks object identity/count, ego yaw within 10⁻⁵ rad and planar positions within 10⁻⁴ m to accommodate Blender's float32 transforms. Playback samples approximately every 0.3 simulation seconds at ten GIF frames per second, followed by a final pause. The GIF uses a shared 192-color palette, no dithering and a mild 3×3 median filter confined to the 3D viewport to reduce rendering noise and download size. HUD text and the recorded timeline are preserved. Identical encoded frames may merge; total duration and resolution are checked.

Provenance records the actual RNE backend, pinned engine revision, complete scenario/summary, input trace SHA-256, renderer command, Blender version, sample count, number of verified scene states and encoded GIF frames. Every render uses a fresh temporary frame directory; stale frames cannot fill gaps in a new capture. Failing, non-RNE or unsupported-schema runs are rejected before rendering. GIF bytes can vary with Blender, fonts and sampling versions.

The RNE CI job renders a real native mission frame with Cycles CPU after physical scenario/replay checks. Rendering remains outside Cargo's dependencies and the required Rust-only workflow. Full GIFs are generated and inspected locally; CI's 3D check is a single-frame smoke test.

## Limits

This is **3D visualization of a planar driving simulation**, not full 3D driving physics, an RNE renderer capture, camera perception, road elevation/suspension/contact-response validation or a photorealistic sensor feed. The road and vehicle meshes do not replace the independent circular collision evaluator. RNE still integrates ego natively and uses Rapier LiDAR; the reactive follower still uses the shared one-dimensional traffic integrator. Rendering never influences a driving command.

The diagnostic top-down renderer and its historical recordings remain available through `scripts/render_demo.py`. Neither rendering style establishes real-vehicle safety, traffic-rule compliance or real-time performance.
