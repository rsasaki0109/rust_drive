# Recorded pedestrians and cyclists in the 3D demo

The README opening GIF shows an actual CPU RNE run with a lead car, a crossing
pedestrian proxy and a cyclist proxy. Ego yields while the pedestrian crosses
and later stops behind the lead. The original procedural display meshes add
smooth articulated human limbs, clothing, facial features, hair, shoes and a
helmet, plus a diamond bicycle frame, forks, tires, rims, 32 spokes per wheel,
chain, chainring, cranks, pedals and brake details. No external meshes, textures
or asset downloads are used; these source assets use the repository's Apache-2.0
license.

Actor ID 0 is displayed as a sedan, ID 1 as a pedestrian and ID 2 as a cyclist.
These assignments exist only in the renderer. Actual recorded positions and
headings determine their placement; limb and wheel articulation uses recorded
speed and time, without changing actor root poses. The street camera frames
local recorded road users within 30 m of ego rather than zooming out for a
cyclist that has left the local view. All recorded actor positions are audited,
including actors beyond that view. Camera fitting uses conservative cosmetic
mesh bounds, separately from the declared physical circles.

![Original pedestrian and cyclist display meshes](../assets/vru-models.png)

This close-up is a separate showroom study of the same meshes used in the GIF.
It shows clothing, articulated limbs and bicycle details at a readable scale.
The [render record](../assets/vru-models.json) retains source/image hashes and
five-pose geometry and root-position audits for each model. It is a display
study, rather than a driving capture. Reproduce it with CPU Blender:

```sh
blender --background --factory-startup --threads 2 \
  --python scripts/render_vru_models.py -- assets/vru-models.png
```

## Physical verification

The unchanged 22-second fixture is checked with seeds 1, 7 and 42. All three
native runs stop, with zero collisions and road violations, against the fixed
1 m clearance requirement. Independent continuous 200 Hz actor-distance bounds
retain at least **1.726 m** to the pedestrian, **5.423 m** to the lead and
**5.969 m** to the cyclist. Ego is stopped during the pedestrian's centerline
crossing at approximately 6.2 seconds. Full sensor-only replay verifies all
**1,323 ticks**. Independent analytic checks reconstruct **1,909,440 native
beams** and **906,015 measured XYZ returns**. Some actor returns are removed by
ground fitting, so perfect preservation is not claimed. [Exact case results and
checker/source hashes](../assets/vru-demo-results.json).

The physical actors remain declared circular/capsule proxies, and motion follows
specified trajectories. Human intent, pedestrian/bicycle semantic recognition,
avatar mesh ray casting, contact response, skeletal physics and production
vehicle calibration are not implemented. Decorative buildings and vegetation
are display assets. The meshes make the recording easier to inspect; their
realistic details do not establish more realistic sensing or road safety.

## Reproduce

From the repository root, install the optional RNE tools with
`bash scripts/setup-rne.sh` and activate `source scripts/env.sh`. Build the locked
reference/native binaries, then run the independent three-seed check:

```sh
cargo build --release --locked --bin rustdrive
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/check-vru-demo.py --compact --output artifacts/vru-demo \
  --report artifacts/vru-demo/report.json
```

Install Blender and the pinned `scripts/requirements-demo.txt` in a Python
virtual environment. Render the actual seed-7 capture:

```sh
python3 scripts/render_demo_3d.py artifacts/vru-demo/seed-7/run.json \
  --native-scene artifacts/vru-demo/seed-7/scene.json \
  --actor-models 0=sedan 1=pedestrian 2=cyclist --camera street \
  --samples 12 --threads 3 --output artifacts/vru-demo/demo.gif \
  --scene-output artifacts/vru-demo/scene.blend
```

Use `--preview-time 6` for one PNG. The saved `.blend` is an editable scene at
the last rendered state. The published GIF has 75 frames, 960 × 640 pixels and
3× playback. [Render provenance](../assets/vru-demo.json) retains input, renderer
and engine hashes, actor model assignments and scene-state verification. To
intentionally update the README GIF, select `--output assets/vru-demo.gif`.
Blender/Pillow/fonts can change visual bytes; physics and sensor replay remain
separate from rendering.

The preceding [ground/body GIF](../assets/ground-demo.gif) and its original
[provenance](../assets/ground-demo.json) remain available. This addition does not
replace its evidence or change default driving algorithms.
