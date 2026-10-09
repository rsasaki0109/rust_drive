# Measured-ground LiDAR

The opt-in native `--lidar-3d --ground-segmentation` mode measures a physical flat road and obstacles with inclined RNE/Rapier rays. The shared Rust pipeline fits a bounded ground plane from the actual XYZ returns, removes only locally supported points near that measured plane, and passes the remaining height-selected XY points into the existing perception, tracking, prediction and planning stack.

This extends the earlier [inclined LiDAR fixtures](lidar-3d.md), whose road was cosmetic and whose driving projection did not separate ground. The new mode uses **SceneV2 `ground_cuboids` with physical top surfaces at road datum Z=0**. Ground/obstacle geometry and simulator truth are evaluation inputs; they are not supplied to the ground fitter or operational detector as point-role labels.

Driving remains planar. Ground fitting is not general terrain reconstruction, volumetric perception, object classification or a claim of real-world driving safety. The native plant has yaw-only body and sensor orientation, without roll, pitch, heave, tire contact response or suspension.

## Run and replay

Use the existing [CPU-only RNE setup](../README.md#cpu-only-robot-native-engine-demo), then run the authored physical-road obstacle fixture:

```sh
source scripts/env.sh
bash scripts/setup-rne.sh
cargo +1.95.0 run --release --locked \
  --manifest-path integrations/rne/Cargo.toml -- \
  --scenario scenarios/native-ground-stop.json \
  --scene scenes/ground-midbeam.json \
  --lidar-3d --ground-segmentation \
  --plant dynamic --seed 7 --output artifacts/ground-mid
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/ground-mid/sensors.jsonl \
  --output artifacts/ground-mid/replay
python3 scripts/check-ground-scenes.py --compact \
  --output artifacts/ground-scenes
```

`--ground-segmentation` requires `--lidar-3d`, which requires `--scene`; multi-height sensing remains exclusive. The mode writes raw typed XYZ sensor logs and `scene.json`, including the calibration and acquisition-time ground fit diagnostics. Replay recomputes the fit, projection and driving outputs from those raw measured returns; it does not read the scene sidecar.

## Beam and measured-ground contract

The new native mode uses 180 azimuth columns and 16 uniformly spaced elevation rings from −15° to +15°, with a 0.6 m mount, 0.2–45 m range, instantaneous acquisition and 0.008 m radial noise. The [XYZ and firing-ordinal contract](lidar-3d.md#calibrated-beam-and-coordinate-contract) is retained. Every return is validated for finite coordinates, unique firing ordinal, calibrated range and beam direction before any ground or height selection. There is no scan-motion deskew. Calibration recorded in a particular accepted run is authoritative for that run.

Ground fitting uses the optional `lidar3d.ground` calibration:

```json
{
  "reference_height_m": 0.0,
  "max_slope": 0.05,
  "max_height_offset_m": 0.03,
  "residual_threshold_m": 0.02,
  "fit_radius_m": 8.0,
  "min_inliers": 200,
  "min_sector_inliers": 20,
  "min_cell_inliers": 6
}
```

These values are bounded priors and support requirements, not a supplied ground plane. Candidate returns lie 0.5–8 m from the sensor in XY and within the configured prior height band. A deterministic set of at most 48 hypotheses uses spatially spread measured anchors; up to eight trimmed least-squares refinements fit `Z = aX + bY + c` to measured inliers. The inlier mask must stabilize; a non-converged fit fails confidence, reports no removals and latches braking instead of supplying a normal projected scan. A converged fit must satisfy the slope and height-offset bounds, have at least 200 inliers, and retain at least 20 inliers in each of eight angular sectors.

After the fit passes, returns within its 2 cm residual band are removable only in their own supported cell: eight angular sectors and 5 m radial bands, with at least six nearby-plane returns per cell. A point near an unsupported portion of the plane is preserved. This avoids treating an extrapolated plane as evidence of local road support. The 10 cm slab fixture can have lower-face returns within the ground residual band removed; its higher measured surfaces remain and cause a stop. Ground fitting does not promise zero obstacle-surface removal. Preserved returns pass through the configured measured-Z collision interval and deterministic firing-ordinal / 5 cm body-XY voxel projection. Downstream clustering and tracking remain two-dimensional.

`PipelineOutput.ground` records the acquisition stamp, fitted plane, candidate/inlier counts, eight sector counts, supported-cell count, removed/preserved return counts, maximum/RMS inlier residual and fit confidence. Confidence describes geometric support; acquisition validity and freshness are separate. It is not a probability of safe drivable terrain.

An unavailable or unsupported fit causes invalid-LiDAR braking. Advanced-sensor faults remain latched: missing, duplicate or delayed pre-fault clouds cannot clear them. Recovery requires a complete valid supported acquisition stamped strictly after the fault control epoch and last accepted scan, with an available acquisition-time localization pose. The existing 0.35 s acquisition history and no-extrapolation rules still apply. Ordinary sensor modes omit the new optional fields.

## Research body envelope

`--vehicle-body` additionally requires ground mode. The optional body envelope is an authored **4.2 × 1.8 × 1.5 m upright cuboid**, bottom at 0.15 m, centered in XY on the native planar plant reference. These are research dimensions, not measured specifications for a real vehicle or a rear-axle calibration. Its top is 1.65 m. The body-mode sensing gate additionally includes a 1 m clearance reserve, from −0.85 to 2.65 m; that gate is distinct from the physical body dimensions. The ordinary planar planner uses the box's circumscribed 2.284732 m circular radius, and body-mode routes require at least 3 m half-width. This is a conservative planar approximation, not oriented-body motion planning.

A conservative independent upright-box guard checks recorded native translation and yaw against static obstacles, including swept translation/rotation allowance between 5 ms plant samples. Physical road support is excluded from obstacle-clearance acceptance. Force-free native sensor overlap witnesses can identify obstacle intersections without claiming chassis/contact-force simulation. In body mode, `scene.json` records `operating_mode: "lidar3d_ground_body"` and `body_guard`; its body report supplies physical acceptance, while the older capsule result is retained as `capsule_diagnostic`. The older broad capsule is not a calibrated car body. A render wire envelope shows the actual research box separately from the cosmetic vehicle model. Moving actors still use native capsule query proxies and circular planar traffic clearance; this does not establish swept cuboid-to-cuboid traffic acceptance. With no static obstacle cuboids, body-guard clearance is null rather than a measured large distance.

## Verification and limits

[Complete measured matrix, source/input hashes and mutations](../assets/ground-results.json).

The authored scene separates physical road cuboids from obstacle cuboids to build and independently audit the query world. Operational sensor inputs contain only calibrated raw measured points. Independent ray reconstruction, raw sensor-log correspondence, measured fit checks, sensor-only replay and body/capsule guard evidence serve different purposes; matching rendered geometry does not replace them.

The completed ground/body matrix verifies **66 episodes: 36 original-envelope ground episodes and 30 authored-body episodes**, across both native plants and seeds 1, 7 and 42. Sixty episodes have confident sensing and complete goal arrival or a sensed stop; six separate narrow-support episodes verify confidence-failure braking. Previous inclined-scan blind-zone failures remain separate evidence; this ground matrix does not establish full sensor coverage. Discrete inclined beams, occlusion, finite range and restricted elevation FOV still leave unobserved regions. Ground fitting does not remove those limits. Native f32 capsule queries can also differ from an ideal f64 surface at grazing boundaries. The checker labels independently reproduced ≤0.1 mm empirical capsule-boundary ambiguity separately from strict reconstructed hits/misses; this threshold is not a certified numerical bound. The [retained grazing proof](../scripts/fixtures/native-capsule-grazing-proof.json) records that numerical limitation.

The bounded fitter assumes a locally supported near-flat road datum. Sparse ground, sharp changes, curbs, elevated platforms, vegetation, arbitrary sensor extrinsics, real-world scan distortion and general terrain classification are not established by these fixtures. Very shallow objects within the residual/support band can be mistaken for ground; exact geometry and sensor resolution still matter. The authored support patch can be 100 × 50 m while the mapped driving corridor has a much smaller half-width; support geometry is not lane semantics. A physically narrow road may lack the eight-sector near-ground support needed for a confident fit, and unsupported far cells preserve ground as possible obstacles, causing conservative false blocking. Flat road cuboids provide LiDAR query surfaces, not a demonstrated tire/road dynamics model.

## Audited replay rendering

![Native ground-measured RNE run follows a moving capsule lead into a stop, with a separate research body wire envelope](../assets/ground-demo.gif)

The published dynamic-plant, seed-7 episode contains **22 seconds / 441 sensor ticks**, with **302,290 genuine XYZ returns** across 221 acquisitions. All 221 measured ground fits are confident. The fitter removes 294,135 returns and preserves 8,155; the independent oracle classifies 7,263 measured returns from the lead capsule, of which **7,258 survive**. Five near-plane actor returns are removed, so successful stopping does not imply perfect ground/actor classification. The checker reconstructs 4,726 expected height-selected XY voxel points across the episode; this is separate from exporting the pipeline's internal projected cloud.

Ego stops with **5.715014 m minimum circular traffic clearance**, zero recorded collisions and zero road violations. The lead uses native capsule query geometry; traffic acceptance uses conservative circumscribed-circle sweeps. This scene has no static obstacle cuboids, so its static-body guard has zero obstacle checks and a **null minimum clearance**, rather than a demonstrated swept-box traffic clearance. The separate body-obstacle fixtures in the complete matrix test the upright box guard and force-free static-obstacle witnesses.

The GIF contains **75 audited / decoded frames**, at 960 × 640 pixels, 3× playback and an 8.8 s encoded duration including the final pause. Eight Cycles CPU samples and three threads produce a 1.49 MB asset. It checks all 441 recorded display poses, plus exact road and body-envelope corners at all 75 displayed states. [GIF provenance](../assets/ground-demo.json) records calibrated beams, ground diagnostic consistency, scope limits, the accepted trace/scene hashes, renderer fingerprint and decoded GIF hash/duration.

The ground-mode renderer draws physical road cuboids at their exact recorded top height of zero, along with actual obstacle cuboids when present. Decorative markings, buildings, trees and vehicle models remain visual scenery. The separate body wireframe uses recorded research calibration and native pose. The cosmetic car can extend beyond this wire envelope, and its roof sensor housing is decorative: actual sensor extrinsics come from the recorded 0.6 m calibration, not model geometry.

Reproduce the published two-vehicle case after the setup above, then use Blender/Pillow from [3D replay setup](3d-demo.md#reproduce-the-closure-detour-gif):

```sh
cargo +1.95.0 run --release --locked \
  --manifest-path integrations/rne/Cargo.toml -- \
  --scenario scenarios/native-body-traffic-stop.json \
  --scene scenes/ground-moving-traffic.json \
  --lidar-3d --ground-segmentation --vehicle-body \
  --plant dynamic --seed 7 --output artifacts/ground-moving
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/ground-moving/sensors.jsonl \
  --output artifacts/ground-moving/replay
python3 scripts/render_demo_3d.py artifacts/ground-moving/run.json \
  --native-scene artifacts/ground-moving/scene.json \
  --output artifacts/ground-moving/demo.gif --samples 8 --threads 3 \
  --camera traffic --traffic-models sedan
```

The renderer audits raw XYZ/ordinal/range consistency, calibrated beams, measured-fit count/plane consistency, exact road/obstacle meshes and recorded body poses. Its fit-metadata audit does not independently reconstruct the operational fitter; the acceptance checker and sensor replay provide that separate evidence. The replay does not draw invented sensor rays or infer operational object labels from simulator geometry.
