# Japanese urban intersection and construction replay

The README Hero is an authored CPU RNE simulation styled as a Japanese urban
street. Original storefronts, balconies, air conditioners, horizontal signals,
left-side road markings and a guarded work zone dress the recorded drive. It
is not a surveyed Japanese city or a claim of complete Japanese traffic-law
compliance. The previous [urban Hero](../assets/city-demo.gif) is preserved.

The new recording includes 23 physical road-user proxies: seven passenger
vehicles, three trucks, eight pedestrians, four bicycles and a dog. Original
meshes include a child, a parent pushing a stroller with a visible baby, an
elder with a cane, a dog walker and a worker with a hardhat and shovel. Actor
roles are renderer metadata, not operational semantic classifications.

## Intersections and construction

The ego follows the +X route at lateral 0 m; the opposing lane is at −8 m,
placing both directions on their left side of the main road center at −4 m.
A two-way crossing street is centered at x=15 m with half-width 3.4 m.
The crossing car travels +Y at x=13.8 m, within its left-side half. The
second crossing is a 2.7 m wide one-way +Y alley centered at x=36 m; its
car follows that center with a 2.2 m SI mirror span inside the street.
Street geometry and directional markings match these actual recordings.

Mapped main-route signals at s=11 m and s=33 m change to green at 8 and
24 seconds. Their display heads use the recorded colors. The independent
checks require actual red holding, fresh green crossings, recorded crossing
traffic clearance and the full ego body exiting the second junction. In the
first design, the second intersection extended beyond the final stopping
position. That design trial is retained separately; the final intersection
layout explicitly checks complete exit rather than equating entry with passage.
The [failed scenario](baselines/japan-city-v1/scenario.json) and
[unchanged scene snapshot](baselines/japan-city-v1/scene.json) reproduce the
first trial with the same native CLI flags; they are not positive acceptance
fixtures. Its longest red standstill was 6.4 seconds against the unchanged
8-second requirement.

A second layout passed its narrower ego-only gates, but visual review found
the lead sedan waiting partly inside the first junction. Its
[scenario](baselines/japan-city-v2/scenario.json),
[positive bounded report](baselines/japan-city-v2/results.json) and
[review rejection](baselines/japan-city-v2/review.json) are preserved. The
final layout separates the junctions so ego and the queued sedan can wait
fully between them. The checker now independently rejects stationary SI
footprints of all three queued forward vehicles inside either junction,
and verifies ego's complete body throughout its red queue hold. Both stop
lines are placed before their respective street envelopes.

A [third trial](baselines/japan-city-v3/result.json) passed sensing, replay,
clearance and signal checks but failed the new no-stationary-body junction
gate: conservative eight-second forecasts briefly projected an opposing
track across ego's route, stopping it partly inside the second wide street.
The [scenario](baselines/japan-city-v3/scenario.json) remains reproducible.
The final authored alley places this braking outside its crossing envelope;
its traffic timing and adjacent pedestrian position are explicitly changed.
The planner's conservative forecast and emergency braking are retained.
This is a scenario-layout correction, not a fix for long-horizon lateral
tracking drift or proof of arbitrary intersection handling.

A real native static cuboid represents the construction obstruction near
x=64 m. Its shape participates in sensing and independent clearance evaluation.
Cones, fencing and work-zone markings are decorative; the worker and parked
truck have separately recorded physical proxies. Forward queued cars start
farther downstream in this new fixture to avoid spawning inside the obstruction.
The episode stops before the work zone. It does not implement construction-zone
negotiation, an automatically chosen construction detour or a perception label
for roadworks. All three seeds have zero returns from the work-zone cuboid because traffic occludes it, so this episode does not demonstrate construction detection.

## Evidence and boundaries

[Physical results](../assets/japan-city-demo-results.json) retain full sensor
replay, native inclined LiDAR/ray and measured-ground checks, native ego-body
checks, all 253 actor pairs and all actor-to-construction clearance checks.
The existing 1 m clearance floor and oracle tolerances are preserved. Native
ego motion remains planar; actors use physical capsule/circle proxies rather
than contact-enabled avatar meshes.

[Japanese display audit](../assets/japan-display-audit.json) measures worker
root scale, hand/shovel contact and 10 display poses, plus raised sidewalk
exclusion of all 2,436 raised sidewalk faces from both live junction rectangles.
The second alley has no two-way center divider and three measured +Y arrows;
its crossing hatchback measures 2.184 m wide including wheels and mirrors,
leaving 0.258 m of display clearance on each side. These are display-fit
measurements, separate from native clearance acceptance. The car, truck/dog and family
[SI mesh audits](road-users.md) remain applicable. Storefronts and animation
are original Apache-2.0 display geometry. Optional Japanese road lettering
uses an installed OFL-1.1 Noto CJK font; no font or external model is downloaded
or redistributed by the renderer, and stop-sign lettering is not added to
signal-only lines.

[GIF provenance](../assets/japan-city-demo.json) records renderer and transitive
asset hashes, actual trace/scene hashes, recorded pose checks and fixed camera
scale. Cosmetic exclusion paths are compressed only when they represent the
same stationary or axis-aligned polyline; measured vehicle/actor poses are
unchanged. Scenery and road markings do not add operational map lanes or rules.
A completed display trial was [rejected on visual review](baselines/japan-display-v4/review.json)
because legacy pedestrian-demo zebra markings overlapped the final stopping
position. The Japanese renderer now uses junction-specific crosswalks only;
their measured mesh bounds stay inside the independently validated junction
envelopes. Actual trajectories, clearance floors and braking are unchanged.
Unmarked prescribed pedestrian crossings remain unmarked; this is not a
pedestrian route-planning or general crosswalk-right-of-way implementation.


The three accepted seeds (7, 1, 42) independently verify all 2,763 sensor
replay ticks, 3,983,040 beam slots and 27,603 native 200 Hz body poses.
Red-queue standstill lasts at least 10.50 seconds. The final conservative
body rear clears the second street by at least 1.353237 m; no stationary
ego body sample occupies either junction. All 253 actor-pair clearance
bounds retain the 1 m floor (minimum 1.003840 m). The
[local validation record](../assets/japan-city-validation.json) also separates
physical results, display measurements and complete GIF capture.

## Reproduction

Follow the [README commands](../README.md#reproduce-the-gif) to run the native
46-second fixture, replay all 921 observations and render the complete episode.
The opening GIF uses seed 7, `urban-japan`, the street camera and 3× playback.
The encoded GIF has 155 frames at 960×640, lasts 16.8 seconds including
the final hold, and is 7,509,338 bytes. Run the independent physical matrix
and display audit with:

```sh
python3 scripts/check-japan-city-demo.py --compact --output artifacts/japan-city-demo
blender --background --factory-startup --threads 2 --python-exit-code 1 \
  --python scripts/check_japan_display.py -- --output artifacts/japan-display-audit.json \
  --physical-report assets/japan-city-demo-results.json
```

