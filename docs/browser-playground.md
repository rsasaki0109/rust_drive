# City Playground, urban hills and stacked roads

[Play RustDriving](https://rsasaki0109.github.io/rust_driving/) in a desktop
WebGL 2 / WebAssembly browser, or use the on-screen pointer controls on a phone.
There is no login, backend or save requirement. The static build bundles its
renderer and physics; it makes no runtime CDN or map-service requests.

The city has five connected districts on one freely drivable map: **Neon City**, **Harbor
Bay**, **Canopy Park**, **Ironworks** and **Drift Square**. The port has container
yards and cranes, the forest has six curved shortcuts, the factory has tanks and
chimneys, and the central squares leave open circles for drifting. Roads connect
every area without loading screens. Choose a starting area on the opening/pause
screen; changing it ends any active pursuit and returns the car to rest.

[Area overview](../assets/mixed-districts-preview.png) shows captured frames from
the five starting areas, after driving with keyboard input.

## Hillside city, backstreets and station loop

Choose **坂のある市街地 / HILLSIDE CITY** for the street leading into the
urban hill, or **駅前の周回道路 / CHUO STATION LOOP** for the curved boulevard.
Mixed-height buildings, shopfronts and vertical signs line the incline.
Two narrow one-way backstreets connect the hill junctions; the lower boulevard
joins the original city by its outer return road. Traffic and police share
that connected waypoint graph and follow the Rust road height.

Commuter trains run on a 10 m viaduct above the boulevard. The station platform
and a pedestrian bridge have physical upper decks and underside clearance.
The uphill avenue continues beneath the pedestrian bridge to a guarded dead end.
A normal component above 12 m/s can fracture a guardrail section: Rust spends
90 kJ of normal kinetic energy for the authored 1,250 kg car, reducing speed.
Slower impacts use the solid collider. Broken steel sections leave visible debris;
R restores the rail along with the car.

This is an original Tokyo-inspired arcade layout, rather than a geographic model.

## Elevated road and underground passage

Choose **高架道路 / SKYWAY +12 m** to climb the two 70 m ramps onto a 12 m
viaduct. **高架の下 / LOWER ROAD** starts on the perpendicular road passing
under its beams. That lower road also crosses the tunnel roof at city grade.

Choose **地下トンネル / UNDERGROUND -8 m** to descend an open cutting, drive
through the covered -8 m passage and climb the eastern exit ramp. Its roof
underside is at -1.2 m. The excavation has side walls, lit portals and ceiling
lights. The outer eastern road connects these routes to the ridge return and city.

Rust selects a reachable support layer using the preceding physical height.
Driving below a deck keeps the car on the lower road; landing from above can
reach the upper deck. Slab sides and undersides block the body. The renderer
samples the same floor/deck functions, and tire effects use the current layer.
The camera stays below overhead slabs and inside the tunnel; the eastern
panorama shows the incline from an oblique angle. R returns to the current
area's approach. These are authored arcade surfaces, not a volumetric rigid-body
or suspension simulation.

## City hill: climb, guardrail impact and landing

Choose **市街地の坂・崖ダイブ / URBAN RIDGE** on the opening/pause screen, or
drive east along the city's northernmost road. A 140 m ascent rises to a 48 m
plateau, followed by a short launch lip at 49 m. The unsupported edge drops into
a lower city district with a curved station boulevard. The lower return road loops west to the city.
Keep W/Up pressed to climb, break through the guardrail and launch; R returns to the foot of the climb and restores the rail.

Rust integrates downhill gravity on the slope and world-space ballistic motion
after support is lost. Airborne throttle, steering and handbrake add no tire
forces. Landing absorbs vertical motion and reduces horizontal speed according
to impact speed. The car mesh, camera, altitude/airtime HUD and grounded tire
effects follow the integrated height and slope. The renderer samples Rust's
authored height function directly. The minimap zooms to the ridge route.

This is an arcade height-field/contact model with a planar two-axle tire model
on the ground, not a six-degree-of-freedom rigid body or suspension model.
Authored road, pedestrian and railway decks and the tunnel roof provide upper surfaces. Traffic and police use the connected city/hillside/station street graph.

The map has 107 buildings, 54 signalized intersections and 40 independently moving
traffic cars. Chase mode adds two police cars; leave them more than 80 m behind
for 12 seconds to escape. Drift chains add score after the slide ends. All
artwork is original procedural geometry.

| Control | Action |
|---|---|
| W / Up | Accelerate |
| S / Down | Brake, then reverse |
| A / D or Left / Right | Steer and countersteer |
| Space | Handbrake: reduce rear grip |
| C / camera button | Chase, city and hood cameras |
| R / recover button | Return to the nearest road |
| Esc / pause button | Pause or resume |
| Chase / Dusk / Sound buttons | Pursuit, time of day and optional engine audio |

Releasing focus or hiding the page pauses the game and clears held controls.
Touch buttons support simultaneous acceleration, steering and handbrake input.

## Physics and scope

`integrations/playground` is a standalone Rust crate compiled to WebAssembly.
Above the low-speed blend it integrates body-forward velocity, body-lateral
velocity and yaw rate with front/rear tire slip forces. The handbrake reduces
rear tire force capacity. The rendered car uses that integrated position and
body yaw; smoke and skid marks require measured rear slip and lateral motion.
Steering the display mesh sideways is not used to manufacture a drift.

The original city ground is flat; the eastern courses add slope gravity,
ballistic flight, independently reachable upper decks and an excavated tunnel. Speeds and approximate arcade contact impulses are bounded. Traffic follows independent waypoint/signal rules; police
use the connected street graph to approach the player. It is separate from the sensor-driven
autonomous research pipeline and the pinned native RNE adapter. Browser scores
and pursuit behavior do not establish autonomous-driving capability.

## Build and verify

Use the repository's Rust 1.90.0 toolchain and Node 24:

```sh
source scripts/env.sh
rustup target add wasm32-unknown-unknown
npm ci --prefix web --ignore-scripts
bash scripts/build-playground.sh
npm --prefix web test
npm --prefix web run dev
```

Open the local URL printed by Vite. `web/public/drive.wasm` and `web/dist/` are
generated outputs. For a project Pages path, set `PAGES_BASE=/rust_driving/`
when building and previewing; ordinary local development defaults to `./`.

```sh
cargo fmt --manifest-path integrations/playground/Cargo.toml --check
cargo clippy --locked --manifest-path integrations/playground/Cargo.toml --all-targets -- -D warnings
cargo test --locked --manifest-path integrations/playground/Cargo.toml
```

The Rust and built-WASM checks exercise acceleration, braking, reversing, real
rear slip, invalid inputs and contact recovery, plus slope rollback, cliff
entry from below, independent free-fall equations, absence of air tire forces,
landing and vertical-state reset. Layer tests also drive the bridge and the road
underneath it, enter and exit the tunnel, cross its roof, hit an underside and
land on an upper deck. Actual mesh raycasts check the two levels and open ramp
excavations; body-width sweeps check the structural colliders. The browser test drives the
actual production UI with keyboard and pointer events, checks smoke/skid marks,
all five city starting areas, three cameras, pursuit, pause/focus handling, recovery
and mobile controls. Its fixed browser clock advances actual rendered frames;
acceleration checks use simulation time rather than depending on CI GPU speed. Geometry checks independently sweep the street lanes,
forest corridors and plaza drift circles against the actual scene colliders:

```sh
python -m pip install playwright==1.56.0
python -m playwright install chromium
npm --prefix web run preview -- --host 127.0.0.1 --port 4173
# In another terminal:
python web/tests/browser_smoke.py --output web/test-results
```

The `Browser playground` GitHub Actions workflow repeats these checks, retains
browser screenshots, and deploys `web/dist` from `main`. Repository Settings →
Pages → Source must be **GitHub Actions**. The workflow derives the base path
from the repository name, so it follows repository renames.

## Hero recording

The first gallery cell records the actual upper road, lower crossing and underground
passage ([provenance](../assets/stacked-roads-demo.json)). The **Stacked road evidence**
workflow runs the required core/game checks and deterministic browser drives,
then intentionally refreshes both the ridge and stacked-road recordings. It
publishes only verified formatting and named media to the unchanged feature branch.

```sh
python web/tests/structures_smoke.py --url http://127.0.0.1:4173/rust_driving/ \
  --output web/test-results/structures --record assets/stacked-roads-demo.gif
```

The README hero records the actual urban pursuit, hillside climb, guardrail fracture,
landing and station curve. Its [provenance](../assets/urban-hills-chase-demo.json)
binds source, WASM, trace and output hashes with normal-speed timing and observed
impact, flight and landing. The **Urban city evidence** workflow runs the required
core suite, native/WASM tests and all production browser checks first. It
publishes formatting and the six named urban media files only if every check
passes and the feature branch has not moved. It excludes `main` and `develop`
as publication targets.

After building and starting the preview, install Playwright, Pillow and FFmpeg
and intentionally refresh the urban hero with:

```sh
python web/tests/urban_smoke.py --url http://127.0.0.1:4173/rust_driving/ \
  --output web/test-results/urban --record assets/urban-hills-chase-demo.gif
```

The clip has one visible cut between two public area selections. Both segments
use real keyboard input and the production game; the recorder never writes
vehicle poses or NPC state.

The older [ridge recording](../assets/ridge-dive-demo.gif) and its
[provenance](../assets/ridge-dive-demo.json) remain available. **Ridge course
evidence** refreshes those three ridge assets after core and game checks:

```sh
python web/tests/ridge_smoke.py --url http://127.0.0.1:4173/rust_driving/ \
  --output web/test-results/ridge --record assets/ridge-dive-demo.gif
```

The [city chase recording](../assets/city-chase-demo.gif) shows two pursuing police
cars, signalized intersections, handbrake slides, smoke and skid marks. It uses
keyboard events and the existing chase/camera buttons. A filming fixture reads
the diagnostic snapshot to choose repeatable inputs; it never writes poses or
NPC state. This is arcade gameplay, separate from autonomous-driving validation.
The [native dense-traffic recording](dense-traffic-demo.md) remains available.

The **Record playground evidence** workflow can refresh the older city chase and area
overview on a feature branch. It runs the core checks, geometry checks and real
browser controls first, retains the evidence, and commits only the media paths.
It refuses to publish if the branch moved while it ran. It never writes to
`main` or `develop`; merge the resulting media through the usual PR.

After building and starting the production preview above, install Playwright,
Pillow and FFmpeg, then intentionally refresh the city chase with:

```sh
python scripts/capture-city-chase.py \
  --url http://127.0.0.1:4173/rust_driving/ \
  --evidence artifacts/city-chase-capture
```

Use `PAGES_BASE=/rust_driving/` for that build and preview. The capture advances
the browser clock in 40 ms steps and renders the unchanged game at 25 Hz; its
120 Hz physics accumulator still performs the motion integration. Screenshots
sample every other rendered frame, giving a 960 × 540, 12.5 fps GIF at normal
playback speed. The first six seconds let the police approach before recording.
The camera changes from chase to city view and back during the clip.

[Provenance](../assets/city-chase-demo.json) records source/WASM/output hashes,
frame timing and observed movement/slip. The full input and snapshot timeline
is retained in the ignored evidence directory; browser or renderer versions
may change pixels. The recorder checks moving pursuit, measured rear slip,
smoke, frame count, playback duration and uncaught browser errors before
publishing the asset. The [poster](../assets/city-chase-demo.png) is a captured
frame, with no added vehicles or effects.

