# City Playground

[Play RustDriving](https://rsasaki0109.github.io/rust_driving/) in a desktop
WebGL 2 / WebAssembly browser, or use the on-screen pointer controls on a phone.
There is no login, backend or save requirement. The static build bundles its
renderer and physics; it makes no runtime CDN or map-service requests.

The authored city has 102 buildings, 49 intersections, changing signals, parks,
shop signs and 20 independently moving traffic cars. Chase mode adds two police
cars; leave them more than 80 m behind for 12 seconds to escape. Drift chains
add score after the slide ends. All artwork is original procedural geometry.

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

This is a flat, planar arcade playground with bounded speeds and approximate
contact impulses. Traffic follows independent waypoint/signal rules; police
use the street grid to approach the player. It is separate from the sensor-driven
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
rear slip, invalid inputs and contact recovery. The browser test drives the
actual production UI with keyboard and pointer events, checks smoke/skid marks,
three cameras, pursuit, pause/focus handling, recovery and mobile controls:

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
