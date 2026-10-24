# Storm taxi lab

[Open Storm Taxi Lab](https://rsasaki0109.github.io/rust_driving/?mode=fleet&lab=1).

The optional lab connects a horizontal LiDAR instrument, the existing Rust
weather model and **LidarClusters** to an arcade taxi brake governor. Rain
and snow also generate building-origin taxi demand. A reserve policy adds
drivers from the physical depot rather than teleporting them to customers.

Choose **CLEAR / RAIN / SNOW**, adjust **STRENGTH**, and switch
**DEPLOY RESERVE CABS**. Compare waiting riders, completed trips, pickup wait,
empty travel, cumulative sensor-governed time and body-proxy contacts.
**INSPECT TAXI** selects a cab's acquired cloud; **FOLLOW / ROAD / CITY**
change the camera. **HIDE POINTS** changes the overlay while sensing continues.
Start/pause, 1x/4x speed, train transfers and morning/dusk/night remain available.

## Sensor and driving boundary

Each cab acquires 360 first-return rays over 360 degrees at 5 Hz, with a 45 m
maximum range and a horizontal plane 1.1 m above its road support. The source
world contains the game's collision prisms, oriented vehicle-body boxes and
active pedestrian discs, with vertical separation and nearer occlusion.
Office/club bodies use declared footprint proxies. This is an analytic proxy
instrument, not a full triangle/optical LiDAR renderer.

Body-frame **x forward / y left** returns enter WebAssembly. Rust applies the
same synthetic extinction/foreground-particle/range-jitter model documented
in [the offline study](lidar-weather.md), then runs the unchanged detector.
The brake governor receives only returned points, detection geometry, speed
and a supplied-map turn intent. Clean returns, actor identities, weather
coefficients and weather source labels stay outside the governor.

The mapped steering, service stops and simulated signal/reservation service
remain arcade systems. The complete research DrivingPipeline and its
localization are not installed in this browser mode. Sensor-guided taxis skip
the legacy actor-position following shortcut. A missing, invalid or >0.6 s
stale packet holds the cab; a fresh measured cloud supplies the speed limit.

Supported returned points constrain a 1.15 m corridor along the map's current
turn and outgoing tangent. A 5 m/s² brake calibration, 0.3 s reaction allowance
and 3.5 m sensor-origin reserve set the speed limit. Near returns also have a
two-neighbor emergency support rule: returns must be within 0.25 m and
0.03 rad of one another. Separated precipitation speckles do not qualify. Weather can create false braking or hide an
obstacle: an empty degraded cloud is not a clear-road certificate. Stationary boarding/alighting transitions are exempt only for their own carrier.
Proxy overlap episodes are independently evaluated and retained, without imposing
physical contact forces or declaring real-world safety.

## Fleet operations

The original eight/day, ten/evening and twelve/night targets remain the base.
Reserve staffing adds **ceil(2 x strength)** for rain or
**ceil(4 x strength)** for snow, capped at 16 cabs. Disabling reserve preserves
the base target. Retiring drivers complete passenger trips before returning
to one of eight distinct depot bays. Passenger assignment/boarding/dropoff
still require actual travel, stopped arrival and dwell time.

A new storm releases its first additional travel plan after two physical seconds,
then uses a strength-dependent interval at the office or nightclub.
People walk through the actual building entrance before requesting a taxi.
The model never creates served trips by advancing counters. Existing
passengers and assignments survive weather and staffing changes.

This is authored demand and an experimental controller, without a calibrated
weather/population model. Reserve/base comparisons use the same initial seed,
weather transitions and passenger generator; post-treatment traffic and
individual journeys can diverge. One paired run is not a fleet-wide
efficiency claim, and poor operation/contact outcomes remain reported.

## Reproduce

~~~sh
npm ci --prefix web --ignore-scripts
bash scripts/build-playground.sh
npm --prefix web test
npm --prefix web run preview -- --host 127.0.0.1 --port 4173
~~~

With Python 3.12, Playwright 1.56.0, Pillow 12.0.0 and Chromium:

~~~sh
python web/tests/storm_smoke.py --url http://127.0.0.1:4173/ --output web/test-results/storm
python web/tests/storm_smoke.py --url http://127.0.0.1:4173/ --output web/test-results/storm --record assets/storm-taxi-demo.gif
~~~

The recorded clear/rain/snow intervals run at 1x after an unrecorded 20 s
pre-roll. Both staffing policies use identical physical weather-switch times
and run to 90 physical seconds. The receipt contains source/frame/media hashes,
actual metrics, contact episodes and control timelines. Tests operate UI
events and a bounded frame clock; they never assign agent poses or counters.

The **Storm taxi lab** workflow checks the required core suite, standalone
Rust ABI, production Wasm and JavaScript tests, actual browser proof images,
and paired longer operation. Verified media are published only after these
gates pass and the feature head still matches the tested source.
