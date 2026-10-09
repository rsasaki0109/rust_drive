# Bounded OpenStreetMap import

RustDrive imports a local OpenStreetMap Overpass JSON document into its existing directed road network. The Rust implementation does not download maps, require Python/ROS, use OSMnx at runtime, or substitute map coordinates for sensing/localization. The checked-in [historical external extract and ODbL provenance](../maps/osm/SOURCE.md) provide real geographic data; the separate authored junction fixture tests topology without claiming external data.

## Import, route and drive

```sh
cargo run --release --locked --bin rustdrive -- import-osm \
  --input maps/osm/german-road-extract.json \
  --output artifacts/osm/map.json \
  --origin-lat 48.136 --origin-lon 10.0695 --default-half-width 3.0 \
  --scenario-output artifacts/osm/scenario.json \
  --start osm-node-7119017425 --goal osm-node-274969423 \
  --cruise-speed 2 --duration 180
cargo run --release --locked --bin rustdrive -- run \
  --scenario artifacts/osm/scenario.json --seed 7 --output artifacts/osm/reference
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/osm/reference/sensors.jsonl --output artifacts/osm/reference/replay

# Pinned native RNE dynamics and sensor acquisition; setup-rne.sh first.
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --plant dynamic --scenario artifacts/osm/scenario.json --seed 7 \
  --output artifacts/osm/rne-dynamic
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/osm/rne-dynamic/sensors.jsonl --output artifacts/osm/rne-dynamic/replay
```

The importer writes a plain `RoadNetworkSpec` to `map.json`, counts/projection calibration to `map.import.json`, and optionally a runnable scenario with the existing `navigation` network/start/goal/closure fields. Graph node IDs are `osm-node-ID`; directed edges are `osm-way-ID-PART-forward` or `-reverse`. The scenario uses deterministic Dijkstra, calibrated initial pose/heading, noisy GNSS/odometry and synthetic LiDAR in the same operational pipeline as authored maps. OSM coordinates are supplied map geometry, not ground-truth localization observations.

`--closed-edge ID` may be repeated when generating a scenario. Unknown closures or an unreachable goal fail with exit 2 before generation. Start/goal must be graph nodes, including retained way endpoints or shared junctions; arbitrary mid-way spawning is absent. `--cruise-speed` defaults to 2 m/s and `--duration` to 180 s. Scenario generation validates the existing 20–1000 m route and calibration domain; importing a valid larger graph does not guarantee every route can become a simulation scenario.

`--local-route-geometry` explicitly enables the optional corner-fitting planner mode when generating a scenario. It defaults to false and does not alter imported coordinates or the physical corridor. Its capability and measured turn regressions are separate from the default Goethestraße road reproduction above.

## Geometry and access contract

WGS84 latitude/longitude at zero height converts to Earth-centered Cartesian coordinates and then the origin's east/north tangent plane. Every supplied node must be within a **2 km chord radius** of the configured origin. Latitude is bounded to ±85° and longitude to ±180°. Elevation, banking, geoid corrections and 3D road contact are absent. Independent known equatorial distances and a dateline case test the projection.

Supported motor-road classes include residential, service, living street, unclassified and motorway/trunk/primary/secondary/tertiary roads and links. Pedestrian/footway/track, unknown road classes and conditional tags are conservatively filtered. Motorcar access overrides motor-vehicle, vehicle and generic access tags; only absent access or `yes`, `permissive`, `designated` is accepted. Private/no/destination and unknown restricted access are filtered. Node-level access restrictions and barrier tags also filter affected ways; operating gates or deciding destination access are not implemented.

`oneway=yes/1/true` keeps forward order; `-1/reverse` reverses it; `no/0/false` creates both directions. Roundabouts and motorway/motorway-link ways default to forward one-way unless explicitly overridden. Unknown/reversible one-way values are errors. Shared OSM node identities split intersections; geometric crossings without shared IDs are not connected. Every intermediate way coordinate remains in edge geometry. Closed rings are split into non-self-loop directed edges. Nodes, ways and part IDs are processed deterministically, so input ordering does not change the graph or Dijkstra tie decisions.

Metric scalar `width` tags supply half the tagged total width. Otherwise **`--default-half-width` is explicit simulation corridor calibration**, not a measured or inferred OSM lane width. It defaults to 3 m and must be 1.2–12 m; scenario generation retains its narrower existing geometry limits. No lane counts, vehicle clearance or legal speed are inferred. OSM `maxspeed` tags, lane boundaries, turn-restriction relations, signals/signs and access schedules are not operationally imported.

The JSON CLI accepts at most 16 MiB; the typed importer permits at most 50,000 elements, 4,096 references per retained way, 200,000 retained references and 10,000 directed edges. Missing nodes, duplicate/zero IDs, invalid coordinates, repeated interior nodes, coincident consecutive positions, unsupported widths and malformed direction values fail explicitly. Cropped single-node road ways are filtered and counted separately; empty ways are errors. Unsupported relation/element counts and filtered/default-width counts are visible in the import report. Relations are counted but not resolved into driving rules.

## Actual evidence and limits

The real extract contains **281 nodes, 56 ways and three relations**. Four supported motor-road ways produce **six graph nodes and ten directed edges**. Fifty-two ways are filtered, including two clipped single-node residential ways; all four imported widths use the declared 3 m corridor calibration. Source hashes and the original XML allow independent verification of this derived database.

The supplied Goethestraße route is about **113.06 m**. It reaches the goal in both reference and native dynamic plants across seeds 1, 7 and 42, with all sensor logs fully recomputed. Native seed 7 completes at **57.55 s / 1152 ticks**, with zero collisions/road violations, zero emergency ticks and a maximum position error of **0.149481 m**. [Initial local probe summaries and map/scenario hashes](../maps/osm/initial-validation.json) preserve these measurements separately from final frozen-source verification. These initial external-map runs contain no obstacle actors and do not establish general external-map driving, physical map accuracy or road legality.

Two additional exploratory routes exposed the default planner's geometry limits. A sparse authored closure detour with a sharp bend failed its 250 s goal deadline, stopping at 28.30 m without collisions/road violations. A real Goethestraße-to-Haydnstraße branch route using the same 3 m calibration also failed its 180 s goal deadline at 10.325 m. The directed routes were valid, but default candidate smoothing/containment could not execute these sharper sparse geometries. Their source coordinates, widths, speeds and deadlines remain unchanged in the opt-in corner-fitting fixtures.

The optional `local_route_geometry` mode fits local circular fillets with radius at least `1.005 * wheelbase / tan(max_steer)`, sampled at 0.25 m and checked against the original physical corridor. Overlapping/infeasible fits fail rather than widening roads or dropping the bend. The local preview is 0.8–1.5 m, with an explicit lateral-acceleration calibration. This mode is incompatible with mapped signal/sign/priority arc lengths and is supported only for the tested reference circle-vehicle fixtures. The final checker passes six reference runs and all complete sensor replays across both fixtures and seeds 1, 7 and 42. The real branch completes at **78.45 / 78.60 / 78.55 s**, with sampled original-circle corridor margins **0.103353 / 0.085487 / 0.091996 m**. The authored detour completes at **156.70 / 156.85 / 156.85 s**. No original coordinates, widths, speeds or deadlines are changed.

Native dynamics remain a known failure for the approximately 101.7° narrow branch: native seed 1 fails the unchanged **180 s** goal deadline, stopping at **62.632034 m** with **2,953 emergency ticks**, no collisions/road violations and final speed zero. Its **3,601 sensor ticks** fully replay, and its sampled original-circle corridor margin is **0.342293 m**; deterministic safe stopping does not establish goal completion. The larger cuboid-body envelope cannot fit this branch's calibrated geometry. Neither is counted as supported sharp-turn driving. `python3 scripts/check-local-corners.py --check-known-native-limit` accepts the reference fixtures and independently verifies the retained native failure; explicitly requesting native/all positive backends keeps failures strict. The six positive native/reference mild-road runs and the separate ground/body mild-road matrix do not repair this sharp native branch. General intersections, lane topology, native sharp-route feasibility and imported traffic rules remain future work.

Meaningful routing tests cover independent projection distances, one-way/reverse/roundabout reachability, shared junction splits with intermediate geometry, deterministic permutations, closure detours, access/barrier filtering, closed loops and malformed/bounded inputs. Replay verifies the computation separately from physical goal/corridor acceptance. The import is a working external-data adapter within this bounded prototype, not a replacement for an HD map importer.

## Real map, measured ground and a native body

The separate integration cases combine the same **113.060503 m** imported road with an actual flat Rapier ground platform, unlabeled inclined XYZ LiDAR, measured plane fitting/removal, and the authored **4.2 × 1.8 × 1.5 m** cuboid body. The platform is simulator calibration, not terrain imported from OSM or a measured geographic road surface. Its wide physical support ensures all ground-fit sectors are observed. The original map coordinates, 3 m half-width and 2 m/s cruise are retained. Integration scenarios use a stricter 100 s deadline because native scene evidence is bounded to 120 s; the original 180 s scenario remains separate.

```sh
cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- \
  --plant dynamic --scene scenes/osm-ground-clear.json \
  --scenario maps/osm/german-road-ground-scenario.json --seed 7 \
  --lidar-3d --ground-segmentation --vehicle-body --output artifacts/osm-ground
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/osm-ground/sensors.jsonl --output artifacts/osm-ground/replay

# Six clear goals (two native plants × three seeds) and three dynamic barrier stops.
python3 scripts/check-osm-scenes.py --output artifacts/osm-scenes --compact
```

The matrix retains source-map, scenario, ground/obstacle and checker hashes. Independent Python checks original XML-to-JSON geographic identity, ENU projection, preserved way points and the actual selected route. It reconstructs native rays, ground confidence/removal, body sweeps and Rapier sensor witnesses using the shared scene oracle. A further 200 Hz circumscribed-body corridor check retains the supplied 3 m width and 0.03 m bounded-motion reserve. Static map replay uses the resolved route; full Dijkstra recomputation is not claimed for these logs. Ground/body labels, obstacle identities and physical overlap results stay outside operational sensor inputs.

The [final frozen-source matrix](../assets/osm-ground-results.json) passes all **nine cases** and **12,914 complete sensor-replay ticks**. Six clear goals complete in **57.45–57.60 s** across both native plants and seeds 1, 7 and 42. Three dynamic barrier cases stop without overlap, with minimum independently reconstructed body clearance **7.516491 m**. Across the matrix, **18,604,800 beam entries** are audited, all **9,302,400 actual ground returns** are removed, and all **59,510 actual obstacle returns** are preserved. The minimum circumscribed-body corridor reserve is **0.065024 m after the additional 0.03 m motion reserve**, so this narrow calibrated success is not evidence of a general tracking-error margin.

The generated `artifacts/osm-scenes/report.json` records `passed=true`, source/checker fingerprints, fixture and raw-log hashes, and per-case archive manifests. `--compact` verifies full archive hashes and gzip CRC before removing generated raw cases, retaining dynamic seed-7 clear and barrier-stop raw cases for inspection/rendering. The published summary preserves these results and hashes; rerun the command above to produce full logs locally.

Native motion remains planar, with no tire/ground contact response, suspension, banking, terrain reconstruction or six-degree-of-freedom motion. A ground platform and authored cuboid do not establish a physical external-world simulator or real-road safety. OSM attribution and ODbL obligations continue to apply to the geographic data and derived map databases.

The published [corner proof](../assets/local-corner-results.json) retains its pre-ground-refinement fingerprint and the native known failure. [Corridor mutations](../assets/local-corner-mutations.json) reject width changes, omitted bends and shifted truth poses. The final ground fix does not enable ground processing in those reference corner fixtures. The [integrated map proof](../assets/osm-ground-results.json) additionally records all nine fresh final-source native episodes and full replays, with byte-identical raw and recomputed files allowing reuse of the explicitly preserved prior physical-oracle evidence.
