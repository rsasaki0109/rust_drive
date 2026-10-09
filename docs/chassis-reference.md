# Chassis reference and low-speed course

RNE dynamic vehicle poses are at the chassis/center of mass. The existing reference bicycle moves along the rear-axle/body heading. Those references differ during a turn: for a no-slip chassis, lateral velocity is `rear_axle_offset_m * measured_yaw_rate`. Reusing a rear-axle estimator and controller at the chassis caused the real imported sharp-branch failure.

## Operational contract

`PipelineConfig.rear_axle_offset_m` is optional and omitted from older/default sensor-log headers. A supplied offset must be finite, positive, less than wheelbase, and used with `local_route_geometry`. The RNE adapter selects the declared 1.5 m rear distance from its pinned native dynamics calibration only for dynamic local-route mode. Reference and default native modes retain their previous computation and serialization.

The opt-in EKF predicts chassis displacement from noisy measured longitudinal wheel speed `v` and gyro `r`: body velocity `(v, offset*r)` rotates through the midpoint body heading. Its yaw Jacobian uses the integrated displacement; the existing covariance noise and GNSS innovation policy remain. Estimated speed is the chassis/course magnitude `hypot(v, offset*r)`. Recorded pose yaw remains **body heading**, preserving LiDAR frame conversion and acquisition-pose history. No native lateral-velocity state, future pose, actor identity or simulator geometry crosses the sensing boundary.

Above 0.1 m/s measured longitudinal speed, local planning and pursuit use course `body_yaw + atan2(offset*r, v)`; below that threshold they retain body yaw to avoid a gyro-noise direction at standstill. This course is a temporary planning/control view, not a replacement sensor pose. Reachable trajectory profiles and speed feedback use the estimated chassis speed.

For course curvature `k`, no-slip chassis pursuit requests `atan(wheelbase*k / sqrt(1-(offset*k)^2))`, then retains the existing 0.55 rad steering limit and 0.7 rad/s command rate. Unreachable curvature saturates finitely. Offset zero delegates the preceding estimator and steering arithmetic exactly. The local fillet remains a guidance curve; its rear-axle radius validation is not a chassis tracking guarantee. Every candidate still passes original-corridor containment and speed/collision checks.

## Measured repair

The preceding seed-1 native branch run stalled at 62.632034 m after 180 s. Near the turn its actual displacement course differed from body yaw by about 0.30 rad. At 32 s, true inward offset was about 0.478 m while the estimate was about 0.253 m. The recovery candidate at 32.1 s exceeded the unchanged 1.75 m center-clearance budget, while longitudinal speed feasibility still passed. Shortening only the heading join did not solve this reference mismatch.

The repaired matrix reaches both original goals in all six reference and six native dynamic cases, with zero recorded collisions or road violations and full replay. Native branch times are 78.65 / 79.70 / 78.50 s, under the original 180 s deadline; minimum sampled native branch corridor margin is 0.099672 m. There are still 9 / 19 / 9 emergency ticks. These margins are sampled circular-footprint measurements, not certified continuous margins or calibrated rectangular-body corner acceptance. [Full results](../assets/local-corner-chassis-results.json); [original-map oracle](osm-import.md).

Three edited logs change the chassis distance, remove it, or alter a measured turn gyro. Each is rejected by exact Rust replay. Analytic unit tests verify left/right displacement, speed, covariance, circle-to-steering inversion, saturation and unchanged zero-offset behavior. Default reference/native hazard outputs and the accepted opening-GIF ground/body episode retain their bytes. [Default regression and validation proof](../assets/chassis-validation.json).

## Limits and reproduction

The established domain is these 2 m/s clear-road fixtures and a low-speed no-slip model. This does not estimate tire slip, support GNSS-denied motion, establish high-speed course accuracy, remove discrete perception blind zones or guarantee arbitrary bends. Mapped signals/signs/priority arc lengths remain excluded from local-route geometry. Vehicle dimensions, source road coordinates, widths, cruise settings, deadlines and existing acceptance floors are unchanged.

```sh
bash scripts/setup-rne.sh
source scripts/env.sh
cargo build --release --workspace --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/check-local-corners.py --backend all --compact --output artifacts/local-corners
```

[Historical failure](../assets/local-corner-results.json) remains associated with its original source fingerprint. The preceding Windows CI failure occurred before driving: checkout converted pinned OSM files to CRLF. Narrow `.gitattributes` rules preserve their exact source bytes; a real autocrlf checkout and tamper test leaves the checksum gates intact. CI executes that check on all three Rust platforms and the complete native turn matrix on Linux. Remote results are reported separately from local validation.
