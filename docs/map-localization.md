# Bounded scan-to-map localization

`PipelineConfig.localization_map` opts into local SE(2) registration against a supplied fixed world-XY point map. It is absent by default. The driver receives measured body-XY LiDAR, odometry and genuine GNSS observations; simulator actor identities, live poses and fault schedules remain outside operational input. The demonstrated prior is an **authored offline surface map**, not a real survey or a map generated from current truth.

Registration uses a deterministic spatial grid, unique map correspondences, trimming, centered closed-form SE(2) fits and one main plus six bounded local seeds. It chooses the best accepted fit and rejects distinct comparable fits. Convergence, overlap, residual, geometry, conditioning, covariance, pose jumps and a global point-comparison budget bound acceptance. This is local matching, without global localization, SLAM, 6DOF, deskew or a proof of unique association. Dynamic surfaces can produce misleading correspondences; local ambiguity probes do not resolve every repeating scene.

Accepted matches pass a joint EKF XY/body-yaw innovation gate and Joseph covariance correction. The pipeline scales measurement covariance to at least 0.01 m² for X/Y and 1e-4 rad² for yaw. Those conservative floors are assumptions, not measured statistical calibration. Full raw and fused covariance, conditioning, work counts and observed/accepted timestamps remain in diagnostics and exact sensor replay.

A match cannot invent an accepted GNSS timestamp. Only after an actual initial GNSS fix, a fresh accepted synchronous map scan may bridge GNSS age over 0.75 s for a configured maximum of **10 seconds since that actual fix**. Map freshness expires after 0.2 s. Rejected, malformed, repeated, delayed or future scans cannot extend the bridge; prolonged outage and LiDAR failure brake. Existing GNSS innovation holds, odometry freshness, covariance limits and numeric guards still apply. Fresh healthy GNSS can continue operation when a configured map match is rejected.

```sh
source scripts/env.sh
cargo build --workspace --release --locked
python scripts/check-map-localization.py --backend reference --output artifacts/map-localization
# Optional CPU RNE adapter, using its separate pinned toolchain/lockfile:
bash scripts/setup-rne.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python scripts/check-map-localization.py --backend all --output artifacts/map-localization
```

Six cases across three seeds and two plants exercise a five-second moving outage and real-fix recovery, a prolonged outage, an unrelated map, collinear map geometry, LiDAR loss after accepted matches, and the same outage with matching disabled. The independent checker reads full 20 Hz physical telemetry only to score motion, error, road containment and clearance. It verifies actual GNSS acquisitions and unchanged accepted timestamps during loss. Fixed healthy accuracy gates are 0.5 m position and 0.1 rad yaw, plus the existing 1 m physical-clearance floor. Fault negatives require finite timely braking and unchanged physical gates; localization accuracy is gated only while accepted map matches are fresh. Untrusted-map errors remain reported. A historical negative reached 0.103962 rad yaw error after loss, disproving a whole-episode 0.1 rad guarantee.

Four altered logs—map shift, acceptance timestamp, covariance and body-scan translation—must fail full recomputation. Verified raw telemetry is compressed losslessly with uncompressed and compressed hashes. [Measurements and scope](../assets/map-localization-results.json). Running these cases does not establish general GNSS-denied autonomy, map correctness or automotive LiDAR accuracy.
