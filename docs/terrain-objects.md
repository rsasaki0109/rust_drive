# Experimental measured XYZ terrain and objects

`PipelineConfig.perception3d` opts into bounded progressive morphological ground classification and XYZ Euclidean components over validated `Lidar3dScan` returns. It requires explicit beam/mount/range/height calibration and excludes the separate near-flat plane-removal configuration. All beam ordinals, finite coordinates, ranges and directions are validated before processing. Unsupported local ground geometry holds sensor-fault braking until a newer valid acquisition; there is no silent planar fallback.

Output preserves acquisition-body XYZ AABBs, point counts and original return-vector indices. Those indices differ from firing ordinals. Bounds describe observed surfaces, including any connected ground residuals; they do not reconstruct hidden dimensions or semantic vehicle classes. The calibrated vertical collision interval selects relevant components. Their measured XY box diagonal supplies a planar circular detection envelope, transformed using the acquisition-time EKF pose. Tracking, prediction, planning and motion remain planar.

```sh
bash scripts/setup-rne.sh
source scripts/env.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
integrations/rne/target/release/rustdrive-rne \
  --scenario scenarios/native-ground-traffic-stop.json \
  --scene scenes/ground-moving-traffic.json --plant dynamic \
  --lidar-3d --terrain-objects --seed 7 --output artifacts/terrain-objects/run
python scripts/check-terrain-objects.py --compact --output artifacts/terrain-objects
```

The native mode uses the existing 720 × 16 inclined ray profile and an explicitly authored research sensing interval **0.15–1.65 m**, taken from the existing body calibration. This selects sensing height; `--vehicle-body` is excluded and physical acceptance retains the ordinary circular vehicle. Ground surfaces enter genuine Rapier queries, while ground/actor roles stay outside pipeline inputs. No classifier settings were changed after real-data evaluation.

The unchanged 22-second moving-lead stop scenario passes in the dynamic plant at seed 7: progress 23.69 m, final speed zero, minimum circular actor clearance 4.91 m, no collisions/road violations and nine emergency ticks. The independent 200 Hz motion bound retains at least 4.72 m clearance. The oracle reconstructs 2,545,920 query rays and 1,216,923 delivered XYZ returns; 41,743 actual actor points enter measured AABBs, including 41,504 in collision-relevant components. Full replay verifies all 441 sensor ticks. There are 148 mixed components containing 10,607 ground samples; a proposed zero-ground-member condition fails and is retained as a counterexample. The actual gate excludes ground-only collision components, without demanding label purity of connected observed surfaces. This is one authored case, without claims of broader traffic coverage or reliable real-data terrain performance. [Complete scope and provenance](../assets/terrain-objects-results.json).

The first 180-column, broad-height acquisition is preserved as a **failed two-second probe**: ground residuals become obstacle tracks, ego makes no progress and sparse actor points produce no object component. Its measured sensor recording is included for current-pipeline replay, rather than claiming that the revised CLI reconstructs the old acquisition profile:

```sh
python -c "import gzip,pathlib; pathlib.Path('artifacts').mkdir(exist_ok=True); pathlib.Path('artifacts/terrain-sparse.jsonl').write_bytes(gzip.decompress(pathlib.Path('assets/native-terrain-sparse-sensors.jsonl.gz').read_bytes()))"
cargo run --release --locked --bin rustdrive -- replay \
  --log artifacts/terrain-sparse.jsonl --output artifacts/terrain-sparse-replay
```

That 41-tick replay is verified; repeatability does not make its perception correct. [Failure metadata](../assets/terrain-objects-sparse-failure.json) and the [complete original simulation evidence](../assets/native-terrain-sparse-evidence.tar.gz) retain hashes, physical telemetry and all measured inputs. Increasing acquisition density and declaring the research height interval repairs this specific native stop episode while preserving the original failed capture. The PMF still has six zero-F1 real-data sites. Mixed actor/ground components can inflate envelopes; broad roofs can resemble terrain, and the default within-cell residual allowance of approximately 0.574 m can remove small low objects. Local geometric confidence is not road semantics or a calibrated probability. [Frozen measured-data failures](datasets.md).
