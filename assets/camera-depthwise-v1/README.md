# Safe Nano depthwise evidence v1

This packet retains the first 16-forward pilot and every observation from the
392-forward main comparison. The opt-in path replaces thirty validated Nano
3×3 depthwise evaluations with safe row-wise Rust arithmetic, using the existing
tract executor and original bias/tap order. It also retains the earlier guarded
Resize2x lowering. Models, images, four views, thresholds, postprocessing and the
200 ms/two-sample guard are unchanged.

FIFO availability improves from 1/8 to 2/8 in both paired observations. The
supervised comparison improves from 3/8 and 1/8 to 4/8 in both observations; its
baseline lacks both optimizations. Neither result meets full availability, and
both optimized supervisors exceed the 10 ms periodic target. The main serial
comparison isolates depthwise work against Resize-only execution: aggregate
time falls 9.79%, with one 4.29% regression among sixteen retained pairs.

All 19,306,560 main raw f32 values match bitwise. Current original serial and
parallel detections, merges and decisions also match; independent postprocessing
and eight complete standalone guard replays check the results. There is no new
holdout, quality score, identity evaluation, live camera or driving integration.
The inherited Nano BDD quality gates still fail. [Contract and results](../../docs/camera-depthwise.md).

`files.json` indexes every packet file except itself. `raw-files.json` pins exact
uncompressed observations; gzip uses deterministic mtime 0. The initial pilot
checker is preserved byte for byte; its body is unchanged in the main checker,
which appends the experiment driver. The complete main checker is archived before
its first timed inference. The later reproduction wrapper explicitly executes
its own pilot and main phases; its auditor is checked on the saved pilot without
re-running inference. A second full orchestrated execution is unobserved locally.

The 149-file source closure reuses all 137 earlier sources from the Nano,
latest-ready, Resize and supervised packets; only twelve new source files are
copied. All 76 earlier indexes / 6,033 indexed entries and all 245 core outputs
are unchanged. Neither weights, executables nor original camera images are
included. Two authored 8×6 PNG controls and intentionally invalid image text
remain; the nine-byte invalid model is omitted with a hash record.

The model/code is Apache-2.0 with original provenance in the parent Nano packet.
TUM/BDD-derived measurement reports retain their complete original dataset
notices. This packet adds no original image or BDD annotation. Maturity stays
about 20% (subjective), below the 30% waypoint. Remote CI for this new variant
has not been observed.

```sh
python -B assets/camera-depthwise-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p test_camera_depthwise.py -v
```

Both commands execute no model and need no original camera images.
