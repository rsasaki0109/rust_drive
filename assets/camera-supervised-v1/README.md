# Supervised camera evidence v1

This packet freezes 100 actual Nano forwards (68 timed + 32 current serial
references) and eight complete standalone guard replays. The unchanged original
Nano inference, four views, postprocessing and 200 ms/two-sample guard remain
reproducible. A new optional scoped worker permits the caller to deliver failures
and detect expiry during inference. Original acquisition times are never renewed.

Injected failure delivery falls from 102–105 ms to 2.8–3.1 ms in the two paired
local observations. Both supervised ordinary trials still supply 0/8 available
results, including one late inference in the repeat. Maximum periodic poll gaps
exceed the 10 ms target in every supervised trial. These failures are retained;
there is no hard real-time, live camera, new holdout, quality or driving claim.
See [the contract and results](../../docs/camera-supervised.md).

`files.json` indexes every file except itself. `raw-files.json` pins the exact
uncompressed observations; gzip uses deterministic mtime 0. The before-inference
protocol and complete initial checker are archived before any valid inference.
The 137-file source closure reuses 130 earlier frozen sources from
`camera-nano-v1`, `camera-latest-v1` and `camera-nano-repeat2x-v1`; only seven new
source files are copied. All 75 earlier indexes / 5,917 indexed entries, and all
245 core outputs, remain unchanged. No model weight, executable or original
camera image is included. Two 8×6 authored PNG controls are retained; the third
`.png` is intentionally invalid text. A nine-byte invalid-model control is
omitted and identified by hash.

The unchanged Nano model/code uses Apache-2.0 (see `LICENSE-code.txt` and the
parent model provenance). TUM-derived measurements retain the original TUM
notice. The BDD license is preserved alongside the inherited source/evaluation
references; this packet adds no BDD annotations or accuracy scoring. Every
injected sensor failure is an authored test event.

```sh
python -B assets/camera-supervised-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p test_camera_supervised.py -v
```

Both commands execute no model and need no original images. The parent Nano PR
has all ten CI jobs successful; the new supervised variant is locally checked
and has no remote CI observation. Maturity remains about 20% (subjective), below
the 30% waypoint.
