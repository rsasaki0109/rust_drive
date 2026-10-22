# Complete small-object grid evidence v1

The new native twelve-view grid improves fresh-sample recall from 33.53% to
43.71% but reduces precision from 57.44% to 50.52%. The sixteen new images score
112 TP / 83 FP / 222 FN for the four-view baseline and 146 / 143 / 188 for the
grid. It gains 37 reference identities and loses three. All original absolute
quality gates fail; precision regresses in both fresh and viewed groups. The
candidate is **not adopted**. [Full method and results](../../docs/camera-grid.md).

The complete grid preserves the original four crop measurements, requires all
new views and uses at most four scoped workers at a time. Two reversed-order
passes plus current serial original-graph references run 1,760 unique forwards
and save 200 image reports. Every decoded measurement pair and complete
postprocessing comparison passes; this increment makes no raw tensor claim.
Model-free pixel exports, forty inventory inspections, eight valid fixtures,
nine rejected fixtures and one no-clobber check contribute no model forwards.

`verify.py` checks the first-sixteen selection against the previously frozen
complete Git tree, rebuilds all postprocessing and recomputes both metric groups
from original pinned annotations. The original 24 annotations and seven parent
source archives are reused unchanged. Only sixteen new annotations and ten new
source files are copied. The 177-file source closure preserves 167 previous
sources, 79 indexes / 6,795 entries and 245 core output pins.

`files.json` indexes every file except itself; `raw-files.json` pins original
uncompressed bytes and gzip uses mtime zero. Original images, generated PNGs,
lossless pixel exports, weights and executables are excluded. The included BDD
notice applies to original annotations and derived evidence separately from
Apache-2.0 implementation code. The first portrait-layout assertion failure is
retained as a pre-inference development check; its expected order was corrected
before the native source and model comparison were frozen.

The workflow is unobserved remotely. A green integrity check does not establish
passed quality gates. Additional compute, live transport, driving integration,
identity accuracy and measured availability remain unresolved. Subjective
maturity remains about 20%, with 30% unmet.

```sh
python -B assets/camera-grid-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p test_camera_grid.py -v
```

Both commands execute no model and need no original images, weights or OpenCV.
