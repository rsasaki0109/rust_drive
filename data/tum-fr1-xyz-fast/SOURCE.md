# TUM RGB-D consecutive-frame temporal comparison

This subset uses the same measured `rgbd_dataset_freiburg1_xyz` physical sequence,
camera model, original depth index and motion-capture trajectory as
[the original subset](../tum-fr1-xyz/SOURCE.md). It inherits that source's explicit
calibration uncertainty and unverified original redistribution-license limits.
Raw files remain ignored; this manifest does not supply a redistribution grant.

The original twelve frames, spaced about 0.333 seconds apart, produced eleven
rejections with the frozen matcher. Their manifest and failed report are retained.
After that failure, a **different temporal interval** was preregistered before
running any new fits: original `depth.txt` entries **120–131 inclusive**, twelve
consecutive frames from 1305031106.166330 to 1305031106.528267, about 0.362 seconds.
Frame intervals are about 0.029–0.036 seconds. Selection used fixed source-index
positions, not point density, reference motion or fit scores. All eleven
consecutive pairs must remain in the denominator.

The first three frames form a temporal calibration partition and the remaining
nine a temporal held-out partition. These frames were not used for algorithm
tuning, but they share the original room and acquisition. **This is not a new
independent environment or a repair of the original longer-interval failure.**
A source/configuration/protocol freeze must be recorded before scoring this subset;
matcher defaults, identity initialization and preprocessing stay unchanged.

`manifest.json` pins the same teaching repository revision and exact original
paths, plus per-file size and SHA-256. The depth index and mocap trajectory are
unaltered byte-for-byte copies of the original subset's inputs. New PNGs are
original measured frames with no resampling or synthetic transformation.

```sh
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz-fast
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz-fast --verify-only
```
