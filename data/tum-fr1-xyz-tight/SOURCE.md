# TUM RGB-D later temporal comparison

This subset uses the same recorded `rgbd_dataset_freiburg1_xyz` physical room,
camera, original depth index and motion-capture trajectory as
[the original subset](../tum-fr1-xyz/SOURCE.md). Its source provenance, registered
depth calibration uncertainty and unverified original redistribution-license
limits remain unchanged. Raw files stay ignored and are not redistributed.

The original ~0.333-second pair benchmark and the later fast subset at original
indices 120–131 both failed their frozen matcher stages. Their manifests,
reports and source archives are retained. After those failures, and before any
new fits or optimizer inspection of these frames, **original depth-index entries
140–151 inclusive** were declared as twelve consecutive frames. The first three
are labelled calibration and the remaining nine temporal held-out. All eleven
consecutive pairs must remain in the denominator. Selection uses fixed original
index positions, not reference motion, point density or registration scores.

The new source/configuration/protocol freeze must be recorded after optimizer
changes and before evaluating this interval. The previously viewed fast subset
is regression data for an optimized matcher; its historic frozen-stage split
labels do not regain unseen status. **This later interval is temporal withheld
data from the same physical sequence, not a new independent environment.**
Neither its eventual outcome nor a faster matcher can erase the earlier failures.

`manifest.json` pins the same public teaching mirror revision, original paths,
exact byte counts and SHA-256. Depth PNGs are original measured frames; no
synthetic transform, resampling or modified depth value is introduced. The full
original depth index and mocap file remain unaltered byte-for-byte copies of the
original subset's inputs. Ground truth remains evaluation-only.

```sh
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz-tight
python3 scripts/fetch-additional-datasets.py --dataset tum-fr1-xyz-tight --verify-only
```
