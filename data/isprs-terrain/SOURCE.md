# ISPRS airborne terrain reference data

- Repository: [PointCloudLibrary/data](https://github.com/PointCloudLibrary/data).
- Pinned revision: `5c26bdd0591ba150b91858b5c9fe5e91cb39ae86`.
- Source directory: [`terrain/`](https://github.com/PointCloudLibrary/data/tree/5c26bdd0591ba150b91858b5c9fe5e91cb39ae86/terrain).
- Per-file SHA-256, byte sizes, point counts and splits: [manifest.json](manifest.json).
- Thirty unchanged files: **4,985,216 bytes**, **637,042 points** across input clouds and ground references combined. These counts are acquisition metadata, not evaluation scores.

The pinned [terrain README](https://github.com/PointCloudLibrary/data/blob/5c26bdd0591ba150b91858b5c9fe5e91cb39ae86/terrain/README.md) identifies the data as the 2003 ISPRS filter comparison, described by Sithole and Vosselman (2004), *Experimental comparison of filter algorithms for bare-earth extraction from airborne laser scanning point clouds*, ISPRS Journal of Photogrammetry and Remote Sensing 59(1–2), 85–101, DOI [10.1016/j.isprsjprs.2004.05.004](https://doi.org/10.1016/j.isprsjprs.2004.05.004).

PCL projected the original measurements to UTM zone 32U and converted ASCII → LAS → PCD. First and last returns are no longer distinguished. Each sample provides an all-points cloud and a ground-only reference cloud. The files are `PCD binary_compressed`, field-major XYZ float32 with LZF compression; the coordinate quantization is already present in the upstream float32 files. They do not provide sensor timestamps, a vehicle trajectory, raw beam angles or semantic classes beyond the ground reference.

Freeze the following split before algorithm calibration:

| Use | Samples | Sites |
| --- | --- | --- |
| Calibration | samp11, samp12, samp21, samp22, samp23, samp24 | 1–2 |
| Held-out evaluation | samp31, samp41, samp42, samp51, samp52, samp53, samp54, samp61, samp71 | 3–7 |

Keep ground-reference points in the evaluator only. A common, input-derived local origin may translate both clouds consistently; independently centering the input and reference would corrupt labels. Missing or ambiguous coordinate associations must be reported, not silently assigned. Ground labels do not supply a plane to the algorithm. This airborne benchmark is different from near-ground automotive LiDAR and cannot establish automotive perception safety or operational compatibility.

## Acquisition and license scope

```sh
python scripts/fetch-datasets.py --dataset isprs-terrain
python scripts/fetch-datasets.py --dataset isprs-terrain --verify-only
```

The repository declares BSD-3-Clause, copyright © 2013 Point Cloud Library; its exact notice is preserved in [UPSTREAM-LICENSE.txt](UPSTREAM-LICENSE.txt). The terrain README identifies a third-party original dataset, but no separate original ISPRS redistribution grant is documented there. RustDriving therefore **does not redistribute the raw clouds or claim the original dataset is Apache-2.0/BSD licensed**. The fetch script reads revision-pinned public files for local research; `raw/` is ignored. Obtain any necessary rights before redistributing original or derived data. The repository license and a public download do not establish permissions beyond their documented scope.
