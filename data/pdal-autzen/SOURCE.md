# Autzen measured LiDAR fixture

This is a new physical environment for the terrain evaluator: Autzen Stadium,
Oregon, rather than another crop of the previously viewed ISPRS sites. It is
**one environment and one thinned scan**, not evidence of general scene coverage.

`manifest.json` pins `PDAL/PDAL` revision
`64c11506cccdf790b9677458d5ee884bdf4e754b`, the original LAS path, exact byte count
and SHA-256. The upstream [clipping tutorial](https://github.com/PDAL/PDAL/blob/64c11506cccdf790b9677458d5ee884bdf4e754b/doc/tutorial/clipping/index.md)
identifies the measured Autzen point cloud and Oregon feet coordinate system.
The selected LAS header names TerraScan as producer; it contains 10,653 records
in uncompressed LAS 1.2 point format 3. Its WKT specifies international feet:
**multiply all three coordinates by 0.3048 before local recentering and processing**.
The LAS integer scales/offsets must be decoded first. Do not infer metre units
from the filename or silently treat projected coordinates as sensor coordinates.

The original embedded LAS classifications are evaluation references that
predate RustDrive. Class 2 is ground; classes 3–6 and 9–11 are non-ground.
Unclassified/default classes 0/1, noise 7, key-point/reserved 8/12–31 and withheld
points are excluded. The label generation/independent manual review process is
not established here; call them **upstream reference labels**, not audited manual
ground truth. Classification bytes must never be passed into the algorithm.

The [upstream license](https://github.com/PDAL/PDAL/blob/64c11506cccdf790b9677458d5ee884bdf4e754b/LICENSE.txt)
says that, unless otherwise indicated, all files in the distribution use the
BSD license. Its full notice is retained as `UPSTREAM-LICENSE.txt`. Original
survey acquisition terms are not separately established. RustDrive commits
only acquisition metadata and keeps raw bytes ignored; it does not redistribute
the scan or assert that the repository license replaces third-party rights.

```sh
python3 scripts/fetch-additional-datasets.py --dataset pdal-autzen
python3 scripts/fetch-additional-datasets.py --dataset pdal-autzen --verify-only
```

The file was selected and pinned before any RustDrive classification score was
computed. A useful held-out result requires an independently recorded algorithm
freeze. Do not select another crop or tune parameters after seeing its score.
