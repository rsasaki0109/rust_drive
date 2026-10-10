# Third-party licensing

The optional packaged OpenStreetMap extract and derived ENU road database are © OpenStreetMap contributors, under ODbL 1.0, separately from RustDriving's Apache-2.0 code. [Pinned source, attribution, conversion and redistribution terms](maps/osm/SOURCE.md). No OSMnx implementation is copied or required at runtime. The extract is historical and cropped; its presence does not establish current physical map accuracy.

RustDriving code is original Apache-2.0. No Autoware, Apollo or openpilot implementation is vendored. This is the dependency inventory for the committed Cargo.lock, recorded on 2026-10-08 from Cargo metadata.

| Package | Locked version | SPDX expression |
|---|---|---|
| itoa | 1.0.18 | MIT OR Apache-2.0 |
| memchr | 2.8.3 | Unlicense OR MIT |
| proc-macro2 | 1.0.107 | MIT OR Apache-2.0 |
| quote | 1.0.47 | MIT OR Apache-2.0 |
| serde | 1.0.229 | MIT OR Apache-2.0 |
| serde_core | 1.0.229 | MIT OR Apache-2.0 |
| serde_derive | 1.0.229 | MIT OR Apache-2.0 |
| serde_json | 1.0.151 | MIT OR Apache-2.0 |
| syn | 3.0.6 | MIT OR Apache-2.0 |
| unicode-ident | 1.0.26 | (MIT OR Apache-2.0) AND Unicode-3.0 |
| zmij | 1.0.23 | MIT |

For dual MIT/Apache-2.0 crates, preserve the applicable license when redistributing. `unicode-ident` also includes Unicode-3.0 terms; `zmij` is MIT. Dependency license texts distributed with these exact packages are retained in [THIRD_PARTY_LICENSES.txt](THIRD_PARTY_LICENSES.txt). This inventory covers direct and transitive workspace Rust dependencies, not future simulator/model assets.

Optional visualization uses Pillow 12.3.0, MIT-CMU. It is a development tool, not linked into the Rust executable. The renderer prefers system DejaVu fonts (Bitstream Vera / DejaVu terms) and does not vendor the font files. GIF/PNG output is generated from original simulated data; the README contains no images from reference projects.

Optional 3D visualization invokes the separately installed Blender executable (GPL-3.0-or-later) as an external rendering tool. Blender is not vendored or linked into the Rust executable. The procedural road, hatchback, sedan, van, pickup, traffic-light, stop-sign, yield-sign, perpendicular crossing-road, tree, streetlight and building meshes are original; no external model or texture assets are packaged. The Japanese urban storefronts, balconies, air conditioners, construction dressing, worker, truck, cyclist, pedestrian, family and dog meshes are also original Apache-2.0 procedural geometry. Optional Japanese road lettering uses a separately installed Noto CJK font under OFL-1.1, with a system-font fallback; neither the font nor an external model is packaged, and the signal-only Hero does not use stop-sign lettering. The optional editable `.blend` snapshot contains these generated meshes and materials. Preserve Blender's own license and notices if distributing its binaries. Local capture uses Blender 4.3.2 with the Cycles CPU backend.

Autoware and Apollo root licenses are Apache-2.0; openpilot is MIT at the researched revisions. These projects are architectural references rather than dependencies. See [research](docs/research.md) for source links and artifact-specific licensing restrictions.

The optional standalone RNE adapter has a separate lockfile and [dependency inventory](integrations/rne/THIRD_PARTY.md). RNE is MIT OR Apache-2.0; no RNE source is vendored into the default RustDriving workspace.

The optional [urban-camera diagnostic](docs/road-camera-evaluation.md) reproduces
reference-box metadata from the COCO Consortium, *Microsoft COCO: Common Objects
in Context* (Lin et al., 2014), under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
The report selects seven categories, converts box coordinates and associates
predictions; these are adaptations of annotations, separately licensed from
Apache-2.0 evaluator code. [Pinned official terms, mirror and image attribution](integrations/onnx/README.md#urban-input-provenance-and-separate-licenses).
Four original photographs have CC BY 2.0 metadata and two have CC BY-ND 2.0
metadata. Optional downloads preserve original bytes under ignored artifacts;
the repository and CI uploads contain no original or altered photographs.

The optional [original BDD dashcam diagnostic](docs/bdd-road-evaluation.md)
uses separately pinned BDD100K images and legacy annotations. Its published
box-derived reports retain the Regents of the University of California / BAIR
notice and research, educational and not-for-profit terms in
[LICENSE-data.txt](assets/recorded-bdd/LICENSE-data.txt); these data and
derivatives are not relicensed under Apache-2.0. General commercial rights are
not granted. Raw JPEGs and original JSON are ignored and excluded from CI
uploads. The evaluator implementation remains Apache-2.0.

Optional TUM RGB-D inputs and pinned calibration documents are acquired into
ignored `data/*/raw/` folders. Raw depth, mocap and calibration files are not
redistributed. The source mirrors do not establish the original data's licence;
the official TUM terms have not been independently verified in this session.
[Dataset provenance and limitations](docs/recorded-submaps.md).

The optional [RGB-D visual-motion evaluator](docs/recorded-visual-odometry.md)
also fetches pinned original RGB PNGs into these ignored raw directories.
Images are not redistributed or relicensed. FAST-9, intensity-centroid
orientation, binary intensity comparisons and Horn rigid fitting are
implemented directly in Rust; the deterministic descriptor sampling pattern is
original, without copied OpenCV code or its learned ORB pattern. Independent
pixel reconstruction uses optional NumPy (BSD-3-Clause) and Pillow
(HPND); their versions are pinned in `scripts/requirements-visual.txt`.
