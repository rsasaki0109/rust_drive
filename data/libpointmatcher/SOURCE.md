# ASL apartment scans distributed with libpointmatcher

- Repository: [ethz-asl/libpointmatcher](https://github.com/ethz-asl/libpointmatcher).
- Pinned revision: `62b347813e6aa4f0129457859e12a0371cc7a5da`.
- Source files: [`examples/icp_tutorial/cloud_0.vtk`](https://github.com/ethz-asl/libpointmatcher/blob/62b347813e6aa4f0129457859e12a0371cc7a5da/examples/icp_tutorial/cloud_0.vtk) and [`cloud_1.vtk`](https://github.com/ethz-asl/libpointmatcher/blob/62b347813e6aa4f0129457859e12a0371cc7a5da/examples/icp_tutorial/cloud_1.vtk).
- Hashes, byte sizes and frozen split: [manifest.json](manifest.json).
- Two unchanged ASCII VTK files: **3,205,194 bytes**, **36,674 / 36,670 points**.

The pinned [ICP tutorial](https://github.com/ethz-asl/libpointmatcher/blob/62b347813e6aa4f0129457859e12a0371cc7a5da/doc/ICPIntro.md) explicitly identifies these two views as scans from the ASL/ETH Zurich apartment dataset and links its original acquisition site. This documents measured geometry, unlike an authored point cloud. The exported fixtures do not provide acquisition timestamps, a sensor calibration or independently measured relative poses. Their normals/descriptors, when present, are upstream processing products and are not new RustDrive measurements.

Use `cloud_0.vtk` for calibration and `cloud_1.vtk` for held-out perturbation evaluation. These are different views of the **same apartment**, so this split cannot establish generalization to independent environments. Do not tune algorithm settings using the held-out cloud or its evaluation output.

Known SE(2)/SE(3) perturbations of measured geometry can test transform recovery, but their motion is **synthetic**. Report these as semi-synthetic registration tests. Natural cloud-to-cloud alignment may report residuals, overlap and convergence; without an independent reference pose it cannot report physical translation/rotation accuracy. Upstream coordinate units should be preserved; a meter interpretation and any reduction to a horizontal model are explicit evaluation calibration, not recovered acquisition metadata.

The separate `examples/data/car_cloud400.csv`, `car_cloud401.csv` and `carCloudList.csv` were investigated but are not included in this acquisition manifest. The [basic registration tutorial](https://github.com/ethz-asl/libpointmatcher/blob/62b347813e6aa4f0129457859e12a0371cc7a5da/doc/BasicRegistration.md) calls them a car example without documenting sensor/location acquisition. [Commit 33aa994](https://github.com/ethz-asl/libpointmatcher/commit/33aa99455b77fdda9abfb36f0c45d9cb334b0353) changes the car cloud under the message “enhance example code and add noise to position of test data” and adds an explicit point transformation to an example converter. The listed transforms are therefore not defensible independent physical-pose truth. They must not be promoted to measured automotive registration accuracy.

## Acquisition and license scope

```sh
python scripts/fetch-datasets.py --dataset libpointmatcher
python scripts/fetch-datasets.py --dataset libpointmatcher --verify-only
```

The repository declares BSD-3-Clause, copyright © 2010–2023 the libpointmatcher authors; its notice is preserved in [UPSTREAM-LICENSE.txt](UPSTREAM-LICENSE.txt). The tutorial identifies an external original ASL dataset, but the pinned example does not establish separate original data redistribution terms. RustDrive therefore does not redistribute these raw files or relicense them under Apache-2.0. Revision-pinned acquisition is for local research; `raw/` is ignored. Document any separate permission before redistributing original or derived clouds.
