//! Bounded loader and exact processed-image projection for KITTI raw calibration.
//! Translations are metres. Native camera K/D are provenance, never an extra warp.
use serde::Serialize;
use std::collections::BTreeMap;

pub const MAX_CALIBRATION_BYTES: usize = 64 * 1024;
pub type Matrix3 = [[f64; 3]; 3];
pub type Matrix34 = [[f64; 4]; 3];
#[derive(Clone, Debug, Serialize)]
pub struct Rigid {
    pub rotation: Matrix3,
    pub translation_m: [f64; 3],
}
impl Rigid {
    pub fn apply(&self, point: [f64; 3]) -> [f64; 3] {
        let p = multiply(self.rotation, point);
        std::array::from_fn(|i| p[i] + self.translation_m[i])
    }
    pub fn compose(&self, other: &Self) -> Self {
        let rotation = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                (0..3)
                    .map(|k| self.rotation[i][k] * other.rotation[k][j])
                    .sum()
            })
        });
        Self {
            rotation,
            translation_m: self.apply(other.translation_m),
        }
    }
}
fn multiply(m: Matrix3, point: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| (0..3).map(|j| m[i][j] * point[j]).sum())
}
fn identity() -> Matrix3 {
    [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
}
#[derive(Clone, Debug, Serialize)]
pub struct Camera {
    pub native_size: [usize; 2],
    pub native_k: Matrix3,
    pub native_d: [f64; 5],
    pub native_cam_from_cam0: Rigid,
    pub rectified_size: [usize; 2],
    pub declared_rectification: Matrix3,
    pub projection: Matrix34,
}
#[derive(Clone, Debug, Serialize)]
pub struct Calibration {
    pub cameras: [Camera; 4],
    pub cam0_from_velo: Rigid,
    pub velo_from_imu: Rigid,
    pub calibration_times: [Option<String>; 3],
    pub corner_distance_m: Option<f64>,
    pub velo_delta_f: Option<[f64; 2]>,
    pub velo_delta_c: Option<[f64; 2]>,
}
#[derive(Clone, Debug, Serialize)]
pub struct ProjectionModel {
    pub camera_index: usize,
    pub rectified_size: [usize; 2],
    pub k: Matrix3,
    pub rectified_baseline_m: [f64; 3],
    pub rectified_camera_center_in_rect0_m: [f64; 3],
    pub shared_r_rect_00: Matrix3,
    pub p_rect_selected: Matrix34,
    pub rectified_camera_from_velo: Rigid,
    pub rectified_camera_from_imu: Rigid,
}
#[derive(Clone, Debug, Serialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    Inframe,
    Outside,
    Behind,
}
#[derive(Clone, Debug, Serialize)]
pub struct Projected {
    pub status: Status,
    pub camera_xyz_m: [f64; 3],
    pub pixel: Option<[f64; 2]>,
}
impl ProjectionModel {
    pub fn project(&self, point: [f64; 3]) -> Result<Projected, String> {
        if point.iter().any(|v| !v.is_finite()) {
            return Err("nonfinite Velodyne point".into());
        }
        let camera = self.rectified_camera_from_velo.apply(point);
        if camera.iter().any(|v| !v.is_finite()) {
            return Err("nonfinite transformed point".into());
        }
        if camera[2] <= 0. {
            return Ok(Projected {
                status: Status::Behind,
                camera_xyz_m: camera,
                pixel: None,
            });
        }
        let homogeneous = multiply(self.k, camera);
        let pixel = [
            homogeneous[0] / homogeneous[2],
            homogeneous[1] / homogeneous[2],
        ];
        if pixel.iter().any(|v| !v.is_finite()) {
            return Err("nonfinite projected pixel".into());
        }
        let status = if pixel[0] >= 0.
            && pixel[1] >= 0.
            && pixel[0] < self.rectified_size[0] as f64
            && pixel[1] < self.rectified_size[1] as f64
        {
            Status::Inframe
        } else {
            Status::Outside
        };
        Ok(Projected {
            status,
            camera_xyz_m: camera,
            pixel: Some(pixel),
        })
    }
}
impl Calibration {
    pub fn from_texts(camera: &str, velo: &str, imu: &str) -> Result<Self, String> {
        let mut camera = fields(camera)?;
        let camera_time = time(&mut camera)?;
        let corner_distance_m = if camera.contains_key("corner_dist") {
            let distance = values::<1>(&mut camera, "corner_dist")?[0];
            if !(0. ..=10.).contains(&distance) || distance == 0. {
                return Err("invalid checkerboard corner distance metadata".into());
            }
            Some(distance)
        } else {
            None
        };
        let mut cameras = Vec::new();
        for index in 0..4 {
            let suffix = format!("_{index:02}");
            let native_size = size(values(&mut camera, &format!("S{suffix}"))?)?;
            let native_k = matrix3(values(&mut camera, &format!("K{suffix}"))?);
            validate_k(native_k)?;
            let native_d = values(&mut camera, &format!("D{suffix}"))?;
            let rotation = matrix3(values(&mut camera, &format!("R{suffix}"))?);
            validate_rotation(rotation)?;
            let translation_m = values(&mut camera, &format!("T{suffix}"))?;
            validate_translation(translation_m)?;
            let rectified_size = size(values(&mut camera, &format!("S_rect{suffix}"))?)?;
            let declared_rectification = matrix3(values(&mut camera, &format!("R_rect{suffix}"))?);
            validate_rotation(declared_rectification)?;
            let p: [f64; 12] = values(&mut camera, &format!("P_rect{suffix}"))?;
            let projection = std::array::from_fn(|i| std::array::from_fn(|j| p[4 * i + j]));
            validate_k(std::array::from_fn(|i| {
                std::array::from_fn(|j| projection[i][j])
            }))?;
            if projection.iter().flatten().any(|v| v.abs() > 1e9) {
                return Err("projection coefficient exceeds bound".into());
            }
            cameras.push(Camera {
                native_size,
                native_k,
                native_d,
                native_cam_from_cam0: Rigid {
                    rotation,
                    translation_m,
                },
                rectified_size,
                declared_rectification,
                projection,
            });
        }
        empty(camera)?;
        let (cam0_from_velo, velo_time, velo_delta_f, velo_delta_c) = transform(velo, true)?;
        let (velo_from_imu, imu_time, _, _) = transform(imu, false)?;
        Ok(Self {
            cameras: cameras.try_into().map_err(|_| "camera count")?,
            cam0_from_velo,
            velo_from_imu,
            calibration_times: [camera_time, velo_time, imu_time],
            corner_distance_m,
            velo_delta_f,
            velo_delta_c,
        })
    }
    pub fn model(&self, index: usize) -> Result<ProjectionModel, String> {
        let camera = self.cameras.get(index).ok_or("camera index must be0..3")?;
        let p = camera.projection;
        let k = std::array::from_fn(|i| std::array::from_fn(|j| p[i][j]));
        // Exact triangular inverse K^-1*p4. Both y baseline and p23 survive;
        // tx=p03/fx would be wrong when either skew or z baseline is nonzero.
        let bz = p[2][3];
        let by = (p[1][3] - k[1][2] * bz) / k[1][1];
        let bx = (p[0][3] - k[0][1] * by - k[0][2] * bz) / k[0][0];
        let baseline = [bx, by, bz];
        validate_translation(baseline)?;
        let shared = self.cameras[0].declared_rectification;
        let rectified_camera_from_velo = Rigid {
            rotation: identity(),
            translation_m: baseline,
        }
        .compose(&Rigid {
            rotation: shared,
            translation_m: [0.; 3],
        })
        .compose(&self.cam0_from_velo);
        let rectified_camera_from_imu = rectified_camera_from_velo.compose(&self.velo_from_imu);
        Ok(ProjectionModel {
            camera_index: index,
            rectified_size: camera.rectified_size,
            k,
            rectified_baseline_m: baseline,
            rectified_camera_center_in_rect0_m: baseline.map(|v| -v),
            shared_r_rect_00: shared,
            p_rect_selected: p,
            rectified_camera_from_velo,
            rectified_camera_from_imu,
        })
    }
}
fn fields(text: &str) -> Result<BTreeMap<String, String>, String> {
    if text.is_empty() || text.len() > MAX_CALIBRATION_BYTES || !text.is_ascii() {
        return Err("calibration byte/ASCII bound".into());
    }
    let mut out = BTreeMap::new();
    for line in text.lines() {
        if line.trim().is_empty() {
            continue;
        }
        if line.len() > 4096 {
            return Err("calibration line bound".into());
        }
        let (key, value) = line
            .split_once(':')
            .ok_or("calibration field missing colon")?;
        if key.is_empty()
            || key.len() > 32
            || !key.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
            || value.trim().is_empty()
            || out.insert(key.into(), value.trim().into()).is_some()
        {
            return Err("invalid or duplicate calibration field".into());
        }
        if out.len() > 64 {
            return Err("calibration field count bound".into());
        }
    }
    Ok(out)
}
fn time(fields: &mut BTreeMap<String, String>) -> Result<Option<String>, String> {
    let Some(t) = fields.remove("calib_time") else {
        return Ok(None);
    };
    if t.len() > 128 || t.bytes().any(|b| b.is_ascii_control()) {
        return Err("invalid calib_time metadata".into());
    }
    Ok(Some(t))
}
fn values<const N: usize>(
    fields: &mut BTreeMap<String, String>,
    key: &str,
) -> Result<[f64; N], String> {
    let text = fields
        .remove(key)
        .ok_or_else(|| format!("required calibration field missing: {key}"))?;
    let tokens: Vec<_> = text.split_whitespace().collect();
    if tokens.len() != N {
        return Err(format!("calibration cardinality: {key}"));
    }
    let mut values: [f64; N] = [0.; N];
    for (i, token) in tokens.iter().enumerate() {
        values[i] = token
            .parse()
            .map_err(|_| format!("invalid numeric calibration: {key}"))?;
        if !values[i].is_finite() {
            return Err(format!("nonfinite calibration: {key}"));
        }
    }
    Ok(values)
}
fn empty(fields: BTreeMap<String, String>) -> Result<(), String> {
    if fields.is_empty() {
        Ok(())
    } else {
        Err(format!(
            "unknown calibration fields: {:?}",
            fields.keys().collect::<Vec<_>>()
        ))
    }
}
fn matrix3(values: [f64; 9]) -> Matrix3 {
    std::array::from_fn(|i| std::array::from_fn(|j| values[i * 3 + j]))
}
fn size(values: [f64; 2]) -> Result<[usize; 2], String> {
    if values
        .iter()
        .any(|&v| !(1. ..=8192.).contains(&v) || v.fract() != 0.)
        || values[0] * values[1] > 16_777_216.
    {
        return Err("image canvas size/capacity bound".into());
    }
    Ok([values[0] as usize, values[1] as usize])
}
fn validate_k(k: Matrix3) -> Result<(), String> {
    if !(1. ..=1e6).contains(&k[0][0])
        || !(1. ..=1e6).contains(&k[1][1])
        || k.iter().flatten().any(|v| !v.is_finite() || v.abs() > 1e6)
        || k[1][0] != 0.
        || k[2] != [0., 0., 1.]
    {
        return Err("camera K must be finite positive upper-triangular normalized pinhole".into());
    }
    Ok(())
}
fn validate_translation(t: [f64; 3]) -> Result<(), String> {
    if t.iter().any(|v| !v.is_finite() || v.abs() > 100.) {
        return Err("metre translation exceeds calibration bound".into());
    }
    Ok(())
}
fn validate_rotation(r: Matrix3) -> Result<(), String> {
    for i in 0..3 {
        for j in 0..3 {
            let dot: f64 = (0..3).map(|k| r[k][i] * r[k][j]).sum();
            if !dot.is_finite() || (dot - if i == j { 1. } else { 0. }).abs() > 1e-4 {
                return Err("non-orthonormal calibration rotation".into());
            }
        }
    }
    let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    if !det.is_finite() || (det - 1.).abs() > 1e-4 {
        return Err("calibration rotation must have positive unit determinant".into());
    }
    Ok(())
}
type TransformFields = (Rigid, Option<String>, Option<[f64; 2]>, Option<[f64; 2]>);
fn transform(text: &str, allow_delta: bool) -> Result<TransformFields, String> {
    let mut f = fields(text)?;
    let time = time(&mut f)?;
    let rotation = matrix3(values(&mut f, "R")?);
    validate_rotation(rotation)?;
    let translation_m = values(&mut f, "T")?;
    validate_translation(translation_m)?;
    let df = if allow_delta && f.contains_key("delta_f") {
        Some(values(&mut f, "delta_f")?)
    } else {
        None
    };
    let dc = if allow_delta && f.contains_key("delta_c") {
        Some(values(&mut f, "delta_c")?)
    } else {
        None
    };
    empty(f)?;
    if df.is_some() != dc.is_some() {
        return Err("delta_f/delta_c must both be present or absent".into());
    }
    Ok((
        Rigid {
            rotation,
            translation_m,
        },
        time,
        df,
        dc,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    // Explicit analytical schema fixture, NOT a KITTI recording/calibration sample.
    fn camera_text() -> String {
        let mut s = "calib_time: analytical fixture\ncorner_dist: 0.1\n".to_string();
        for i in 0..4 {
            s += &format!(
                "S_{i:02}: 100 80\nK_{i:02}: 900 0 40 0 900 30 0 0 1\nD_{i:02}: 0.2 0.3 0.4 0.5 0.6\nR_{i:02}: 1 0 0 0 1 0 0 0 1\nT_{i:02}: 0 0 0\nS_rect_{i:02}: 100 80\nR_rect_{i:02}: {}\nP_rect_{i:02}: 100 10 40 18 0 200 30 -5 0 0 1 0.2\n",
                if i == 0 {
                    "0 -1 0 1 0 0 0 0 1"
                } else {
                    "1 0 0 0 1 0 0 0 1"
                }
            );
        }
        s
    }
    fn velo_text() -> &'static str {
        "calib_time: analytical fixture\nR: 1 0 0 0 0 -1 0 1 0\nT: 1 2 3\ndelta_f: 0 0\ndelta_c: 0 0\n"
    }
    fn imu_text() -> &'static str {
        "calib_time: analytical fixture\nR: 1 0 0 0 1 0 0 0 1\nT: 2 0 0\n"
    }
    #[test]
    fn hand_geometry_retains_y_z_baselines_shared_rectification_and_skew() {
        let c = Calibration::from_texts(&camera_text(), velo_text(), imu_text()).unwrap();
        let m = c.model(2).unwrap();
        // K^-1*[18,-5,0.2]=[0.1055,-0.055,0.2]. Velo[2,4,6]
        // ->cam0[3,-4,7]->shared rect[4,3,7]->selected[4.1055,2.945,7.2].
        for (a, b) in m
            .rectified_baseline_m
            .into_iter()
            .zip([0.1055, -0.055, 0.2])
        {
            assert!((a - b).abs() < 1e-12);
        }
        let p = m.project([2., 4., 6.]).unwrap();
        assert_eq!(p.status, Status::Outside);
        for (a, b) in p.camera_xyz_m.into_iter().zip([4.1055, 2.945, 7.2]) {
            assert!((a - b).abs() < 1e-12);
        }
        let pixel = p.pixel.unwrap();
        assert!((pixel[0] - 728. / 7.2).abs() < 1e-12);
        assert!((pixel[1] - 805. / 7.2).abs() < 1e-12);
        let imu = m.rectified_camera_from_imu.apply([0., 4., 6.]);
        for (a, b) in imu.into_iter().zip(p.camera_xyz_m) {
            assert!((a - b).abs() < 1e-12);
        }
        // NativeK=900/native distortion are deliberately different and unused.
        assert_eq!(m.k[0][0], 100.);
        assert_eq!(c.cameras[2].native_d[0], 0.2);
    }
    #[test]
    fn hand_geometry_canvas_boundaries_and_nonpositive_depth() {
        let m = Calibration::from_texts(&camera_text(), velo_text(), imu_text())
            .unwrap()
            .model(2)
            .unwrap();
        // Invert the explicit rigid chain to target selected-camera[0,0,1].
        let p = m.project([-0.945, -2.2, 1.8945]).unwrap();
        assert_eq!(p.status, Status::Inframe);
        let pixel = p.pixel.unwrap();
        assert!((pixel[0] - 40.).abs() < 1e-10 && (pixel[1] - 30.).abs() < 1e-10);
        assert_eq!(m.project([0., -4., 0.]).unwrap().status, Status::Behind);
        assert!(m.project([f64::NAN, 0., 0.]).is_err());
        assert!(
            Calibration::from_texts(&camera_text(), velo_text(), imu_text())
                .unwrap()
                .model(4)
                .is_err()
        );
    }
    #[test]
    fn independent_half_open_rectified_canvas_edges() {
        let model = ProjectionModel {
            camera_index: 0,
            rectified_size: [100, 80],
            k: identity(),
            rectified_baseline_m: [0.; 3],
            rectified_camera_center_in_rect0_m: [0.; 3],
            shared_r_rect_00: identity(),
            p_rect_selected: [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.]],
            rectified_camera_from_velo: Rigid {
                rotation: identity(),
                translation_m: [0.; 3],
            },
            rectified_camera_from_imu: Rigid {
                rotation: identity(),
                translation_m: [0.; 3],
            },
        };
        for (point, expected) in [
            ([0., 0., 1.], Status::Inframe),
            ([99.5, 79.5, 1.], Status::Inframe),
            ([100., 0., 1.], Status::Outside),
            ([0., 80., 1.], Status::Outside),
            ([-0.5, 0., 1.], Status::Outside),
            ([0., -0.5, 1.], Status::Outside),
            ([0., 0., 0.], Status::Behind),
        ] {
            let projected = model.project(point).unwrap();
            assert_eq!(projected.status, expected);
            assert_eq!(projected.pixel.is_some(), point[2] > 0.);
        }
    }
    #[test]
    fn independent_schema_negative_cases_fail_closed() {
        let valid = camera_text();
        for bad in [
            format!("{valid}unknown: 1\n"),
            format!("{valid}S_00: 100 80\n"),
            valid.replace("K_02: 900", "K_02: NaN"),
            valid.replace("S_rect_02: 100 80", "S_rect_02: 100.5 80"),
            valid.replace("D_02: 0.2 0.3 0.4 0.5 0.6", "D_02: 0.2 0.3"),
            valid.replace(
                "R_rect_02: 1 0 0 0 1 0 0 0 1",
                "R_rect_02: -1 0 0 0 1 0 0 0 1",
            ),
            valid.replace("R_02: 1 0 0 0 1 0 0 0 1", "R_02: 2 0 0 0 1 0 0 0 1"),
            valid.replace("P_rect_02: 100", "P_rect_02: -100"),
            valid.replace(" 0 0 1 0.2", " 0 0 2 0.2"),
        ] {
            assert!(Calibration::from_texts(&bad, velo_text(), imu_text()).is_err());
        }
        assert!(
            Calibration::from_texts(
                &" ".repeat(MAX_CALIBRATION_BYTES + 1),
                velo_text(),
                imu_text()
            )
            .is_err()
        );
        assert!(
            Calibration::from_texts(
                &valid,
                &velo_text().replace("delta_c: 0 0\n", ""),
                imu_text()
            )
            .is_err()
        );
        assert!(
            Calibration::from_texts(&valid, velo_text(), &format!("{}extra: 0\n", imu_text()))
                .is_err()
        );
    }
}
