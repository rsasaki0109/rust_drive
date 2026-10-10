//! Explicit TUM FR1 registered RGB/depth pixel-grid interpretation.
//! This nominal profile follows publisher guidance; it is not a new calibration
//! estimate. No raw image warping or second depth-scale correction occurs.
use rustdriving_core::Vec3;
use serde::{Deserialize, Serialize};
use serde_json::Value;

pub const PROFILE_ID: &str = "tum-fr1-registered-ros-default-v1";

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Operational {
    pub width: usize,
    pub height: usize,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub units_per_metre: f64,
    pub invalid_depth: u16,
    pub depth_quantity: String,
    pub depth_scale_multiplier: f64,
    pub additional_undistortion: bool,
    pub pixel_domain: String,
    pub lens_model: String,
}
impl Operational {
    pub fn nominal() -> Self {
        Self {
            width: 640,
            height: 480,
            fx: 525.,
            fy: 525.,
            cx: 319.5,
            cy: 239.5,
            units_per_metre: 5000.,
            invalid_depth: 0,
            depth_quantity: "axial_z".into(),
            depth_scale_multiplier: 1.,
            additional_undistortion: false,
            pixel_domain: "original_registered_rgb_depth_grid".into(),
            lens_model: "pinhole_on_registered_grid".into(),
        }
    }
}
#[derive(Clone, Debug)]
pub struct RegisteredGrid {
    operational: Operational,
}
impl RegisteredGrid {
    pub fn from_profile(profile: &Value) -> Result<Self, String> {
        let fields = [
            "schema_version",
            "profile_id",
            "operational",
            "source_rgb_provenance",
            "authority",
            "quotations",
            "limitations",
        ];
        if !profile
            .as_object()
            .is_some_and(|o| o.keys().all(|k| fields.contains(&k.as_str())))
        {
            return Err("registered-grid top-level schema mismatch".into());
        }
        if profile["schema_version"] != 1 || profile["profile_id"] != PROFILE_ID {
            return Err("registered-grid profile identity mismatch".into());
        }
        let operational: Operational = serde_json::from_value(profile["operational"].clone())
            .map_err(|e| format!("registered-grid operational schema: {e}"))?;
        if operational != Operational::nominal() {
            return Err("registered-grid operational override rejected".into());
        }
        Ok(Self { operational })
    }
    pub fn operational(&self) -> &Operational {
        &self.operational
    }
    pub fn backproject(&self, x: f64, y: f64, raw_depth: u16) -> Result<Vec3, String> {
        let c = &self.operational;
        if !x.is_finite()
            || !y.is_finite()
            || !(0. ..=(c.width - 1) as f64).contains(&x)
            || !(0. ..=(c.height - 1) as f64).contains(&y)
            || raw_depth == c.invalid_depth
        {
            return Err("invalid registered-grid pixel or depth".into());
        }
        let z = f64::from(raw_depth) / c.units_per_metre;
        Ok(Vec3::new((x - c.cx) * z / c.fx, (y - c.cy) * z / c.fy, z))
    }
    pub fn project(&self, point: Vec3) -> Result<[f64; 2], String> {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() || point.z <= 0. {
            return Err("invalid registered-grid camera point".into());
        }
        let c = &self.operational;
        let pixel = [
            c.fx * point.x / point.z + c.cx,
            c.fy * point.y / point.z + c.cy,
        ];
        if pixel.iter().any(|v| !v.is_finite()) {
            return Err("registered-grid projection overflow".into());
        }
        Ok(pixel)
    }
    pub fn validate_buffers(
        &self,
        width: usize,
        height: usize,
        gray: usize,
        depth: usize,
    ) -> Result<(), String> {
        let c = &self.operational;
        let pixels = width
            .checked_mul(height)
            .ok_or("registered-grid capacity overflow")?;
        if width != c.width
            || height != c.height
            || pixels > 307_200
            || gray != pixels
            || depth != pixels * 2
        {
            return Err("registered-grid bounded image shape mismatch".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustdriving_localization::registration3d::{Pose3, Quaternion};
    use serde_json::json;
    fn profile() -> Value {
        json!({"schema_version":1,"profile_id":PROFILE_ID,"operational":Operational::nominal()})
    }
    #[test]
    fn fractional_registered_ray_projection_and_axial_depth() {
        let grid = RegisteredGrid::from_profile(&profile()).unwrap();
        for (x, y, raw) in [
            (319.5, 239.5, 5000),
            (0., 0., 10000),
            (638.5, 478.5, 7500),
            (187.25, 201.75, 23456),
        ] {
            let point = grid.backproject(x, y, raw).unwrap();
            assert_eq!(point.z, f64::from(raw) / 5000.);
            let pixel = grid.project(point).unwrap();
            assert!((pixel[0] - x).abs() < 1e-12 && (pixel[1] - y).abs() < 1e-12);
        }
        let center = grid.backproject(319.5, 239.5, 5000).unwrap();
        assert_eq!([center.x, center.y, center.z], [0., 0., 1.]);
        let off = grid.backproject(0., 0., 5000).unwrap();
        assert!(off.x.hypot(off.y).hypot(off.z) > off.z);
        assert_eq!(off.z, 1.); // Already scaled; never apply FR1 correction1.035 twice.
    }
    #[test]
    fn closed_profile_rejects_overrides_unknowns_and_cross_profile_identity() {
        for key in [
            "fx",
            "fy",
            "cx",
            "cy",
            "units_per_metre",
            "depth_scale_multiplier",
        ] {
            let mut p = profile();
            p["operational"][key] = json!(17.);
            assert!(RegisteredGrid::from_profile(&p).is_err());
        }
        let mut p = profile();
        p["override"] = json!(525.);
        assert!(RegisteredGrid::from_profile(&p).is_err());
        let mut p = profile();
        p["operational"]["k1"] = json!(0.262383);
        assert!(RegisteredGrid::from_profile(&p).is_err());
        let mut p = profile();
        p["profile_id"] = json!("tum-fr2-registered-ros-default-v1");
        assert!(RegisteredGrid::from_profile(&p).is_err());
        let mut p = profile();
        p["operational"]["additional_undistortion"] = json!(true);
        assert!(RegisteredGrid::from_profile(&p).is_err());
    }
    #[test]
    fn finite_geometry_and_bounded_vga_buffers() {
        let grid = RegisteredGrid::from_profile(&profile()).unwrap();
        grid.validate_buffers(640, 480, 307200, 614400).unwrap();
        for args in [
            (640, 480, 307201, 614400),
            (640, 480, 307200, 614401),
            (320, 240, 76800, 153600),
            (usize::MAX, 2, 0, 0),
        ] {
            assert!(
                grid.validate_buffers(args.0, args.1, args.2, args.3)
                    .is_err()
            );
        }
        for point in [
            Vec3::new(0., 0., 0.),
            Vec3::new(0., 0., -1.),
            Vec3::new(f64::NAN, 0., 1.),
            Vec3::new(f64::MAX, 0., f64::MIN_POSITIVE),
        ] {
            assert!(grid.project(point).is_err());
        }
        for (x, y, raw) in [
            (f64::NAN, 1., 5000),
            (640., 0., 5000),
            (1., -1., 5000),
            (1., 1., 0),
        ] {
            assert!(grid.backproject(x, y, raw).is_err());
        }
    }
    #[test]
    fn known_rigid_motion_uses_one_registered_pixel_domain() {
        let grid = RegisteredGrid::from_profile(&profile()).unwrap();
        let pose = Pose3 {
            translation: Vec3::new(0.01, -0.02, 0.03),
            rotation: Quaternion::from_axis_angle(Vec3::new(0., 1., 0.), 0.04).unwrap(),
        };
        for (x, y, raw) in [
            (100., 100., 5000),
            (400., 140., 10000),
            (320., 380., 7500),
            (600., 420., 12500),
        ] {
            let previous = grid.backproject(x, y, raw).unwrap();
            let current = pose.inverse().transform(previous);
            let pixel = grid.project(current).unwrap();
            // Analytical continuous axial depth, not quantized synthetic sensor range.
            let reconstructed = Vec3::new(
                (pixel[0] - 319.5) * current.z / 525.,
                (pixel[1] - 239.5) * current.z / 525.,
                current.z,
            );
            let recovered = pose.transform(reconstructed);
            assert!(
                (recovered.x - previous.x).abs() < 1e-12
                    && (recovered.y - previous.y).abs() < 1e-12
                    && (recovered.z - previous.z).abs() < 1e-12
            );
        }
    }
}
