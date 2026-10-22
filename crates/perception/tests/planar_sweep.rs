use rustdriving_core::{
    ClearSweepRecoveryConfig, LidarScan, PlanarLidarCalibration, PlanarLidarSweep,
    PlanarRayOutcome as Ray, Vec2,
};
use rustdriving_perception::planar_sweep::{
    certify_empty_sweep, validate_calibration, validate_recovery, validate_sweep,
};
use std::f64::consts::{PI, TAU};

fn config() -> ClearSweepRecoveryConfig {
    ClearSweepRecoveryConfig {
        lidar: PlanarLidarCalibration {
            ray_count: 720,
            first_angle_rad: -PI,
            angular_step_rad: TAU / 720.,
            min_range_m: 0.2,
            max_range_m: 45.,
            range_error_m: 0.05,
        },
        checked_center_radius_m: 40.,
        minimum_resolved_disc_radius_m: 0.4,
    }
}
fn empty() -> LidarScan {
    LidarScan {
        stamp: 1.,
        points: vec![],
        sweep: Some(PlanarLidarSweep {
            calibration: config().lidar,
            rays: vec![Ray::Clear; 720],
        }),
    }
}

#[test]
fn complete_empty_rays_have_an_independent_circle_resolution_bound() {
    let c = config();
    let proof = certify_empty_sweep(&empty(), &c).unwrap().unwrap();
    assert!((proof.maximum_transverse_gap_m - 40. * (PI / 720.).sin()).abs() < 1e-12);
    // Even at a worst azimuth halfway between adjacent beams, the declared
    // minimum circular body has a positive ray/circle intersection.
    let offset = 40. * (c.lidar.angular_step_rad / 2.).sin();
    let discriminant = c.minimum_resolved_disc_radius_m.powi(2) - offset.powi(2);
    assert!(discriminant > 0.);
    assert!(
        40. * (c.lidar.angular_step_rad / 2.).cos() - discriminant.sqrt() < c.lidar.max_range_m
    );
}

#[test]
fn a_natively_reversed_inclusive_pattern_keeps_its_actual_gap() {
    let mut c = config();
    c.lidar.first_angle_rad = PI;
    c.lidar.angular_step_rad = -TAU / 719.;
    let mut s = empty();
    s.sweep.as_mut().unwrap().calibration = c.lidar;
    let p = certify_empty_sweep(&s, &c).unwrap().unwrap();
    assert!((p.maximum_angular_gap_rad - TAU / 719.).abs() < 1e-12);
}

#[test]
fn an_unannotated_empty_cloud_is_not_negative_sensing_evidence() {
    let mut s = empty();
    s.sweep = None;
    assert!(certify_empty_sweep(&s, &config()).is_err());
}

#[test]
fn incomplete_unavailable_and_duplicate_point_ownership_are_rejected() {
    for mode in 0..4 {
        let mut s = empty();
        match mode {
            0 => {
                s.sweep.as_mut().unwrap().rays.pop();
            }
            1 => s.sweep.as_mut().unwrap().rays[3] = Ray::Unavailable,
            2 => s.sweep.as_mut().unwrap().rays[3] = Ray::Return(0),
            _ => {
                s.points.push(Vec2::new(-5., 0.));
                let rays = &mut s.sweep.as_mut().unwrap().rays;
                rays[0] = Ray::Return(0);
                rays[719] = Ray::Return(0);
            }
        }
        assert!(validate_sweep(&s).is_err());
    }
}

#[test]
fn every_original_return_is_bound_to_one_actual_beam_and_vetoes_empty_confirmation() {
    let mut s = empty();
    s.points.push(Vec2::new(5., 0.));
    s.sweep.as_mut().unwrap().rays[360] = Ray::Return(0);
    assert_eq!(certify_empty_sweep(&s, &config()).unwrap(), None);
    s.sweep.as_mut().unwrap().rays[360] = Ray::Clear;
    assert!(validate_sweep(&s).is_err());
}

#[test]
fn changed_bearing_range_or_declared_calibration_is_rejected() {
    for point in [
        Vec2::new(5., 1.),
        Vec2::new(100., 0.),
        Vec2::new(f64::NAN, 0.),
    ] {
        let mut s = empty();
        s.points.push(point);
        s.sweep.as_mut().unwrap().rays[360] = Ray::Return(0);
        assert!(validate_sweep(&s).is_err());
    }
    let mut s = empty();
    s.sweep.as_mut().unwrap().calibration.max_range_m = 44.;
    assert!(certify_empty_sweep(&s, &config()).is_err());
}

#[test]
fn incomplete_azimuth_coverage_and_an_unresolved_disc_cannot_be_certified() {
    let mut c = config();
    c.lidar.angular_step_rad = PI / 720.;
    assert!(validate_calibration(&c.lidar).is_err());
    let mut c = config();
    c.lidar.ray_count = 64;
    c.lidar.angular_step_rad = TAU / 64.;
    assert!(validate_recovery(&c).is_err());
    let mut c = config();
    c.checked_center_radius_m = 45.;
    assert!(validate_recovery(&c).is_err());
    let mut c = config();
    c.minimum_resolved_disc_radius_m = 0.1;
    assert!(validate_recovery(&c).is_err());
}
