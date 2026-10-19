use rustdriving_core::{Pose, Vec2};
use rustdriving_sim::{Scenario, pipeline_config};

#[test]
fn physical_mount_never_becomes_operational_calibration() {
    let mut s: Scenario =
        serde_json::from_str(include_str!("../../../scenarios/gnss-delay-curve.json")).unwrap();
    let pose = Pose {
        position: Vec2::new(3., 4.),
        yaw: std::f64::consts::FRAC_PI_2,
    };
    assert_eq!(s.gnss_antenna_position(pose), pose.position);
    s.gnss_antenna_mount_m = Some(Vec2::new(1.2, 0.35));
    s.validate().unwrap();
    assert!(s.gnss_antenna_position(pose).distance(Vec2::new(2.65, 5.2)) < 1e-14);
    assert!(pipeline_config(&s).gnss_antenna_offset_m.is_none());
    let header = serde_json::to_value(pipeline_config(&s)).unwrap();
    assert!(header.get("gnss_antenna_mount_m").is_none());
    assert!(header.get("gnss_antenna_offset_m").is_none());
    s.gnss_antenna_offset_m = Some(Vec2::new(-0.4, 0.2));
    assert_eq!(
        pipeline_config(&s).gnss_antenna_offset_m,
        s.gnss_antenna_offset_m
    );
    assert!(s.gnss_antenna_position(pose).distance(Vec2::new(2.65, 5.2)) < 1e-14);
    for offset in [Vec2::new(0., f64::NAN), Vec2::new(11., 0.)] {
        s.gnss_antenna_mount_m = Some(offset);
        assert!(s.validate().is_err());
        s.gnss_antenna_mount_m = None;
        s.gnss_antenna_offset_m = Some(offset);
        assert!(s.validate().is_err());
    }
}
