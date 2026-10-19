use rustdriving_sim::{Scenario, pipeline_config};

fn scenario() -> Scenario {
    serde_json::from_str(include_str!("../../../scenarios/gnss-delay-straight.json")).unwrap()
}

#[test]
fn simulation_noise_is_not_inferred_as_filter_calibration_and_is_validated_independently() {
    let mut s = scenario();
    assert!(s.gnss_noise(7, 12).is_none());
    s.gnss_noise_shape = Some([[1., 0.9], [0.9, 1.]]);
    s.validate().unwrap();
    let c = pipeline_config(&s);
    assert!(c.gnss_covariance_shape.is_none());
    let header = serde_json::to_value(c).unwrap();
    assert!(header.get("gnss_noise_shape").is_none());
    assert!(header.get("gnss_covariance_shape").is_none());
    let fixed = s.gnss_noise(7, 12).unwrap();
    s.objects.clear();
    s.lidar_dropout = Some(0.);
    assert_eq!(s.gnss_noise(7, 12).unwrap(), fixed);
    assert_ne!(s.gnss_noise(42, 12).unwrap(), fixed);
    assert_ne!(s.gnss_noise(7, 16).unwrap(), fixed);
    s.gnss_covariance_shape = Some([[1.9, 0.], [0., 0.1]]);
    assert_eq!(
        pipeline_config(&s).gnss_covariance_shape,
        s.gnss_covariance_shape
    );
    assert_eq!(s.gnss_noise(7, 12).unwrap(), fixed);
    s.gnss_noise_shape = Some([[1., 2.], [2., 1.]]);
    assert!(s.validate().is_err());
}

#[test]
fn keyed_uniform_noise_has_declared_second_moments_for_both_axis_shapes() {
    for shape in [[[1.9, 0.], [0., 0.1]], [[1., 0.9], [0.9, 1.]]] {
        let mut s = scenario();
        s.gnss_noise_shape = Some(shape);
        let mut mean = [0.; 2];
        let mut moment = [[0.; 2]; 2];
        let n = 100_000;
        for i in 0..n {
            let v = s.gnss_noise(313, i * 4).unwrap();
            let v = [v.x, v.y];
            for a in 0..2 {
                mean[a] += v[a] / n as f64;
                for b in 0..2 {
                    moment[a][b] += v[a] * v[b] / n as f64;
                }
            }
        }
        assert!(mean.iter().all(|v| v.abs() < 0.002));
        for a in 0..2 {
            for b in 0..2 {
                assert!((moment[a][b] - 0.02 * shape[a][b]).abs() < 0.0006);
            }
        }
    }
}
