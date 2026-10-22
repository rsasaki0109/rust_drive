use rustdriving_core::{Prediction, Vec2};

#[test]
fn occupied_radius_interpolates_and_holds_the_endpoint() {
    let mut p = Prediction {
        id: 1,
        positions: vec![Vec2::default(); 3],
        radius: 0.7,
        dt: 0.2,
        additional_radius_m: vec![],
    };
    assert_eq!(p.radius_at(0.1), 0.7);
    assert_eq!(p.radius_at(100.), 0.7);
    p.additional_radius_m = vec![0., 1., 0.5];
    assert!(p.valid_radii());
    for (t, r) in [(0., 0.7), (0.1, 1.2), (0.2, 1.7), (0.3, 1.45), (10., 1.2)] {
        assert!((p.radius_at(t) - r).abs() < 1e-12);
    }
    p.additional_radius_m.pop();
    assert!(!p.valid_radii());
    assert_eq!(p.radius_at(0.), f64::INFINITY);
    p.additional_radius_m = vec![0., f64::NAN, 0.];
    assert!(!p.valid_radii());
    assert_eq!(p.radius_at(0.1), f64::INFINITY);
}
