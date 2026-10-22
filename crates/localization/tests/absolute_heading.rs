use rustdriving_core::{AbsoluteHeading, HeadingDecision, Pose, wrap_angle};
use rustdriving_localization::Ekf;

fn sample(stamp: f64, yaw_rad: f64, variance_rad2: f64) -> AbsoluteHeading {
    AbsoluteHeading {
        stamp,
        yaw_rad,
        variance_rad2,
    }
}

#[test]
fn a_wrapped_observation_uses_the_short_arc_and_keeps_gnss_clocks_separate() {
    let mut filter = Ekf::new(Pose {
        yaw: std::f64::consts::PI - 0.01,
        ..Pose::default()
    });
    let before = filter.state().pose.position;
    assert_eq!(
        filter.correct_heading(sample(0., -std::f64::consts::PI + 0.01, 1e-4)),
        HeadingDecision::Accepted
    );
    assert!(wrap_angle(filter.state().pose.yaw - (-std::f64::consts::PI + 0.01)).abs() < 1e-4);
    assert_eq!(filter.state().pose.position.distance(before), 0.);
    assert!(filter.last_gnss.is_infinite());
    assert_eq!(filter.diagnostics().accepted_fixes, 0);
    assert!((filter.pose_covariance()[2][2] - 0.03 * 1e-4 / 0.0301).abs() < 1e-15);
}

#[test]
fn a_correlated_prior_matches_the_closed_form_conditioned_gaussian() {
    let prior = [[1., 0.1, 0.02], [0.1, 1., -0.01], [0.02, -0.01, 0.03]];
    let mut filter = Ekf::with_pose_covariance(Pose::default(), prior).unwrap();
    assert_eq!(
        filter.correct_heading(sample(1., 0.1, 0.01)),
        HeadingDecision::Accepted
    );
    let mean = [
        filter.state().pose.position.x,
        filter.state().pose.position.y,
        filter.state().pose.yaw,
    ];
    for i in 0..3 {
        assert!((mean[i] - prior[i][2] / 0.04 * 0.1).abs() < 1e-14);
        for j in 0..3 {
            let expected = prior[i][j] - prior[i][2] * prior[2][j] / 0.04;
            assert!((filter.pose_covariance()[i][j] - expected).abs() < 1e-14);
        }
    }
}

#[test]
fn rejection_and_timestamp_watermarks_cannot_be_replaced_by_an_old_good_sample() {
    let mut filter = Ekf::new(Pose::default());
    let before = filter.pose_covariance();
    assert_eq!(
        filter.correct_heading(sample(1., 0.53, 1e-4)),
        HeadingDecision::RejectedInnovation
    );
    assert_eq!(filter.pose_covariance(), before);
    assert_eq!(filter.state().pose.yaw, 0.);
    assert_eq!(filter.heading_diagnostics().last_accepted_stamp, None);
    assert_eq!(
        filter.correct_heading(sample(1., 0., 1e-4)),
        HeadingDecision::IgnoredTimestamp
    );
    assert_eq!(
        filter.correct_heading(sample(0., 0., 1e-4)),
        HeadingDecision::IgnoredTimestamp
    );
    assert_eq!(filter.heading_diagnostics().rejected_samples, 1);
    assert_eq!(
        filter.correct_heading(sample(2., 0., 1e-4)),
        HeadingDecision::Accepted
    );
    assert_eq!(filter.heading_diagnostics().last_accepted_stamp, Some(2.));
}

#[test]
fn malformed_observations_leave_pose_covariance_and_all_clocks_unchanged() {
    let mut filter = Ekf::new(Pose::default());
    for bad in [
        sample(f64::NAN, 0., 0.01),
        sample(-1., 0., 0.01),
        sample(0., f64::INFINITY, 0.01),
        sample(0., 4., 0.01),
        sample(0., 0., 0.),
        sample(0., 0., -1.),
        sample(0., 0., f64::NAN),
        sample(0., 0., 10.),
    ] {
        let before = filter.pose_covariance();
        assert_eq!(filter.correct_heading(bad), HeadingDecision::Invalid);
        assert_eq!(filter.pose_covariance(), before);
        assert_eq!(filter.state().pose.yaw, 0.);
        assert_eq!(filter.heading_diagnostics().last_observed_stamp, None);
        assert_eq!(filter.heading_diagnostics().accepted_samples, 0);
        assert_eq!(filter.diagnostics().last_observed_stamp, None);
    }
}
