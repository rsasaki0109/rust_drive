use rustdriving_core::{
    CandidateDisposition, EgoState, Planner, PlanningFailure, Pose, Prediction, Route, Vec2,
};
use rustdriving_planning::LatticePlanner;

fn road(width: f64) -> Route {
    Route::new(vec![Vec2::default(), Vec2::new(100., 0.)], width).unwrap()
}
fn obstacle(x: f64) -> Prediction {
    Prediction {
        id: 77,
        positions: vec![Vec2::new(x, 0.); 41],
        radius: 1.,
        additional_radius_m: vec![],
        dt: 0.2,
    }
}
fn ego() -> EgoState {
    EgoState {
        pose: Pose::default(),
        speed: 4.,
    }
}

#[test]
fn a_nominal_conflict_identifies_the_forecast_and_conservative_arrival_time() {
    let mut planner = LatticePlanner::default();
    planner.set_diagnostics_enabled(true);
    let path = planner.plan(ego(), &road(5.5), &[obstacle(20.)]);
    let report = planner.diagnostics().unwrap();
    assert!(report.failure.is_none());
    assert_eq!(report.candidates.len(), 3);
    let center = &report.candidates[0];
    let conflict = center.nominal_forecast_conflict.as_ref().unwrap();
    assert_eq!(conflict.prediction_id, 77);
    // Straight acceleration 2 m/s² from 4 to 8 m/s: x(t)=8t-4 after 2s.
    // 20-x(t)=1+1.25+0.3+0.06t gives 21.45/8.06. The segment reserve
    // conservatively uses its end time, so its bound can be slightly earlier.
    assert!((conflict.relative_time_s - 21.45 / 8.06).abs() < 0.001);
    assert_eq!(conflict.forecast_radius_m, 1.);
    assert_eq!(conflict.forecast_horizon_s, 8.);
    assert_eq!(conflict.minimum_clearance_floor_m, 0.);
    let selected = report
        .candidates
        .iter()
        .find(|c| Some(c.index) == report.selected_candidate)
        .unwrap();
    assert_eq!(selected.lateral_target_m, path.lateral_target);
    assert_eq!(selected.disposition, CandidateDisposition::CompletePath);
    let profile = selected.profile.as_ref().unwrap();
    assert_eq!(profile.point_count, path.points.len());
    assert_eq!(profile.end_time_s, path.points.last().unwrap().time);
    assert!((profile.end_progress_m - path.points.last().unwrap().position.x).abs() < 1e-12);
}

#[test]
fn blocked_candidates_report_checked_holds_and_infeasible_stops_separately() {
    let mut planner = LatticePlanner::default();
    planner.set_diagnostics_enabled(true);
    planner.settle_obstruction_stops = true;
    planner.search_obstruction_stops = true;
    let path = planner.plan(ego(), &road(2.), &[obstacle(20.)]);
    let report = planner.diagnostics().unwrap();
    let selected = &report.candidates[report.selected_candidate.unwrap() as usize];
    assert_eq!(selected.disposition, CandidateDisposition::CheckedStop);
    assert_eq!(selected.profile.as_ref().unwrap().terminal_speed_m_s, 0.);
    let n = path.points.len();
    assert_eq!(path.points[n - 1].time - path.points[n - 2].time, 8.);
    assert!(
        report.candidates[1..]
            .iter()
            .all(|c| c.disposition == CandidateDisposition::OutsideTargetCorridor)
    );
    let path = planner.plan(ego(), &road(2.), &[obstacle(1.)]);
    assert!(path.points.is_empty());
    let report = planner.diagnostics().unwrap();
    assert_eq!(report.failure, Some(PlanningFailure::NoSelectableCandidate));
    assert!(report.selected_candidate.is_none());
    assert_eq!(
        report.candidates[0].disposition,
        CandidateDisposition::NoCheckedStop
    );
}

#[test]
fn invalid_input_replaces_old_records_without_serializing_invalid_numbers() {
    let mut planner = LatticePlanner::default();
    planner.set_diagnostics_enabled(true);
    planner.plan(ego(), &road(5.5), &[]);
    planner.plan(
        EgoState {
            speed: f64::NAN,
            ..ego()
        },
        &road(5.5),
        &[],
    );
    let report = planner.diagnostics().unwrap();
    assert_eq!(report.failure, Some(PlanningFailure::InvalidInput));
    assert!(report.candidates.is_empty());
    assert!(report.route_progress_m.is_none());
    planner.reset_route_state();
    assert!(planner.diagnostics().is_none());
}

#[test]
fn refinements_report_fresh_support_and_all_nine_bounded_proposals() {
    let mut planner = LatticePlanner::default();
    planner.lateral_candidate_margin_m = Some(0.35);
    planner.refine_lateral_candidates = true;
    planner.set_diagnostics_enabled(true);
    let mut center = obstacle(30.);
    center.radius = 0.5;
    let mut left = obstacle(30.);
    left.id = 78;
    left.radius = 1.5;
    left.positions = vec![Vec2::new(30., 7.1); 41];
    let mut right = left.clone();
    right.id = 79;
    right.positions = vec![Vec2::new(30., -7.1); 41];
    let objects = [center, left, right];
    for stamp in [0., 0., 0.1, 0.2] {
        planner.observe_refinement_scene(Some(stamp));
        let path = planner.plan(ego(), &road(5.5), &objects);
        let report = planner.diagnostics().unwrap();
        assert_eq!(report.candidates.len(), 9);
        assert_eq!(report.scene_acquisition_s, Some(stamp));
        let support = report.refinement_confirmation.as_ref().unwrap();
        assert_eq!(support.confirmed, stamp == 0.2);
        assert_eq!(
            support.acquisition_count,
            if stamp == 0. {
                1
            } else if stamp == 0.1 {
                2
            } else {
                3
            }
        );
        assert_eq!(path.lateral_target, if stamp == 0.2 { 3.0625 } else { 3.5 });
    }
}

#[test]
fn disabled_capture_retains_no_candidate_history() {
    let mut planner = LatticePlanner::default();
    planner.plan(ego(), &road(5.5), &[obstacle(20.)]);
    assert!(planner.diagnostics().is_none());
    planner.set_diagnostics_enabled(true);
    planner.plan(ego(), &road(5.5), &[]);
    assert!(planner.diagnostics().is_some());
    planner.set_diagnostics_enabled(false);
    assert!(planner.diagnostics().is_none());
    planner.plan(ego(), &road(5.5), &[]);
    assert!(planner.diagnostics().is_none());
}
