use rustdriving_core::{
    CandidateDisposition, PlanningCandidateDiagnostic, PlanningDiagnostics,
    PlanningForecastConflict, PlanningProfileSummary, Route, TrajectoryPoint,
};

/// One record per enumerated candidate, including every early rejection.
/// Disabled capture neither allocates a record nor owns decision state.
pub(super) struct CandidateTrace<'a> {
    destination: Option<&'a mut Vec<PlanningCandidateDiagnostic>>,
    record: Option<PlanningCandidateDiagnostic>,
}
impl<'a> CandidateTrace<'a> {
    pub fn new(
        diagnostics: Option<&'a mut PlanningDiagnostics>,
        index: usize,
        target: f64,
    ) -> Self {
        Self {
            record: diagnostics.as_ref().map(|_| PlanningCandidateDiagnostic {
                index: index as u8,
                lateral_target_m: target,
                disposition: CandidateDisposition::UnreachableSpeedProfile,
                nominal_forecast_conflict: None,
                profile: None,
            }),
            destination: diagnostics.map(|d| &mut d.candidates),
        }
    }
    pub fn enabled(&self) -> bool {
        self.record.is_some()
    }
    pub fn reject(&mut self, disposition: CandidateDisposition) {
        if let Some(record) = &mut self.record {
            record.disposition = disposition;
        }
    }
    pub fn conflict(&mut self, conflict: Option<PlanningForecastConflict>) {
        if let Some(record) = &mut self.record {
            record.nominal_forecast_conflict = conflict;
        }
    }
    pub fn accept(
        &mut self,
        points: &[TrajectoryPoint],
        route: &Route,
        blocked: bool,
        searched: bool,
    ) {
        if let Some(record) = &mut self.record {
            record.disposition = if blocked {
                CandidateDisposition::CheckedStop
            } else {
                CandidateDisposition::CompletePath
            };
            record.profile = points.last().map(|last| PlanningProfileSummary {
                point_count: points.len(),
                end_progress_m: route.project(last.position).0,
                end_time_s: last.time,
                terminal_speed_m_s: last.speed,
                searched_stop: searched,
            });
        }
    }
}
impl Drop for CandidateTrace<'_> {
    fn drop(&mut self) {
        if let (Some(destination), Some(record)) = (&mut self.destination, self.record.take()) {
            destination.push(record);
        }
    }
}
