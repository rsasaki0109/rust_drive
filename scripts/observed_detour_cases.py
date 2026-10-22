"""Fixed maps and paired inputs for sensor-observed, stopped route detours."""
import copy
import json
import math
from pathlib import Path

REPO=Path(__file__).resolve().parents[1]
MODES=['detour-on','detour-off','known-closure','no-alternative-on','no-alternative-off',
       'moving-on','moving-off','late-on','late-off','external-on','external-off']
GATES=dict(default_enabled=False,minimum_acquisitions=3,minimum_span_s=.2,maximum_gap_s=.35,
    maximum_measured_speed_m_s=.2,maximum_anchor_displacement_m=.15,maximum_retained_edges=8,
    clearance_m=1.,road_reserve_m=.1,rmse_m=.3,maximum_error_m=.6,maximum_yaw_rad=.1,
    source_duration_s=70.,moving_control_duration_s=90.,minimum_forecast_coverage=.99,
    forecast_horizons_s=[.2,1.,2.,4.,8.],minimum_coverage_samples=100,
    forecast_budgets=dict(position_error_m=.2,velocity_error_m_s=.3,acceleration_error_m_s2=.1),
    handover_confirmations=3,maximum_handover_speed_m_s=.05,
    maximum_previous_witness_age_s=.15,minimum_measured_radius_m=.4,edge_interior_reserve_m=1.,
    require_all_nominal_candidates_blocked_by_one_identity=True,
    separate_external_closure_ownership=True,retain_preferences_for_mission=True,
    preserve_pending_on_irrelevant_external_updates=True,
    unchanged_in_pairs='Original map, bodies, sensors, budgets, motion limits, deadlines and acceptance; only navigation.avoid_observed_obstructions differs')


def specifications(plants):
    return [dict(plant=p,world=m.rsplit('-',1)[0] if m.endswith(('-on','-off')) else m,
                 mode=m,seed=s,enabled=m.endswith('-on')) for p in plants for s in [7,42] for m in MODES]


def materialize(spec):
    source='scenarios/route-handover.json';value=json.loads((REPO/source).read_text())
    value.update(name='Acquired obstruction route detour',full_rate_recording=True,min_clearance_m=1.,
        objects=[dict(s=65.,lateral=0.,radius=1.)],navigation_updates=[],
        record_planning_diagnostics=True,record_planar_sweep=True,
        fit_sparse_circles=True,fit_recent_track_motion=True,enclose_track_measurements=True,
        settle_obstruction_stops=True,search_obstruction_stops=True,lateral_candidate_margin_m=.35,
        forecast_envelope=copy.deepcopy(GATES['forecast_budgets']),forecast_consistency_guard=True,
        absolute_heading_fusion=True,heading_sensor=dict(period_ticks=4,variance_rad2=1e-4))
    value['navigation']['hold_for_handover']=True
    if spec['enabled']:value['navigation']['avoid_observed_obstructions']=True
    for edge in value['navigation']['network']['edges']:
        if edge['id']=='detour':
            edge['points']=[dict(x=float(x),y=20.*math.sin(math.pi*(x-40.)/120.)**2) for x in range(40,161,2)]
            edge['points'][0]['y']=edge['points'][-1]['y']=0.
    if spec['world']=='no-alternative':
        value['navigation']['network']['edges']=[e for e in value['navigation']['network']['edges'] if e['id']!='detour']
        value['expected']='stop'
    elif spec['world']=='moving':
        value['objects']=[dict(s=65.,lateral=20.,radius=1.,lateral_speed=-2.5)]
        value['duration']=90.
    elif spec['world']=='late':
        value['objects'][0]['active_from']=12.;value['expected']='stop'
    elif spec['world']=='external':
        value['navigation_updates']=[dict(stamp=3.,revision=1,closed_edges=['south-branch']),
                                     dict(stamp=10.,revision=2,closed_edges=[])]
    elif spec['world']=='known-closure':
        value['navigation_updates']=[dict(stamp=3.,revision=1,closed_edges=['main'])]
    elif spec['world']!='detour':raise ValueError('unknown fixed detour world')
    from check_clear_sweep_recovery import recovery_policy
    value['clear_sweep_recovery']=recovery_policy(spec['plant'])
    return source,value
