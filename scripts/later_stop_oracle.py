"""Certify every emitted checked stop, including both search directions.

The existing search marker does not encode direction. Full sensor replay binds
candidate generation; this inspector checks actual paths regardless of marker.
"""
import math

from check_obstruction_stops import certify_trajectory


def reconstruct(config,ticks):
    if config.get('search_later_obstruction_stops') is not True or config.get('replan_checked_approaches') is not True:
        raise ValueError('later stopping policy missing')
    limits=config.get('motion_limits') or {};accel=limits.get('max_acceleration_m_s2',2.)
    decel=limits.get('max_deceleration_m_s2',2.5)
    if not all(type(v) in [int,float] and math.isfinite(v) and v>0. for v in [accel,decel]):
        raise ValueError('invalid stopping authority')
    count=searched=bounds=0
    for tick in ticks:
        out=tick['expected'];command=out['command']
        if out['health'] or out.get('emergency',False):
            braking=command['acceleration']
            if not math.isfinite(braking) or braking>-5.99:raise ValueError('fault did not retain braking')
            continue
        decisions=out.get('planning_diagnostics')
        if not isinstance(decisions,dict):raise ValueError('stopping inspection requires planning records')
        selected=decisions.get('selected_candidate')
        if selected is None:continue
        record=decisions['candidates'][selected]
        if record['disposition']!='CheckedStop':continue
        marker=record['profile']['searched_stop']
        if type(marker) is not bool:raise ValueError('invalid search marker')
        points=out['trajectory']['points']
        if (len(points)<2 or out['trajectory']['mode']!='Yield' or points[-1]['speed']!=0.
                or points[-1]['position']!=points[-2]['position']
                or points[-1]['time']-points[-2]['time']<8.-1e-9):
            raise ValueError('checked stop lost its complete stationary tail')
        for a,b in zip(points,points[1:]):
            duration=b['time']-a['time'];distance=math.dist([a['position'][k] for k in ['x','y']],[b['position'][k] for k in ['x','y']])
            if (not all(math.isfinite(v) for v in [duration,distance,a['speed'],b['speed']])
                    or duration<=0. or min(a['speed'],b['speed'])<0.
                    or abs(distance-(a['speed']+b['speed'])*duration/2)>1e-7*max(1.,distance)
                    or not -decel-1e-7<=(b['speed']-a['speed'])/duration<=accel+1e-7):
                raise ValueError('unreachable stopping segment')
        proof=certify_trajectory(out,config['vehicle']['radius']);count+=1;searched+=marker;bounds+=proof['interval_bounds']
    return dict(passed=True,verified_control_ticks=len(ticks),certified_checked_stops=count,
        certified_searched_stops=searched,interval_bounds=bounds,all_search_directions_checked=True,
        minimum_stationary_tail_s=8.,actor_truth_used=False,counterfactual_optimality_verified=False)
