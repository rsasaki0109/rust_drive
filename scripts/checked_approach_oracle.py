"""Independent acquisition confirmation and emitted checked-stop certification.

The proposed endpoint is recorded planner evidence. This inspector validates its
support and the adopted path, not optimality of unpublished counterfactual paths.
"""
import math

from check_obstruction_stops import certify_trajectory
from cold_motion_oracle import ColdMotionOracle
from observed_detour_oracle import equal
from tracking_frame import continuous_observations


def blank():
    return dict(supporting_acquisitions=0,first_acquisition_s=None,last_acquisition_s=None,
        proposal_end_progress_m=None,proposal_endpoint=None,proposal_lateral_target_m=None,
        blocking_prediction_ids=[],adopted=False)


class CheckedApproachOracle:
    def __init__(self,config):
        if (config.get('replan_checked_approaches') is not True
                or config.get('forecast_recent_velocity_hypotheses') is not True
                or config.get('settle_obstruction_stops') is not True
                or config.get('search_obstruction_stops') is not True
                or config.get('lateral_candidate_margin_m') is None):
            raise ValueError('checked approach prerequisites missing')
        if any(config.get(k) for k in ['lane_change','traffic_controls','stop_signs','intersections']):
            raise ValueError('checked approach inspection requires fixed or navigation routes without mapped duties')
        self.config=config;self.consumed=None;self.support=None;self.epoch=0;self.switches=0
        self.last_time=None;self.ticks=0;self.adoptions=[];self.bounds=0

    def observe(self,tick,cold):
        out=tick['expected'];now=tick['input']['time'];report=out.get('checked_approach')
        if not isinstance(report,dict):raise ValueError('checked approach status missing')
        epoch=out.get('localization_epoch',0);nav=out.get('navigation') or {};switches=nav.get('switches',0)
        if epoch!=self.epoch:self.support=None;self.consumed=None;self.last_time=None;self.epoch=epoch
        if switches!=self.switches:self.support=None;self.switches=switches
        if not math.isfinite(now) or now<0. or (self.last_time is not None and now<self.last_time):
            raise ValueError('invalid checked approach control clock')
        self.last_time=now;scene=cold.accepted
        constrained=nav.get('phase') not in [None,'Following'] and nav.get('stop_s') is not None
        eligible=not out['health'] and not constrained and all(s['report']['confirmed'] for s in cold.states.values())
        valid=(eligible and scene is not None and math.isfinite(scene) and 0.<=scene<=now
            and now-scene<=.35+1e-9 and (self.consumed is None or scene>=self.consumed))
        fresh=valid and (self.consumed is None or scene>self.consumed)
        if not valid:
            self.consumed=max(now,self.consumed or 0.);self.support=None
            if not equal(report,blank()):raise ValueError('unsupported scene authorized a checked approach')
            self.ticks+=1;return
        if fresh:self.consumed=scene
        endpoint=report.get('proposal_endpoint');progress=report.get('proposal_end_progress_m')
        if endpoint is None:
            self.support=None
            if not equal(report,blank()):raise ValueError('empty checked approach retained confirmation')
            self.ticks+=1;return
        decisions=out.get('planning_diagnostics')
        if not isinstance(decisions,dict):raise ValueError('inspection requires recorded planning decisions')
        here=decisions['route_progress_m'];position=out['estimate']['pose']['position'];target=report['proposal_lateral_target_m']
        ids=report['blocking_prediction_ids'];point=lambda p:[p[k] for k in ['x','y']]
        if (not all(type(v) in [int,float] and math.isfinite(v) for v in [here,progress,target]+point(position)+point(endpoint))
                or progress-here<2. or abs(out['estimate']['speed'])>.2
                or not isinstance(ids,list) or not 1<=len(ids)<=9
                or any(type(i) is not int or i<0 for i in ids) or ids!=sorted(set(ids))
                or not set(ids).issubset(p['id'] for p in out['predictions'])):
            raise ValueError('invalid checked approach endpoint or witness')
        s=self.support
        same=(s is not None and scene-s['last']<=.35+1e-9 and math.dist(point(position),point(s['position']))<=.2
            and abs(here-s['progress'])<=.2 and math.dist(point(endpoint),point(s['endpoint']))<=1.
            and abs(target-s['target'])<=.01 and ids==s['ids'])
        if same:
            if fresh:s['last']=scene;s['count']=min(3,s['count']+1)
        elif fresh:
            s=self.support=dict(first=scene,last=scene,count=1,position=position,progress=here,endpoint=endpoint,target=target,ids=ids)
        else:raise ValueError('cached scene seeded a replacement checked approach')
        adopted=fresh and s['count']>=3 and s['last']-s['first']>=.2-1e-9
        expected=dict(supporting_acquisitions=s['count'],first_acquisition_s=s['first'],last_acquisition_s=s['last'],
            proposal_end_progress_m=progress,proposal_endpoint=endpoint,proposal_lateral_target_m=target,
            blocking_prediction_ids=ids,adopted=adopted)
        if not equal(report,expected):raise ValueError('checked approach clocks or authorization differ')
        path=out['trajectory'];points=path['points']
        if adopted:
            chosen=decisions['candidates'][decisions['selected_candidate']]
            if (chosen['disposition']!='CheckedStop' or path['mode']!='Yield' or path['lateral_target']!=target
                    or len(points)<2 or points[-1]['position']!=endpoint or points[-1]['speed']!=0.
                    or points[-2]['position']!=endpoint or points[-1]['time']-points[-2]['time']<8.-1e-9
                    or abs(chosen['profile']['end_progress_m']-progress)>1e-8):
                raise ValueError('adopted approach lost its complete stopped endpoint')
            proof=certify_trajectory(out,self.config['vehicle']['radius']);self.bounds+=proof['interval_bounds']
            self.adoptions.append(dict(time_s=now,**report));self.support=None
        elif (path['mode']!='Yield' or len(points)<2 or any(not 0.<=p['speed']<=.2
                or math.dist(point(p['position']),point(position))>.2 for p in points)):
            raise ValueError('unconfirmed checked approach restarted movement')
        self.ticks+=1


def reconstruct(config,ticks):
    cold=ColdMotionOracle(config);oracle=CheckedApproachOracle(config)
    for tick in ticks:
        tracks,_=continuous_observations(tick['expected'],config.get('forecast_pose_sigma'))
        cold.observe(tick,tracks);oracle.observe(tick,cold)
    return dict(passed=True,verified_ticks=oracle.ticks,adoptions=oracle.adoptions,
        certified_adopted_paths=len(oracle.adoptions),interval_bounds=oracle.bounds,
        cached_acquisitions_cannot_authorize=True,actor_truth_used=False,
        counterfactual_candidate_optimality_verified=False)
