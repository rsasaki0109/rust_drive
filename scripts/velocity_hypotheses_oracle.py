"""Independent acquisition history and geometry for measured velocity point alternatives."""
import copy
import math

from cold_motion_oracle import ColdMotionOracle
from observed_detour_oracle import equal
from tracking_frame import continuous_observations


class VelocityHypothesesOracle:
    def __init__(self,config):
        if (config.get('forecast_recent_velocity_hypotheses') is not True
                or config.get('forecast_cold_max_speed_m_s') is None or config.get('forecast_envelope') is None):
            raise ValueError('recent velocity prerequisites missing')
        self.position_error=config['forecast_envelope']['position_error_m']
        self.history={};self.consumed=None;self.previous_time=None;self.epoch=0
        self.ticks=self.point_discs=0;self.maximum_samples=0;self.maximum_velocity_difference=0.

    def observe(self,tick,tracks,cold):
        out=tick['expected'];now=tick['input']['time'];epoch=out.get('localization_epoch',0)
        if epoch!=self.epoch:self.history={};self.consumed=self.previous_time=None;self.epoch=epoch
        if not math.isfinite(now) or now<0. or (self.previous_time is not None and now<self.previous_time):
            raise ValueError('recent velocity control clock regressed')
        self.previous_time=now;ids={t['id'] for t in tracks}
        if len(ids)!=len(tracks) or len(ids)>128:raise ValueError('invalid recent velocity identities')
        self.history={i:h for i,h in self.history.items() if i in ids}
        failed=any(h in out['health'] for h in ['InvalidLidar','AcquisitionFailed']);scene=cold.accepted
        fresh=(scene is not None and 0.<=scene<=now and now-scene<=.35+1e-9
            and (self.consumed is None or scene>self.consumed))
        if failed:self.history={};self.consumed=max(now,self.consumed or 0.)
        elif fresh:self.consumed=scene
        for t in tracks:
            if (not all(math.isfinite(v) for k in ['position','velocity'] for v in t[k].values())
                    or math.hypot(*t['velocity'].values())>15.+1e-9 or not math.isfinite(t['radius']) or t['radius']<0.
                    or not math.isfinite(t['last_seen']) or not 0.<=t['last_seen']<=now):
                raise ValueError('invalid acquired recent velocity')
            h=self.history.setdefault(t['id'],[])
            if failed or not cold.states[t['id']]['report']['confirmed'] or now-t['last_seen']>.35+1e-9:
                h.clear();continue
            if h and now-h[-1]['stamp']>.35+1e-9:h.clear()
            if fresh and t['last_seen']==scene:h.append(dict(stamp=scene,velocity=copy.deepcopy(t['velocity'])))
            h[:]=[s for s in h if now-s['stamp']<=.8+1e-9][-32:]
            self.maximum_samples=max(self.maximum_samples,len(h))
            current=[t['velocity'][k] for k in ['x','y']]
            self.maximum_velocity_difference=max([self.maximum_velocity_difference]
                +[math.dist(current,[s['velocity'][k] for k in ['x','y']]) for s in h])
        expected=dict(window_s=.8,maximum_samples_per_identity=32,tracks=[dict(id=i,sample_count=len(h),
            first_acquisition_s=h[0]['stamp'] if h else None,last_acquisition_s=h[-1]['stamp'] if h else None)
            for i,h in sorted(self.history.items())])
        if not equal(out.get('recent_velocity_hypotheses'),expected):raise ValueError('recent velocity acquisition ownership differs')
        self.ticks+=1

    def floor(self,track,forecast,now):
        h=self.history[track['id']]
        if not h:return None
        result=[];age=now-track['last_seen'];origin=[track['position'][k] for k in ['x','y']]
        for i,p in enumerate(forecast['positions']):
            elapsed=age+i*forecast['dt'];center=[p[k] for k in ['x','y']]
            alternatives=[[origin[j]+s['velocity'][k]*elapsed for j,k in enumerate(['x','y'])] for s in h]
            result.append(max(math.dist(center,q)+self.position_error for q in alternatives))
        self.point_discs+=len(h)*len(result)
        return result


def reconstruct(config,ticks):
    cold=ColdMotionOracle(config);recent=VelocityHypothesesOracle(config)
    for tick in ticks:
        tracks,predictions=continuous_observations(tick['expected'],config.get('forecast_pose_sigma'))
        cold.observe(tick,tracks);recent.observe(tick,tracks,cold);by_id={t['id']:t for t in tracks}
        for p in predictions:
            radii=recent.floor(by_id[p['id']],p,tick['input']['time'])
            if radii is not None and any(a>b+1e-8 for a,b in zip(radii,p['additional_radius_m'])):
                raise ValueError('published occupancy lost a recent velocity alternative')
    return dict(passed=True,verified_ticks=recent.ticks,verified_point_discs=recent.point_discs,
        maximum_retained_samples=recent.maximum_samples,maximum_velocity_difference_m_s=recent.maximum_velocity_difference,
        window_s=.8,maximum_samples_per_identity=32,actor_truth_used=False,confidence_calibrated=False,
        cached_acquisitions_not_renewed=True,nominal_centers_preserved=True)
