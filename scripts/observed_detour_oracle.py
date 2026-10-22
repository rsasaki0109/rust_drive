"""Batch-acquisition reference for advisory observed edge avoidance (no actor truth)."""
import copy
import math

from check_mid_edge import reference


def project(points,position):
    best=None;cursor=0.
    for a,b in zip(points,points[1:]):
        delta=[b[k]-a[k] for k in ['x','y']];length=math.hypot(*delta)
        offset=[position[k]-a[k] for k in ['x','y']]
        fraction=min(1.,max(0.,sum(x*y for x,y in zip(delta,offset))/(length*length)))
        distance=math.hypot(*(x-fraction*y for x,y in zip(offset,delta)))
        candidate=(distance,cursor+fraction*length)
        if best is None or candidate[0]<best[0]:best=candidate
        cursor+=length
    return best


def equal(actual,expected):
    if isinstance(expected,dict):
        if not isinstance(actual,dict) or actual.keys()!=expected.keys():return False
        return all(equal(actual[k],v) for k,v in expected.items())
    if isinstance(expected,list):
        return isinstance(actual,list) and len(actual)==len(expected) and all(equal(a,b) for a,b in zip(actual,expected))
    if type(expected) is float:
        return type(actual) in [int,float] and math.isfinite(actual) and abs(actual-expected)<=1e-9
    return type(actual) is type(expected) and actual==expected


class ObservedDetours:
    def __init__(self,config):
        self.config=config;self.nav=config['navigation'];self.radius=config['vehicle']['radius']
        self.deceleration=(config.get('motion_limits') or {}).get('max_deceleration_m_s2',2.5)
        self.enabled=self.nav.get('avoid_observed_obstructions',False)
        if type(self.enabled) is not bool:raise ValueError('mistyped observed-detour policy')
        self.previous=None;self.last_scan=None;self.consumed=None;self.fault=None;self.prefix=[]
        self.report=dict(retained_edges=[],candidate=None,last_decision=None,last_decision_control_s=None)
        self.attempts=[]

    def witness(self,tick,active,phase,revision,stamp):
        out=tick['expected'];now=out['time'];update=tick['input'].get('navigation_update');d=self.previous
        if out['health'] or phase!='Following' or (update is not None and update['revision']>revision) or d is None:return None
        if any(not t['confirmed'] for t in (out.get('cold_motion') or {}).get('tracks',[])):return None
        scene=d['scene_acquisition_s'];records=d['candidates']
        if scene is None or not math.isfinite(scene) or scene>stamp or now-scene>.15+1e-9 or not 1<=len(records)<=9:return None
        if any(r['disposition']=='CompletePath' or not r['nominal_forecast_conflict'] for r in records):return None
        ids={r['nominal_forecast_conflict']['prediction_id'] for r in records}
        if len(ids)!=1:return None
        ident=next(iter(ids));track=next((t for t in out['tracks'] if t['id']==ident and t['last_seen']==stamp),None)
        if track is None or not all(math.isfinite(t[k]) for t in [track['position'],track['velocity']] for k in ['x','y']):return None
        if math.hypot(track['velocity']['x'],track['velocity']['y'])>.2 or not math.isfinite(track['radius']) or track['radius']<.4:return None
        center=track['position'];ego=out['estimate']
        if project(active['points'],center)[1]<=project(active['points'],ego['pose']['position'])[1]+self.radius+2.:return None
        matches=[];retained={e['evidence']['edge'] for e in self.report['retained_edges']}
        for span in active['traversals']:
            ident_edge=span['edge']
            if ident_edge in retained:continue
            edge=next(e for e in self.nav['network']['edges'] if e['id']==ident_edge)
            distance,s=project(edge['points'],center)
            if (span['start_s_m']+track['radius']+1.<s<span['end_s_m']-track['radius']-1.
                and distance+track['radius']<=edge['half_width']):matches.append(ident_edge)
        if len(matches)!=1:return None
        return dict(stamp=stamp,edge=matches[0],prediction_id=ident,position=copy.deepcopy(center))

    def prospective(self,active,ego,closed,edge):
        excluded=list(dict.fromkeys(closed+[r['evidence']['edge'] for r in self.report['retained_edges']]+[edge]))
        wanted=reference(self.nav,excluded)
        if wanted is None:return 'NoAlternative'
        # Independently find the first divergent traversal, then the first
        # excluded traversal. These are map arcs, never actor/simulator arcs.
        spans=active['traversals'];common=0.
        for a,b in zip(spans,wanted['traversals']):
            if a['edge']!=b['edge']:break
            common+=a['end_s_m']-a['start_s_m']
        boundaries=[common];cursor=0.
        for span in spans:
            if span['edge'] in excluded:boundaries.append(cursor)
            cursor+=span['end_s_m']-span['start_s_m']
        stop=max(0.,min(boundaries)-self.radius-2.);progress=project(active['points'],ego['pose']['position'])[1]
        reachable=(stop-progress>=max(0.,ego['speed'])**2/(1.6*self.deceleration)+1.
                   or (abs(ego['speed'])<=.05 and progress<=stop+.5))
        return ('DetourRequested' if reachable and wanted['half_width']>self.radius and len(wanted['points'])<=5000
                and wanted['length_m']<=1500. else 'UnreachableDivergence')

    def observe(self,tick,active,phase,revision,closed):
        if not self.enabled:
            self.previous=tick['expected'].get('planning_diagnostics');return []
        source=tick['input'];out=tick['expected'];now=out['time'];scan=source.get('lidar')
        if source.get('lidar_failed'):self.fault=now
        if scan is not None:
            stamp=scan['stamp'];valid=(type(stamp) in [int,float] and math.isfinite(stamp) and 0.<=stamp<=now+1e-9
                and len(scan['points'])<=20000 and all(math.isfinite(v) for p in scan['points'] for v in p.values())
                and all(math.hypot(p['x'],p['y'])<=200. for p in scan['points']))
            if not valid or source.get('lidar_failed'):self.fault=now
            elif ('InvalidLidar' not in out['health'] and 'MotionDiscontinuity' not in out['health']
                  and (self.last_scan is None or stamp>self.last_scan) and (self.fault is None or stamp>self.fault)):
                self.last_scan=stamp;self.fault=None
        valid_scene=self.last_scan is not None and now-self.last_scan<=.35+1e-9 and (self.consumed is None or self.last_scan>=self.consumed)
        fresh=valid_scene and (self.consumed is None or self.last_scan>self.consumed)
        if fresh:self.consumed=self.last_scan
        point=self.witness(tick,active,phase,revision,self.last_scan) if valid_scene else None
        compatible=(point is not None and self.prefix and point['edge']==self.prefix[0]['edge']
            and point['prediction_id']==self.prefix[0]['prediction_id']
            and self.last_scan-self.prefix[-1]['stamp']<=.35+1e-9
            and math.dist([point['position'][k] for k in ['x','y']],[self.prefix[0]['position'][k] for k in ['x','y']])<=.15)
        if point is None:self.prefix=[]
        elif fresh:self.prefix=self.prefix+[point] if compatible else [point]
        elif not compatible:self.prefix=[]
        candidate=None
        if self.prefix:
            first,last=self.prefix[0],self.prefix[-1]
            candidate=dict(edge=first['edge'],prediction_id=first['prediction_id'],first_acquisition_s=first['stamp'],
                last_acquisition_s=last['stamp'],acquisition_count=len(self.prefix),anchor_position=copy.deepcopy(first['position']))
        self.report['candidate']=candidate
        if fresh and candidate is not None and len(self.prefix)>=3 and self.prefix[-1]['stamp']-self.prefix[0]['stamp']>=.2-1e-9:
            decision=('CapacityReached' if len(self.report['retained_edges'])>=8 else self.prospective(active,out['estimate'],closed,candidate['edge']))
            self.report.update(last_decision=decision,last_decision_control_s=now)
            self.attempts.append(dict(control_s=now,decision=decision,edge=candidate['edge']))
            if decision=='DetourRequested':self.report['retained_edges'].append(dict(evidence=copy.deepcopy(candidate),requested_at_control_s=now))
        self.previous=out.get('planning_diagnostics')
        return [r['evidence']['edge'] for r in self.report['retained_edges']]

    def audit_status(self,actual):
        if not self.enabled:
            if actual is not None:raise ValueError('undeclared observed-detour status')
        elif not equal(actual,self.report):raise ValueError('independent observed-detour acquisition, preference or decision differs')

    def summary(self):
        return dict(passed=True,enabled=self.enabled,retained_edges=copy.deepcopy(self.report['retained_edges']),
                    attempts=copy.deepcopy(self.attempts),final_decision=self.report['last_decision'],sensor_only=True)
