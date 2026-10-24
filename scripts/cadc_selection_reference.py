"""Independent all-pairs component selection with numerical circle-lens integration."""
import math
from cadc_recorded import require

def proxy(component,common):
    lower=component['min'];upper=component['max']
    centre={k:(lower[k]+upper[k])*.5 for k in ('x','y','z')}
    radius=min(common['max_proxy_radius_m'],max(common['min_proxy_radius_m'],
        math.hypot(upper['x']-lower['x'],upper['y']-lower['y'])*.5+common['radius_padding_m']))
    return centre,radius
def circle_iou_integral(a,b):
    from scipy.integrate import quad
    ac,ar=a;bc,br=b;distance=math.hypot(ac['x']-bc['x'],ac['y']-bc['y'])
    if distance>=ar+br:return 0.
    if distance<=abs(ar-br):return min(ar,br)**2/max(ar,br)**2
    left=max(-ar,distance-br);right=min(ar,distance+br)
    crossing=(distance**2+ar**2-br**2)/(2*distance)
    def height(x):
        return 2*min(math.sqrt(max(0,ar**2-x*x)),math.sqrt(max(0,br**2-(x-distance)**2)))
    stops=[left]+([crossing] if left<crossing<right else [])+[right]
    area=math.fsum(quad(height,x,y,epsabs=1e-10,epsrel=1e-10,limit=150)[0] for x,y in zip(stops,stops[1:]))
    return min(1.,max(0.,area/(math.pi*(ar*ar+br*br)-area)))
def select_reference(components,common,policy):
    ids=[];proxies=[]
    for component in components:
        require(component['point_count']==len(component['indices'])>0,'invalid support count')
        require(component['indices']==sorted(set(component['indices'])),'invalid support ordinals')
        ids.extend(component['indices']);proxies.append(proxy(component,common))
    require(len(ids)==len(set(ids)),'overlapping source-return components')
    require(all(0<=i<common['max_return_indices'] for i in ids),'return ordinal outside bounds')
    order=sorted(range(len(components)),key=lambda i:(-components[i]['point_count'],components[i]['indices'][0],i))
    kept=[];low=[];suppressed=[]
    for i in order:
        centre,_=proxies[i];distance=math.hypot(centre['x'],centre['y'])
        band=0 if distance<common['range_breaks_m'][0] else 1 if distance<common['range_breaks_m'][1] else 2
        if components[i]['point_count']<policy['min_points_by_range'][band]:
            low.append(i);continue
        winner=None
        if policy['overlap_threshold'] is not None:
            for j in kept:
                other,_=proxies[j]
                if abs(centre['z']-other['z'])<=common['max_center_height_gap_m'] and circle_iou_integral(proxies[i],proxies[j])>=policy['overlap_threshold']:
                    winner=j;break
        if winner is None:kept.append(i)
        else:suppressed.append([i,winner])
    return {'kept_indices':sorted(kept),'low_support_indices':sorted(low),'suppressed':sorted(suppressed)}
def aggregate(frames,policy_ids):
    result={}
    for domain in ('clear','winter'):
        rows=[r for r in frames if r['domain']==domain]
        if not rows:continue
        result[domain]={}
        for policy in ['raw']+list(policy_ids):
            scores=[r['methods'][policy] for r in rows]
            eligible=sum(s['eligible'] for s in scores);matched=sum(s['matched'] for s in scores)
            result[domain][policy]={'frames':len(rows),'eligible':eligible,'matched':matched,
                'matchFraction':matched/eligible if eligible else None,
                'outputs':sum(s['outputs'] for s in scores),
                'outputsPerFrame':sum(s['outputs'] for s in scores)/len(rows),
                'unassociatedPerFrame':sum(s['unassociated'] for s in scores)/len(rows),
                'rangeBins':[{'near':lo,'far':hi,
                    'eligible':sum(b['eligible'] for s in scores for b in s['rangeBins'] if b['near']==lo),
                    'matched':sum(b['matched'] for s in scores for b in s['rangeBins'] if b['near']==lo)}
                    for lo,hi in ((2,15),(15,30),(30,45))],
                'context':{key:sum(s['context'][key] for s in scores) for key in
                    ('matched','unassignedEligibleOverlap','outsideEligibilityOnly','noAnnotatedOverlap')}}
    return result
def retention_constraints(summary,policy,minimum):
    constraints=[]
    for domain in ('clear','winter'):
        raw=summary[domain]['raw'];candidate=summary[domain][policy]
        far_raw=raw['rangeBins'][-1]['matched'];far_candidate=candidate['rangeBins'][-1]['matched']
        for name,numerator,denominator in [('all',candidate['matched'],raw['matched']),('30-45m',far_candidate,far_raw)]:
            ratio=numerator/denominator if denominator else None
            constraints.append({'domain':domain,'range':name,'retained':numerator,'rawMatched':denominator,
                'ratio':ratio,'passes':ratio is not None and ratio>=minimum})
    return constraints
def choose_policy(summary,protocol):
    candidates=[]
    for policy in protocol['policies']:
        constraints=retention_constraints(summary,policy['id'],protocol['choice']['minimum_development_retention'])
        if all(c['passes'] for c in constraints):
            outputs=sum(summary[d][policy['id']]['outputs'] for d in ('clear','winter'))
            candidates.append((outputs,policy['id'],constraints))
    require(candidates,'no policy satisfies development retention; undefined matches must not qualify')
    _,identity,constraints=min(candidates,key=lambda row:(row[0],row[1]))
    return {'id':identity,'policy':next(p for p in protocol['policies'] if p['id']==identity),'constraints':constraints,
            'objectiveOutputs':sum(summary[d][identity]['outputs'] for d in ('clear','winter'))}
