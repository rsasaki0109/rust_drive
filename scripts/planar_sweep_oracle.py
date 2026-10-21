"""Independently bind acquired ray outcomes and conditional angular resolution.

An unannotated empty point list has no negative sensing meaning. Clear denotes
a completed physical query, Return indexes an original measured point, and an
unavailable ray prevents certification. This is a planar circular-body model.
"""
import math


def calibration(c):
    fields = {'ray_count','first_angle_rad','angular_step_rad','min_range_m','max_range_m','range_error_m'}
    if not isinstance(c,dict) or c.keys()!=fields:raise ValueError('planar calibration schema differs')
    count=c['ray_count'];step=abs(c['angular_step_rad']);span=step*(count-1)
    if (type(count) is not int or not 64<=count<=4096
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in c.values())
            or abs(c['first_angle_rad'])>math.tau or not 1e-5<=step<=.1
            or not math.tau-step-1e-9<=span<=math.tau+1e-9
            or not 0.<=c['min_range_m']<=1. or not 1.<=c['max_range_m']<=200.
            or c['max_range_m']<=c['min_range_m'] or not 0.<=c['range_error_m']<=.1):
        raise ValueError('invalid full-azimuth calibration')
    # Sort actual directions and measure every cyclic gap, including the seam.
    angles=sorted((c['first_angle_rad']+i*c['angular_step_rad'])%math.tau for i in range(count))
    return max(b-a for a,b in zip(angles,angles[1:]+[angles[0]+math.tau]))


def validate_policy(p):
    if not isinstance(p,dict) or p.keys()!={'lidar','checked_center_radius_m','minimum_resolved_disc_radius_m'}:
        raise ValueError('empty-sweep policy schema differs')
    gap=calibration(p['lidar']);radius=p['checked_center_radius_m'];minimum=p['minimum_resolved_disc_radius_m'];c=p['lidar']
    if (not all(type(x) in (int,float) and math.isfinite(x) for x in [radius,minimum])
            or not 1.<=radius<=100. or not .2<=minimum<=3.
            or radius+minimum+c['range_error_m']>c['max_range_m'] or c['min_range_m']>=minimum
            or radius*math.sin(gap/2.)>=minimum):raise ValueError('unresolved empty-sweep region')
    return gap


def acquired(scan,policy=None):
    sweep=scan.get('sweep')
    if not isinstance(sweep,dict) or sweep.keys()!={'calibration','rays'}:raise ValueError('acquired ray outcomes missing')
    c=sweep['calibration'];gap=calibration(c);rays=sweep['rays'];points=scan['points']
    if policy is not None:
        validate_policy(policy)
        if c!=policy['lidar']:raise ValueError('acquired/declared ray calibration differs')
    if len(rays)!=c['ray_count'] or len(points)>len(rays):raise ValueError('incomplete acquired sweep')
    owners=set()
    for i,ray in enumerate(rays):
        if ray=='Clear':continue
        if not isinstance(ray,dict) or ray.keys()!={'Return'}:raise ValueError('unavailable acquired ray')
        index=ray['Return']
        if type(index) is not int or index in owners or not 0<=index<len(points):raise ValueError('ray/point ownership differs')
        owners.add(index);point=points[index];x,y=point['x'],point['y'];a=c['first_angle_rad']+i*c['angular_step_rad']
        if not all(math.isfinite(v) for v in [x,y]):raise ValueError('invalid measured point')
        # A radial projection and an oriented line residual bind this point to
        # its commanded body-frame ray without nearest-beam reassociation.
        along=x*math.cos(a)+y*math.sin(a);across=abs(x*math.sin(a)-y*math.cos(a))
        if (along<=0. or not c['min_range_m']-c['range_error_m']<=along<=c['max_range_m']+c['range_error_m']
                or across>1e-5*(1.+along)):raise ValueError('measured return differs from commanded beam')
    if len(owners)!=len(points):raise ValueError('measured point has no acquired ray')
    if points or policy is None:return None
    return dict(acquisition_s=scan['stamp'],checked_center_radius_m=policy['checked_center_radius_m'],
                minimum_resolved_disc_radius_m=policy['minimum_resolved_disc_radius_m'],maximum_angular_gap_rad=gap,
                maximum_transverse_gap_m=policy['checked_center_radius_m']*math.sin(gap/2.),ray_count=c['ray_count'])


def blank_recovery():
    return dict(last_evidence=None,confirmation_samples=0,first_acquisition_s=None,last_acquisition_s=None,
                release_count=0,last_release=None)


def audit_stream(config,ticks):
    policy=config.get('clear_sweep_recovery');scans=empty=returns=rays=0;maximum_gap=0.
    if policy is not None:validate_policy(policy)
    for tick in ticks:
        scan=tick['input'].get('lidar')
        if scan is None:continue
        acquired(scan,policy);scans+=1;empty+=not scan['points'];returns+=len(scan['points']);rays+=len(scan['sweep']['rays'])
        maximum_gap=max(maximum_gap,calibration(scan['sweep']['calibration']))
    return dict(passed=True,acquired_sweeps=scans,complete_empty_sweeps=empty,acquired_rays=rays,
                bound_original_points=returns,maximum_angular_gap_rad=maximum_gap,sensor_only=True)
