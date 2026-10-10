#!/usr/bin/env python3
"""Independent viewed calibration oracle for measured pyramidal pixel tracking.

The pinned reprojection oracle supplies unchanged pixel reconstruction, Kabsch
coarse consensus, numerical-Jacobian/SVD refinement, clocks and physical scoring.
A static state/physical audit keeps all179updates continuous. Previous measured
feature pixels track to explicit current endpoints; endpoints have no BRIEF descriptors.
Qualification is independently recomputed from timestamp columns and row arity;
no image pixels or ground-truth pose values enter metadata preregistration.
"""
import argparse
from bisect import bisect_left
import copy
from functools import lru_cache
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
OLD_REPROJECTION_SHA = 'fd1425f9c73c21b104d9453a7c2b298c21fa22effd6ecb5f0015bbc41f79cef7'
ALGORITHM = 'bounded_visual_reprojection_pyramidal_tracked'
OLD_TEMPORAL_SHA = '70cb0837aff6882e1d5d13798e4d7d52b6969c79d7ae49f5d3debc66404e06de'
DATASET = 'tum-fr1-room-temporal'
REPOSITORY = 'edrishakimi1/Indoor-SLAM-Floorplan-with-Gaussian-Splatting'
REVISION = '1d5b2c3e1ee186abc1709042b739e9cd8a3b41d1'
PREFIX = 'data/rgbd_dataset_freiburg1_room/'
OLD_PATH = Path(__file__).with_name('check-recorded-reprojection.py')
if hashlib.sha256(OLD_PATH.read_bytes()).hexdigest() != OLD_REPROJECTION_SHA:
    raise ValueError('original reprojection checker must remain immutable')
_spec = importlib.util.spec_from_file_location('immutable_reprojection_oracle', OLD_PATH)
g = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g)
r, k, v = g.r, g.k, g.v


@lru_cache(maxsize=180)
def _pixel_pyramid(pixels, validity=None):
    image = g.np.frombuffer(pixels,dtype=g.np.uint8).reshape(480,640)
    mask = (g.np.ones_like(image,dtype=bool) if validity is None else
            g.np.frombuffer(validity,dtype=g.np.uint8).reshape(480,640).astype(bool))
    output = [(image,mask)]
    weights = (1,4,6,4,1)
    for _ in range(2):
        height,width = image.shape
        sums = g.np.zeros((height//2,width//2),dtype=g.np.uint32)
        valid = g.np.ones_like(sums,dtype=bool)
        for dy,wy in enumerate(weights):
            rows = g.np.clip(g.np.arange(height//2)*2+dy-2,0,height-1)
            for dx,wx in enumerate(weights):
                columns = g.np.clip(g.np.arange(width//2)*2+dx-2,0,width-1)
                indices = g.np.ix_(rows,columns)
                sums += wy*wx*image[indices].astype(g.np.uint32)
                valid &= mask[indices]
        image = ((sums+128)//256).astype(g.np.uint8);mask=valid
        output.append((image,mask))
    return output


class _PixelRejection(Exception):
    pass


def _sample_coordinates(image, coordinates, counter):
    pixels,mask=image;height,width=pixels.shape
    xy=g.np.asarray(coordinates,dtype=float)
    inside=g.np.isfinite(xy).all(axis=1)&(xy[:,0]>=0)&(xy[:,1]>=0)&(xy[:,0]<width-1)&(xy[:,1]<height-1)
    safe=g.np.where(inside[:,None],xy,0.)
    columns=g.np.floor(safe[:,0]).astype(int);rows=g.np.floor(safe[:,1]).astype(int)
    valid=mask[rows,columns]&mask[rows,columns+1]&mask[rows+1,columns]&mask[rows+1,columns+1]
    bad=g.np.flatnonzero(~inside|~valid)
    if len(bad):
        counter[0]+=int(bad[0])+1
        raise _PixelRejection('Border' if not inside[bad[0]] else 'Masked')
    counter[0]+=len(xy)
    dx=xy[:,0]-columns;dy=xy[:,1]-rows
    top=pixels[rows,columns]*(1.-dx)+pixels[rows,columns+1]*dx
    bottom=pixels[rows+1,columns]*(1.-dx)+pixels[rows+1,columns+1]*dx
    return top*(1.-dy)+bottom*dy


_PATCH_OFFSETS = g.np.asarray([(x,y) for y in range(-4,5) for x in range(-4,5)],dtype=float)
_GRADIENT_OFFSETS = g.np.asarray([(0,0),(1,0),(-1,0),(0,1),(0,-1)],dtype=float)


def _pixel_patch(image,center,counter):
    values=_sample_coordinates(image,_PATCH_OFFSETS+center,counter)
    return values-values.mean()


def _pixel_level(previous,current,point,initial,level):
    trace=dict(level=level,start=[float(x) for x in initial],endpoint=[float(x) for x in initial],iterations=0,bilinear_samples=0,
        min_eigenvalue=0.,condition_number=0.,photometric_rms=None,converged=False,rejection=None)
    counter=[0]
    try:
        coordinates=(_PATCH_OFFSETS[:,None,:]+_GRADIENT_OFFSETS[None,:,:]+point).reshape(-1,2)
        samples=_sample_coordinates(previous,coordinates,counter).reshape(81,5)
        reference=samples[:,0]-samples[:,0].mean()
        gradients=g.np.column_stack(((samples[:,1]-samples[:,2])*.5,(samples[:,3]-samples[:,4])*.5))
        gradients-=gradients.mean(axis=0)
        hessian=gradients.T@gradients
        eigenvalues=g.np.linalg.eigvalsh(hessian/81.)
        a,b,c=float(hessian[0,0]/81.),float(hessian[0,1]/81.),float(hessian[1,1]/81.)
        discriminant=math.sqrt((a-c)**2+4*b*b)
        minimum=max(0.,(a+c-discriminant)*.5);maximum=(a+c+discriminant)*.5
        r.require(abs(minimum-max(0.,float(eigenvalues[0])))<=1e-7,
                  'independent eigen diagnostic disagrees with NumPy symmetric eigensolve')
        condition=maximum/minimum if minimum>0 else float(g.np.finfo(float).max)
        trace.update(min_eigenvalue=minimum,condition_number=condition)
        if max(0.,float(eigenvalues[0]))<4.:raise _PixelRejection('LowTexture')
        if float(eigenvalues[1]/eigenvalues[0])>100.:raise _PixelRejection('IllConditioned')
        for _ in range(10):
            trace['iterations']+=1
            measured=_pixel_patch(current,trace['endpoint'],counter)
            step=g.np.linalg.solve(hessian,gradients.T@(measured-reference))
            length=float(g.np.linalg.norm(step))
            if not math.isfinite(length):raise _PixelRejection('NonConvergence')
            if length>2.:step*=2./length
            trace['endpoint']=(g.np.asarray(trace['endpoint'])-step).tolist()
            if length<=.01:
                trace['converged']=True;break
        if not trace['converged']:raise _PixelRejection('NonConvergence')
        residual=_pixel_patch(current,trace['endpoint'],counter)-reference
        trace['photometric_rms']=float(g.np.sqrt(g.np.mean(residual**2)))
        if trace['photometric_rms']>15.:raise _PixelRejection('Photometric')
    except _PixelRejection as error:
        trace['rejection']=str(error)
    trace['bilinear_samples']=counter[0]
    return trace


def _pixel_direction(previous,current,point):
    endpoint=g.np.asarray(point,dtype=float)/4.;levels=[]
    for level in (2,1,0):
        if level!=2:endpoint*=2.
        trace=_pixel_level(previous[level],current[level],g.np.asarray(point)/(2**level),endpoint,level)
        levels.append(trace);endpoint=g.np.asarray(trace['endpoint'])
        if trace['rejection'] is not None:return None,levels,trace['rejection']
    return endpoint.tolist(),levels,None


@lru_cache(maxsize=180)
def _independent_pixel_tracks(previous,current,coordinates,previous_mask=None,current_mask=None):
    if not coordinates:return dict(points=[],bilinear_samples=0,iterations=0,pyramid_output_pixels=0)
    a=_pixel_pyramid(previous,previous_mask);b=_pixel_pyramid(current,current_mask)
    output=[]
    for index,point in enumerate(coordinates):
        forward,fl,rejection=_pixel_direction(a,b,point)
        backward=None;bl=[];distance=None
        if forward is not None:
            backward,bl,rejection=_pixel_direction(b,a,forward)
            if backward is not None:
                distance=math.hypot(backward[0]-point[0],backward[1]-point[1])
                if distance>1.:rejection='ForwardBackward'
        output.append(dict(point_index=index,start=[float(x) for x in point],forward_endpoint=forward,
            backward_endpoint=backward,forward_backward_error=distance,accepted=rejection is None,
            rejection=rejection,forward_levels=fl,backward_levels=bl))
    levels=[level for point in output for level in point['forward_levels']+point['backward_levels']]
    return dict(points=output,bilinear_samples=sum(x['bilinear_samples'] for x in levels),
        iterations=sum(x['iterations'] for x in levels),pyramid_output_pixels=192000)


def independent_pixel_tracks(previous,current,coordinates,previous_mask=None,current_mask=None):
    for image in (previous,current):
        r.require(image.shape==(480,640) and image.dtype==g.np.uint8,'invalid tracker image dimensions/layout')
    for mask in (previous_mask,current_mask):
        if mask is not None:
            r.require(mask.shape==(480,640) and g.np.isin(mask,(0,1)).all(),'invalid tracker binary mask')
    r.require(len(coordinates)<=400 and all(len(p)==2 and all(math.isfinite(x) for x in p)
        and 0<=p[0]<640 and 0<=p[1]<480 for p in coordinates),'invalid original tracker points')
    return _independent_pixel_tracks(previous.tobytes(),current.tobytes(),tuple(tuple(p) for p in coordinates),
        None if previous_mask is None else previous_mask.astype(g.np.uint8).tobytes(),
        None if current_mask is None else current_mask.astype(g.np.uint8).tobytes())


def tracking_witness_check(actual,expected,message='invented pixel tracking witness'):
    if isinstance(expected,bool):r.require(actual is expected,message)
    elif isinstance(expected,dict):
        r.require(isinstance(actual,dict) and actual.keys()==expected.keys(),message)
        for key in expected:tracking_witness_check(actual[key],expected[key],message+' '+key)
    elif isinstance(expected,list):
        r.require(isinstance(actual,list) and len(actual)==len(expected),message)
        for a,e in zip(actual,expected):tracking_witness_check(a,e,message)
    elif type(expected) is int:r.require(type(actual)is int and actual==expected,message)
    elif isinstance(expected,float):
        r.require(type(actual) in (int,float) and math.isfinite(actual),message+' numeric type/bound')
        if abs(expected)>1e200:r.require(actual==expected,message)
        else:r.close(actual,expected,message,1e-7)
    else:r.require(actual==expected,message)


def no_flow_check(row):
    r.require(row['flow_attempted'] is False and row['flow_results']==[]
              and row['tracked_endpoints']==[] and row['flow_work'] is None,
              'performed or invented tracking without accepted reference/clock permission')


def tracked_geometry(row,previous_features,previous_gray,current_gray,previous_depth,current_depth,calibration):
    r.require(row['flow_attempted'] is True,'concealed measured optical flow invocation')
    flow=independent_pixel_tracks(previous_gray,current_gray,[[f['x'],f['y']] for f in previous_features])
    tracking_witness_check(row['flow_results'],flow['points'])
    tracking_witness_check(row['flow_work'],{key:flow[key] for key in ('bilinear_samples','iterations','pyramid_output_pixels')})
    r.require(flow['bilinear_samples']<=3110400 and flow['iterations']<=24000,'tracker work exceeds fixed resource bound')
    endpoints=[dict(previous_feature_index=p['point_index'],current_pixel_xy=p['forward_endpoint'])
               for p in flow['points'] if p['accepted']]
    tracking_witness_check(row['tracked_endpoints'],endpoints,'invented/dropped accepted tracking endpoints')
    r.require(len(row['depth_matches'])==len(endpoints),'omitted tracked endpoint depth evidence')
    pairs=[]
    for index,(endpoint,actual) in enumerate(zip(endpoints,row['depth_matches'])):
        previous_index=endpoint['previous_feature_index']
        r.require(type(actual['previous_index']) is int and actual['previous_index']==previous_index
                  and type(actual['current_endpoint_index']) is int and actual['current_endpoint_index']==index,
                  'invented original feature/current endpoint association')
        previous,p_error=g.depth_point_status(previous_features[previous_index],previous_depth,calibration)
        xy=endpoint['current_pixel_xy'];current,c_error=g.depth_point_status(dict(x=xy[0],y=xy[1]),current_depth,calibration)
        valid=previous is not None and current is not None
        selected=valid and len(pairs)<256
        r.require(actual['accepted'] is valid and actual['correspondence_selected'] is selected,
                  'invented tracked depth validity or fixed correspondence selection')
        if valid:
            v.vector_check(actual['previous_xyz'],previous,'invented tracked previous depth point')
            v.vector_check(actual['current_xyz'],current,'invented tracked current depth point')
            r.require('previous_rejection' not in actual and 'current_rejection' not in actual,
                      'valid tracked depth retains contradictory rejection')
            if selected:
                r.require(type(actual['correspondence_index']) is int and actual['correspondence_index']==len(pairs)
                          and 'selection_rejection' not in actual,'invented original selected correspondence order')
                pairs.append(dict(previous=previous,current=current))
            else:
                r.require('correspondence_index' not in actual and actual['selection_rejection']==
                          'measured correspondence count exceeds unchanged256-point registration bound',
                          'concealed unselected measured depth witness')
        else:
            r.require(actual['previous_rejection']==p_error and actual['current_rejection']==c_error
                      and not any(name in actual for name in ('previous_xyz','current_xyz','correspondence_index','selection_rejection')),
                      'concealed invalid tracked depth')
    r.require(len(row['correspondences'])==len(pairs),'invented/dropped selected tracked correspondences')
    for a,e in zip(row['correspondences'],pairs):
        for side in ('previous','current'):v.vector_check(a[side],e[side],'invented tracked '+side+' correspondence')
    return pairs,flow,endpoints


def tracked_refinement_observations(row,fit,endpoints,pairs):
    mapping={item['correspondence_index']:item for item in row['depth_matches'] if item['correspondence_selected']}
    expected=[]
    for index in fit['inlier_indices']:
        matched=mapping[index]
        expected.append(dict(correspondence_index=index,previous_index=matched['previous_index'],
            current_endpoint_index=matched['current_endpoint_index'],previous_xyz=pairs[index]['previous'],
            current_pixel_xy=endpoints[matched['current_endpoint_index']]['current_pixel_xy']))
    tracking_witness_check(row['refinement_observations'],expected,'invented tracked inlier pixel/depth refinement support')
    return expected


def pixel_contract_checks():
    yy,xx=g.np.mgrid[:480,:640]
    periodic=lambda x,y:128+50*g.np.sin(2*g.np.pi*x/16)+50*g.np.cos(2*g.np.pi*y/16)
    render=lambda values:g.np.floor(g.np.clip(values,0,255)+.5).astype(g.np.uint8)
    previous=render(periodic(xx,yy));current=g.np.roll(previous,1,axis=1)
    points=[[120.,120.],[180.,240.],[300.,360.],[240.5,180.25]]
    tracks=independent_pixel_tracks(previous,current,points)
    r.require(all(p['accepted'] and math.hypot(p['forward_endpoint'][0]-q[0]-1.,
        p['forward_endpoint'][1]-q[1])<.001 for p,q in zip(tracks['points'],points)),
        'known integer translation pixels not measured')
    r.require(all(p['forward_backward_error']<.001 for p in tracks['points']),
              'known reverse pixel motion failed')
    blank=g.np.full((480,640),100,dtype=g.np.uint8)
    flat=independent_pixel_tracks(blank,blank,[[120.,120.]])['points'][0]
    r.require(flat['rejection']=='LowTexture' and flat['forward_endpoint'] is None
              and flat['forward_levels'][0]['condition_number']==g.np.finfo(float).max,
              'rank-zero low-texture sentinel/guard changed')
    diagonal=render(128+50*g.np.sin(2*g.np.pi*(xx+yy)/16))
    rank_one=independent_pixel_tracks(diagonal,diagonal,[[120.,120.]])['points'][0]
    r.require(rank_one['rejection']=='LowTexture' and rank_one['forward_levels'][0]['min_eigenvalue']==0.
              and rank_one['forward_levels'][0]['condition_number']==g.np.finfo(float).max,
              'rank-one analytic diagnostic and independent eigensolve changed')
    border=independent_pixel_tracks(previous,current,[[0.,0.]])['points'][0]
    r.require(border['rejection']=='Border' and border['forward_levels'][0]['bilinear_samples']==1,
              'first attempted border sample must count and reject')
    mask=g.np.zeros((480,640),dtype=g.np.uint8)
    masked=independent_pixel_tracks(previous,current,[[120.,120.]],previous_mask=mask)['points'][0]
    r.require(masked['rejection']=='Masked' and masked['forward_levels'][0]['bilinear_samples']==1,
              'first masked attempted sample must count and reject')
    repeated=independent_pixel_tracks(previous,g.np.roll(previous,32,axis=1),[[120.,120.]])['points'][0]
    r.require(repeated['accepted'] and r.norm([a-b for a,b in zip(repeated['forward_endpoint'],[152.,120.])])>31.99,
              'known repeated-texture false acceptance must remain disclosed')
    texture=lambda x,y:128+40*g.np.sin(.20*x+.10*y)+35*g.np.cos(.13*x-.24*y)+30*g.np.sin(.39*x+.31*y)
    c=math.cos(.1);s=math.sin(.1)
    rotated=render(texture(320+c*(xx-320)+s*(yy-240),240-s*(xx-320)+c*(yy-240)))
    fb=independent_pixel_tracks(render(texture(xx,yy)),rotated,[[190.,370.]])['points'][0]
    r.require(fb['rejection']=='ForwardBackward' and fb['forward_backward_error']>1.
              and all(x['converged'] and x['rejection'] is None for x in fb['forward_levels']+fb['backward_levels']),
              'specific measured forward-backward rejection control changed')
    rejected=[]
    for name,change in [('wrong_endpoint',lambda x:x['points'][0]['forward_endpoint'].__setitem__(0,0.)),
        ('wrong_backward_endpoint',lambda x:x['points'][0]['backward_endpoint'].__setitem__(0,0.)),
        ('wrong_forward_backward_error',lambda x:x['points'][0].__setitem__('forward_backward_error',10.)),
        ('conceal_sample_work',lambda x:x.__setitem__('bilinear_samples',0)),
        ('conceal_iterations',lambda x:x.__setitem__('iterations',0)),
        ('conceal_pyramid_pixels',lambda x:x.__setitem__('pyramid_output_pixels',0)),
        ('forge_level_eigen',lambda x:x['points'][0]['forward_levels'][0].__setitem__('min_eigenvalue',0.)),
        ('forge_level_RMS',lambda x:x['points'][0]['forward_levels'][0].__setitem__('photometric_rms',100.)),
        ('forge_level_work',lambda x:x['points'][0]['forward_levels'][0].__setitem__('bilinear_samples',0))]:
        altered=copy.deepcopy(tracks);change(altered)
        try:tracking_witness_check(altered,tracks)
        except ValueError:rejected.append(name)
        else:raise ValueError('corrupted known pixel witness accepted '+name)
    for name,actual,expected in [('false_border_acceptance',dict(border,accepted=True),border),
        ('false_masked_acceptance',dict(masked,accepted=True),masked),
        ('false_FB_acceptance',dict(fb,accepted=True,rejection=None),fb),
        ('rank_deficient_condition_forgery',dict(rank_one,forward_levels=[dict(rank_one['forward_levels'][0],condition_number=0.)]),rank_one)]:
        try:tracking_witness_check(actual,expected)
        except ValueError:rejected.append(name)
        else:raise ValueError('corrupted rejected pixel witness accepted '+name)
    for name,mask_value,coordinates in [('invalid_binary_mask',g.np.full((480,640),2,dtype=g.np.uint8),points),
        ('nonfinite_original_point',None,[[math.nan,1.]]),('too_many_original_points',None,[[120.,120.]]*401)]:
        try:independent_pixel_tracks(previous,current,coordinates,previous_mask=mask_value)
        except ValueError:rejected.append(name)
        else:raise ValueError('invalid tracker input accepted '+name)
    return dict(passed=['known_integer_translation_and_subpixel_start','known_backward_motion',
        'rank_zero_low_texture_sentinel','rank_one_eigen_diagnostic_and_independent_decision',
        'attempted_border_and_masked_sample_counted','specific_FB_rejection_after_six_converged_levels',
        'periodic_known_wrong_accepted_endpoint_preserved'],mutations_rejected=rejected,
        accepted_periodic_endpoint_error_pixels=32.,specific_forward_backward_error_pixels=fb['forward_backward_error'],
        actual_sensor_data_read=False,calibrated_confidence_claim=False)


def correspondence_selection_contract_checks():
    yy,xx=g.np.mgrid[:480,:640]
    image=g.np.floor(128+50*g.np.sin(2*g.np.pi*xx/16)+50*g.np.cos(2*g.np.pi*yy/16)+.5).astype(g.np.uint8)
    features=[dict(x=float(50+20*x),y=float(50+20*y)) for y in range(15) for x in range(20)]
    flow=independent_pixel_tracks(image,image,[[f['x'],f['y']] for f in features])
    r.require(all(p['accepted'] for p in flow['points']),'stationary300point selection positive control failed')
    endpoints=[dict(previous_feature_index=p['point_index'],current_pixel_xy=p['forward_endpoint']) for p in flow['points']]
    depth=g.np.full((480,640),7500,dtype=g.np.uint16)
    camera=dict(fx=517.306408,fy=516.469215,cx=318.643040,cy=255.313989,units_per_metre=5000.)
    depth_rows=[];pairs=[]
    for i,endpoint in enumerate(endpoints):
        previous=g.depth_point_status(features[i],depth,camera)[0]
        xy=endpoint['current_pixel_xy'];current=g.depth_point_status(dict(x=xy[0],y=xy[1]),depth,camera)[0]
        row=dict(previous_index=i,current_endpoint_index=i,accepted=True,correspondence_selected=i<256,
                 previous_xyz=previous,current_xyz=current)
        if i<256:row['correspondence_index']=i;pairs.append(dict(previous=previous,current=current))
        else:row['selection_rejection']='measured correspondence count exceeds unchanged256-point registration bound'
        depth_rows.append(row)
    report=dict(flow_attempted=True,flow_results=flow['points'],flow_work={key:flow[key] for key in ('bilinear_samples','iterations','pyramid_output_pixels')},
                tracked_endpoints=endpoints,depth_matches=depth_rows,correspondences=pairs)
    result,_,_=tracked_geometry(report,features,image,image,depth,depth,camera)
    r.require(len(result)==256 and len(report['depth_matches'])==300,'fixed selection drops overflow evidence')
    failed=[]
    for name,change in [('select_overflow_pair',lambda x:x['depth_matches'][256].__setitem__('correspondence_selected',True)),
        ('drop_unselected_tail_witness',lambda x:x['depth_matches'].pop()),
        ('omit_overflow_reason',lambda x:x['depth_matches'][256].pop('selection_rejection')),
        ('invent_overflow_correspondence_ID',lambda x:x['depth_matches'][256].__setitem__('correspondence_index',256)),
        ('omit_valid_selected_pair',lambda x:x['depth_matches'][0].__setitem__('correspondence_selected',False))]:
        altered=copy.deepcopy(report);change(altered)
        try:tracked_geometry(altered,features,image,image,depth,depth,camera)
        except (ValueError,KeyError):failed.append(name)
        else:raise ValueError('fixed selection mutation accepted '+name)
    # The first accepted flow endpoint need not have valid depth. Reproduce the
    # v1 negative-test selection failure without changing any tracking gate.
    mixed_depth=depth.copy();x=int(features[0]['x']);y=int(features[0]['y'])
    mixed_depth[y-1:y+2,x-1:x+2]=0
    mixed=copy.deepcopy(report)
    mixed['depth_matches'][0]=dict(previous_index=0,current_endpoint_index=0,
        accepted=False,correspondence_selected=False,
        previous_rejection=g.depth_point_status(features[0],mixed_depth,camera)[1],
        current_rejection=g.depth_point_status(dict(x=endpoints[0]['current_pixel_xy'][0],
            y=endpoints[0]['current_pixel_xy'][1]),mixed_depth,camera)[1])
    mixed['correspondences']=[]
    for index,item in enumerate(mixed['depth_matches'][1:],1):
        selected=len(mixed['correspondences'])<256
        item['correspondence_selected']=selected
        item.pop('correspondence_index',None);item.pop('selection_rejection',None)
        if selected:
            item['correspondence_index']=len(mixed['correspondences'])
            mixed['correspondences'].append(dict(previous=item['previous_xyz'],current=item['current_xyz']))
        else:item['selection_rejection']='measured correspondence count exceeds unchanged256-point registration bound'
    tracked_geometry(mixed,features,image,image,mixed_depth,mixed_depth,camera)
    wrapped=dict(frames=[mixed]);frame,index=selected_depth_witness(wrapped)
    r.require((frame,index)==(0,1),'selected witness must skip invalid first depth')
    noop=copy.deepcopy(mixed);noop['depth_matches'][0]['correspondence_selected']=False
    r.require(noop==mixed,'original v1 mixed-depth mutation was not a no-op')
    tracked_geometry(noop,features,image,image,mixed_depth,mixed_depth,camera)
    altered=copy.deepcopy(wrapped)
    altered['frames'][frame]['depth_matches'][index]['correspondence_selected']=False
    r.require(altered!=wrapped,'mixed-depth selected witness mutation must change baseline')
    try:tracked_geometry(altered['frames'][0],features,image,image,mixed_depth,mixed_depth,camera)
    except (ValueError,KeyError):failed.append('omit_selected_pair_after_invalid_first_depth')
    else:raise ValueError('mixed-depth selected pair mutation accepted')
    return dict(passed=['300measured_endpoints_replayed','first256valid_pairs_selected_in_original_order',
        'all44overflow_depth_witnesses_retained','mixed_invalid_first_depth_valid_later_positive',
        'original_v1_first_depth_mutation_reproduced_as_noop'],mutations_rejected=failed,actual_sensor_data_read=False)


@lru_cache(maxsize=180)
def _temporal_rigid_fit(frozen):
    return v._robust_svd.__wrapped__(frozen)


def temporal_rigid_fit(pairs):
    frozen = tuple(tuple(pair['previous'])+tuple(pair['current']) for pair in pairs)
    return _temporal_rigid_fit(frozen)


@lru_cache(maxsize=180)
def _temporal_refinement(frozen, calibration, coarse):
    return g._refine_independent.__wrapped__(frozen, calibration, coarse)


def temporal_refinement(observations, camera, coarse):
    frozen = tuple(tuple(obs['previous_xyz'])+tuple(obs['current_pixel_xy']) for obs in observations)
    calibration = tuple(camera[key] for key in ('fx','fy','cx','cy'))
    initial = tuple(coarse[0])+tuple(coarse[1])
    return _temporal_refinement(frozen, calibration, initial)


def cache_contract_checks():
    """Exact keys retain full solver results, including rejection/work/trace."""
    previous = [(x/10., y/10., 1.5) for y in range(3) for x in range(4)]
    pairs = [dict(previous=list(p), current=[p[0]-.01, p[1], p[2]]) for p in previous]
    frozen = tuple(tuple(p['previous'])+tuple(p['current']) for p in pairs)
    uncached = v._robust_svd.__wrapped__(frozen)
    r.require(uncached[1] is None and temporal_rigid_fit(pairs) == uncached,
              'cached rigid solver differs from uncached positive control')
    before = _temporal_rigid_fit.cache_info().hits
    r.require(temporal_rigid_fit(pairs) == uncached
              and _temporal_rigid_fit.cache_info().hits == before+1,
              'exact rigid-fit cache key failed')
    camera = dict(fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989)
    observations = [dict(previous_xyz=p['previous'], current_pixel_xy=[
        camera['fx']*p['current'][0]/p['current'][2]+camera['cx'],
        camera['fy']*p['current'][1]/p['current'][2]+camera['cy']]) for p in pairs]
    coarse = uncached[0]['estimate']
    key = (tuple(tuple(o['previous_xyz'])+tuple(o['current_pixel_xy']) for o in observations),
           tuple(camera[k] for k in ('fx','fy','cx','cy')), tuple(coarse[0])+tuple(coarse[1]))
    direct = g._refine_independent.__wrapped__(*key)
    r.require(direct[1] is None and temporal_refinement(observations,camera,coarse) == direct,
              'cached refinement differs from uncached positive control')
    before = _temporal_refinement.cache_info().hits
    r.require(temporal_refinement(observations,camera,coarse) == direct
              and _temporal_refinement.cache_info().hits == before+1,
              'exact refinement cache key failed')
    _temporal_rigid_fit.cache_clear()
    _temporal_refinement.cache_clear()
    return dict(passed=['exact_cached_vs_uncached_rigid_result_including_work',
        'exact_cached_vs_uncached_refinement_including_trace',
        'repeated_exact_keys_hit_local_caches'], maxsize=180,
        immutable_helper_globals_modified=False, actual_sensor_data_read=False)

SOURCES = dict(g.v.SOURCES)
for _field in ('visual_source_sha256', 'reprojection_source_sha256',
               'reprojection_acquisition_source_sha256'):
    SOURCES.pop(_field)
SOURCES.update(
    evaluator_source_sha256='integrations/rgbd/src/bin/rustdriving-rgbd-tracked.rs',
    tracked_source_sha256='integrations/rgbd/src/tracked.rs',
    independent_checker_sha256='scripts/check-recorded-tracked.py',
    pyramidal_tracker_source_sha256='integrations/rgbd/src/pyramidal_tracking.rs',
    temporal_checker_sha256='scripts/check-recorded-temporal.py',
    reprojection_checker_sha256='scripts/check-recorded-reprojection.py',
    qualification_source_sha256='scripts/qualify-rgbd-temporal.py',
    acquisition_source_sha256='scripts/fetch-temporal-dataset.py')


def timestamp_rows(content, columns, image_kind=None):
    """Never convert or interpret the seven ground-truth pose tokens."""
    r.require(len(content) <= 4194304, 'metadata file exceeds declared byte bound')
    rows = []
    for number, raw_line in enumerate(content.decode('utf8').splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        fields = line.split()
        r.require(len(fields) == columns, f'metadata row arity line {number}')
        try:
            stamp = float(fields[0])
        except ValueError:
            raise ValueError(f'invalid metadata timestamp line {number}') from None
        r.require(math.isfinite(stamp) and (not rows or stamp > rows[-1][0]),
                  f'nonfinite or nonmonotonic metadata timestamp line {number}')
        if image_kind is not None:
            name = fields[1]
            r.require(name.startswith(image_kind+'/') and name.endswith('.png')
                      and '\\' not in name and len(Path(name).parts) == 2
                      and '..' not in Path(name).parts, 'unsafe image index path')
            rows.append((stamp, name))
        else:
            rows.append((stamp,))
        r.require(len(rows) <= (20000 if image_kind is not None else 30000), 'metadata source row bound')
    r.require(len(rows) >= 2, 'insufficient metadata timestamps')
    return rows


def metadata_inputs(manifest, raw_path):
    files = {item['file']: item for item in manifest['files']}
    result = {}
    for name in ('depth.txt', 'rgb.txt', 'groundtruth.txt'):
        item = files[name]
        content = r.bounded(raw_path/name, 4194304)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'],
                  'changed full-source qualification metadata bytes')
        if 'git_blob_sha1' in item:
            blob = b'blob '+str(len(content)).encode()+b'\0'+content
            r.require(hashlib.sha1(blob).hexdigest() == item['git_blob_sha1'],
                      'changed qualification metadata Git blob')
        result[name] = content
    return result


def independently_qualify(manifest, metadata):
    depth = timestamp_rows(metadata['depth.txt'], 2, 'depth')
    rgb = timestamp_rows(metadata['rgb.txt'], 2, 'rgb')
    truth = timestamp_rows(metadata['groundtruth.txt'], 8)
    truth_times = [row[0] for row in truth]
    rgb_times = [row[0] for row in rgb]
    frames = manifest['frames']
    r.require(len(frames) == 180 and [f['source_index'] for f in frames] == list(range(100, 280)),
              'changed fixed consecutive source window')
    maximum_bracket, maximum_gap = 0., 0.
    for ordinal, frame in enumerate(frames):
        r.require(frame['split'] == ('initialization' if ordinal == 0 else ('viewed_prefix' if ordinal <= 35 else 'unviewed_extension')),
                  'changed preregistered source split')
        for kind, index, rows in [('depth', frame['source_index'], depth),
                                  ('rgb', frame['rgb_source_index'], rgb)]:
            r.require(type(index) is int and 0 <= index < len(rows), 'invalid source index')
            stamp, filename = rows[index]
            r.require(stamp == frame[kind+'_timestamp']
                      and frame[kind+'_file'] == kind+'-'+filename[len(kind)+1:],
                      'changed original source acquisition')
        stamp = frame['depth_timestamp']
        right = bisect_left(rgb_times, stamp)
        choices = [i for i in (right-1, right) if 0 <= i < len(rgb)]
        nearest = min(choices, key=lambda i: (abs(rgb_times[i]-stamp), i))
        r.require(frame['rgb_source_index'] == nearest, 'RGB is not nearest timestamp with earlier tie')
        gap = abs(frame['depth_timestamp']-frame['rgb_timestamp'])
        r.require(gap <= .02, 'qualification RGB/depth gap exceeds exact gate')
        v.same_pair_gap(frame['pair_gap_seconds'], gap)
        maximum_gap = max(maximum_gap, gap)
        right = bisect_left(truth_times, stamp)
        if right < len(truth_times) and truth_times[right] == stamp:
            bracket = 0.
        else:
            r.require(0 < right < len(truth_times), 'qualification requires GT bracket without extrapolation')
            bracket = truth_times[right]-truth_times[right-1]
            r.require(0 < bracket <= .02, 'qualification GT bracket exceeds exact gate')
        maximum_bracket = max(maximum_bracket, bracket)
    return dict(depth_rows=len(depth), rgb_rows=len(rgb), ground_truth_rows=len(truth),
                frame_count=180, reference_bracketed_frames=180,
                unique_rgb_frames=len({(f['rgb_source_index'], f['rgb_timestamp'], f['rgb_file']) for f in frames}),
                maximum_observed_ground_truth_bracket_us=math.floor(maximum_bracket*1e6+.5),
                maximum_pair_gap_us=math.floor(maximum_gap*1e6+.5),
                summary_time_unit='rounded microseconds from source timestamps; qualification gates use unrounded seconds')


def pinned_acquisition_check(manifest, source_root):
    """The hash-bound fetcher supplies source pins, never qualification results."""
    path = source_root/'scripts/fetch-temporal-dataset.py'
    spec = importlib.util.spec_from_file_location('temporal_acquisition_pins', path)
    module = importlib.util.module_from_spec(spec)
    before = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = before
    module.verify_manifest(manifest)


def exact_json(actual, expected, message):
    # Treat JSON booleans separately from numbers (Python True equals 1).
    if isinstance(expected, bool):
        r.require(type(actual) is bool and actual == expected, message)
    elif isinstance(expected, dict):
        r.require(isinstance(actual, dict) and set(actual) == set(expected), message)
        for key in expected:
            exact_json(actual[key], expected[key], message+' '+key)
    elif isinstance(expected, list):
        r.require(isinstance(actual, list) and len(actual) == len(expected), message)
        for a, e in zip(actual, expected):
            exact_json(a, e, message)
    elif isinstance(expected, (int, float)):
        r.require(type(actual) in (int, float) and math.isfinite(actual) and actual == expected, message)
    else:
        r.require(actual == expected, message)


PREPROCESSING = dict(width=640, height=480,
    luma='(77*R+150*G+29*B)>>8; RGB/RGBA8; RGBA must be fully opaque',
    pixel_coordinates='nearest integer feature coordinate, ties round away from zero',
    min_depth_m=.3, max_depth_m=5., depth_units_per_metre=5000., patch_radius_pixels=1,
    patch_validity='all 9 depths valid and range-bounded', patch_max_spread_m=.05,
    point='centre depth at feature pixel; optical x-right y-down z-forward', maximum_pair_gap_s=.02)
FEATURE_POLICY = dict(max_features=400, max_matches=256,
    detector='fixed original FAST-9 threshold20 radius3, deterministic NMS,32pixel tile max2',
    descriptor='intensity-centroid oriented deterministic256bit BRIEF on5x5binomial blur',
    matching='detected FAST/BRIEF features reseeded only on accepted references; pair correspondences from fixed pyramidal tracking, without descriptor matching')
TRACKING_POLICY = dict(max_unobserved_s=.20,
    reference='last accepted measured grayscale image, original detected FAST features and depth image; initial root identity after measured geometry validation',
    pose='reference-root pose composed with measured refined previous_from_current pixel-reprojection pose',
    rejection='no root output or accepted clock/reference update; repeated/stale RGB timestamp versus last observed image rejects, including previously rejected fits; observed image clock advances without permission renewal; accepted-pose expiry checked before RGB freshness; evaluation never resets')
GT_POLICY = dict(method='linear translation and shortest-arc quaternion SLERP', max_bracket_s=.02,
                 extrapolation=False, max_source_rows=30000, max_source_bytes=4194304)
REFINEMENT_POLICY = dict(g.REFINEMENT_POLICY)
REFINEMENT_POLICY['observations'] = 'only original robust3D inlier correspondence indices; previous measured depth point and measured current LK subpixel endpoint; fixed support throughout refinement'
FLOW_CONFIG = dict(max_points=400,pyramid_levels=3,patch_size=9,max_iterations_per_level=10,
    minimum_average_eigenvalue=4.,maximum_condition_number=100.,maximum_step_level_pixels=2.,
    convergence_level_pixels=.01,maximum_photometric_rms_gray=15.,
    maximum_forward_backward_error_pixels=1.,max_bilinear_samples=3110400,max_iterations=24000)
FLOW_POLICY = dict(
    reference='last accepted measured grayscale image, original detected FAST features, depth image and root pose; current FAST reseeded only when accepted, no long-lived feature identity claim',
    attempts='track all original previous features in extraction order, maximum400; independent forward and backward zero-coarse initialization, fixed3level9x9 measured grayscale tracking',
    endpoints='accepted flow endpoints in original previous-feature order, distinct buffer with subpixel pixel coordinates and no measured BRIEF descriptors',
    depth='unchanged all-nine-pixel validity/range/spread gate; sample depth and backproject rounded integer pixel; refinement observes measured LK subpixel endpoint',
    correspondence_selection='first maximum256 valid measured depth pairs in original previous feature order after flow/depth gates; retain every later endpoint/depth witness with correspondence_selected:false',
    masks='native RGB grayscale, no additional pixel masks or simulator truth',
    failure='failed tracking or registration does not renew accepted reference or pose clock; original age expiry and Lost latch unchanged')

RESOURCE_POLICY = dict(max_raw_bytes=134217728,max_report_bytes=134217728,max_frames=180,
                       max_manifest_bytes=524288,max_freeze_bytes=524288,max_qualification_bytes=524288)

TEMPORAL_POLICY = dict(
    viewed_prefix=dict(first_depth_index=100,last_depth_index=135,frames=36),
    extension_window=dict(first_depth_index=136,last_depth_index=279,frames=144),
    continuous_state=dict(initialization_frame_index=100,maximum_initializations=1,
                          chunk_resets=False,lost_recovery=False),
    evidence_scope='same previously evaluated180-frame room recording; pyramidal optical-flow tracking calibration variant; all images and reference labels viewed; no new-recording or fresh-generalization claim')

IMMUTABLE_ALGORITHMS = {
    'feature_source_sha256': '5b5b103d4e798929f753a850387609405cad1a303d49ef8e81433f064e901106',
    'visual_pose_source_sha256': '718192d69b02de0a668c54e8469812b7c5f1903ab74de660e1b221e2ec874d24',
    'reprojection_pose_source_sha256': '2895e1c3eafefcfacdd156fd51706598521f0344f427f24edf420d159e5da486',
    'visual_checker_sha256': g.OLD_VISUAL_SHA,
    'reprojection_checker_sha256': OLD_REPROJECTION_SHA,
    'temporal_checker_sha256': OLD_TEMPORAL_SHA,
    'pyramidal_tracker_source_sha256': '2118ecdd95e7788dc1937a0937209dab4f4f9f7d9b5fc0c28aaa3f2134b3ce8e',
}


def verify_sources(freeze, manifest_raw, qualification_raw, source_root):
    r.require(type(freeze['schema_version']) is int and freeze['schema_version'] == 1
              and type(freeze['protocol_version']) is int and freeze['protocol_version'] == 1
              and freeze['algorithm'] == ALGORITHM, 'unsupported temporal protocol')
    exact_json(freeze['resources'],RESOURCE_POLICY,'changed temporal resource bounds')
    for field, expected in TEMPORAL_POLICY.items():
        exact_json(freeze[field],expected,'changed temporal continuity/evidence policy '+field)
    for field, path in SOURCES.items():
        r.require(freeze[field] == r.digest(r.bounded(source_root/path)), 'changed frozen source '+path)
    for field, expected in IMMUTABLE_ALGORITHMS.items():
        r.require(freeze[field] == expected, 'changed original measured-image mathematics '+field)
    r.require(freeze['manifest_sha256'] == r.digest(manifest_raw), 'changed frozen manifest')
    r.require(freeze['qualification_sha256'] == r.digest(qualification_raw), 'changed qualification proof bytes')
    qualification = json.loads(qualification_raw)
    exact_json(freeze['qualification'], qualification, 'changed frozen qualification object')
    manifest = json.loads(manifest_raw)
    pinned_acquisition_check(manifest, source_root)
    r.require(manifest['dataset'] == freeze['dataset'] == DATASET
              and manifest['repository'] == REPOSITORY and manifest['revision'] == REVISION
              and manifest['source_prefix'] == PREFIX, 'changed pinned source identity')
    r.require(freeze['regression_requested'] is True and freeze['kind'] == 'calibration_regression',
              'all180 frames viewed; unsupported fresh or extension role')
    for field in ('depth_calibration', 'calibration_source'):
        exact_json(freeze[field], manifest[field], 'changed frozen acquisition '+field)
    r.require(len(freeze['frames']) == len(manifest['frames']), 'changed frozen frame count')
    for actual, expected in zip(freeze['frames'], manifest['frames']):
        v.same_acquisition(actual, expected)
    for field, expected in dict(preprocessing=PREPROCESSING, feature_policy=FEATURE_POLICY,
        tracking_policy=TRACKING_POLICY, registration_config=v.CONFIG, refinement_config=g.REFINEMENT_CONFIG,
        refinement_policy=REFINEMENT_POLICY, evaluation_label_policy=g.LABEL_POLICY,
        optical_flow_config=FLOW_CONFIG,optical_flow_policy=FLOW_POLICY,
        ground_truth_interpolation=GT_POLICY, accuracy_gates=dict(translation_m=.1, rotation_rad=.1),
        json_metadata_audit=v.JSON_METADATA_AUDIT).items():
        exact_json(freeze[field], expected, 'changed fixed policy '+field)
    for field in ('depth_calibration', 'calibration_source', 'preprocessing', 'feature_policy',
                  'tracking_policy', 'registration_config', 'refinement_config', 'refinement_policy',
                  'evaluation_label_policy','optical_flow_config','optical_flow_policy'):
        expected = (g.refinement_config_digest(freeze[field]) if field == 'refinement_config'
                    else r.canonical(freeze[field]))
        r.require(freeze[field+'_sha256'] == expected, 'changed fixed policy hash '+field)
    calibration = manifest['depth_calibration']
    for key, value in dict(fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989,
                          width=640, height=480, units_per_metre=5000., invalid_depth=0).items():
        r.require(type(calibration[key]) in (int, float) and calibration[key] == value,
                  'changed pinned source camera profile '+key)
    return manifest, qualification


def qualification_check(freeze, manifest_raw, qualification, metadata):
    manifest = json.loads(manifest_raw)
    measured = independently_qualify(manifest, metadata)
    files = {item['file']: item for item in manifest['files']}
    expected = dict(schema_version=1, dataset=DATASET, manifest_sha256=r.digest(manifest_raw),
        source_commit=REVISION, window=dict(first_depth_index=100,last_depth_index=279,frames=180,updates=179),
        metadata_files={name:{key:files[name][key] for key in ('bytes','sha256','git_blob_sha1')}
                        for name in ('depth.txt','rgb.txt','groundtruth.txt')},
        timestamps=dict(depth_index_rows=measured['depth_rows'], rgb_index_rows=measured['rgb_rows'],
            ground_truth_rows=measured['ground_truth_rows'], strict_depth_order=True, strict_rgb_order=True,
            strict_ground_truth_order=True, all_frame_brackets_valid=True, max_ground_truth_bracket_s=.02,
            max_pair_gap_s=.02, maximum_observed_ground_truth_bracket_us=measured['maximum_observed_ground_truth_bracket_us'],
            maximum_pair_gap_us=measured['maximum_pair_gap_us'], unique_rgb_acquisitions=measured['unique_rgb_frames'],
            duplicate_rgb_associations=180-measured['unique_rgb_frames'], summary_time_unit=measured['summary_time_unit']),
        passed=True, pixels_read=False, image_headers_read=False, ground_truth_pose_values_parsed=False,
        features_or_fits_run=False, helper_sha256=freeze['qualification_source_sha256'],
        acquisition_helper_sha256=freeze['acquisition_source_sha256'])
    exact_json(qualification, expected, 'invented qualification proof')
    return measured


def continuous_audit(report, manifest, all_features, depths, gt, label_failure=None, grays=None):
    r.require(grays is not None and len(grays)==180, 'missing full measured grayscale sequence')
    g.label_failure_check(report, label_failure)
    r.require(report['schema_version'] == 1 and report['ground_truth_operational'] is False
              and report['raw_redistributed'] is False
              and report['algorithm'] == ALGORITHM,
              'schema/truth/redistribution/algorithm')
    for name in ('dataset', 'repository', 'revision'):
        r.require(report[name] == manifest[name], 'changed source identity')
    frames, rows = manifest['frames'], report['frames']
    r.require(len(rows) == len(frames) == 180, 'omitted observed/rejected acquisition')
    r.require(report['freeze']['registration_config'] == v.CONFIG, 'weakened robust registration gates')
    r.require(report['freeze']['refinement_config'] == g.REFINEMENT_CONFIG, 'weakened pixel refinement gates')
    r.require(report['freeze']['tracking_policy']['max_unobserved_s'] == .20,
              'changed accepted-pose age')
    calibration = manifest['depth_calibration']
    origin = k.timed_pose(gt, frames[0]['depth_timestamp'])
    reference, root_reference, last_accepted, last_observed_rgb, lost = None, v.IDENTITY, None, None, False
    counters = dict(initialized_frames=0, accepted_updates=0, rejected_updates=0,
                    accurate_root_updates=0, reference_valid_updates=0)
    records = []
    for i, (row, frame, observed) in enumerate(zip(rows, frames, all_features)):
        for field in ('source_index', 'depth_file', 'depth_timestamp', 'rgb_file', 'rgb_timestamp',
                      'rgb_source_index', 'pair_gap_seconds', 'split'):
            if field == 'pair_gap_seconds':
                v.same_pair_gap(row[field], frame[field])
            else:
                r.require(row[field] == frame[field], 'changed paired observation/clock')
        r.require(type(row['accepted']) is bool and type(row['initialized']) is bool,
                  'invalid acceptance/initialization')
        r.require(isinstance(row['cpu_wall_seconds'], (int, float))
                  and math.isfinite(row['cpu_wall_seconds']) and row['cpu_wall_seconds'] >= 0,
                  'invalid CPU timing')
        r.require(row['last_accepted_stamp_before'] == last_accepted
                  and row['reference_frame_index_before'] == (frames[reference]['source_index']
                                                              if reference is not None else None)
                  and row['lost_before'] == lost
                  and row['last_observed_rgb_stamp_before'] == last_observed_rgb,
                  'invented before-acquisition pose/reference/clock state')
        stamp, rgb_stamp = frame['depth_timestamp'], frame['rgb_timestamp']
        blocked = v.clock_block_reason(lost, last_accepted, last_observed_rgb, stamp, rgb_stamp)
        computation_allowed = blocked is None
        r.require(row['features_computed'] is computation_allowed
                  and row['feature_count'] == (len(observed) if computation_allowed else 0),
                  'computed features before clock/freshness guard')
        v.feature_check(row['features'], observed if computation_allowed else [])
        truth = k.truth_relative(origin, k.timed_pose(gt, stamp))
        if i:
            counters['reference_valid_updates'] += int(truth is not None)
        rejected = None
        predicted = None
        refinement_attempted = False
        evidence = dict(source_index=frame['source_index'], feature_count=row['feature_count'],
                        features_computed=computation_allowed)
        if blocked:
            no_flow_check(row)
            rejected = blocked
            lost = lost or 'localization lost' in blocked
        else:
            last_observed_rgb = rgb_stamp
            if reference is None:
                no_flow_check(row)
                r.require(row['depth_matches'] == [] and row['correspondences'] == []
                          and not row.get('refinement_observations')
                          and not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement', 'relative_estimate')),
                          'invented initialization fit')
                good, ratio, point_count = v.initialized_geometry(row, observed, depths[i], calibration)
                evidence.update(initialization_geometry_ratio=ratio,
                                initialization_depth_points=point_count)
                if good:
                    predicted = v.IDENTITY
                    r.require(row['initialized'] is True, 'concealed initialization')
                    counters['initialized_frames'] += 1
                else:
                    lost = True
                    rejected = row.get('rejection')
                    r.require(bool(rejected), 'concealed initialization failure')
            else:
                r.require(row['reference_frame_index'] == frames[reference]['source_index']
                          and row['reference_stamp'] == frames[reference]['depth_timestamp'],
                          'invented reference acquisition')
                k.same_pose(r.pose(row['root_from_reference']), root_reference, 'invented reference root')
                pairs,flow,endpoints = tracked_geometry(row,all_features[reference],grays[reference],grays[i],
                    depths[reference],depths[i],calibration)
                independently, rejected = temporal_rigid_fit(pairs)
                evidence.update(flow_attempted=True,flow_work={key:flow[key] for key in ('bilinear_samples','iterations','pyramid_output_pixels')},
                    tracked_points=len(flow['points']),accepted_endpoints=len(endpoints),valid_depth_matches=len(pairs))
                if independently is not None:
                    r.require(row['initialized'] is False, 'silently changed root origin')
                    coarse = r.pose(row['coarse_relative_estimate'])
                    v.fit_check(row['fit'], coarse, pairs, independently)
                    observations = tracked_refinement_observations(row,independently,endpoints,pairs)
                    refinement_attempted = True
                    refined, rejected = temporal_refinement(observations, calibration, coarse)
                    if refined is not None:
                        g.refinement_check(row['refinement'], r.pose(row['relative_estimate']), observations, calibration, coarse, refined)
                        predicted = k.compose(root_reference, r.pose(row['relative_estimate']))
                        evidence['refinement'] = refined
                    else:
                        r.require('refinement' not in row and 'relative_estimate' not in row, 'concealed refinement failure')
                        r.require(row['refinement_rejection'] == rejected, 'concealed backend refinement rejection')
                    evidence['fit'] = {field: value for field, value in independently.items() if field != 'estimate'}
                else:
                    r.require(not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement', 'relative_estimate'))
                              and not row.get('refinement_observations'), 'invented refinement without coarse consensus')
        accepted = predicted is not None
        r.require(row['refinement_attempted'] is refinement_attempted,
                  'invented or concealed refinement invocation')
        if not refinement_attempted or accepted:
            r.require('refinement_rejection' not in row, 'invented refinement rejection')
        r.require(row['accepted'] is accepted, 'changed independently reconstructed acceptance')
        evidence['accepted'] = accepted
        score = row['root_accuracy']
        if label_failure is not None:
            r.require(score.get('reference_rejection') == label_failure['reason'],
                      'concealed strict source-label failure')
        r.require(score['valid'] == accepted, 'invented root validity')
        if accepted:
            k.same_pose(r.pose(row['root_estimate']), predicted, 'invented root pose/composition')
            evidence['root'] = k.scored_pose(score, predicted, truth, 'visual root')
            reference, root_reference, last_accepted = i, predicted, stamp
            if i:
                counters['accepted_updates'] += 1
                counters['accurate_root_updates'] += int(score['within_accuracy_gates'])
            r.require('rejection' not in row, 'accepted pose retains contradictory rejection')
        else:
            r.require(row.get('rejection') == rejected and row['initialized'] is False
                      and not any(field in row for field in ('root_estimate', 'relative_estimate', 'refinement')),
                      'concealed failure/invented rejected pose')
            k.scored_pose(score, None, truth, 'rejected visual root')
            if i:
                counters['rejected_updates'] += 1
            evidence['rejection'] = rejected
        if rejected and ('lost' in rejected or 'duplicate or stale' in rejected):
            no_flow_check(row)
            r.require(row['depth_matches'] == [] and row['correspondences'] == []
                      and not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement'))
                      and not row.get('refinement_observations'),
                      'performed visual fit after expired/duplicate acquisition')
        r.require(row['last_accepted_stamp_after'] == last_accepted
                  and row['reference_frame_index_after'] == (frames[reference]['source_index']
                                                             if reference is not None else None)
                  and row['lost_after'] == lost
                  and row['last_observed_rgb_stamp_after'] == last_observed_rgb,
                  'invented after-acquisition pose/reference/clock state')
        records.append(evidence)
    updates = len(frames)-1
    passed = (counters['initialized_frames'] == 1 and counters['accepted_updates'] == updates
              and counters['accurate_root_updates'] == updates
              and counters['reference_valid_updates'] == updates)
    expected = dict(frames=len(frames), updates=updates, **counters, lost=lost,
                    all_updates_passed=passed)
    r.require(report['summary'] == expected, 'summary hides rejected/unscorable acquisitions')
    return records



def audit(report, manifest, features, depths, gt, label_failure, grays=None):
    r.require(report['freeze']['regression_requested'] is True and report['freeze']['kind']=='calibration_regression',
              'all180 frames viewed; unsupported fresh tracked role')
    exact_json(report['freeze']['optical_flow_config'],FLOW_CONFIG,'changed measured tracker config')
    exact_json(report['freeze']['optical_flow_policy'],FLOW_POLICY,'changed measured tracker policy')
    r.require(report['qualification_verified'] is True, 'qualification gate not reported')
    exact_json(report['qualification'], report['freeze']['qualification'], 'report changed qualification proof')
    r.require(report['qualification_sha256'] == report['freeze']['qualification_sha256'],
              'report changed qualification proof hash')
    for field, expected in TEMPORAL_POLICY.items():
        exact_json(report['freeze'][field], expected, 'changed continuous temporal policy '+field)
    return continuous_audit(report, manifest, features, depths, gt, label_failure,grays)


def mutation_checks(report, manifest, all_features, depths, gt, label_failure=None, grays=None):
    audit(report,manifest,all_features,depths,gt,label_failure,grays)
    tests = [
        ('omit_acquisition', lambda x: x['frames'].pop()),
        ('invent_summary', lambda x: x['summary'].__setitem__('accurate_root_updates', 999)),
        ('operational_truth', lambda x: x.__setitem__('ground_truth_operational', True)),
        ('source_revision', lambda x: x.__setitem__('revision', 'unverified')),
        ('RGB_clock', lambda x: x['frames'][1].__setitem__('rgb_timestamp', 0.)),
        ('depth_clock', lambda x: x['frames'][1].__setitem__('depth_timestamp', 0.)),
        ('invent_derived_pair_gap', lambda x: x['frames'][1].__setitem__(
            'pair_gap_seconds', manifest['frames'][1]['pair_gap_seconds']+1e-12)),
        ('accepted_clock', lambda x: x['frames'][0].__setitem__('last_accepted_stamp_after', 0.)),
        ('observed_RGB_clock', lambda x: x['frames'][0].__setitem__('last_observed_rgb_stamp_after', 0.)),
        ('reference_clock', lambda x: x['frames'][0].__setitem__('reference_frame_index_after', 999)),
        ('weaken_consensus', lambda x: x['freeze']['registration_config'].__setitem__('min_inliers', 1)),
        ('weaken_ambiguity', lambda x: x['freeze']['registration_config'].__setitem__('ambiguity_support_ratio', 1.1)),
    ]
    if label_failure is not None:
        tests.extend([
            ('conceal_invalid_label_source', lambda x: x.pop('evaluation_label_failure')),
            ('invent_invalid_label_reason', lambda x: x['evaluation_label_failure'].__setitem__('reason', 'invented')),
            ('invent_invalid_label_hash', lambda x: x['evaluation_label_failure'].__setitem__('source_sha256', '0'*64)),
            ('invent_invalid_label_file', lambda x: x['evaluation_label_failure'].__setitem__('file', 'rgb.txt')),
            ('claim_mocap_for_invalid_source', lambda x: x['frames'][0]['root_accuracy'].__setitem__('reference_valid', True)),
            ('claim_accuracy_for_invalid_source', lambda x: x['frames'][0]['root_accuracy'].__setitem__('within_accuracy_gates', True)),
            ('invent_invalid_source_error', lambda x: x['frames'][0]['root_accuracy'].__setitem__('translation_error_m', 0.)),
            ('conceal_row_label_failure', lambda x: x['frames'][0]['root_accuracy'].__setitem__('reference_rejection', 'invented')),
        ])
    else:
        tests.append(('invent_label_failure_for_valid_source', lambda x: x.__setitem__(
            'evaluation_label_failure', dict(file='groundtruth.txt', source_sha256='0'*64, reason='invented'))))
    feature = next((i for i, row in enumerate(report['frames']) if row['features']), None)
    if feature is not None:
        tests.extend([
            ('wrong_feature_pixel', lambda x: x['frames'][feature]['features'][0].__setitem__('x', 0.)),
            ('wrong_FAST_score', lambda x: x['frames'][feature]['features'][0].__setitem__('score', 0)),
            ('wrong_BRIEF', lambda x: x['frames'][feature]['features'][0]['descriptor'].__setitem__(0, x['frames'][feature]['features'][0]['descriptor'][0] ^ 1)),
            ('wrong_orientation', lambda x: x['frames'][feature]['features'][0].__setitem__('orientation', 100.)),
        ])
    accepted = next((i for i, row in enumerate(report['frames']) if i and row['accepted']), None)
    if accepted is not None:
        tests.extend([
            ('invent_depth_point', lambda x: x['frames'][accepted]['correspondences'][0]['current'].__setitem__(0, 999.)),
            ('invent_relative_pose', lambda x: x['frames'][accepted]['relative_estimate']['translation_m'].__setitem__(0, 100.)),
            ('invent_root_pose', lambda x: x['frames'][accepted]['root_estimate']['translation_m'].__setitem__(0, 100.)),
            ('invent_reference_root', lambda x: x['frames'][accepted]['root_from_reference']['translation_m'].__setitem__(0, 100.)),
            ('invent_inlier_membership', lambda x: x['frames'][accepted]['fit']['inlier_indices'].pop()),
            ('invent_inlier_ratio', lambda x: x['frames'][accepted]['fit'].__setitem__('inlier_ratio', 0.)),
            ('invent_RMS', lambda x: x['frames'][accepted]['fit'].__setitem__('rms_m', 100.)),
            ('invent_hypothesis_work', lambda x: x['frames'][accepted]['fit'].__setitem__('point_checks', 0)),
            ('invent_consensus_refits', lambda x: x['frames'][accepted]['fit'].__setitem__('refits', 0)),
            ('invent_candidates', lambda x: x['frames'][accepted]['fit'].__setitem__('candidate_models', 999)),
            ('invent_competing_models', lambda x: x['frames'][accepted]['fit'].__setitem__('competing_models', 1)),
            ('invent_accuracy', lambda x: x['frames'][accepted]['root_accuracy'].__setitem__('translation_error_m', 999.)),
        ])
    rejected = next((i for i, row in enumerate(report['frames']) if not row['accepted']), None)
    if rejected is not None:
        tests.append(('invent_rejected_pose', lambda x: x['frames'][rejected].__setitem__(
            'root_estimate', dict(translation_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]))))
    coarse_failed = next((i for i, row in enumerate(report['frames'])
                         if i and row['features_computed'] and not row['accepted'] and 'fit' not in row), None)
    if coarse_failed is not None:
        tests.extend([
            ('forge_failed_coarse_witness', lambda x: x['frames'][coarse_failed].__setitem__('fit', {})),
            ('forge_failed_coarse_pose', lambda x: x['frames'][coarse_failed].__setitem__('coarse_relative_estimate',
                dict(translation_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]))),
            ('forge_failed_refinement_observation', lambda x: x['frames'][coarse_failed].__setitem__('refinement_observations', [{}])),
        ])
    blocked = next((i for i, row in enumerate(report['frames']) if not row['features_computed']), None)
    if blocked is not None:
        tests.extend([
            ('forge_blocked_coarse_witness', lambda x: x['frames'][blocked].__setitem__('fit', {})),
            ('compute_after_duplicate_guard', lambda x: x['frames'][blocked].__setitem__('features_computed', True)),
            ('renew_duplicate_accepted_clock', lambda x: x['frames'][blocked].__setitem__(
                'last_accepted_stamp_after', x['frames'][blocked]['depth_timestamp'])),
        ])
    refined = next((i for i, row in enumerate(report['frames']) if 'refinement' in row), None)
    if refined is not None:
        tests.extend([
            ('conceal_refinement_invocation', lambda x: x['frames'][refined].__setitem__('refinement_attempted', False)),
            ('invent_refinement_rejection', lambda x: x['frames'][refined].__setitem__('refinement_rejection', 'invented')),
            ('invent_coarse_pose', lambda x: x['frames'][refined]['coarse_relative_estimate']['translation_m'].__setitem__(0, 100.)),
            ('drop_refinement_support', lambda x: x['frames'][refined]['refinement_observations'].pop()),
            ('wrong_refinement_inlier_ID', lambda x: x['frames'][refined]['refinement_observations'][0].__setitem__('correspondence_index', 999)),
            ('wrong_refinement_pixel', lambda x: x['frames'][refined]['refinement_observations'][0]['current_pixel_xy'].__setitem__(0, 0.)),
            ('wrong_refinement_depth_point', lambda x: x['frames'][refined]['refinement_observations'][0]['previous_xyz'].__setitem__(0, 999.)),
            ('invent_pixel_initial_cost', lambda x: x['frames'][refined]['refinement'].__setitem__('initial_huber_cost', 9999.)),
            ('invent_pixel_final_cost', lambda x: x['frames'][refined]['refinement'].__setitem__('final_huber_cost', 9999.)),
            ('invent_pixel_RMS', lambda x: x['frames'][refined]['refinement'].__setitem__('final_rms_px', 9999.)),
            ('invent_pixel_work', lambda x: x['frames'][refined]['refinement'].__setitem__('point_checks', 0)),
            ('invent_pixel_support', lambda x: x['frames'][refined]['refinement'].__setitem__('valid_support', 1)),
            ('invent_pixel_convergence', lambda x: x['frames'][refined]['refinement'].__setitem__('converged', not x['frames'][refined]['refinement']['converged'])),
            ('weaken_pixel_Huber', lambda x: x['freeze']['refinement_config'].__setitem__('huber_delta_px', 1000.)),
            ('weaken_pixel_workcap', lambda x: x['freeze']['refinement_config'].__setitem__('max_point_checks', 1000000)),
        ])
        if report['frames'][refined]['refinement']['trace']:
            tests.extend([
                ('nonmonotonic_pixel_step', lambda x: x['frames'][refined]['refinement']['trace'][0].__setitem__('huber_cost_after', 99999.)),
                ('invent_pixel_increment', lambda x: x['frames'][refined]['refinement']['trace'][0]['increment'].__setitem__(0, 10.)),
                ('invent_pixel_line_search', lambda x: x['frames'][refined]['refinement']['trace'][0].__setitem__('line_search_trials', 999)),
            ])
    failures = []
    for name, change in tests:
        altered = copy.deepcopy(report)
        change(altered)
        try:
            audit(altered, manifest, all_features, depths, gt, label_failure,grays)
        except (ValueError, KeyError, IndexError):
            failures.append(name)
        else:
            raise ValueError('corrupted visual report accepted: '+name)
    return failures


def selected_depth_witness(report):
    for frame,row in enumerate(report['frames']):
        for index,match in enumerate(row['depth_matches']):
            if match['correspondence_selected'] is True:return frame,index
    raise ValueError('no selected depth correspondence for targeted mutation')


def tracked_report_mutations(report,manifest,features,depths,gt,label_failure,grays):
    audit(report,manifest,features,depths,gt,label_failure,grays)
    frame=next(i for i,row in enumerate(report['frames']) if any(p['accepted'] for p in row['flow_results']))
    point=next(i for i,p in enumerate(report['frames'][frame]['flow_results']) if p['accepted'])
    selected_frame,selected_index=selected_depth_witness(report)
    tests=[
        ('conceal_flow_invocation',lambda x:x['frames'][frame].__setitem__('flow_attempted',False)),
        ('drop_original_flow_attempt',lambda x:x['frames'][frame]['flow_results'].pop()),
        ('invent_original_flow_ID',lambda x:x['frames'][frame]['flow_results'][point].__setitem__('point_index',999)),
        ('invent_flow_start',lambda x:x['frames'][frame]['flow_results'][point]['start'].__setitem__(0,0.)),
        ('invent_flow_endpoint',lambda x:x['frames'][frame]['flow_results'][point]['forward_endpoint'].__setitem__(0,0.)),
        ('invent_backward_endpoint',lambda x:x['frames'][frame]['flow_results'][point]['backward_endpoint'].__setitem__(0,0.)),
        ('invent_forward_backward_metric',lambda x:x['frames'][frame]['flow_results'][point].__setitem__('forward_backward_error',10.)),
        ('invent_flow_acceptance',lambda x:x['frames'][frame]['flow_results'][point].__setitem__('accepted',False)),
        ('invent_flow_sample_work',lambda x:x['frames'][frame]['flow_work'].__setitem__('bilinear_samples',0)),
        ('invent_flow_iteration_work',lambda x:x['frames'][frame]['flow_work'].__setitem__('iterations',0)),
        ('invent_pyramid_work',lambda x:x['frames'][frame]['flow_work'].__setitem__('pyramid_output_pixels',0)),
        ('invent_level_sample_work',lambda x:x['frames'][frame]['flow_results'][point]['forward_levels'][0].__setitem__('bilinear_samples',0)),
        ('invent_level_eigenvalue',lambda x:x['frames'][frame]['flow_results'][point]['forward_levels'][0].__setitem__('min_eigenvalue',0.)),
        ('invent_level_condition',lambda x:x['frames'][frame]['flow_results'][point]['forward_levels'][0].__setitem__('condition_number',0.)),
        ('invent_level_photo_RMS',lambda x:x['frames'][frame]['flow_results'][point]['forward_levels'][0].__setitem__('photometric_rms',100.)),
        ('invent_level_convergence',lambda x:x['frames'][frame]['flow_results'][point]['forward_levels'][0].__setitem__('converged',False)),
        ('invent_initialization_flow',lambda x:x['frames'][0].__setitem__('flow_attempted',True)),
        ('invent_BRIEF_on_endpoint',lambda x:x['frames'][frame]['tracked_endpoints'][0].__setitem__('descriptor',[0]*4)),
        ('drop_accepted_endpoint',lambda x:x['frames'][frame]['tracked_endpoints'].pop()),
        ('invent_tracked_subpixel',lambda x:x['frames'][frame]['tracked_endpoints'][0]['current_pixel_xy'].__setitem__(0,0.)),
        ('invent_endpoint_buffer_ID',lambda x:x['frames'][frame]['depth_matches'][0].__setitem__('current_endpoint_index',999)),
        ('drop_selected_correspondence',lambda x:x['frames'][selected_frame]['depth_matches'][selected_index].__setitem__('correspondence_selected',False)),
        ('weaken_forward_backward_gate',lambda x:x['freeze']['optical_flow_config'].__setitem__('maximum_forward_backward_error_pixels',100.)),
        ('replace_measured_flow_with_truth',lambda x:x['freeze']['optical_flow_policy'].__setitem__('masks','simulator truth')),
    ]
    failed=[]
    for name,change in tests:
        changed=copy.deepcopy(report);change(changed)
        r.require(changed!=report,'tracked mutation did not change baseline '+name)
        try:audit(changed,manifest,features,depths,gt,label_failure,grays)
        except (ValueError,KeyError,IndexError):failed.append(name)
        else:raise ValueError('tracked report corruption accepted '+name)
    return failed


def write_result(output, result):
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')


def qualification_mutations(freeze, manifest_raw, proof_raw, metadata, source_root):
    rejected = []
    for name,change in [('hide_viewed_regression',lambda x:x.__setitem__('regression_requested',False)),
        ('claim_unviewed_extension',lambda x:x.__setitem__('kind','preregistered_temporal_extension'))]:
        changed=copy.deepcopy(freeze);change(changed)
        try:verify_sources(changed,manifest_raw,proof_raw,source_root)
        except (ValueError,KeyError):rejected.append(name)
        else:raise ValueError('viewed tracking misclassification accepted '+name)
    for field in SOURCES:
        changed = copy.deepcopy(freeze); changed[field] = '0'*64
        try:
            verify_sources(changed, manifest_raw, proof_raw, source_root)
        except (ValueError, KeyError):
            rejected.append('source_'+field)
        else:
            raise ValueError('changed source accepted '+field)
    policy_changes = [('refinement_config','max_iterations',100),
        ('registration_config','min_inliers',1), ('tracking_policy','max_unobserved_s',2.),
        ('preprocessing','patch_max_spread_m',1.), ('feature_policy','max_features',4000),
        ('ground_truth_interpolation','max_bracket_s',1.),
        ('evaluation_label_policy','invalid_source','deduplicate invalid reference poses'),
        ('optical_flow_config','max_points',1000),('optical_flow_policy','masks','simulator truth')]
    for field, key, value in policy_changes:
        changed = copy.deepcopy(freeze); changed[field][key] = value
        if field+'_sha256' in changed:
            changed[field+'_sha256'] = (g.refinement_config_digest(changed[field]) if field == 'refinement_config'
                                        else r.canonical(changed[field]))
        try:
            verify_sources(changed, manifest_raw, proof_raw, source_root)
        except (ValueError, KeyError):
            rejected.append('policy_'+field)
        else:
            raise ValueError('weakened policy accepted '+field)
    proof = json.loads(proof_raw)
    changes = [ ('hide_metadata_failure',lambda x:x.__setitem__('passed',False)),
        ('invent_gt_row_order',lambda x:x['timestamps'].__setitem__('strict_ground_truth_order',False)),
        ('invent_gt_row_count',lambda x:x['timestamps'].__setitem__('ground_truth_rows',2)),
        ('invent_gt_bracket_stat',lambda x:x['timestamps'].__setitem__('maximum_observed_ground_truth_bracket_us',0)),
        ('invent_pair_gap_stat',lambda x:x['timestamps'].__setitem__('maximum_pair_gap_us',0)),
        ('weaken_pair_gap_gate',lambda x:x['timestamps'].__setitem__('max_pair_gap_s',2.)),
        ('invent_unique_rgb_count',lambda x:x['timestamps'].__setitem__('unique_rgb_acquisitions',999)),
        ('invent_qualification_window',lambda x:x['window'].__setitem__('first_depth_index',0)),
        ('invent_qualification_metadata_hash',lambda x:x['metadata_files']['groundtruth.txt'].__setitem__('sha256','0'*64)),
        ('invent_qualification_helper',lambda x:x.__setitem__('helper_sha256','0'*64)),
        ('qualification_parsed_pose_values',lambda x:x.__setitem__('ground_truth_pose_values_parsed',True)),
        ('qualification_decoded_pixels',lambda x:x.__setitem__('pixels_read',True)),
        ('qualification_opened_image_header',lambda x:x.__setitem__('image_headers_read',True)),
        ('qualification_ran_features',lambda x:x.__setitem__('features_or_fits_run',True)),
    ]
    for name, change in changes:
        altered_proof = copy.deepcopy(proof); change(altered_proof)
        altered_raw = json.dumps(altered_proof,sort_keys=True).encode()
        altered_freeze = copy.deepcopy(freeze)
        altered_freeze['qualification'] = altered_proof
        altered_freeze['qualification_sha256'] = r.digest(altered_raw)
        try:
            verify_sources(altered_freeze,manifest_raw,altered_raw,source_root)
            qualification_check(altered_freeze,manifest_raw,altered_proof,metadata)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('self-consistently rehashed qualification accepted '+name)
    manifest = json.loads(manifest_raw)
    for name, field, value in [('rename_repository','repository','unverified/repository'),
                               ('rename_source_commit','revision','0'*40)]:
        changed = copy.deepcopy(manifest); changed[field] = value
        changed_raw = json.dumps(changed,sort_keys=True).encode()
        changed_freeze = copy.deepcopy(freeze); changed_freeze['manifest_sha256'] = r.digest(changed_raw)
        try:
            verify_sources(changed_freeze,changed_raw,proof_raw,source_root)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('self-consistently rehashed source accepted '+name)
    return rejected


def metadata_contract_checks():
    frames = []
    depth_rows, rgb_rows = [], []
    for i in range(280):
        stamp, rgb_stamp = i/64.+1/128., i/64.
        dep = f'{stamp:.9f}.png'; rgb = f'{rgb_stamp:.9f}.png'
        depth_rows.append(f'{stamp:.9f} depth/{dep}\n')
        rgb_rows.append(f'{rgb_stamp:.9f} rgb/{rgb}\n')
        if i >= 100:
            frames.append(dict(source_index=i,depth_timestamp=stamp,depth_file='depth-'+dep,
                rgb_source_index=i,rgb_timestamp=rgb_stamp,rgb_file='rgb-'+rgb,
                pair_gap_seconds=1/128.,split='initialization' if i==100 else ('viewed_prefix' if i<=135 else 'unviewed_extension')))
    # Pose tokens deliberately are not numeric: qualification must treat them
    # as opaque while still enforcing eight-column metadata syntax.
    labels = ''.join(f'{i/128.:.9f} opaque opaque opaque opaque opaque opaque opaque\n' for i in range(570)).encode()
    metadata = {'depth.txt':''.join(depth_rows).encode(),'rgb.txt':''.join(rgb_rows).encode(),
                'groundtruth.txt':labels}
    manifest = dict(frames=frames)
    result = independently_qualify(manifest,metadata)
    r.require(result['unique_rgb_frames']==180 and result['maximum_pair_gap_us']==7813
              and result['maximum_observed_ground_truth_bracket_us']==0,
              'known earlier-index RGB tie or opaque-pose qualification changed')
    positive = ['opaque_pose_tokens_not_converted','earlier_rgb_index_on_exact_tie',
                'exact_reference_timestamp_zero_bracket','all_180_rows_retained']
    rejected = []
    corrupted = [ ('duplicate_gt_timestamp',dict(metadata,**{'groundtruth.txt':labels+labels.splitlines()[-1]+b'\n'})),
        ('nonfinite_gt_timestamp',dict(metadata,**{'groundtruth.txt':b'nan opaque opaque opaque opaque opaque opaque opaque\n'+labels})),
        ('wrong_gt_row_arity',dict(metadata,**{'groundtruth.txt':b'0 opaque\n'+labels})),
        ('duplicate_depth_timestamp',dict(metadata,**{'depth.txt':metadata['depth.txt']+depth_rows[-1].encode()})),
        ('oversized_gt_metadata',dict(metadata,**{'groundtruth.txt':b' '*(4194304+1)})) ]
    for name, altered in corrupted:
        try:
            independently_qualify(manifest,altered)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('corrupted qualification metadata accepted '+name)
    for name, change in [('wrong_original_depth_index',lambda x:x['frames'][0].__setitem__('source_index',0)),
        ('wrong_nearest_rgb_index',lambda x:x['frames'][0].__setitem__('rgb_source_index',101)),
        ('invent_pair_gap',lambda x:x['frames'][0].__setitem__('pair_gap_seconds',.001)),
        ('omit_observed_depth_row',lambda x:x['frames'].pop())]:
        altered = copy.deepcopy(manifest); change(altered)
        try:
            independently_qualify(altered,metadata)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('corrupted source association accepted '+name)
    return dict(passed=positive,mutations_rejected=rejected)


def temporal_report_mutations(report, manifest, features, depths, gt, label_failure, grays=None):
    audit(report,manifest,features,depths,gt,label_failure,grays)
    failures = []
    for name, change in [ ('conceal_qualification_gate',lambda x:x.__setitem__('qualification_verified',False)),
        ('conceal_report_qualification',lambda x:x.pop('qualification')),
        ('invent_report_qualification_hash',lambda x:x.__setitem__('qualification_sha256','0'*64)),
        ('invent_report_qualification_count',lambda x:x['qualification']['timestamps'].__setitem__('ground_truth_rows',2)),
        ('invent_report_algorithm',lambda x:x.__setitem__('algorithm','unverified_algorithm')),
        ('drop_unviewed_tail',lambda x:x.__setitem__('frames',x['frames'][:36])),
        ('omit_final_observation',lambda x:x['frames'].pop()),
        ('reclassify_unviewed_tail',lambda x:x['frames'][36].__setitem__('split','viewed_prefix')),
        ('forge_midsequence_initialization',lambda x:x['frames'][36].__setitem__('initialized',True)),
        ('forge_boundary_clock_reset',lambda x:x['frames'][36].__setitem__('last_accepted_stamp_before',None)),
        ('forge_boundary_reference_reset',lambda x:x['frames'][36].__setitem__('reference_frame_index_before',None)),
        ('allow_chunk_resets',lambda x:x['freeze']['continuous_state'].__setitem__('chunk_resets',True)),
        ('allow_multiple_initializations',lambda x:x['freeze']['continuous_state'].__setitem__('maximum_initializations',2)),
        ('claim_fresh_generalization',lambda x:x['freeze'].__setitem__('evidence_scope','fresh environment generalization')),
        ('reverse_temporal_boundary',lambda x:x['frames'].__setitem__(slice(35,37),list(reversed(x['frames'][35:37])))) ]:
        changed = copy.deepcopy(report); change(changed)
        try:
            audit(changed,manifest,features,depths,gt,label_failure,grays)
        except (ValueError,KeyError):
            failures.append(name)
        else:
            raise ValueError('temporal report corruption accepted '+name)
    return failures


def verified_inputs(manifest, raw_path, report=None):
    if report is None:
        report = dict(calibration_source=manifest['calibration_source'], calibration_sha256_verified=True)
    v.calibration_check(report, manifest, raw_path)
    raw, total = {}, 0
    for item in manifest['files']:
        filename = item['file']
        r.require(Path(filename).name == filename and '/' not in filename and '\\' not in filename,
                  'unsafe raw filename')
        content = r.bounded(raw_path/filename)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'],
                  'measured RGB/depth/mocap byte mismatch')
        if 'git_blob_sha1' in item:
            blob = b'blob '+str(len(content)).encode()+b'\0'+content
            r.require(hashlib.sha1(blob).hexdigest() == item['git_blob_sha1'], 'changed pinned Git source blob')
        raw[filename] = content
        total += len(content)
    r.require(total+manifest['calibration_source']['bytes'] <= 128*1024*1024, 'raw acquisition total bound')
    if 'files' in report:
        r.require(report['files'] == [{key: item[key] for key in ('file', 'bytes', 'sha256', 'role')}
                                     for item in manifest['files']], 'changed input provenance')
    expected_names = {'depth.txt', 'rgb.txt', 'groundtruth.txt'} | {
        frame[key] for frame in manifest['frames'] for key in ('depth_file', 'rgb_file')}
    r.require(set(raw) == expected_names and len(manifest['files']) == len(expected_names),
              'unexpected/incomplete visual input inventory')
    return raw


@lru_cache(maxsize=1)
def continuous_contract_checks():
    """All180 acquisitions, alternating known nonzero image motion and one root.

    This synthetic fixture has limited cumulative displacement; separate
    inherited composition controls check accumulating rotated poses. Reference
    points are hand identified; the actual-record audit reconstructs FAST/BRIEF.
    """
    import random
    cal=dict(fx=517.306408,fy=516.469215,cx=318.643040,cy=255.313989,units_per_metre=5000.)
    rng=random.Random(42); descriptions=[[rng.getrandbits(64) for _ in range(4)] for _ in range(12)]
    features=[[dict(x=float(x+i%2),y=float(y),score=40,orientation=0.,descriptor=descriptions[j]) for j,(x,y) in enumerate([(x,y) for y in (120,240,360) for x in (120,180,240,300)])] for i in range(180)]
    yy,xx=g.np.mgrid[:480,:640]
    base=g.np.floor(128+50*g.np.sin(2*g.np.pi*xx/16)+50*g.np.cos(2*g.np.pi*yy/16)+.5).astype(g.np.uint8)
    variants=[g.np.roll(base,i,axis=1) for i in range(2)];grays=[variants[i%2] for i in range(180)]
    depth=g.np.full((480,640),7500,dtype=g.np.uint16);depths=[depth]*180
    identity=([0.,0.,0.],[1.,0.,0.,0.]);pose=lambda value:dict(translation_m=value[0],quaternion_wxyz=value[1])
    frames=[dict(source_index=100+i,depth_file=f'depth-{i}.png',depth_timestamp=1.+i/64.,rgb_file=f'rgb-{i}.png',rgb_timestamp=1.+i/64.,rgb_source_index=100+i,pair_gap_seconds=0.,split='initialization' if i==0 else ('viewed_prefix' if i<=35 else 'unviewed_extension')) for i in range(180)]
    manifest=dict(frames=frames,depth_calibration=cal,dataset='hand-continuous-motion',repository='hand/fixture',revision='hand')
    freeze=dict(registration_config=v.CONFIG,refinement_config=g.REFINEMENT_CONFIG,tracking_policy=dict(max_unobserved_s=.2),qualification={},qualification_sha256='hand',regression_requested=True,kind='calibration_regression',optical_flow_config=FLOW_CONFIG,optical_flow_policy=FLOW_POLICY,**TEMPORAL_POLICY)
    report=dict(schema_version=1,algorithm=ALGORITHM,dataset=manifest['dataset'],repository=manifest['repository'],revision=manifest['revision'],ground_truth_operational=False,raw_redistributed=False,qualification_verified=True,qualification={},qualification_sha256='hand',freeze=freeze,frames=[])
    root=identity
    for i,(frame,fs) in enumerate(zip(frames,features)):
     row=dict(frame,accepted=True,initialized=i==0,features=fs,feature_count=len(fs),features_computed=True,cpu_wall_seconds=0.,last_accepted_stamp_before=frames[i-1]['depth_timestamp'] if i else None,reference_frame_index_before=frames[i-1]['source_index'] if i else None,lost_before=False,last_observed_rgb_stamp_before=frames[i-1]['rgb_timestamp'] if i else None,flow_attempted=i>0,flow_results=[],flow_work=None,tracked_endpoints=[],depth_matches=[],correspondences=[],refinement_observations=[],refinement_attempted=i>0)
     if i==0:
      points=[g.depth_point_status(f,depth,cal)[0] for f in fs]
      row['initialization_depth_features']=[dict(feature_index=j,accepted=True,point=p) for j,p in enumerate(points)]
      row['initialization_geometry_ratio']=v.scatter_ratio(points)
     else:
      row['reference_frame_index']=frames[i-1]['source_index'];row['reference_stamp']=frames[i-1]['depth_timestamp'];row['root_from_reference']=pose(root)
      flow=independent_pixel_tracks(grays[i-1],grays[i],[[f['x'],f['y']] for f in features[i-1]])
      assert all(p['accepted'] for p in flow['points']),(i,[(p['point_index'],p['rejection'],p['forward_levels'][-1],p['backward_levels'][-1] if p['backward_levels'] else None) for p in flow['points'] if not p['accepted']])
      endpoints=[dict(previous_feature_index=p['point_index'],current_pixel_xy=p['forward_endpoint']) for p in flow['points']]
      row.update(flow_results=flow['points'],flow_work={key:flow[key] for key in ('bilinear_samples','iterations','pyramid_output_pixels')},tracked_endpoints=endpoints)
      pairs=[];deprows=[]
      for j,endpoint in enumerate(endpoints):
       pp=g.depth_point_status(features[i-1][endpoint['previous_feature_index']],depth,cal)[0];xy=endpoint['current_pixel_xy'];cp=g.depth_point_status(dict(x=xy[0],y=xy[1]),depth,cal)[0]
       pairs.append(dict(previous=pp,current=cp));deprows.append(dict(previous_index=endpoint['previous_feature_index'],current_endpoint_index=j,accepted=True,correspondence_selected=True,previous_xyz=pp,current_xyz=cp,correspondence_index=j))
      fit,error=temporal_rigid_fit(pairs);assert error is None,error
      coarse=fit['estimate'];row.update(depth_matches=deprows,correspondences=pairs,fit={a:b for a,b in fit.items() if a!='estimate'},coarse_relative_estimate=pose(coarse))
      observations=[dict(correspondence_index=j,previous_index=endpoints[j]['previous_feature_index'],current_endpoint_index=j,previous_xyz=pairs[j]['previous'],current_pixel_xy=endpoints[j]['current_pixel_xy']) for j in fit['inlier_indices']]
      refined,error=temporal_refinement(observations,cal,coarse);assert error is None,error
      row['refinement_observations']=observations;row['relative_estimate']=pose(refined['estimate']);external={a:copy.deepcopy(b) for a,b in refined.items() if a!='estimate'}
      for step in external['trace']:step['current_from_previous']=pose(step['current_from_previous'])
      row['refinement']=external;root=k.compose(root,refined['estimate'])
     truth=([-(i%2)*1.5/cal['fx'],0.,0.],[1.,0.,0.,0.]);position=r.norm([a-b for a,b in zip(root[0],truth[0])]);angle=r.quaternion_error(root[1],truth[1]);assert position<1e-4 and angle<1e-4,(i,position,angle)
     row['root_estimate']=pose(root);row['root_accuracy']=dict(valid=True,estimate=pose(root),reference_valid=True,evaluation_only_truth=pose(truth),translation_error_m=position,rotation_error_rad=angle,within_accuracy_gates=True)
     row.update(last_accepted_stamp_after=frame['depth_timestamp'],reference_frame_index_after=frame['source_index'],lost_after=False,last_observed_rgb_stamp_after=frame['rgb_timestamp']);report['frames'].append(row)
    report['qualification']={'timestamps':{'ground_truth_rows':180}}
    freeze['qualification']=copy.deepcopy(report['qualification'])
    report['summary']=dict(frames=180,updates=179,initialized_frames=1,accepted_updates=179,rejected_updates=0,accurate_root_updates=179,reference_valid_updates=179,lost=False,all_updates_passed=True)
    gt=[(f['depth_timestamp'],[-(i%2)*1.5/cal['fx'],0.,0.],[1.,0.,0.,0.]) for i,f in enumerate(frames)]
    records=audit(report,manifest,features,depths,gt,None,grays);assert len(records)==180
    mutants=temporal_report_mutations(report,manifest,features,depths,gt,None,grays)+tracked_report_mutations(report,manifest,features,depths,gt,None,grays)+mutation_checks(report,manifest,features,depths,gt,None,grays)
    out=dict(passed_integrity=True,hand_fixture_only=True,actual_temporal_images_or_pose_values_read=False,positive=['complete180frame179update_known_motion_passes_same_auditor','exactlyone_initialization','continuous_alternating_nonzero_known_pixel_motion','all179_physical_scores_include_tail'],mutations_rejected=mutants,terminal_known_translation_m=-1.5/cal['fx'])
    _independent_pixel_tracks.cache_clear()
    _pixel_pyramid.cache_clear()
    _temporal_rigid_fit.cache_clear()
    _temporal_refinement.cache_clear()
    return out


def main():
    if sys.argv[1:] == ['--self-test']:
        print(json.dumps(dict(continuous_contract_checks=continuous_contract_checks(),
            pixel_contract_checks=pixel_contract_checks(),mathematical_contract_checks=g.mathematical_contract_checks(),
            correspondence_selection_contract_checks=correspondence_selection_contract_checks(),
            missing_reference_score_mutations_rejected=k.missing_reference_contract_checks())))
        return
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'raw', 'freeze', 'qualification', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--source-snapshot', type=Path)
    parser.add_argument('--preregister-only', action='store_true')
    args = parser.parse_args()
    manifest_raw = r.bounded(args.manifest, 512*1024)
    freeze_raw = r.bounded(args.freeze, 512*1024)
    qualification_raw = r.bounded(args.qualification, 512*1024)
    freeze = json.loads(freeze_raw)
    source_root = args.source_snapshot or ROOT
    manifest, qualification = verify_sources(freeze, manifest_raw, qualification_raw, source_root)
    metadata = metadata_inputs(manifest, args.raw)
    measured = qualification_check(freeze, manifest_raw, qualification, metadata)
    common = dict(schema_version=1, passed_integrity=True, source_freeze_verified=True,
        synthetic_image_controls_executed=True,actual_recorded_image_pixels_read=False,
        frozen_source_count=len(SOURCES), dataset=DATASET, kind=freeze['kind'],
        manifest_sha256=r.digest(manifest_raw), freeze_sha256=r.digest(freeze_raw),
        qualification_sha256=r.digest(qualification_raw), qualification_independently_recomputed=True,
        qualification_method='whole-source row arity and timestamp columns only; nearest RGB and strict brackets',
        temporal_timestamp_summary=measured, running_checker_sha256=r.digest(r.bounded(Path(__file__))),
        frozen_checker_sha256=freeze['independent_checker_sha256'],
        archived_source_snapshot=args.source_snapshot is not None)
    source_mutations = qualification_mutations(freeze,manifest_raw,qualification_raw,metadata,source_root)
    metadata_contracts = metadata_contract_checks()
    cache_contracts = cache_contract_checks()
    common.update(frozen_provenance_and_qualification_mutations_rejected=source_mutations,
                  metadata_contract_checks=metadata_contracts,
                  exact_solver_cache_contract_checks=cache_contracts,
                  continuous_contract_checks=continuous_contract_checks(),pixel_contract_checks=pixel_contract_checks(),
                  correspondence_selection_contract_checks=correspondence_selection_contract_checks())
    if args.preregister_only:
        r.require(args.report is None, 'metadata preregistration must not consume fit reports')
        common.update(metadata_only=True,
                      evidence_scope=TEMPORAL_POLICY['evidence_scope'],
                      reference_source_previously_evaluated=True, metadata_files_read=['depth.txt','rgb.txt','groundtruth.txt'],
                      image_bytes_read=False, image_headers_read=False, pixels_read=False,
                      ground_truth_pose_values_parsed=False, features_or_fits_run=False)
        write_result(args.output, common)
        print(json.dumps(common))
        return
    r.require(args.report is not None, 'fit audit requires --report')
    report_raw = r.bounded(args.report, 128*1024*1024)
    report = json.loads(report_raw)
    exact_json(report['freeze'], freeze, 'external freeze mismatch')
    r.require(report['freeze_sha256'] == r.digest(freeze_raw)
              and report['manifest_sha256'] == r.digest(manifest_raw), 'external source/freeze mismatch')
    raw = verified_inputs(manifest, args.raw, report)
    features, depths, grays, cache = [], [], [], {}
    for frame in manifest['frames']:
        filename = frame['rgb_file']
        if filename not in cache:
            gray=v.png_image(raw[filename],False)
            cache[filename] = (gray,v.features(gray))
        grays.append(cache[filename][0])
        features.append(cache[filename][1])
        depths.append(v.png_image(raw[frame['depth_file']], True))
    # Reference pose columns are parsed only after the frozen operational report
    # and the independent measured-feature/refinement reconstruction exist.
    gt, label_failure = g.evaluation_labels(raw['groundtruth.txt'])
    records = audit(report, manifest, features, depths, gt, label_failure,grays)
    old_mutations = mutation_checks(report, manifest, features, depths, gt, label_failure,grays)
    temporal_mutations = temporal_report_mutations(report,manifest,features,depths,gt,label_failure,grays)
    flow_mutations = tracked_report_mutations(report,manifest,features,depths,gt,label_failure,grays)
    common.update(summary=report['summary'], frames=records, report_sha256=r.digest(report_raw),
        actual_recorded_image_pixels_read=True,
        raw_sha256_verified=True, evaluation_label_failure=label_failure,
        physical_pose_independently_scored=label_failure is None,
        unscorable_updates=report['summary']['updates']-report['summary']['reference_valid_updates'],
        mutations_rejected=old_mutations+temporal_mutations+flow_mutations, mathematical_contract_checks=g.mathematical_contract_checks(),
        missing_reference_score_mutations_rejected=k.missing_reference_contract_checks(),
        pixel_features_descriptors_and_associations_independently_reconstructed=True,
        pyramidal_tracking_endpoints_and_work_independently_reconstructed=True,
        endpoint_buffer_separate_from_detected_features_verified=True,
        rigid_fit_independently_replayed='Kabsch SVD vs operational Horn quaternion/Jacobi',
        pixel_refinement_independently_replayed='central numerical Jacobian and SVD vs analytic Jacobian and Cholesky',
        fixed_original_inlier_support_independently_checked=True,
        monotonic_search_and_terminal_step_accounting_independently_checked=True,
        reference_and_clock_state_independently_reconstructed=True,
        evidence_role='viewed calibration regression',
        evidence_scope=TEMPORAL_POLICY['evidence_scope'],
        all_179_updates_audited_continuously=True,
        maximum_initializations=1,chunk_resets=False,lost_recovery=False,
        calibrated_covariance_or_root_confidence_claim=False,
        oracle_dependencies=dict(numpy=g.np.__version__, pillow=v.Image.__version__),
        continuous_audit_method='static copy of original state/physical audit with full180frame179update length; same pinned feature/3D/refinement math and no frame chunk resets',
        **g.maximum_scored_errors(records))
    write_result(args.output, common)
    print(json.dumps(report['summary']))


if __name__ == '__main__':
    main()
