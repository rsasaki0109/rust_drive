"""Independent vectorized depth geometry; never imports an operational Rust/Python fitter."""
import numpy as np

from projective_depth_common import pose, require


def vertices(depth, camera):
    y,x = np.indices(depth.shape)
    return np.stack([(x-camera['cx'])*depth/camera['fx'],
                     (y-camera['cy'])*depth/camera['fy'],depth],axis=-1)


def pyramid(raw, camera, normal_policy='central_difference'):
    require(normal_policy in ('central_difference','inverse_depth_plane'),'declared normal policy')
    depths = {1:raw.copy()}
    for scale in (2,4):
        a = depths[scale//2]
        block = np.stack([a[dy::2,dx::2] for dy,dx in ((0,0),(0,1),(1,0),(1,1))])
        lo,hi = block.min(axis=0),block.max(axis=0)
        valid = (lo >= .3)&(hi <= 5)&(hi-lo <= .05+.01*lo)
        depths[scale] = np.where(valid,block.mean(axis=0),0.)
    result = {}
    for scale,depth in depths.items():
        c = dict(width=camera['width']//scale,height=camera['height']//scale,
                 fx=camera['fx']/scale,fy=camera['fy']/scale,
                 cx=(camera['cx']+.5)/scale-.5,cy=(camera['cy']+.5)/scale-.5)
        xyz = vertices(depth,c)
        normal = np.zeros_like(xyz)
        center = depth[1:-1,1:-1]
        valid = (center > .3)&(center < 5)
        for neighbour in (depth[1:-1,2:],depth[1:-1,:-2],depth[2:,1:-1],depth[:-2,1:-1]):
            valid &= (neighbour > .3)&(np.abs(neighbour-center) < .05+.01*center)
        cross = np.cross(xyz[1:-1,2:]-xyz[1:-1,:-2],xyz[2:,1:-1]-xyz[:-2,1:-1])
        length = np.linalg.norm(cross,axis=-1)
        valid &= length > 1e-10
        normal[1:-1,1:-1][valid] = cross[valid]/length[valid,None]
        if normal_policy == 'inverse_depth_plane' and scale == 1:
            planes = inverse_depth_normals(depth,c)
            use_plane = np.sum(planes*planes,axis=-1)>.25
            normal[use_plane] = planes[use_plane]
        result[scale] = dict(vertices=xyz,normals=normal,camera=c,depth=depth)
    return result


def inverse_depth_normals(depth,camera):
    """Independent uniform least squares on complete inverse-depth patches."""
    windows = np.lib.stride_tricks.sliding_window_view(depth,(5,5))
    center = depth[2:-2,2:-2]
    valid = ((windows>.3)&(windows<5)&
             (abs(windows-center[:,:,None,None])<.05+.01*center[:,:,None,None])).all(axis=(-2,-1))
    yy,xx = np.mgrid[-2:3,-2:3]
    design = np.column_stack([xx.ravel(),yy.ravel(),np.ones(25)])
    # NumPy QR/SVD pseudo-inverse; no operational coefficient implementation.
    coefficients = np.einsum('ij,abj->abi',np.linalg.pinv(design),
                             (1/np.maximum(windows,1e-15)).reshape(*center.shape,25))
    prediction = np.einsum('ji,abi->abj',design,coefficients).reshape(*center.shape,5,5)
    rms = np.sqrt(np.mean((1/np.maximum(prediction,1e-15)-windows)**2,axis=(-2,-1)))
    valid &= rms<=.002
    a,b,g = np.moveaxis(coefficients,-1,0)
    y,x = np.mgrid[2:depth.shape[0]-2,2:depth.shape[1]-2]
    n = np.stack([camera['fx']*a,camera['fy']*b,
                  g-(x-camera['cx'])*a-(y-camera['cy'])*b],axis=-1)
    length = np.linalg.norm(n,axis=-1)
    normal = np.zeros((*depth.shape,3))
    supported = valid&(length>1e-10)
    normal[2:-2,2:-2][supported] = n[supported]/length[supported,None]
    return normal


def normal_equation(target, source, transform, scale, correspondence='forward', residual_policy='target_plane'):
    if residual_policy=='depth_whitened_plane':
        require(correspondence=='forward','qualified standardized range-weighted correspondence policy')
        from projective_whitened_raw import equation
        return equation(target,source,transform,scale)
    if residual_policy=='depth_variance_plane':
        require(correspondence=='forward','qualified range-weighted correspondence policy')
        from projective_range_raw import pairs,equation
        return equation(pairs(target,source,transform,scale),True)
    if residual_policy=='bidirectional_midpoint':
        require(correspondence=='forward','qualified bidirectional correspondence policy')
        forward=normal_equation(target,source,transform,scale,residual_policy='symmetric_midpoint')
        reverse=normal_equation(source,target,np.linalg.inv(transform),scale,residual_policy='symmetric_midpoint')
        # Reverse left increments are -Ad(T^-1) times target-frame increments.
        # Unlike the Rust direct-pair accumulation, transport whole equations.
        inverse=np.linalg.inv(transform);rotation=inverse[:3,:3];x,y,z=inverse[:3,3]
        cross=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        adjoint=np.block([[rotation,cross@rotation],[np.zeros((3,3)),rotation]])
        count=forward['count']+reverse['count']
        squared=forward['count']*forward['rms_m']**2+reverse['count']*reverse['rms_m']**2
        return dict(count=count,rms_m=float(np.sqrt(squared/count)) if count else 0.,
                    h=forward['h']+adjoint.T@reverse['h']@adjoint,
                    g=forward['g']-adjoint.T@reverse['g'],
                    direction_counts=[forward['count'],reverse['count']])
    stride = 2 if scale == 1 else 1
    xyz = source['vertices'][::stride,::stride].reshape(-1,3)
    normals = source['normals'][::stride,::stride].reshape(-1,3)
    indices = np.arange(source['depth'].size).reshape(source['depth'].shape)[::stride,::stride].ravel()
    valid = np.sum(normals*normals,axis=1) > .25
    xyz,normals = xyz[valid],normals[valid]
    indices = indices[valid]
    query = xyz@transform[:3,:3].T+transform[:3,3]
    transformed_normals = normals@transform[:3,:3].T
    c = target['camera']
    # Reject invalid Z before division or integer conversion.
    front = np.isfinite(query).all(axis=1)&(query[:,2] > .3)
    query,transformed_normals = query[front],transformed_normals[front]
    indices = indices[front]
    uf = c['fx']*query[:,0]/query[:,2]+c['cx']
    vf = c['fy']*query[:,1]/query[:,2]+c['cy']
    bilinear = correspondence=='bilinear'
    u,v = (np.floor(uf),np.floor(vf)) if bilinear else (np.rint(uf),np.rint(vf))
    border = 2 if bilinear else 1
    inside = (u >= 1)&(u < c['width']-border)&(v >= 1)&(v < c['height']-border)
    query,transformed_normals = query[inside],transformed_normals[inside]
    indices = indices[inside]
    u,v = u[inside].astype(np.int64),v[inside].astype(np.int64)
    point = target['vertices'][v,u]
    normal = target['normals'][v,u]
    target_valid = np.ones(len(query),dtype=bool)
    if bilinear:
        xx = u[:,None]+np.array([0,1,0,1]);yy = v[:,None]+np.array([0,0,1,1])
        vertices = target['vertices'][yy,xx];normals = target['normals'][yy,xx];depth = vertices[:,:,2]
        a,b = uf[inside]-u,vf[inside]-v
        blend = np.column_stack([(1-a)*(1-b),a*(1-b),(1-a)*b,a*b])
        point = np.einsum('ni,nij->nj',blend,vertices);normal = np.einsum('ni,nij->nj',blend,normals)
        length = np.linalg.norm(normal,axis=1)
        target_valid = ((depth>.3)&(depth<5)).all(axis=1)&(np.sum(normals*normals,axis=2)>.25).all(axis=1)
        target_valid &= (np.ptp(depth,axis=1)<=.05+.01*np.min(depth,axis=1))&(length>.5)
        normal = normal/np.maximum(length,1e-15)[:,None]
    valid = (target_valid&(np.sum(normal*normal,axis=1) > .25)
             &(np.linalg.norm(query-point,axis=1) < (.12 if scale == 4 else .08))
             &(np.sum(transformed_normals*normal,axis=1) > .8))
    if correspondence=='reciprocal':
        back = (point-transform[:3,3])@transform[:3,:3]
        front = np.isfinite(back).all(axis=1)&(back[:,2]>.3)
        sc = source['camera'];bu,bv = np.full(len(back),np.nan),np.full(len(back),np.nan)
        bu[front] = np.rint(sc['fx']*back[front,0]/back[front,2]+sc['cx'])
        bv[front] = np.rint(sc['fy']*back[front,1]/back[front,2]+sc['cy'])
        valid &= front&(bu==indices%sc['width'])&(bv==indices//sc['width'])
    query,point,normal = query[valid],point[valid],normal[valid]
    residual = np.sum(normal*(query-point),axis=1)
    jacobian = np.column_stack([normal,np.cross(query,normal)])
    if residual_policy=='symmetric_midpoint':
        ns = transformed_normals[valid]
        midpoint_normal = (normal+ns)/2
        residual = np.sum(midpoint_normal*(query-point),axis=1)
        jacobian = np.column_stack([midpoint_normal,
                                   (np.cross(query,normal)+np.cross(point,ns))/2])
    weights = np.minimum(1.,.01/np.maximum(np.abs(residual),1e-15))
    h = jacobian.T@(weights[:,None]*jacobian)
    g = jacobian.T@(weights*residual)
    squared = residual*residual
    if residual_policy=='symmetric_planes':
        ns = transformed_normals[valid]
        rs = np.einsum('ij,ij->i',ns,query-point)
        js = np.column_stack([ns,np.cross(point,ns)])
        ws = np.minimum(1.,.01/np.maximum(abs(rs),1e-15))
        h = .5*(h+js.T@(ws[:,None]*js));g = .5*(g+js.T@(ws*rs))
        squared = .5*(squared+rs*rs)
    rms = float(np.sqrt(np.mean(squared))) if len(residual) else 0.
    return dict(count=len(residual),rms_m=rms,h=h,g=g)


def compare_equation(actual, expected):
    require(type(actual['count']) is int and actual['count'] == expected['count'],'exact raw correspondence count')
    require(actual.get('direction_counts')==expected.get('direction_counts'),'exact independent direction counts')
    np.testing.assert_allclose(actual['h'],expected['h'],atol=1e-7,rtol=1e-10,
                               err_msg='raw-pixel normal matrix')
    np.testing.assert_allclose(actual['g'],expected['g'],atol=1e-9,rtol=1e-10,
                               err_msg='raw-pixel gradient')
    require(np.isfinite(actual['rms_m']) and abs(actual['rms_m']-expected['rms_m']) <= 1e-10,
            'raw-pixel residual RMS')


def raster(depth, weights, transform, camera, normals=None):
    valid = (depth > .3)&(depth < 5)
    indices = np.flatnonzero(valid.ravel())
    query = vertices(depth,camera).reshape(-1,3)[indices]@transform[:3,:3].T+transform[:3,3]
    support = weights.ravel()[indices]
    stats = dict(source_valid=len(indices),projected=0,selected=0)
    front = np.isfinite(query).all(axis=1)&(query[:,2] > .3)&(query[:,2] < 5)
    query,support,indices = query[front],support[front],indices[front]
    x = np.rint(camera['fx']*query[:,0]/query[:,2]+camera['cx'])
    y = np.rint(camera['fy']*query[:,1]/query[:,2]+camera['cy'])
    inside = (x >= 0)&(x < camera['width'])&(y >= 0)&(y < camera['height'])
    cells = y[inside].astype(np.int64)*camera['width']+x[inside].astype(np.int64)
    z,support,indices = query[inside,2],support[inside],indices[inside]
    adjusted = 0
    if normals is not None:
        n = normals.reshape(-1,3)[indices]@transform[:3,:3].T
        rays = np.column_stack([(x[inside]-camera['cx'])/camera['fx'],
                                (y[inside]-camera['cy'])/camera['fy'],np.ones(len(z))])
        denominator = np.einsum('ij,ij->i',n,rays)
        eligible = (np.einsum('ij,ij->i',n,n)>.25)&(abs(denominator)>.1*np.linalg.norm(rays,axis=1))
        plane_z = np.zeros(len(z))
        np.divide(np.einsum('ij,ij->i',n,query[inside]),denominator,out=plane_z,where=eligible)
        eligible &= np.isfinite(plane_z)&(plane_z>.3)&(plane_z<5)&(abs(plane_z-z)<.03+.01*z)
        z = np.where(eligible,plane_z,z);adjusted = int(eligible.sum())
    stats['projected'] = len(z)
    output = np.zeros(depth.size)
    output_weight = np.zeros(depth.size,dtype=np.uint8)
    # Explicit independent lexicographic selection: cell, axial Z, source index.
    order = np.lexsort((indices,z,cells))
    ordered_cells = cells[order]
    first = np.r_[True,ordered_cells[1:] != ordered_cells[:-1]] if len(order) else np.array([],dtype=bool)
    winners = order[first]
    output[cells[winners]],output_weight[cells[winners]] = z[winners],support[winners]
    stats['selected'] = len(winners)
    if adjusted:stats['tangent_adjusted'] = adjusted
    return output.reshape(depth.shape),output_weight.reshape(depth.shape),stats


def fuse(model, weights, raw, transform, camera, normals=None):
    projected,present,stats = raster(raw,np.ones(raw.shape,dtype=np.uint8),transform,camera,normals)
    valid = present > 0
    filled = valid&(weights == 0)
    averaged = valid&(weights > 0)&(np.abs(projected-model) < .03+.01*model)
    replaced = valid&(weights > 0)&~averaged&(projected < model-.03-.01*model)
    ignored = valid&~filled&~averaged&~replaced
    output = model.copy()
    support = weights.copy()
    prior = np.minimum(16,weights[averaged]).astype(float)
    output[averaged] = (prior*model[averaged]+projected[averaged])/(prior+1)
    support[averaged] = np.minimum(16,weights[averaged]+1)
    output[filled|replaced] = projected[filled|replaced]
    support[filled|replaced] = 1
    counts = dict(warp=None,measurement=stats,filled=int(filled.sum()),averaged=int(averaged.sum()),
                  replaced=int(replaced.sum()),ignored=int(ignored.sum()))
    return output,support,counts


def reconstruct_update(model, weights, raw, camera, update, reprojection='point', correspondence='forward', residual_policy='target_plane', normal_policy='central_difference', geometry='depth', image_context=None, fusion_sampling='nearest', visibility='disabled'):
    """Validate all raw equations and rebuild the next model only after acceptance."""
    require(raw.shape == (camera['height'],camera['width']) and np.isfinite(raw).all()
            and np.all((raw >= 0)&(raw <= 100)),'bounded decoded depth')
    require(geometry in ('depth','vertices'),'declared model geometry')
    require(fusion_sampling in ('nearest','visible_mean'),'declared fusion sampling policy')
    require(visibility in ('disabled','free_space_decay'),'declared visibility policy')
    if visibility=='free_space_decay':
        require((geometry,reprojection,correspondence,residual_policy,normal_policy,fusion_sampling)==
                ('depth','point','forward','target_plane','central_difference','nearest') and image_context is None,
                'qualified visibility-decay policies')
    if fusion_sampling=='visible_mean':
        require((geometry,reprojection,correspondence,residual_policy,normal_policy)==
                ('depth','point','forward','target_plane','central_difference') and image_context is None,
                'qualified visible-mean fusion policies')
    if geometry=='vertices':
        require((reprojection,correspondence,residual_policy,normal_policy)==
                ('point','forward','target_plane','central_difference'),'qualified vertex policies')
        from projective_vertex_raw import reconstruct
        return reconstruct(model,weights,raw,camera,update)
    source = pyramid(raw,camera,normal_policy)
    colors=None
    if 'photometric' in update:
        require(image_context is not None and geometry=='depth','RGB source context required')
        from projective_photo_raw import metadata,gray_pyramid
        metadata(update,image_context)
        if image_context['target'] is not None:
            colors=(gray_pyramid(image_context['target']),gray_pyramid(image_context['source']))
        else:
            require(all('photometric' not in s for s in update['iterations']) and 'final_photometric' not in update,'no invented color anchor')
    else:require(image_context is None,'no silently ignored image context')
    if model is None:
        require(update['accepted'] and not update['iterations'],'valid first initialization')
        require(np.sum(np.sum(source[4]['normals']**2,axis=-1) > .25) >= 200,'initial surface support')
        return raw.copy(),(raw > .3).astype(np.uint8),dict(iterations=0,final_equations=0,model_replaced=False)
    target = pyramid(model,camera,normal_policy)
    for step in update['iterations']:
        scale = step['scale']
        expected = normal_equation(target[scale],source[scale],pose(step['before']),scale,correspondence,residual_policy)
        compare_equation(step['equation'],expected)
        if colors is not None:
            from projective_photo_raw import equation,compare
            compare(step['photometric'],equation(target[scale],source[scale],colors[0][scale],colors[1][scale],pose(step['before']),scale,image_context['qualified'] and image_context['fresh'],affine='illumination' in step['photometric'],warps=image_context.get('warps')))
        if 'step_control' in step:
            from projective_step_raw import reconstruct,compare
            require('photometric' in update,'step control belongs to the RGB-D mode')
            compare(step['step_control'],reconstruct(target[scale],source[scale],colors,step))
    if 'information_failure' in update:
        from projective_information_raw import audit
        audit(update)
        f=update['information_failure'];scale=f['scale'];transform=pose(f['pose'])
        compare_equation(f['equation'],normal_equation(target[scale],source[scale],transform,scale,correspondence,residual_policy))
        if colors is not None:
            from projective_photo_raw import equation,compare
            compare(f['photometric'],equation(target[scale],source[scale],colors[0][scale],colors[1][scale],transform,scale,image_context['qualified'] and image_context['fresh'],affine=True,warps=image_context.get('warps')))
        else:require(f['photometric'] is None,'no invented failure image anchor')
    if not update['accepted']:
        require(update['fusion'] is None and update['first_from_current'] is None,'no rejected-state commit')
        return model,weights,dict(iterations=len(update['iterations']),final_equations=0,model_replaced=False)
    relative = pose(update['iterations'][-1]['after'])
    final = normal_equation(target[1],source[1],relative,1,correspondence,residual_policy)
    compare_equation(update['final_equation'],final)
    if colors is not None:
        from projective_photo_raw import equation,compare
        compare(update['final_photometric'],equation(target[1],source[1],colors[0][1],colors[1][1],relative,1,image_context['qualified'] and image_context['fresh'],affine='illumination' in update['final_photometric'],warps=image_context.get('warps')))
    visibility_stats = None
    if visibility=='free_space_decay':
        from projective_visibility_raw import decay
        model,weights,visibility_stats = decay(model,weights,raw,source[1]['normals'],np.linalg.inv(relative),camera)
    warp = None
    if update['keyframe_replaced']:
        inverse = np.eye(4)
        inverse[:3,:3] = relative[:3,:3].T
        inverse[:3,3] = -relative[:3,:3].T@relative[:3,3]
        if fusion_sampling=='visible_mean':
            from projective_fusion_raw import raster as model_raster
            model,weights,warp = model_raster(model,weights,inverse,camera)
        else:
            model,weights,warp = raster(model,weights,inverse,camera,
                                      target[1]['normals'] if reprojection=='tangent_plane' else None)
        relative = np.eye(4)
    if fusion_sampling=='visible_mean':
        from projective_fusion_raw import fuse as model_fuse
        result,support,counts = model_fuse(model,weights,raw,relative,camera)
    else:
        result,support,counts = fuse(model,weights,raw,relative,camera,
                                   source[1]['normals'] if reprojection=='tangent_plane' else None)
    counts['warp'] = warp
    if visibility_stats is not None:counts['visibility'] = visibility_stats
    require(update['fusion'] == counts,'exact independently reconstructed raster/fusion categories')
    return result,support,dict(iterations=len(update['iterations']),final_equations=1,
                              model_replaced=update['keyframe_replaced'])
