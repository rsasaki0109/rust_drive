"""Independent array morphology and k-d tree graph connectivity references for CADC XYZ."""
import math
import struct
from cadc_recorded import require,MAX_MEMBER

def crop(data,profile):
    import numpy as np
    require(data and len(data)%16==0 and len(data)<=MAX_MEMBER,'invalid recorded XYZI rows')
    rows=[];points=[];invalid=0
    for i,(x,y,z,intensity) in enumerate(struct.iter_unpack('<ffff',data)):
        if not all(math.isfinite(v) for v in (x,y,z)):invalid+=1;continue
        if profile['min_range']<=math.hypot(x,y)<=profile['max_range'] and profile['z_min']<=z<=profile['z_max']:
            rows.append(i);points.append((x,y,z))
    return {'rows':rows,'points':np.asarray(points,dtype=float).reshape((-1,3)),
            'total_rows':len(data)//16,'invalid_xyz_rows':invalid}

def ground_reference(points,cfg):
    import numpy as np
    from scipy.ndimage import minimum_filter1d,maximum_filter1d
    points=np.asarray(points,dtype=float).reshape((-1,3))
    require(len(points)<=cfg['max_points'] and np.isfinite(points).all(),'invalid terrain points')
    require(not len(points) or np.abs(points).max()<=cfg['coordinate_bound_m'],'terrain coordinate bounds')
    if not len(points):
        return [],{'occupied_cells':0,'candidate_ground_cells':0,'supported_cells':0,'rejected_cells':0,
                   'candidate_work':0,'supported_fraction':0.,'confident':False}
    keys=np.floor(points[:,:2]/cfg['cell_size_m']).astype('int64')
    low=keys.min(axis=0);offset=keys-low;shape=tuple((offset.max(axis=0)+1).tolist())
    require(math.prod(shape)<=4_000_000,'independent terrain raster bound exceeded')
    original=np.full(shape,np.inf);np.minimum.at(original,(offset[:,0],offset[:,1]),points[:,2])
    occupied=np.isfinite(original);n=int(occupied.sum())
    require(n<=cfg['max_cells'],'terrain occupied cell bound')
    estimate=n*(16+sum(4*(2*r+1) for r in cfg['window_radii_cells']))
    require(estimate<=cfg['max_candidate_work'],'terrain candidate work bound')
    original=np.where(occupied,original,np.nan)
    def support(mask):
        heights=np.pad(original,1,constant_values=np.nan);valid=np.pad(mask,1,constant_values=False)
        count=np.zeros(shape,dtype='int16')
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                if dx==dy==0:continue
                nearby=heights[1+dx:1+dx+shape[0],1+dy:1+dy+shape[1]]
                present=valid[1+dx:1+dx+shape[0],1+dy:1+dy+shape[1]]
                threshold=cfg['initial_height_m']+cfg['max_slope']*cfg['cell_size_m']*math.hypot(dx,dy)
                count+=present & (np.abs(original-nearby)<=threshold)
        return count
    active=occupied & (support(occupied)>=cfg['min_support_neighbors'])
    candidate=active.copy();field=original.copy();previous=0
    def extreme(values,radius,axis,minimum):
        fill=np.inf if minimum else -np.inf
        method=minimum_filter1d if minimum else maximum_filter1d
        filtered=method(np.where(active,values,fill),size=2*radius+1,axis=axis,mode='constant',cval=fill)
        return np.where(active,filtered,np.nan)
    for radius in cfg['window_radii_cells']:
        eroded=extreme(extreme(field,radius,0,True),radius,1,True)
        opened=extreme(extreme(eroded,radius,0,False),radius,1,False)
        threshold=min(cfg['max_height_m'],cfg['initial_height_m']+cfg['max_slope']*cfg['cell_size_m']*2*(radius-previous))
        candidate &= ~(field-opened>threshold)
        field=np.where(active,np.minimum(field,opened),np.nan);previous=radius
    supported=candidate & (support(candidate)>=cfg['min_support_neighbors'])
    accepted=int(supported.sum());count=int(candidate.sum());fraction=accepted/n
    threshold=cfg['initial_height_m']+cfg['max_slope']*cfg['cell_size_m']*math.sqrt(2)
    ground=supported[offset[:,0],offset[:,1]] & (np.abs(points[:,2]-original[offset[:,0],offset[:,1]])<=threshold)
    work=16*n+int(active.sum())*sum(4*(2*r+1) for r in cfg['window_radii_cells'])
    return np.flatnonzero(ground).tolist(),{
        'occupied_cells':n,'candidate_ground_cells':count,'supported_cells':accepted,'rejected_cells':n-count,
        'candidate_work':work,'supported_fraction':fraction,
        'confident':accepted>=cfg['min_supported_cells'] and fraction>=cfg['min_supported_fraction']}

def xyz_reference(points,indices,cfg,proxy):
    import numpy as np
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    points=np.asarray(points,dtype=float).reshape((-1,3));ids=np.asarray(indices,dtype=int)
    require(len(points)<=cfg['max_points'] and len(set(indices))==len(indices),'invalid XYZ input')
    if not len(ids):return [],[]
    values=points[ids];tree=cKDTree(values);tolerance=cfg['tolerance_m']
    require(tree.count_neighbors(tree,math.nextafter(tolerance,math.inf))<=16_000_000,'independent connectivity edge bound')
    pairs=tree.query_pairs(math.nextafter(tolerance,math.inf),output_type='ndarray')
    if len(pairs):
        pairs=pairs[((values[pairs[:,0]]-values[pairs[:,1]])**2).sum(axis=1)<=tolerance*tolerance]
    graph=coo_matrix((np.ones(len(pairs),dtype=bool),(pairs[:,0],pairs[:,1])),shape=(len(ids),len(ids))).tocsr()
    _,labels=connected_components(graph,directed=False)
    groups=[sorted(ids[labels==label].tolist()) for label in np.unique(labels)]
    groups.sort(key=lambda group:group[0]);objects=[];noise=[]
    for group in groups:
        if len(group)<cfg['min_points']:noise.extend(group);continue
        require(len(group)<=cfg['max_cluster_points'] and len(objects)<cfg['max_clusters'],'XYZ component bound')
        minimum=points[group].min(axis=0);maximum=points[group].max(axis=0);centre=(minimum+maximum)*.5
        radius=min(proxy['max_radius_m'],max(proxy['min_radius_m'],
            math.hypot(*(maximum-minimum)[:2])*.5+proxy['padding_m']))
        xyz=lambda p:{k:float(v) for k,v in zip(('x','y','z'),p)}
        objects.append({'min':xyz(minimum),'max':xyz(maximum),'indices':group,'point_count':len(group),
            'proxy':{'center':{'x':float(centre[0]),'y':float(centre[1])},'radius':radius}})
    return objects,sorted(noise)
