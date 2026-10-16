"""Shared depth slopes using measured in-range sides without crossing jumps."""
import numpy as np

from depth_slope_factors import limited,sample


def field(layer):
    depth = layer['depth'];center = depth[2:-2,2:-2];h,w = center.shape
    present = (depth>.3)&(depth<5);rho = np.zeros_like(depth);np.divide(1.,depth,out=rho,where=present)
    valid = present[2:-2,2:-2].copy();result = dict(du=np.zeros_like(depth),dv=np.zeros_like(depth))
    for key,low,high in (('du',(2,1),(2,3)),('dv',(1,2),(3,2))):
        ly,lx = low;hy,hx = high;lo = depth[ly:ly+h,lx:lx+w];hi = depth[hy:hy+h,hx:hx+w]
        left = present[ly:ly+h,lx:lx+w]&(abs(lo-center)<.05+.01*center)
        right = present[hy:hy+h,hx:hx+w]&(abs(hi-center)<.05+.01*center)
        a = rho[2:-2,2:-2]-rho[ly:ly+h,lx:lx+w];b = rho[hy:hy+h,hx:hx+w]-rho[2:-2,2:-2]
        slope = limited(a,b);slope[left&~right] = a[left&~right];slope[right&~left] = b[right&~left]
        result[key][2:-2,2:-2] = slope;valid &= left|right
    support = np.zeros_like(depth,dtype=bool);support[2:-2,2:-2] = valid;result['valid'] = support
    for key in ('du','dv'):result[key][~support] = 0.
    return result


def prepare(nodes):
    return [dict(node,surface={s:dict(layer,depth_slopes=field(layer)) for s,layer in node['surface'].items()}) for node in nodes]
