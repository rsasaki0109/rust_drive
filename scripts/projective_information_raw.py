"""Independent metre/radian observability; a structural guard, not covariance."""
import numpy as np
from projective_depth_common import audit_direction_support,pose,require


def combined(e,p=None):
    h=np.asarray(e['h'],dtype=float)
    if p is not None and p['used']:h=h+p['scale_m2_per_gray2']*np.asarray(p['h'])
    return h


def spectrum(h):
    require(h.shape==(6,6) and np.isfinite(h).all(),'finite physical information')
    scale=float(np.diag(h).max())
    if scale<=0:return None
    eigen=np.linalg.eigvalsh(h/scale)
    return dict(translation_scale_m=1.,rotation_scale_rad=1.,matrix_scale=scale,eigenvalues=eigen,
        condition=float(eigen[-1]/eigen[0]) if eigen[0]>0 else None)


def admitted(s):return s is not None and s['condition'] is not None and s['condition']<=1e6


def compare(actual,expected):
    if expected is None:require(actual is None,'no fabricated physical spectrum');return
    require(actual is not None and actual.keys()==expected.keys(),'complete physical spectrum')
    for key in ('translation_scale_m','rotation_scale_rad','matrix_scale'):require(actual[key]==expected[key],'fixed physical spectrum units/scale')
    np.testing.assert_allclose(actual['eigenvalues'],expected['eigenvalues'],atol=2e-13,rtol=1e-11)
    if expected['eigenvalues'][0]>1e-12:np.testing.assert_allclose(actual['condition'],expected['condition'],atol=1e-7,rtol=1e-7)
    else:require(actual['condition'] is None or actual['condition']>1e6,'numerically singular information cannot authorize motion')


def audit(update):
    for step in update['iterations']:
        audit_direction_support(step['equation'])
        require(admitted(spectrum(combined(step['equation'],step.get('photometric')))),'every admitted solve has physical directional support')
    if update['accepted'] and update['iterations']:
        audit_direction_support(update['final_equation'])
        require(admitted(spectrum(combined(update['final_equation'],update.get('final_photometric')))),'returned pose has physical directional support')
    f=update.get('information_failure')
    if update['rejection']=='insufficient physical observability condition':require(f is not None,'required observability failure witness')
    if f is None:return
    audit_direction_support(f['equation'])
    require(not update['accepted'] and update['fusion'] is None and update['first_from_current'] is None,'weak information never commits state')
    require(update['rejection']=='insufficient physical observability condition' and f['scale'] in (1,2,4) and type(f['final_pose']) is bool and f['equation']['count']>=200,'bounded failure stage/support')
    s=spectrum(combined(f['equation'],f['photometric']));compare(f['spectrum'],s)
    require(not admitted(s),'observability rejection has independently weak information')
    if update['iterations']:np.testing.assert_allclose(pose(f['pose']),pose(update['iterations'][-1]['after']),atol=1e-12,rtol=0)
    if f['final_pose']:require(f['scale']==1 and bool(update['iterations']),'final pose witness follows actual fitting')
