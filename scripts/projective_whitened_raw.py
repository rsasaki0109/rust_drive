"""Independent standardized-Huber equations for the provisional axial variance."""
import numpy as np
from projective_range_raw import pairs,coefficient


def equation(target,source,transform,scale):
    samples=pairs(target,source,transform,scale)
    r=samples['residual'];j=samples['jacobian']
    relative=coefficient(samples['source'][:,2],samples['target'][:,2])
    standardized=r*np.sqrt(relative)
    # Direct robust weight; Rust instead corrects the existing Huber
    # accumulator by a ratio of robust multipliers.
    weight=relative*np.minimum(1.,.01/np.maximum(abs(standardized),1e-15))
    return dict(count=len(r),rms_m=float(np.sqrt(np.mean(r**2))) if len(r) else 0.,
                h=j.T@(weight[:,None]*j),g=j.T@(weight*r))


def controls():
    residual=np.array([0.,.005,.02,.04,.08])
    relative=coefficient(2.,2.)
    actual=relative*np.minimum(1.,.01/np.maximum(abs(residual)*np.sqrt(relative),1e-15))
    np.testing.assert_allclose(actual,[1/16,1/16,1/16,1/16,1/32],atol=0,rtol=1e-15)
    unit=coefficient(.5,1.)
    assert unit==1.
    return dict(standardized_quadratic_region=True,standardized_robust_tail=True,
                unit_variance_retains_original_huber=True,measured_noise_calibration=False)
