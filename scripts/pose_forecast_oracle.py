"""Independent pose-ellipsoid projection using Cholesky factors, no actor truth."""
import math


def validate_sigma(value):
    if type(value) not in [int,float] or not math.isfinite(value) or not 1.<=value<=5.:
        raise ValueError('invalid pose forecast sigma')


def factor(matrix):
    if (not isinstance(matrix,list) or len(matrix)!=3 or any(not isinstance(row,list) or len(row)!=3 for row in matrix)
            or any(type(v) not in [int,float] or not math.isfinite(v) for row in matrix for v in row)):
        raise ValueError('invalid pose covariance')
    p=[list(row) for row in matrix];l=[[0.]*3 for _ in range(3)]
    for i in range(3):
        if p[i][i]<0.:raise ValueError('negative pose variance')
        for j in range(i):
            scale=math.sqrt(p[i][i])*math.sqrt(p[j][j])
            if abs(p[i][j]-p[j][i])>1e-10*scale:raise ValueError('asymmetric pose covariance')
            p[i][j]=p[j][i]=(p[i][j]+p[j][i])/2
    for i in range(3):
        for j in range(i+1):
            residual=p[i][j]-math.fsum(l[i][k]*l[j][k] for k in range(j))
            if i==j:
                if residual<0.:raise ValueError('indefinite pose covariance')
                l[i][j]=math.sqrt(residual)
            elif l[j][j]:l[i][j]=residual/l[j][j]
            elif residual!=0.:raise ValueError('inconsistent singular pose covariance')
    return l


def padding(position,pose,covariance,sigma):
    validate_sigma(sigma)
    return projected_padding(position,pose,covariance,sigma,factor(covariance))


def projected_padding(position,pose,covariance,sigma,l):
    q=[position[k]-pose['position'][k] for k in ['x','y']]
    if not all(math.isfinite(v) for v in q+[pose['yaw']]):raise ValueError('nonfinite pose forecast reference')
    projected=[[l[0][k]-q[1]*l[2][k] for k in range(3)],
               [l[1][k]+q[0]*l[2][k] for k in range(3)]]
    diagonal=[math.fsum(v*v for v in row) for row in projected]
    cross=math.fsum(a*b for a,b in zip(*projected))
    major=(sum(diagonal)+math.hypot(diagonal[0]-diagonal[1],2*cross))/2
    radius=sigma*math.sqrt(max(0.,major))+math.hypot(*q)*sigma*sigma*covariance[2][2]/2
    if not math.isfinite(radius):raise ValueError('pose forecast overflow')
    return radius


def nominal_output(output,sigma):
    """Remove only the independently computed post-guard world-pose margin.

    Existing nominal-motion checks must still verify every remaining radius.
    Localization reconstruction separately verifies the covariance's inputs.
    """
    validate_sigma(sigma);p=output.get('pose_covariance');l=factor(p)
    predictions=[]
    for forecast in output['predictions']:
        extra=forecast.get('additional_radius_m',[])
        if len(extra)!=len(forecast['positions']):raise ValueError('pose forecast lost per-knot radii')
        corrected=[]
        for point,radius in zip(forecast['positions'],extra):
            margin=projected_padding(point,output['estimate']['pose'],p,sigma,l)
            residual=radius-margin
            if not math.isfinite(residual) or residual < -1e-9:raise ValueError('pose forecast omits required occupancy')
            corrected.append(max(0.,residual))
        predictions.append(dict(forecast,additional_radius_m=corrected))
    return dict(output,predictions=predictions)


def reconstruct(config,ticks):
    sigma=config.get('forecast_pose_sigma');validate_sigma(sigma)
    if config.get('forecast_envelope') is None or config.get('absolute_heading_fusion') is not True:
        raise ValueError('pose forecast prerequisites missing')
    from check_heading_history import reconstruct as localization
    posterior=localization(config,ticks)
    maximum=0.;knots=0
    for tick in ticks:
        output=tick['expected'];nominal_output(output,sigma);l=factor(output['pose_covariance'])
        for forecast in output['predictions']:
            for point in forecast['positions']:
                maximum=max(maximum,projected_padding(point,output['estimate']['pose'],output['pose_covariance'],sigma,l));knots+=1
    return dict(passed=True,verified_ticks=len(ticks),verified_knots=knots,sigma=sigma,
        maximum_added_radius_m=maximum,localization=posterior,
        continuous_guard_uses_unexpanded_nominal=True,actor_truth_used=False,
        confidence_calibrated=False)
