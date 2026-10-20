"""Independent disc-containment checks for declared motion error budgets."""
import math


def validate(bounds):
    limits = {'position_error_m': 20., 'velocity_error_m_s': 15., 'acceleration_error_m_s2': 6.}
    if not isinstance(bounds, dict) or set(bounds) != set(limits) or any(
            isinstance(bounds[k], bool) or not isinstance(bounds[k], (int, float))
            or not math.isfinite(bounds[k]) or not 0 <= bounds[k] <= maximum
            for k, maximum in limits.items()):
        raise ValueError('invalid declared forecast error budgets')


def error_at(bounds, elapsed):
    return (bounds['position_error_m'] + elapsed*bounds['velocity_error_m_s']
            + elapsed**2*bounds['acceleration_error_m_s2']/2)


def audit_envelope(bounds, track, now, forecast, nominal, nominal_extra):
    """Check containment and the optimal enclosing radius, without constructing its center.

    The driver is free to choose a center only when both input discs fit inside
    the mathematically smallest possible enclosing disc. The checks also imply
    inter-knot containment of the affine nominal path and convex CV error tube.
    """
    validate(bounds)
    radii = forecast.get('additional_radius_m', [])
    if len(radii) != len(nominal) or len(forecast['positions']) != len(nominal):
        raise ValueError('forecast envelope must retain every sampled radius and position')
    age = now-track['last_seen']
    if not math.isfinite(age) or age < 0:
        raise ValueError('invalid forecast acquisition age')
    for i, (point, old) in enumerate(zip(forecast['positions'], nominal)):
        t = age+i*forecast['dt']
        cv = tuple(track['position'][axis]+track['velocity'][axis]*t for axis in ['x', 'y'])
        center = (point['x'], point['y'])
        previous = nominal_extra[i] if nominal_extra else 0.
        error = error_at(bounds, t)
        radius = radii[i]
        minimum = max(previous, error, (math.dist(old, cv)+previous+error)/2)
        if (not all(math.isfinite(v) for v in [*center, radius]) or radius < 0
                or abs(radius-minimum) > 1e-8
                or math.dist(center, old)+previous > radius+1e-8
                or math.dist(center, cv)+error > radius+1e-8):
            raise ValueError('motion envelope lost an alternative, reset its age or changed its error budget')
