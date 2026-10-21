"""Read world-coordinate observations in their logged continuous tracking frame.

Historical outputs without the optional transform already tracked in world
coordinates. This only changes coordinates; it never renews observation times.
"""
import math


def continuous_observations(output,pose_sigma=None):
    if pose_sigma is not None:
        from pose_forecast_oracle import nominal_output
        output=nominal_output(output,pose_sigma)
    transform = output.get('tracking_frame')
    if transform is None:
        return output['tracks'], output['predictions']
    x, y = transform['position']['x'], transform['position']['y']
    yaw = transform['yaw']
    if not all(math.isfinite(v) for v in [x, y, yaw]):
        raise ValueError('nonfinite continuous-to-world tracking transform')
    c, s = math.cos(yaw), math.sin(yaw)

    def point(p, translated):
        dx = p['x'] - (x if translated else 0.)
        dy = p['y'] - (y if translated else 0.)
        return {'x': c*dx+s*dy, 'y': -s*dx+c*dy}

    tracks = [dict(t, position=point(t['position'], True), velocity=point(t['velocity'], False))
              for t in output['tracks']]
    predictions = [dict(p, positions=[point(q, True) for q in p['positions']])
                   for p in output['predictions']]
    return tracks, predictions
