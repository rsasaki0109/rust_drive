"""Compare historical geometry reports across finite floating-point backends.

Source/protocol freezes are checked separately and remain exact. Discrete
results, clocks and configuration values must also match exactly here.
"""
import math
import sys
import hashlib
from pathlib import Path

EPSILON_MULTIPLE = 8


def compare_recorded(recorded, reproduced):
    proof = dict(exact=True, rounded_values=0, maximum_absolute_difference=0.,
                 epsilon_multiple=EPSILON_MULTIPLE, clocks_and_configuration_exact=True,
                 comparison_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())

    def visit(left, right, path):
        label = '/'.join(path) or '<root>'
        if type(left) is not type(right):
            raise ValueError('historical result type changed: ' + label)
        if isinstance(left, dict):
            if left.keys() != right.keys():
                raise ValueError('historical result fields changed: ' + label)
            for name in left:
                visit(left[name], right[name], path + [name])
        elif isinstance(left, list):
            if len(left) != len(right):
                raise ValueError('historical result coverage changed: ' + label)
            for index, (a, b) in enumerate(zip(left, right)):
                visit(a, b, path + [str(index)])
        elif isinstance(left, float):
            if not math.isfinite(left) or not math.isfinite(right):
                raise ValueError('nonfinite historical result: ' + label)
            if left == right:
                return
            exact = any('time' in p or 'stamp' in p or p in
                        {'dt', 'age_s', 'delay_s', 'period_s', 'freeze', 'protocol',
                         'config', 'configuration', 'calibration', 'gates', 'policy',
                         'settings', 'parameters'}
                        or p.endswith(('_config', '_policy', '_gates', '_calibration',
                                       '_settings', '_parameters')) for p in path)
            limit = EPSILON_MULTIPLE * sys.float_info.epsilon * max(1., abs(left), abs(right))
            delta = abs(left - right)
            if exact or delta > limit:
                raise ValueError(f'historical numeric result changed: {label}: '
                                 f'{left!r} -> {right!r} (roundoff bound {limit!r})')
            proof['exact'] = False
            proof['rounded_values'] += 1
            proof['maximum_absolute_difference'] = max(proof['maximum_absolute_difference'], delta)
        elif left != right:
            raise ValueError(f'historical discrete result changed: {label}: {left!r} -> {right!r}')

    visit(recorded, reproduced, [])
    return proof
