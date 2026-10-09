#!/usr/bin/env python3
"""Generate explicitly surveyed synthetic priors before any simulation is run.

This prior is an authored offline map of fixed landmark surfaces, not a map built
from a simulator's live poses or sensor labels. The matching pipeline receives
only the fixed x/y surface points and measured body-frame LiDAR observations.
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LANDMARKS = [(6, 5, .9), (13, -6, 1.2), (22, 4.5, .7),
             (30, -5, 1), (40, 7, 1.4), (47, -4.5, .8)]


def fixtures():
    points = [{'x': x+r*math.cos(i*math.tau/360),
               'y': y+r*math.sin(i*math.tau/360)}
              for x, y, r in LANDMARKS for i in range(360)]
    for case in ['map-gnss-recovery', 'map-gnss-prolonged',
                 'map-gnss-no-overlap', 'map-gnss-degenerate',
                 'map-gnss-lidar-loss', 'map-gnss-disabled']:
        scenario = {'name': case.replace('-', ' ').title(), 'duration': 25,
                    'road_length': 50, 'half_width': 2.1, 'expected': 'goal',
                    'cruise_speed': 3, 'min_clearance_m': 1.0,
                    'objects': [{'s': x, 'lateral': y, 'radius': r}
                                for x, y, r in LANDMARKS],
                    'localization_map': {'points': points, 'max_gnss_outage_s': 10.0},
                    'gnss_dropout_windows': [{'from': 3, 'until': 8}]}
        if case == 'map-gnss-prolonged':
            scenario.update(duration=18, road_length=100, expected='fault', gnss_dropout=3)
            scenario.pop('gnss_dropout_windows')
        elif case == 'map-gnss-no-overlap':
            scenario.update(duration=8, road_length=50, expected='fault')
            scenario['localization_map']['points'] = [{'x': p['x']+200, 'y': p['y']} for p in points]
        elif case == 'map-gnss-degenerate':
            scenario.update(duration=8, road_length=50, expected='fault')
            scenario['localization_map']['points'] = [{'x': i*.05, 'y': 5.0} for i in range(1101)]
        elif case == 'map-gnss-lidar-loss':
            scenario.update(duration=10, expected='fault', lidar_dropout=6)
            scenario['gnss_dropout_windows'] = [{'from': 3, 'until': 10}]
        elif case == 'map-gnss-disabled':
            scenario.update(duration=8, expected='fault')
            scenario.pop('localization_map')
        yield case, scenario


def main():
    for case, scenario in fixtures():
        (ROOT/'scenarios'/f'{case}.json').write_text(json.dumps(scenario, separators=(',', ':'))+'\n')


if __name__ == '__main__':
    main()
