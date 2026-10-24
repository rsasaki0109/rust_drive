// Shared production scene and spawn setup for the UI and render-free studies.
import { createCity, Traffic, phase } from './city.js';
import { createRidge } from './ridge.js';
import { createStructures } from './structures.js';
import { createUrbanDistrict } from './urban-city.js';
import { FLEET_STOPS } from './fleet.js';

export const FLEET_OBSERVER = Object.freeze({ x: -1000, y: -1000, height: 0, yaw: 0 });
export function createFleetWorld(scene, drive) {
  const city = createCity(scene, { fleet: true });
  createRidge(scene, drive);
  const structures = createStructures(scene, drive);
  const urban = createUrbanDistrict(scene, drive, phase);
  const instrumentColliders = [...city.colliders, ...structures.colliders, ...urban.colliders,
    { x: -25, y: -24, w: 9, h: 10, bottom: 0, top: 14 },
    { x: -31, y: 76, w: 6, h: 12, bottom: 0, top: 13 }];
  const traffic = new Traffic(scene, { drive, urban: true, cityCars: 14 });
  return { city, structures, urban, instrumentColliders, traffic };
}
export function spawnFleetCab(traffic, index, arriving = false) {
  const stop = FLEET_STOPS[index % FLEET_STOPS.length];
  const from = arriving ? { x: -150, y: -150 } : stop.from;
  const to = arriving ? { x: -100, y: -150 } : stop.to;
  const car = traffic.spawn(false, 0xffc34b, { ...from, to,
    advance: arriving ? 2 + index % 4 * 7 : 7 + index % 7 * 2, variant: 'taxi', serviceStop: stop });
  car.desired = 8 + index % 3;
  return car;
}
