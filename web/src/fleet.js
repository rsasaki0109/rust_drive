import { makeStop, pickupDistance, atServiceStop } from './fleet-routing.js';

// This is authored demand in a four-minute arcade scenario, not a population
// model fitted to travel surveys. All displayed metrics come from these agents.
export const FLEET_STOPS = Object.freeze([
  makeStop('office', 'CIVIC OFFICES', { x: -50, y: 0 }, { x: 0, y: 0 }),
  makeStop('market', 'MARKET', { x: 0, y: 0 }, { x: 50, y: 0 }),
  makeStop('homes', 'EAST HOMES', { x: 50, y: 0 }, { x: 50, y: 50 }),
  makeStop('clinic', 'CLINIC', { x: 50, y: 50 }, { x: 50, y: 100 }),
  makeStop('station', 'NORTH STATION', { x: 50, y: 100 }, { x: 0, y: 100 }),
  makeStop('hotel', 'HOTEL', { x: 0, y: 100 }, { x: -50, y: 100 }),
  makeStop('campus', 'CAMPUS', { x: -50, y: 100 }, { x: -50, y: 50 }),
  makeStop('west', 'WEST HOMES', { x: -50, y: 50 }, { x: -50, y: 0 }),
]);
export const PURPOSES = Object.freeze([
  { id: 'commute', label: 'COMMUTE', color: '#72c8ff', origins: [2, 7], destinations: [0, 4], speed: 1.45, patience: 65, dwell: 3, item: 'backpack' },
  { id: 'shopping', label: 'SHOPPING', color: '#edabf5', origins: [1, 2, 7], destinations: [1, 2, 7], speed: 1.05, patience: 95, dwell: 5, item: 'bag' },
  { id: 'connection', label: 'TRAIN', color: '#ffbc72', origins: [5, 6, 2], destinations: [4], speed: 1.3, patience: 32, dwell: 6, item: 'suitcase' },
  { id: 'appointment', label: 'APPOINTMENT', color: '#9be0b4', origins: [2, 7, 5], destinations: [3], speed: .85, patience: 110, dwell: 7, item: 'bag' },
  { id: 'leisure', label: 'MEETUP', color: '#f3df7e', origins: [4, 6], destinations: [1, 5, 7], speed: 1.2, patience: 80, dwell: 4, item: 'backpack' },
]);
const byStop = new Map(FLEET_STOPS.map(s => [s.id, s]));
const byPurpose = new Map(PURPOSES.map(p => [p.id, p]));

export function createDemand(seed = 1701, count = 72) {
  let value = seed >>> 0;
  const random = () => { value = (Math.imul(value, 1664525) + 1013904223) >>> 0; return value / 4294967296; };
  const pick = list => list[Math.floor(random() * list.length)];
  return Array.from({ length: count }, (_, i) => {
    // Morning commuters, a train-arrival pulse, then mixed individual trips.
    const purpose = i < 8 ? PURPOSES[i % PURPOSES.length] : i >= 18 && i < 25 ? PURPOSES[4] : pick(PURPOSES);
    const origin = i >= 18 && i < 25 ? 4 : pick(purpose.origins);
    const destination = pick(purpose.destinations.filter(s => s !== origin));
    return {
      id: 'P' + String(i + 1).padStart(2, '0'), purpose: purpose.id,
      origin: FLEET_STOPS[origin].id, destination: FLEET_STOPS[destination].id,
      release: i < 12 ? i * 1.65 : i < 18 ? 25 + (i - 12) * 3 : i < 25 ? 48 + (i - 18) * .8 : 62 + (i - 25) * 3.7 + random() * 2,
      walkSpeed: purpose.speed * (.88 + random() * .24),
      patience: purpose.patience * (.8 + random() * .4),
      dwell: purpose.dwell + random() * 1.5,
      offset: (random() < .5 ? -1 : 1) * (7 + random() * 4), queueOffset: (i % 7 - 3) * 1.3,
    };
  }).sort((a, b) => a.release - b.release || a.id.localeCompare(b.id));
}

function walk(person, target, dt) {
  const dx = target.x - person.x, dy = target.y - person.y, distance = Math.hypot(dx, dy);
  const step = Math.min(distance, person.walkSpeed * dt);
  if (distance > .001) { person.x += dx / distance * step; person.y += dy / distance * step; person.yaw = Math.atan2(dy, dx); person.walked += step; }
  return distance <= person.walkSpeed * dt + .001;
}

export class TaxiFleet {
  constructor(graph, cars, { seed = 1701, plans = createDemand(seed) } = {}) {
    this.graph = graph; this.seed = seed; this.time = 0; this.nextDispatch = 0; this.events = []; this.waits = [];
    this.taxis = cars.map((car, i) => ({ id: 'T' + String(i + 1).padStart(2, '0'), car, state: 'idle', person: null, capacity: 1, served: 0 }));
    this.people = [...plans].sort((a, b) => a.release - b.release || a.id.localeCompare(b.id)).map(plan => {
      const origin = byStop.get(plan.origin), destination = byStop.get(plan.destination);
      if (!origin || !destination || origin === destination || !byPurpose.has(plan.purpose) ||
          ![plan.release, plan.walkSpeed, plan.patience, plan.dwell, plan.offset].every(Number.isFinite) ||
          plan.release < 0 || plan.walkSpeed <= 0 || plan.patience <= 0 || plan.dwell <= 0) throw new Error('Invalid individual travel plan: ' + plan.id);
      return { ...plan, origin, destination, state: 'scheduled', taxi: null, walked: 0,
        x: origin.curb.x + origin.dx * plan.offset, y: origin.curb.y + origin.dy * plan.offset, yaw: 0,
        waitPoint: { x: origin.curb.x + origin.dx * (plan.queueOffset ?? 0), y: origin.curb.y + origin.dy * (plan.queueOffset ?? 0) } };
    });
    if (new Set(this.people.map(p => p.id)).size !== this.people.length) throw new Error('Duplicate person ID');
  }
  event(type, person, taxi = null, extra = {}) {
    this.events.push({ time: this.time, type, person: person.id, taxi: taxi?.id ?? null, purpose: person.purpose, ...extra });
  }
  update(dt) {
    if (!Number.isFinite(dt) || dt <= 0 || dt > .1) throw new Error('Fleet step must be in (0, .1] seconds');
    this.time += dt;
    for (const p of this.people) {
      if (p.state === 'scheduled' && this.time >= p.release) { p.state = 'walking'; this.event('request', p); }
      if (['walking', 'waiting'].includes(p.state) && this.time - p.release > p.patience) {
        const taxi = this.taxis.find(t => t.id === p.taxi);
        if (taxi) { taxi.state = 'idle'; taxi.person = null; /* Drive to the last assigned bay before idling. */ }
        p.taxi = null; p.state = 'cancelled'; p.finishedAt = this.time; this.event('cancel', p, taxi);
      }
      if (p.state === 'walking' && walk(p, p.waitPoint, dt)) { p.state = 'waiting'; this.event('curb', p); }
      if (p.state === 'leaving' || p.state === 'cancelled') {
        const stop = p.state === 'leaving' ? p.destination : p.origin;
        if (walk(p, { x: stop.curb.x + stop.dx * p.offset, y: stop.curb.y + stop.dy * p.offset }, dt) && p.state === 'leaving') {
          p.state = 'arrived'; p.finishedAt = this.time; this.event('arrived', p);
        }
      }
    }
    if (this.time >= this.nextDispatch) {
      this.nextDispatch = this.time + .5;
      // Oldest requests first; choose a free taxi by legal road distance.
      for (const p of this.people.filter(p => ['walking', 'waiting'].includes(p.state) && !p.taxi)) {
        const choices = this.taxis.filter(t => t.state === 'idle').map(t => ({ t, distance: pickupDistance(this.graph, t.car, p.origin) })).filter(c => Number.isFinite(c.distance));
        choices.sort((a, b) => a.distance - b.distance || a.t.id.localeCompare(b.t.id));
        const taxi = choices[0]?.t;
        if (!taxi) break;
        taxi.person = p; taxi.state = 'pickup'; taxi.car.serviceStop = p.origin; p.taxi = taxi.id;
        this.event('assign', p, taxi, { distance: choices[0].distance });
      }
    }
    for (const taxi of this.taxis) {
      const p = taxi.person, car = taxi.car;
      if (!p) continue;
      if (taxi.state === 'pickup' && p.state === 'waiting' && atServiceStop(car, p.origin)) {
        taxi.state = 'boarding'; p.state = 'boarding'; p.dwellUntil = this.time + p.dwell;
        p.boardAt = this.time; this.waits.push(this.time - p.release);
        this.event('board', p, taxi, { x: car.x, y: car.y, speed: car.speed, stop: p.origin.id });
      }
      if (taxi.state === 'boarding' && atServiceStop(car, p.origin)) {
        const door = { x: car.x + p.origin.normal.x * 1.15, y: car.y + p.origin.normal.y * 1.15 };
        if (walk(p, door, dt) && this.time >= p.dwellUntil) {
          taxi.state = 'occupied'; p.state = 'riding'; car.serviceStop = p.destination;
          this.event('depart', p, taxi);
        }
      } else if (taxi.state === 'occupied') {
        p.x = car.x; p.y = car.y;
        if (atServiceStop(car, p.destination)) {
          taxi.state = 'alighting'; p.state = 'alighting'; p.dwellUntil = this.time + p.dwell * .7;
          this.event('alight', p, taxi, { x: car.x, y: car.y, speed: car.speed, stop: p.destination.id });
        }
      } else if (taxi.state === 'alighting' && atServiceStop(car, p.destination)) {
        if (walk(p, p.destination.curb, dt) && this.time >= p.dwellUntil) {
          p.state = 'leaving'; p.taxi = null; taxi.state = 'idle'; taxi.person = null; taxi.served++;
          this.event('complete', p, taxi);
        }
      }
    }
  }
  snapshot() {
    const count = states => this.people.filter(p => states.includes(p.state)).length;
    return {
      time: this.time, seed: this.seed, demand: 'authored individual travel plans; not calibrated',
      metrics: {
        requested: this.events.filter(e => e.type === 'request').length,
        waiting: count(['walking', 'waiting']), riding: count(['riding']),
        served: this.taxis.reduce((n, t) => n + t.served, 0),
        cancelled: this.events.filter(e => e.type === 'cancel').length,
        meanWait: this.waits.length ? this.waits.reduce((a, b) => a + b, 0) / this.waits.length : 0,
      },
      taxis: this.taxis.map(t => ({ id: t.id, state: t.state, person: t.person?.id ?? null, served: t.served,
        x: t.car.x, y: t.car.y, speed: t.car.speed, distance: t.car.distance,
        destination: t.car.serviceStop?.id ?? null, capacity: t.capacity })),
      people: this.people.filter(p => p.state !== 'scheduled').map(p => ({ id: p.id, purpose: p.purpose,
        origin: p.origin.id, destination: p.destination.id, state: p.state, taxi: p.taxi, x: p.x, y: p.y,
        walked: p.walked, walkSpeed: p.walkSpeed, patience: p.patience, release: p.release })),
      events: this.events.map(e => ({ ...e })),
    };
  }
}
