import { makeStop, pickupDistance, atServiceStop } from './fleet-routing.js';
import { CITY_WEATHER, weatherSupply } from './city-weather.js';
import { makeStandbyStop, standbyPriorities } from './fleet-standby.js';
import { cityConditions, FACILITIES, NorthLine, STATION_PATH, PLATFORM_WAIT, RAIL_Y } from './city-life.js';

// Authored agents, not demand calibrated against a real population.
export const FLEET_STOPS = Object.freeze([
  makeStop('office', 'CIVIC OFFICES', { x: -50, y: 0 }, { x: 0, y: 0 }),
  makeStop('market', 'MARKET', { x: 0, y: 0 }, { x: 50, y: 0 }),
  makeStop('homes', 'EAST HOMES', { x: 50, y: 0 }, { x: 50, y: 50 }),
  makeStop('clinic', 'CLINIC', { x: 50, y: 50 }, { x: 50, y: 100 }),
  makeStop('station', 'NORTH STATION', { x: 50, y: 100 }, { x: 0, y: 100 }),
  makeStop('hotel', 'HOTEL', { x: 0, y: 100 }, { x: -50, y: 100 }),
  makeStop('campus', 'CAMPUS', { x: -50, y: 100 }, { x: -50, y: 50 }),
  makeStop('west', 'WEST HOMES', { x: -50, y: 50 }, { x: -50, y: 0 }),
  makeStop('nightlife', 'MIDNIGHT STRIP', { x: 0, y: 50 }, { x: -50, y: 50 }),
]);
export const PURPOSES = Object.freeze([
  { id: 'commute', label: 'COMMUTE', color: '#72c8ff', origins: [0, 2, 7], destinations: [0, 4], speed: 1.45, patience: 65, dwell: 3, item: 'backpack' },
  { id: 'shopping', label: 'SHOPPING', color: '#edabf5', origins: [0, 1, 2, 7], destinations: [1, 2, 7], speed: 1.05, patience: 95, dwell: 5, item: 'bag' },
  { id: 'connection', label: 'TRAIN', color: '#ffbc72', origins: [0, 5, 6, 2], destinations: [4], speed: 1.3, patience: 32, dwell: 6, item: 'suitcase' },
  { id: 'appointment', label: 'APPOINTMENT', color: '#9be0b4', origins: [0, 2, 7, 5], destinations: [3], speed: .85, patience: 110, dwell: 7, item: 'bag' },
  { id: 'leisure', label: 'MEETUP', color: '#f3df7e', origins: [6, 8], destinations: [1, 5, 7, 8], speed: 1.2, patience: 80, dwell: 4, item: 'backpack' },
]);
export const DEPOT_STOPS = Object.freeze(Array.from({ length: 8 }, (_, i) => {
  const x = i < 4 ? -150 : -100;
  const base = makeStop('depot-' + i, 'FLEET DEPOT', { x, y: -150 }, { x: x+50, y: -150 });
  const along = 12 + i % 4 * 8; return Object.freeze({ ...base, along, x: x + along });
}));
const byStop = new Map(FLEET_STOPS.map(s => [s.id, s]));
const byPurpose = new Map(PURPOSES.map(p => [p.id, p]));

export function createDemand(seed = 1701, count = 72, period = 'morning', startSeconds = 0, serialOffset = 0) {
  let value = seed >>> 0, release = startSeconds;
  const random = () => { value = (Math.imul(value, 1664525) + 1013904223) >>> 0; return value / 4294967296; };
  const pick = list => list[Math.floor(random() * list.length)];
  return Array.from({ length: count }, (_, i) => {
    const clock = cityConditions(period, release), officeWave = !clock.night && i >= 12 && i < 18;
    let purpose = i < 8 ? PURPOSES[i % 5] : officeWave ? PURPOSES[(i - 12) % 4] : pick(PURPOSES);
    let origin = pick(purpose.origins), destination = pick(purpose.destinations.filter(s => s !== origin));
    if (clock.night) {
      origin = i === 0 || random() < .7 ? 8 : pick([0, 1, 2, 5, 7]);
      destination = pick([2, 5, 7].filter(s => s !== origin));
      if (clock.railOpen && random() < .12) { purpose = PURPOSES[2]; destination = 4; }
    } else if (i === 0 || officeWave) {
      origin = 0; destination = pick(purpose.destinations.filter(s => s !== origin));
    }
    const departure = officeWave ? startSeconds + 42 + (i - 12) * 2 : release;
    release += clock.night ? 1.4 + random() * 1.7 : 3 + random() * 3;
    return { id: 'P' + String(serialOffset + i + 1).padStart(3, '0'), purpose: purpose.id,
      origin: FLEET_STOPS[origin].id, destination: FLEET_STOPS[destination].id,
      release: departure, walkSpeed: purpose.speed * (.88 + random() * .24),
      patience: purpose.patience * (.8 + random() * .4), dwell: purpose.dwell + random() * 1.5,
      offset: (random() < .5 ? -1 : 1) * (7 + random() * 4) };
  }).sort((a, b) => a.release - b.release || a.id.localeCompare(b.id));
}

function walk(p, target, dt) {
  const dx = target.x - p.x, dy = target.y - p.y, dz = (target.height ?? 0) - p.height;
  const distance = Math.hypot(dx, dy, dz), step = Math.min(distance, p.walkSpeed * dt);
  if (distance > .001) {
    p.x += dx / distance * step; p.y += dy / distance * step; p.height += dz / distance * step;
    p.yaw = Math.atan2(dy, dx); p.walked += step;
  }
  return distance <= p.walkSpeed * dt + .001;
}

export class TaxiFleet {
  constructor(graph, cars, options = {}) {
    const { seed = 1701, period = 'morning', plans, standby = true } = options;
    this.graph = graph; this.seed = seed; this.period = period; this.time = 0; this.nextDispatch = 0;
    this.weather='clear'; this.weatherStrength=0; this.reserveEnabled=true; this.weatherSequence=0; this.nextWeatherDemand=Infinity; this.weatherEvents=[];
    this.standbyEnabled = Boolean(standby); this.nextStandby = 0; this.standbyEvents = []; this.standbyScores = [];
    this.standbyStops = new Map(FLEET_STOPS.map(s => [s.id, makeStandbyStop(s)]));
    this.events = []; this.waits = []; this.people = []; this.taxis = []; this.auto = plans === undefined;
    this.line = this.auto ? new NorthLine(period) : null; this.railCursor = 0; this.demandBatch = 0;
    this.doors = Object.fromEntries(Object.keys(FACILITIES).map(id => [id, { open: 0 }]));
    for (const car of cars) this.addTaxi(car);
    for (const plan of plans ?? createDemand(seed, 72, period)) this.addPerson(plan);
    this.nextDemand = Math.max(0, ...this.people.map(p => p.release)) + 5;
  }
  addTaxi(car) {
    const taxi = { id: 'T' + String(this.taxis.length + 1).padStart(2, '0'), car, state: 'idle',
      person: null, capacity: 1, served: 0, onDuty: true, standbyStop: null, standbySince: 0,
      idleStop: car.serviceStop, lastDistance: car.distance ?? 0, emptyDistance: 0, repositionDistance: 0 };
    this.taxis.push(taxi); return taxi;
  }
  addPerson(plan) {
    const origin = byStop.get(plan.origin), destination = byStop.get(plan.destination), facility = FACILITIES[plan.origin];
    if (!origin || !destination || origin === destination || !byPurpose.has(plan.purpose) ||
        ![plan.release, plan.walkSpeed, plan.patience, plan.dwell, plan.offset].every(Number.isFinite) ||
        plan.release < 0 || plan.walkSpeed <= 0 || plan.patience <= 0 || plan.dwell <= 0) throw new Error('Invalid travel plan: ' + plan.id);
    if (this.people.some(p => p.id === plan.id)) throw new Error('Duplicate person ID: ' + plan.id);
    const p = { ...plan, origin, destination, state: 'scheduled', taxi: null, walked: 0, height: 0,
      leg: 1, requestAt: null, queueSlot: null, insideBuilding: facility?.id ?? null, facility,
      x: facility ? facility.inside.x + (this.people.length % 3 - 1) * .65 : origin.curb.x + origin.dx * plan.offset,
      y: facility ? facility.inside.y : origin.curb.y + origin.dy * plan.offset, yaw: 0,
      waitPoint: { ...origin.curb, height: 0 } };
    this.people.push(p); return p;
  }
  event(type, p, taxi = null, extra = {}) {
    this.events.push({ time: this.time, type, person: p.id, leg: p.leg, taxi: taxi?.id ?? null,
      purpose: p.purpose, x: p.x, y: p.y, height: p.height, ...extra });
  }
  path(p, points, state, facility = null) {
    p.route = points; p.routeIndex = 0; p.state = state; p.facility = facility;
  }
  travel(p, dt) {
    const target = p.route?.[p.routeIndex];
    if (!target) return true;
    const f = p.facility;
    if (f && (p.y - f.door.y) * (target.y - f.door.y) < 0 && this.doors[f.id].open < .92) return false;
    if (p.state === 'exiting' && this.people.some(other => other !== p && other.state === 'exiting' &&
        other.facility === f && Math.hypot(other.x - p.x, other.y - p.y) < .75 &&
        (other.y - p.y) * f.outward > 0)) return false;
    const reached = walk(p, target, dt);
    if (f && Math.abs(p.x - f.door.x) < 1.35) {
      const side = (p.y - f.door.y) * f.outward;
      if (p.insideBuilding && side > .4) { p.insideBuilding = null; this.event('building-exit', p, null, { building: f.id, doorOpen: this.doors[f.id].open }); }
      if (!p.insideBuilding && side < -.4) { p.insideBuilding = f.id; this.event('building-enter', p, null, { building: f.id, doorOpen: this.doors[f.id].open }); }
    }
    if (reached) p.routeIndex++;
    return p.routeIndex === p.route.length;
  }
  request(p) {
    const occupied = new Set(this.people.filter(q => q !== p && q.origin === p.origin &&
      ['walking', 'waiting'].includes(q.state)).map(q => q.queueSlot));
    let slot = 0; while (occupied.has(slot)) slot++;
    p.queueSlot = slot;
    const column = slot % 21, along = (column ? Math.ceil(column / 2) * (column % 2 ? -1 : 1) : 0) * 1.05;
    const row = Math.floor(slot / 21);
    p.waitPoint = { x: p.origin.curb.x + p.origin.dx * along + p.origin.normal.x * row,
      y: p.origin.curb.y + p.origin.dy * along + p.origin.normal.y * row, height: 0 };
    p.requestAt = this.time;
    this.path(p, p.facility ? [p.facility.apron, p.waitPoint] : [p.waitPoint], 'walking');
    this.event('request', p, null, { source: p.source ?? p.origin.id, stop: p.origin.id });
  }
  finishWalk(p) {
    p.state = p.cancelledJourney ? 'cancelled' : 'arrived'; p.finishedAt = this.time;
    this.event(p.cancelledJourney ? 'returned' : 'arrived', p);
  }
  startLastLeg(p, cancelled = false) {
    const stop = cancelled ? p.origin : p.destination, f = FACILITIES[stop.id];
    p.cancelledJourney = cancelled;
    if (f) this.path(p, [f.apron, f.outside, f.inner, f.inside], cancelled ? 'returning' : 'entering', f);
    else if (!cancelled && stop.id === 'station' && this.line && ['connection', 'commute'].includes(p.purpose)) {
      this.path(p, [...STATION_PATH].reverse().concat(PLATFORM_WAIT), 'to-train');
    } else this.path(p, [{ x: stop.curb.x + stop.dx * p.offset, y: stop.curb.y + stop.dy * p.offset, height: 0 }],
      cancelled ? 'cancelled' : 'leaving');
  }
  unloadTrain(event) {
    const train = this.line.train;
    for (let i = 0; i < event.count; i++) {
      const purpose = PURPOSES[(i + this.line.sequence) % 5], night = cityConditions(this.period, this.time).night;
      const destination = (night ? ['homes', 'west', 'hotel', 'nightlife'] : ['office', 'market', 'homes', 'clinic'])[i % 4];
      const p = this.addPerson({ id: event.train + '-' + (i + 1), purpose: purpose.id, origin: 'station', destination,
        release: this.time + i * .65, walkSpeed: purpose.speed, patience: purpose.patience, dwell: purpose.dwell, offset: 8 });
      Object.assign(p, { state: 'on-train', arrivingTrain: true, trainId: event.train, source: 'train',
        x: train.x + (i % 2 ? 8 : -8), y: RAIL_Y - .8, height: 1, insideBuilding: null });
    }
  }
  targetSupply() { return weatherSupply(cityConditions(this.period,this.time).taxiSupply,this.weather,this.weatherStrength,this.reserveEnabled); }
  setWeather(weather,strength=.5) {
    weatherSupply(8,weather,strength,true);
    this.weather=weather; this.weatherStrength=weather==='clear'?0:strength;
    this.nextWeatherDemand=this.time+(CITY_WEATHER[weather].interval/Math.max(.1,strength));
    this.weatherEvents.push({time:this.time,type:'weather',weather,strength:this.weatherStrength});
  }
  setReserveEnabled(enabled) {
    if(typeof enabled!=='boolean') throw new Error('Reserve policy must be boolean');
    this.reserveEnabled=enabled; this.weatherEvents.push({time:this.time,type:'reserve',enabled});
  }
  stormDemand() {
    if(!this.auto || this.weather==='clear' || this.weatherStrength===0 || this.time<this.nextWeatherDemand) return;
    this.nextWeatherDemand=this.time+CITY_WEATHER[this.weather].interval/Math.max(.1,this.weatherStrength);
    const night=cityConditions(this.period,this.time).night, serial=++this.weatherSequence;
    const purpose=PURPOSES[(serial-1)%PURPOSES.length];
    this.addPerson({id:'W'+String(serial).padStart(4,'0'),purpose:purpose.id,origin:night?'nightlife':'office',
      destination:['homes','west','hotel'][serial%3],release:this.time+.3,walkSpeed:purpose.speed,
      patience:purpose.patience,dwell:purpose.dwell,offset:8,source:'weather-demand'});
  }
  setStandbyEnabled(enabled) {
    if (typeof enabled !== 'boolean') throw new Error('Standby mode must be boolean');
    this.standbyEnabled = enabled; this.nextStandby = this.time;
    if (!enabled) for (const taxi of this.taxis) {
      if (taxi.standbyStop && !taxi.person && taxi.onDuty) taxi.car.serviceStop = taxi.idleStop;
      taxi.standbyStop = null;
    }
  }
  rebalance() {
    for (const taxi of this.taxis) if (taxi.standbyStop && !taxi.person &&
        !taxi.standbyArrived && atServiceStop(taxi.car, taxi.standbyStop)) {
      taxi.standbyArrived = true;
      this.standbyEvents.push({ time: this.time, type: 'standby-arrive', taxi: taxi.id, stop: taxi.standbyStop.id,
        x: taxi.car.x, y: taxi.car.y, speed: taxi.car.speed });
    }
    if (!this.standbyEnabled || this.time < this.nextStandby) return;
    this.nextStandby = this.time + 5;
    const idle = this.taxis.filter(t => !t.person && t.onDuty && !t.retiring);
    const scores = standbyPriorities(FLEET_STOPS, cityConditions(this.period, this.time), this.time,
      this.events, this.people.filter(p => p.state !== 'scheduled'), this.line?.train);
    this.standbyScores = scores.map(s => ({ stop: s.stop.id, label: s.stop.label, score: s.score, reason: s.reason }));
    const desired = new Set(scores.slice(0, idle.length).map(s => s.stop.id)), used = new Set(), free = [];
    for (const taxi of idle) {
      const id = taxi.standbyStop?.id.slice('standby-'.length);
      if (id && !used.has(id) && (desired.has(id) || this.time - taxi.standbySince < 20)) used.add(id);
      else free.push(taxi);
    }
    for (const item of scores) {
      if (!free.length) break;
      if (used.has(item.stop.id)) continue;
      const stop = this.standbyStops.get(item.stop.id);
      const choices = free.map(t => ({ t, distance: pickupDistance(this.graph, t.car, stop) }))
        .filter(c => Number.isFinite(c.distance)).sort((a, b) => a.distance - b.distance || a.t.id.localeCompare(b.t.id));
      const choice = choices[0]; if (!choice) continue;
      const taxi = choice.t;
      taxi.standbyStop = stop; taxi.standbySince = this.time; taxi.standbyArrived = false;
      taxi.car.serviceStop = stop; used.add(item.stop.id); free.splice(free.indexOf(taxi), 1);
      this.standbyEvents.push({ time: this.time, type: 'reposition', taxi: taxi.id, stop: stop.id,
        x: taxi.car.x, y: taxi.car.y, distance: choice.distance, reason: item.reason });
    }
    // More idle cabs than bays cruise through the road network; they do not
    // pile up in a pickup bay or claim another cab's standby space.
    for (const taxi of free) { taxi.standbyStop = null; taxi.car.serviceStop = null; }
  }
  update(dt) {
    if (!Number.isFinite(dt) || dt <= 0 || dt > .1) throw new Error('Fleet step must be in (0, .1] seconds');
    this.time += dt;
    const clock = cityConditions(this.period, this.time), supply = this.targetSupply();
    this.stormDemand();
    this.taxis.forEach((t, i) => {
      const odometer = t.car.distance ?? 0, driven = Math.max(0, odometer - t.lastDistance);
      if (!t.person || t.state === 'pickup') t.emptyDistance += driven;
      if (!t.person && t.standbyStop) t.repositionDistance += driven;
      t.lastDistance = odometer;
      t.onDuty = i < supply || Boolean(t.person); t.retiring = i >= supply;
      if (!t.person) {
        if (!t.onDuty) { t.standbyStop = null; t.car.serviceStop = DEPOT_STOPS[i % DEPOT_STOPS.length]; }
        else if (t.state === 'off-duty') t.car.serviceStop = FLEET_STOPS[i % FLEET_STOPS.length];
        t.state = t.onDuty ? 'idle' : 'off-duty';
      }
    });
    if (this.auto && this.time >= this.nextDemand - 8) {
      this.demandBatch++;
      const plans = createDemand(this.seed + this.demandBatch * 7919, 72, this.period, this.nextDemand, this.demandBatch * 72);
      for (const plan of plans) this.addPerson(plan);
      this.nextDemand = Math.max(...plans.map(p => p.release)) + 5;
    }
    if (this.line) {
      const hold = this.people.some(p => p.state === 'boarding-train' || p.state === 'exiting-train');
      this.line.update(dt, hold);
      while (this.railCursor < this.line.events.length) {
        const event = this.line.events[this.railCursor++];
        if (event.type === 'train-unload') this.unloadTrain(event);
        if (event.type === 'train-exit') for (const p of this.people.filter(p => p.state === 'riding-train' && p.trainId === event.train)) {
          p.x = event.x + (p.trainOffset ?? 0); p.y = RAIL_Y - .8; p.height = 1;
          p.state = 'arrived'; p.finishedAt = this.time; this.event('rail-trip-complete', p);
        }
      }
    }
    for (const p of this.people) if (p.state === 'scheduled' && this.time >= p.release) {
      if (p.facility) { this.path(p, [p.facility.inner, p.facility.outside], 'exiting', p.facility); this.event('building-depart', p, null, { building: p.facility.id }); }
      else this.request(p);
    }
    for (const [id, door] of Object.entries(this.doors)) {
      const f = FACILITIES[id], wanted = this.people.some(p =>
        ['exiting', 'entering', 'returning'].includes(p.state) && p.facility === f &&
        Math.hypot(p.x - f.door.x, p.y - f.door.y) < 4.5);
      door.open = Math.max(0, Math.min(1, door.open + (wanted ? 1.25 : -.9) * dt));
    }
    const train = this.line?.train;
    for (const p of this.people) {
      if (p.state === 'exiting' && this.travel(p, dt)) this.request(p);
      else if (p.state === 'walking' && this.travel(p, dt)) { p.state = 'waiting'; this.event('curb', p); }
      else if (['entering', 'returning', 'leaving', 'cancelled'].includes(p.state) && this.travel(p, dt)) {
        if (!p.finishedAt || p.state !== 'cancelled') this.finishWalk(p);
      } else if (p.state === 'on-train' && p.arrivingTrain && this.time >= p.release &&
                 train?.id === p.trainId && train.phase === 'dwell' && train.doorOpen >= .98 && train.speed < .15) {
        this.path(p, [{ x: p.x, y: 182.7, height: 1 }, ...STATION_PATH], 'exiting-train');
      } else if (p.state === 'exiting-train' || p.state === 'walking-from-train') {
        const complete = this.travel(p, dt);
        if (p.state === 'exiting-train' && p.routeIndex >= 1) {
          p.state = 'walking-from-train'; if (train?.id === p.trainId) train.occupied--;
          this.event('train-alight', p, null, { train: p.trainId });
        }
        if (complete) this.request(p);
      } else if (p.state === 'to-train' && this.travel(p, dt)) { p.state = 'waiting-train'; this.event('rail-platform', p); }
      else if (p.state === 'waiting-train') {
        if (train?.phase === 'dwell' && train.doorOpen >= .98 && train.occupied + train.reserved < train.capacity) {
          train.reserved++; p.trainId = train.id;
          const x = train.x + (p.id.length % 2 ? 8 : -8);
          this.path(p, [{ x, y: 182.7, height: 1 }, { x, y: RAIL_Y - .8, height: 1 }], 'boarding-train');
        } else if (!clock.railOpen && (!train || train.phase === 'depart')) {
          this.event('missed-last-train', p); p.origin = byStop.get('station');
          p.destination = byStop.get(p.id.length % 2 ? 'homes' : 'west'); p.purpose = 'connection';
          p.leg++; p.source = 'missed-train'; p.arrivingTrain = false;
          this.path(p, [...STATION_PATH], 'walking-from-train');
        }
      } else if (p.state === 'boarding-train' && train?.id === p.trainId &&
                 train.phase === 'dwell' && train.doorOpen >= .98 && this.travel(p, dt)) {
        train.reserved--; train.occupied++; p.trainOffset = p.x - train.x; p.state = 'riding-train'; this.event('train-board', p, null, { train: train.id, speed: train.speed });
      }
      if (p.state === 'riding-train' && train?.id === p.trainId) { p.x = train.x + p.trainOffset; p.y = RAIL_Y - .8; p.height = 1; }
      if (['walking', 'waiting'].includes(p.state) && this.time - p.requestAt > p.patience) {
        const taxi = this.taxis.find(t => t.id === p.taxi);
        if (taxi) { taxi.state = 'idle'; taxi.person = null; taxi.idleStop = p.origin; }
        p.taxi = null; this.event('cancel', p, taxi); this.startLastLeg(p, true);
      }
    }
    if (this.time >= this.nextDispatch) {
      this.nextDispatch = this.time + .5;
      const waiting = this.people.filter(p => ['walking', 'waiting'].includes(p.state) && !p.taxi)
        .sort((a, b) => a.requestAt - b.requestAt || a.id.localeCompare(b.id));
      for (const p of waiting) {
        const choices = this.taxis.filter(t => t.state === 'idle' && t.onDuty && !t.retiring)
          .map(t => ({ t, distance: pickupDistance(this.graph, t.car, p.origin) })).filter(c => Number.isFinite(c.distance));
        choices.sort((a, b) => a.distance - b.distance || a.t.id.localeCompare(b.t.id));
        const taxi = choices[0]?.t; if (!taxi) continue;
        taxi.standbyStop = null; taxi.person = p; taxi.state = 'pickup'; taxi.car.serviceStop = p.origin; p.taxi = taxi.id;
        this.event('assign', p, taxi, { distance: choices[0].distance });
      }
    }
    for (const taxi of this.taxis) {
      const p = taxi.person, car = taxi.car; if (!p) continue;
      if (taxi.state === 'pickup' && p.state === 'waiting' && atServiceStop(car, p.origin)) {
        taxi.state = 'boarding'; p.state = 'boarding'; p.queueSlot = null; p.dwellUntil = this.time + p.dwell;
        this.waits.push(this.time - p.requestAt);
        this.event('board', p, taxi, { x: car.x, y: car.y, speed: car.speed, stop: p.origin.id });
      }
      if (taxi.state === 'boarding' && atServiceStop(car, p.origin)) {
        const door = { x: car.x + p.origin.normal.x * 1.15, y: car.y + p.origin.normal.y * 1.15 };
        if (walk(p, door, dt) && this.time >= p.dwellUntil) {
          taxi.state = 'occupied'; p.state = 'riding'; car.serviceStop = p.destination; this.event('depart', p, taxi);
        }
      } else if (taxi.state === 'occupied') {
        p.x = car.x; p.y = car.y; p.height = car.height ?? 0;
        if (atServiceStop(car, p.destination)) {
          taxi.state = 'alighting'; p.state = 'alighting'; p.dwellUntil = this.time + p.dwell * .7;
          this.event('alight', p, taxi, { x: car.x, y: car.y, speed: car.speed, stop: p.destination.id });
        }
      } else if (taxi.state === 'alighting' && atServiceStop(car, p.destination) && walk(p, p.destination.curb, dt) && this.time >= p.dwellUntil) {
        p.taxi = null; taxi.state = 'idle'; taxi.person = null; taxi.served++; taxi.idleStop = p.destination;
        this.event('complete', p, taxi); this.startLastLeg(p);
      }
    }
    this.rebalance();
  }
  snapshot() {
    const count = states => this.people.filter(p => states.includes(p.state)).length;
    return { time: this.time, seed: this.seed, period: this.period, clock: cityConditions(this.period, this.time),
      weather:{name:this.weather,strength:this.weatherStrength,reserve:this.reserveEnabled,targetSupply:this.targetSupply(),events:this.weatherEvents.map(e=>({...e}))},
      demand: 'authored individual travel plans; not calibrated',
      standby: { enabled: this.standbyEnabled, scores: this.standbyScores.map(s => ({ ...s })), events: this.standbyEvents.map(e => ({ ...e })) },
      doors: Object.fromEntries(Object.entries(this.doors).map(([id, door]) => [id, { ...door }])),
      train: this.line?.snapshot() ?? null, railEvents: this.line?.events.map(e => ({ ...e })) ?? [],
      metrics: { targetSupply:this.targetSupply(), weatherRequests:this.events.filter(e=>e.type==='request'&&e.source==='weather-demand').length,
        emptyDistance: this.taxis.reduce((n, t) => n + t.emptyDistance, 0),
        repositionDistance: this.taxis.reduce((n, t) => n + t.repositionDistance, 0),
        repositioning: this.taxis.filter(t => t.standbyStop && !t.person && !atServiceStop(t.car, t.standbyStop)).length,
        standby: this.taxis.filter(t => t.standbyStop && !t.person && atServiceStop(t.car, t.standbyStop)).length,
        requested: this.events.filter(e => e.type === 'request').length, waiting: count(['walking', 'waiting']),
        riding: count(['riding']), served: this.taxis.reduce((n, t) => n + t.served, 0),
        cancelled: this.events.filter(e => e.type === 'cancel').length, onDuty: this.taxis.filter(t => t.onDuty).length,
        railAlighted: this.events.filter(e => e.type === 'train-alight').length, railBoarded: this.events.filter(e => e.type === 'train-board').length,
        meanWait: this.waits.length ? this.waits.reduce((a, b) => a + b, 0) / this.waits.length : 0 },
      taxis: this.taxis.map(t => ({ id: t.id, state: t.state, person: t.person?.id ?? null, served: t.served, onDuty: t.onDuty,
        standbyStop: t.standbyStop?.id ?? null, activity: t.standbyStop && !t.person ? (atServiceStop(t.car, t.standbyStop) ? 'standby' : 'repositioning') : t.state,
        emptyDistance: t.emptyDistance, repositionDistance: t.repositionDistance,
        x: t.car.x, y: t.car.y, speed: t.car.speed, distance: t.car.distance, destination: t.car.serviceStop?.id ?? null, capacity: t.capacity })),
      people: this.people.filter(p => p.state !== 'scheduled').map(p => ({ id: p.id, purpose: p.purpose, origin: p.origin.id,
        destination: p.destination.id, state: p.state, taxi: p.taxi, x: p.x, y: p.y, height: p.height, source: p.source ?? p.origin.id,
        insideBuilding: p.insideBuilding, walked: p.walked, walkSpeed: p.walkSpeed, patience: p.patience,
        release: p.release, requestAt: p.requestAt, leg: p.leg })),
      events: this.events.map(e => ({ ...e })),
    };
  }
}
