// Authored city clock: physical movement uses SI seconds; the clock runs at 15x.
export const PERIODS = Object.freeze({
  morning: { label: 'MORNING / 08:00', start: 8 * 60 },
  dusk: { label: 'DUSK / 18:15', start: 18 * 60 + 15 },
  night: { label: 'LATE NIGHT / 23:40', start: 23 * 60 + 40 },
});
export function cityConditions(period, seconds) {
  const minutes = ((Object.hasOwn(PERIODS, period) ? PERIODS[period] : PERIODS.morning).start + seconds / 4) % 1440;
  const hour = minutes / 60, night = hour >= 22 || hour < 5;
  return { minutes, hour, night, daylight: Math.max(0, Math.min(1, (Math.sin((hour - 6) * Math.PI / 12) + .22) / .98)),
    taxiSupply: night ? 12 : hour >= 17 && hour < 22 ? 10 : 8,
    railOpen: hour >= 5, clockRate: 15 };
}
export const FACILITIES = Object.freeze({
  office: { id: 'office', label: 'CIVIC OFFICES', door: { x: -25, y: -14 }, outward: 1,
    inside: { x: -25, y: -16.6, height: 0 }, inner: { x: -25, y: -15.2, height: 0 },
    outside: { x: -25, y: -12.9, height: 0 }, apron: { x: -25, y: -10.8, height: 0 } },
  nightlife: { id: 'nightlife', label: 'MIDNIGHT CLUB', door: { x: -31, y: 64 }, outward: -1,
    inside: { x: -31, y: 66.6, height: 0 }, inner: { x: -31, y: 65.2, height: 0 },
    outside: { x: -31, y: 62.9, height: 0 }, apron: { x: -31, y: 60.8, height: 0 } },
});
// A real walkable platform/ramp/footbridge path, outside the building footprints.
// The bridge crosses the y=150 avenue at 5 m; people never teleport to the curb.
export const STATION_PATH = Object.freeze([
  { x: 39.2, y: 179, height: 1 }, { x: 39.2, y: 162, height: 5 },
  { x: 39.2, y: 138, height: 5 }, { x: 39.2, y: 112, height: 0 },
  { x: 39.2, y: 110.8, height: 0 }, { x: 25, y: 110.8, height: 0 },
]);
export const PLATFORM_WAIT = Object.freeze({ x: 28, y: 181.7, height: 1 });
export const RAIL_Y = 185;

export class NorthLine {
  constructor(period = 'morning') {
    this.period = period; this.time = 0; this.nextSpawn = 0; this.sequence = 0; this.train = null; this.events = [];
  }
  update(dt, holdDoors = false) {
    this.time += dt;
    if (!this.train && this.time >= this.nextSpawn && cityConditions(this.period, this.time).railOpen) {
      this.train = { id: 'R' + String(++this.sequence).padStart(3, '0'), x: -235, speed: 0,
        phase: 'approach', doorOpen: 0, occupied: 0, reserved: 0, capacity: 10,
        lastService: !cityConditions(this.period, this.time + 72).railOpen, spawnedAt: this.time };
      this.nextSpawn = this.time + 72;
      this.events.push({ type: 'train-spawn', time: this.time, train: this.train.id, lastService: this.train.lastService });
    }
    const t = this.train;
    if (!t) return;
    if (t.phase === 'approach' || t.phase === 'depart') {
      const distance = Math.max(0, 25 - t.x);
      const target = t.phase === 'depart' ? 12 : Math.min(12, Math.sqrt(2.2 * distance), distance * 1.4);
      t.speed += Math.max(-1.3 * dt, Math.min(1.1 * dt, target - t.speed));
      t.x += t.phase === 'approach' ? Math.min(distance, t.speed * dt) : t.speed * dt;
      if (t.phase === 'approach' && 25 - t.x < .08 && t.speed < .15) {
        t.phase = 'dwell'; t.arrivedAt = this.time; t.speed = 0;
        this.events.push({ type: 'train-arrive', time: this.time, train: t.id, x: t.x, speed: t.speed, lastService: t.lastService });
      }
      if (t.phase === 'depart' && t.x > 265) {
        this.events.push({ type: 'train-exit', time: this.time, train: t.id }); this.train = null;
      }
    } else {
      const readyToClose = this.time - t.arrivedAt > 9 && !holdDoors;
      t.doorOpen = Math.max(0, Math.min(1, t.doorOpen + (readyToClose ? -2 : 2) * dt));
      if (!t.unloaded && t.doorOpen >= .98) {
        t.unloaded = true; t.occupied = t.lastService ? 6 : 4;
        this.events.push({ type: 'train-unload', time: this.time, train: t.id, count: t.occupied, x: t.x });
      }
      if (readyToClose && t.doorOpen === 0) {
        t.phase = 'depart'; this.events.push({ type: 'train-depart', time: this.time, train: t.id, occupied: t.occupied });
      }
    }
  }
  snapshot() { return this.train ? { ...this.train } : null; }
}
