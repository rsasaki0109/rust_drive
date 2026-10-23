// Arcade junction admission in east/north metres; separate from the research stack.
const key = node => node.x + ':' + node.y;
const STOP_LINE = 11, ENTERED = 8.5, CLEARED = 12.5;

export class Junctions {
  constructor(graph, { phase, lane, drive }) {
    this.nodes = new Map([...graph].filter(([, node]) => node.signal || node.links.length >= 3));
    this.phase = phase; this.lane = lane; this.drive = drive;
    this.owners = new Map(); this.tickets = new Map(); this.limits = new Map(); this.clock = 0;
  }
  onRoad(car) {
    return Math.abs((car.height ?? 0) - (this.drive?.drive_surface(car.x, car.y, 0) ?? 0)) <= 2.5;
  }
  approach(car) {
    let from = car.node, to = car.next;
    const heading = Math.atan2(to.y - from.y, to.x - from.x), lane = this.lane(car);
    let distance = Math.hypot(to.x + Math.sin(heading) * lane - car.x,
                              to.y - Math.cos(heading) * lane - car.y);
    // Short station-curve segments need braking before the final graph edge.
    for (let i = 0; i < 6; i++) {
      const id = key(to);
      if (this.nodes.has(id)) return { car, key: id, node: to, distance,
        incoming: key(from), axis: Math.abs(to.x - from.x) > Math.abs(to.y - from.y) ? 'x' : 'y' };
      const forward = to.links.filter(node => key(node) !== key(from));
      if (forward.length !== 1) break;
      from = to; to = forward[0]; distance += Math.hypot(to.x - from.x, to.y - from.y);
    }
    return null;
  }
  update(dt, time, cars, player) {
    this.clock += dt; this.limits.clear();
    const live = new Map(cars.filter(car => this.onRoad(car)).map(car => [car.id, car]));
    const slots = new Map(), activeTickets = new Set();
    const slot = id => {
      if (!slots.has(id)) slots.set(id, { requests: [], inside: [] });
      return slots.get(id);
    };
    for (const car of live.values()) {
      const request = this.approach(car);
      if (request && request.distance < Math.max(36, car.speed * car.speed / 7 + 15)) {
        const ticket = car.id + '|' + request.key;
        activeTickets.add(ticket);
        if (!this.tickets.has(ticket)) this.tickets.set(ticket, this.clock);
        request.since = this.tickets.get(ticket);
        request.green = car.police || !request.node.signal ||
          this.phase(request.axis, request.node.x, request.node.y, time) === 'green';
        slot(request.key).requests.push(request);
      }
      // Include cars already clearing a turn, even after their next waypoint changed.
      const nearby = new Set([car.node, car.next, car.previous].filter(Boolean).map(key));
      for (const id of nearby) {
        const node = this.nodes.get(id); if (!node) continue;
        const radius = key(car.node) === id ? CLEARED : ENTERED;
        if (Math.hypot(car.x - node.x, car.y - node.y) < radius) slot(id).inside.push(car);
      }
    }
    for (const id of this.owners.keys()) slot(id);
    for (const ticket of this.tickets.keys()) if (!activeTickets.has(ticket)) this.tickets.delete(ticket);
    const playerOnRoad = this.onRoad(player);
    for (const [id, state] of slots) {
      const node = this.nodes.get(id), requests = state.requests;
      let owner = this.owners.get(id), car = owner && live.get(owner.id);
      const request = car && requests.find(r => r.car.id === car.id);
      if (car) {
        const radius = Math.hypot(car.x - node.x, car.y - node.y);
        owner.entered ||= radius < ENTERED || key(car.node) === id ||
          Boolean(request && request.distance <= STOP_LINE + .3);
        if (owner.entered ? radius > CLEARED : !request || !request.green) owner = null;
      } else owner = null;
      const blockedByPlayer = playerOnRoad && Math.hypot(player.x - node.x, player.y - node.y) < ENTERED;
      const inside = state.inside.sort((a, b) => a.id - b.id);
      if (!owner?.entered && inside.length) owner = { id: inside[0].id, entered: true };
      if (!owner && !blockedByPlayer) {
        // A follower cannot reserve the junction over the car in front of it.
        const eligible = requests.filter(r => r.green && !requests.some(other =>
          other.incoming === r.incoming && other.distance < r.distance - .1));
        eligible.sort((a, b) => Number(b.car.police) - Number(a.car.police) ||
          a.since - b.since || a.car.id - b.car.id);
        if (eligible.length) {
          const first = eligible[0];
          owner = { id: first.car.id, entered: first.distance <= STOP_LINE + .3 };
        }
      }
      if (owner) this.owners.set(id, owner); else this.owners.delete(id);
      for (const r of requests) {
        if (inside.some(c => c.id === r.car.id) ||
            owner?.id === r.car.id && (!blockedByPlayer || owner.entered)) continue;
        const limit = Math.sqrt(7 * Math.max(0, r.distance - STOP_LINE));
        this.limits.set(r.car.id, Math.min(this.limits.get(r.car.id) ?? Infinity, limit));
      }
    }
  }
  limit(car) { return this.limits.get(car.id) ?? Infinity; }
}
