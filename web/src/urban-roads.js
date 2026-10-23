// Authored Tokyo-inspired street layout, in east/north metres; not a real-city map.
export const STATION_START = Object.freeze({ x: 490, y: 85, yaw: 0 });
export const URBAN_START = Object.freeze({ x: 140, y: 150, yaw: 0 });
export const HILL_STREETS = Object.freeze([
  [{ x: 150, y: 150 }, { x: 190, y: 150 }, { x: 240, y: 150 }, { x: 334, y: 150 }, { x: 370, y: 150 }],
  [{ x: 190, y: 118.5 }, { x: 240, y: 118.5 }, { x: 334, y: 118.5 }, { x: 370, y: 118.5 }],
  [{ x: 190, y: 181.5 }, { x: 240, y: 181.5 }, { x: 334, y: 181.5 }, { x: 370, y: 181.5 }],
  ...[190, 240, 334, 370].map(x => [{ x, y: 110 }, { x, y: 118.5 }, { x, y: 150 }, { x, y: 181.5 }, { x, y: 190 }]),
]);
export const LOWER_RING = Object.freeze(Array.from({ length: 65 }, (_, i) => {
  const a = i * Math.PI * 2 / 64;
  return Object.freeze({ x: Number((490 + 55 * Math.cos(a)).toFixed(6)), y: Number((150 + 65 * Math.sin(a)).toFixed(6)) });
}));
export const HILL_TRAFFIC = Object.freeze([
  { x: 190, y: 150 }, { x: 240, y: 150 }, { x: 334, y: 150 }, { x: 370, y: 150 },
  { x: 370, y: 181.5 }, { x: 334, y: 181.5 }, { x: 240, y: 181.5 }, { x: 190, y: 181.5 },
  // Both directions of the avenue carry ordinary traffic as well as the alleys.
  { x: 240, y: 150, to: { x: 334, y: 150 }, advance: 18 },
  { x: 240, y: 150, to: { x: 334, y: 150 }, advance: 63 },
  { x: 334, y: 150, to: { x: 240, y: 150 }, advance: 17 },
  { x: 334, y: 150, to: { x: 240, y: 150 }, advance: 62 },
]);
export const nodeKey = p => p.x + ':' + p.y;

// Subdivide the curved boulevard for actual steering, not a decorative circle.
export function roadGraph(streets, urban = false) {
  const nodes = new Map();
  const add = p => { const key = nodeKey(p); if (!nodes.has(key)) nodes.set(key, { ...p, signal: streets.includes(p.x) && streets.includes(p.y), links: [] }); return nodes.get(key); };
  const connect = (a, b) => { a = add(a); b = add(b); if (!a.links.some(p => nodeKey(p) === nodeKey(b))) a.links.push(b); if (!b.links.some(p => nodeKey(p) === nodeKey(a))) b.links.push(a); };
  for (let i = 0; i < streets.length; i++) for (let j = 0; j < streets.length; j++) {
    const p = { x: streets[i], y: streets[j] }; add(p);
    if (i) connect(p, { x: streets[i - 1], y: streets[j] });
    if (j) connect(p, { x: streets[i], y: streets[j - 1] });
  }
  if (urban) {
    for (const road of HILL_STREETS) for (let i = 1; i < road.length; i++) connect(road[i - 1], road[i]);
    // The endpoints of crossing streets stay guarded; traffic uses junction nodes.
    for (const p of [...nodes.values()]) if (p.x > 180 && [110, 190].includes(p.y)) {
      for (const q of p.links) q.links = q.links.filter(n => nodeKey(n) !== nodeKey(p));
      nodes.delete(nodeKey(p));
    }
    const ring = LOWER_RING.slice(0, -1).filter((_, i) => i % 2 === 0);
    for (let i = 0; i < ring.length; i++) connect(ring[i], ring[(i + 1) % ring.length]);
    const returnRoad = [{ x: 150, y: 50 }, { x: 150, y: 70 }, { x: 190, y: 70 }, { x: 350, y: 70 }, { x: 530, y: 70 }, { x: 530, y: 150 }, { x: 545, y: 150 }];
    for (let i = 1; i < returnRoad.length; i++) connect(returnRoad[i - 1], returnRoad[i]);
    connect({ x: 150, y: 70 }, { x: 150, y: 100 });
    for (const p of nodes.values()) {
      if (p.y === 181.5) p.links = p.links.filter(q => q.y !== p.y || q.x < p.x);
      if (p.y === 118.5) p.links = p.links.filter(q => q.y !== p.y || q.x > p.x);
    }
    // End approaches join the one-way perimeter; avoid mandatory U-turns into shops.
    for (const [from, to] of [['190:150','190:181.5'], ['370:150','370:118.5']])
      nodes.get(from).links = nodes.get(from).links.filter(q => nodeKey(q) !== to);
    for (const [x, y] of [[240, 150], [334, 150], [370, 150], [435, 150], [545, 150]]) nodes.get(x + ':' + y).signal = true;
  }
  return nodes;
}
