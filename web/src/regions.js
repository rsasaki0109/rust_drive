// Planar world coordinates: x east, y north, metres; yaw is counterclockwise.
export const DISTRICTS = Object.freeze([
  { id: 'plaza', label: 'DRIFT SQUARE', name: '中央広場', color: '#d8b57c', spawn: { x: 0, y: -22, yaw: Math.PI / 2 } },
  { id: 'downtown', label: 'NEON CITY', name: 'ネオン街', color: '#a5a4ce', spawn: { x: 0, y: 90, yaw: Math.PI / 2 } },
  { id: 'harbor', label: 'HARBOR BAY', name: '港湾エリア', color: '#79bbc6', spawn: { x: 100, y: -120, yaw: Math.PI / 2 } },
  { id: 'forest', label: 'CANOPY PARK', name: '森の抜け道', color: '#86b987', spawn: { x: -100, y: 70, yaw: Math.PI / 2 } },
  { id: 'factory', label: 'IRONWORKS', name: '工場エリア', color: '#ca9473', spawn: { x: 100, y: 80, yaw: Math.PI / 2 } },
  { id: 'ridge', label: 'SKYLINE RIDGE', name: '高台・崖ダイブ', color: '#d5aa77', spawn: { x: 190, y: 150, yaw: 0 } },
].map(d => Object.freeze({ ...d, spawn: Object.freeze(d.spawn) })));
const byId = new Map(DISTRICTS.map(d => [d.id, d]));

export function districtAt(x, y) {
  if (x > 180) return byId.get('ridge');
  if (x < -50 && y > 0) return byId.get('forest');
  if (x > 0 && y < -100) return byId.get('harbor');
  if (x > 50 && y > 50) return byId.get('factory');
  if (Math.abs(x) <= 50 && Math.abs(y) <= 50) return byId.get('plaza');
  return byId.get('downtown');
}
export const BLOCKS = Object.freeze(Array.from({ length: 36 }, (_, index) => {
  const x = -125 + 50 * Math.floor(index / 6), y = -125 + 50 * (index % 6);
  return Object.freeze({ x, y, district: districtAt(x, y) });
}));

// Every trail connects the west/east road edges of its block, without a loading gate.
export function forestTrail(cx, cy) {
  return Array.from({ length: 49 }, (_, i) => {
    const t = i / 48;
    return { x: cx - 16 + 32 * t, y: cy + 5 * Math.sin(t * Math.PI * 2) };
  });
}
export function distanceToPath(x, y, path) {
  let distance = Infinity;
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1], b = path[i], dx = b.x - a.x, dy = b.y - a.y;
    const t = Math.max(0, Math.min(1, ((x - a.x) * dx + (y - a.y) * dy) / (dx * dx + dy * dy)));
    distance = Math.min(distance, Math.hypot(x - a.x - t * dx, y - a.y - t * dy));
  }
  return distance;
}
export function forestTrees(cx, cy) {
  const path = forestTrail(cx, cy), trees = [];
  for (let ix = 0; ix < 5; ix++) for (let iy = 0; iy < 5; iy++) {
    const x = cx - 12 + ix * 6, y = cy - 12 + iy * 6;
    if (distanceToPath(x, y, path) >= 5.5) trees.push({ x, y, height: 6 + (ix * 3 + iy * 7) % 5 });
  }
  return trees;
}
