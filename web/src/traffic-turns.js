// Lane-centred turn arcs in east/north metres.
const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));

export function turnArc(from, node, next, incomingLane, outgoingLane) {
  const incoming = Math.atan2(node.y - from.y, node.x - from.x);
  const outgoing = Math.atan2(next.y - node.y, next.x - node.x);
  const delta = wrap(outgoing - incoming);
  if (Math.abs(delta) < .45 || Math.abs(delta) > 2.7) return null;
  const a = { x: Math.cos(incoming), y: Math.sin(incoming) };
  const b = { x: Math.cos(outgoing), y: Math.sin(outgoing) };
  const start = { x: node.x - 7 * a.x + a.y * incomingLane,
                  y: node.y - 7 * a.y - a.x * incomingLane };
  const endLine = { x: node.x + b.y * outgoingLane, y: node.y - b.x * outgoingLane };
  const cross = a.x * b.y - a.y * b.x;
  const along = ((endLine.x - start.x) * b.y - (endLine.y - start.y) * b.x) / cross;
  const radius = along / Math.tan(Math.abs(delta) / 2);
  if (!(radius > 1 && radius < 30)) return null;
  const sign = Math.sign(delta), x = start.x - a.y * sign * radius, y = start.y + a.x * sign * radius;
  const angle = Math.atan2(start.y - y, start.x - x);
  return { x, y, radius, angle, delta,
    endX: x + radius * Math.cos(angle + delta), endY: y + radius * Math.sin(angle + delta),
    speed: Math.min(10, Math.sqrt(5 * radius), 1.4 * radius) };
}

export function arcTarget(arc, car) {
  const progress = Math.sign(arc.delta) * wrap(Math.atan2(car.y - arc.y, car.x - arc.x) - arc.angle);
  if (Math.hypot(car.x - arc.endX, car.y - arc.endY) < 1 ||
      progress > Math.abs(arc.delta) - .04 && Math.hypot(car.x - arc.endX, car.y - arc.endY) < 3) return null;
  const angle = arc.angle + Math.sign(arc.delta) *
    Math.min(Math.abs(arc.delta), Math.max(0, progress) + 2 / arc.radius);
  return { x: arc.x + arc.radius * Math.cos(angle), y: arc.y + arc.radius * Math.sin(angle) };
}
