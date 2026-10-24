// Authored arcade fleet routing; east/north metres. No research-stack inputs.
import { nodeKey } from './urban-roads.js';

const same = (a, b) => a && b && nodeKey(a) === nodeKey(b);
export const onStopEdge = (car, stop) => Boolean(stop && same(car.node, stop.from) && same(car.next, stop.to));

export function makeStop(id, label, from, to) {
  const length = Math.hypot(to.x - from.x, to.y - from.y);
  const dx = (to.x - from.x) / length, dy = (to.y - from.y) / length;
  const along = length / 2, normal = { x: dy, y: -dx };
  return Object.freeze({
    id, label, from, to, along, dx, dy, normal, lane: 6.4,
    x: from.x + dx * along + normal.x * 6.4,
    y: from.y + dy * along + normal.y * 6.4,
    curb: { x: from.x + dx * along + normal.x * 10.8, y: from.y + dy * along + normal.y * 10.8 },
  });
}

// Search directed (incoming, current) states: a close point across the street
// is not a legal pickup approach. U-turns are excluded like ordinary traffic.
export function routeToStop(graph, current, previous, stop) {
  if (!graph.has(nodeKey(stop.from)) || !graph.get(nodeKey(stop.from)).links.some(n => same(n, stop.to))) return null;
  const queue = [{ node: current, previous, nodes: [], distance: 0 }], visited = new Set();
  while (queue.length) {
    queue.sort((a, b) => a.distance - b.distance);
    const item = queue.shift(), key = (item.previous ? nodeKey(item.previous) : '') + '>' + nodeKey(item.node);
    if (visited.has(key)) continue;
    visited.add(key);
    if (same(item.node, stop.from) && !same(item.previous, stop.to))
      return { nodes: [...item.nodes, graph.get(nodeKey(stop.to))], distance: item.distance + stop.along };
    for (const next of graph.get(nodeKey(item.node))?.links ?? []) {
      if (same(next, item.previous)) continue;
      queue.push({ node: next, previous: item.node, nodes: [...item.nodes, next],
        distance: item.distance + Math.hypot(next.x - item.node.x, next.y - item.node.y) });
    }
  }
  return null;
}

export function pickupDistance(graph, car, stop) {
  const along = (stop.x - car.x) * stop.dx + (stop.y - car.y) * stop.dy;
  if (onStopEdge(car, stop) && along > -1) return Math.max(0, along);
  const route = routeToStop(graph, car.next, car.node, stop);
  return route ? Math.hypot(car.next.x - car.x, car.next.y - car.y) + route.distance : Infinity;
}

export function stopAhead(car, stop = car.serviceStop) {
  return onStopEdge(car, stop) && (stop.x - car.x) * stop.dx + (stop.y - car.y) * stop.dy >= -.5;
}

export function stopSpeedLimit(car) {
  const stop = car.serviceStop;
  if (!stopAhead(car, stop) || car.arc) return Infinity;
  const along = (stop.x - car.x) * stop.dx + (stop.y - car.y) * stop.dy;
  // Approach continuously and remain stopped until the fleet releases the stop.
  // No position writes: the normal traffic steering/integrator does the driving.
  return Math.max(0, Math.min(along * .9, Math.sqrt(5 * Math.max(0, along))));
}

export function atServiceStop(car, stop) {
  return onStopEdge(car, stop) && Math.hypot(car.x - stop.x, car.y - stop.y) < 1.1 && car.speed < .25;
}
