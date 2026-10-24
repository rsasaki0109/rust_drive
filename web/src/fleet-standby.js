// Empty-cab standby in the authored city, using public time/service state and
// observed requests only. This is a heuristic, not calibrated demand prediction.
export function makeStandbyStop(stop) {
  const along = stop.along - 12;
  return Object.freeze({ ...stop, id: 'standby-' + stop.id, label: stop.label + ' STANDBY',
    along, x: stop.x - stop.dx * 12, y: stop.y - stop.dy * 12 });
}

export function standbyPriorities(stops, clock, time, requests, waiting, train) {
  return stops.map(stop => {
    let score = .25, reason = 'district coverage';
    const add = (value, text) => { score += value; if (value > 0) reason = text; };
    if (clock.night) {
      if (stop.id === 'nightlife') add(5, 'late-night venues');
      if (stop.id === 'hotel') add(1.6, 'night arrivals');
    } else if (clock.hour >= 17) {
      if (stop.id === 'office') add(2, 'evening departures');
      if (stop.id === 'nightlife') add(1.8, 'evening venues');
    } else {
      if (stop.id === 'office') add(2.2, 'daytime departures');
      if (['homes', 'west'].includes(stop.id)) add(1, 'residential coverage');
      if (stop.id === 'market') add(.8, 'daytime visits');
    }
    if (stop.id === 'station' && clock.railOpen) add(1.2, 'rail service');
    if (stop.id === 'station' && train && ['approach', 'dwell'].includes(train.phase))
      add(train.lastService ? 4 : 2.5, train.lastService ? 'last-train connection' : 'inbound rail service');
    const observed = requests.filter(e => e.type === 'request' && e.stop === stop.id &&
      e.time <= time && time - e.time < 60)
      .reduce((n, e) => n + (1 - (time - e.time) / 60), 0);
    add(observed * .8, 'recent ride requests');
    const queued = waiting.filter(p => p.origin.id === stop.id &&
      ['walking', 'waiting'].includes(p.state) && !p.taxi).length;
    add(queued * 2, 'waiting riders');
    return { stop, score, reason };
  }).sort((a, b) => b.score - a.score || a.stop.id.localeCompare(b.stop.id));
}
