// Planar first-return instrument over the arcade world's collision prisms.
// World geometry stays here. The Rust brake receives body-frame returns only.
export const LIDAR_RAYS = 360, LIDAR_RANGE = 45;
export function rayBox(ox, oy, dx, dy, box, height) {
  if (box.broken || height < (box.bottom ?? -1000) || height > (box.top ?? 32)) return Infinity;
  const c = Math.cos(box.yaw ?? 0), s = Math.sin(box.yaw ?? 0);
  const x = (ox - box.x) * c + (oy - box.y) * s, y = -(ox - box.x) * s + (oy - box.y) * c;
  const vx = dx * c + dy * s, vy = -dx * s + dy * c;
  let enter = -Infinity, exit = Infinity;
  for (const [p, v, half] of [[x,vx,box.w],[y,vy,box.h]]) {
    if (Math.abs(v) < 1e-12) { if (Math.abs(p) > half) return Infinity; continue; }
    const a = (-half - p) / v, b = (half - p) / v;
    enter = Math.max(enter, Math.min(a,b)); exit = Math.min(exit, Math.max(a,b));
  }
  return exit >= Math.max(.2,enter) ? Math.max(.2,enter) : Infinity;
}
export function rayDisc(ox, oy, dx, dy, disc, height) {
  if (height < disc.bottom || height > disc.top) return Infinity;
  const x = disc.x - ox, y = disc.y - oy, along = x * dx + y * dy, cross = x * dy - y * dx;
  const discriminant = disc.radius ** 2 - cross ** 2;
  if (discriminant < 0) return Infinity;
  const near = along - Math.sqrt(discriminant), far = along + Math.sqrt(discriminant);
  return far >= Math.max(.2,near) ? Math.max(.2,near) : Infinity;
}
export function visiblePerson(p, time) {
  return !['scheduled','riding','arrived','on-train','riding-train'].includes(p.state) &&
    !(p.state === 'cancelled' && (p.insideBuilding || time - p.finishedAt > 6));
}
export function turnIntent(car) {
  if (!car.arc) return { curvature: 0, bend: 0 };
  const a = car.arc, angle = Math.atan2(car.y - a.y, car.x - a.x);
  const wrap = v => Math.atan2(Math.sin(v), Math.cos(v));
  const progress = Math.max(0, Math.sign(a.delta) * wrap(angle - a.angle));
  return { curvature: Math.sign(a.delta) / a.radius, bend: Math.max(0, Math.abs(a.delta) - progress) * a.radius };
}
export function measureScan(car, cars, people, colliders, time) {
  const height = (car.height ?? 0) + 1.1;
  const near = colliders.filter(b => !b.broken && Math.abs(b.x-car.x) < LIDAR_RANGE + b.w && Math.abs(b.y-car.y) < LIDAR_RANGE + b.h);
  const boxes = [...near, ...cars.filter(c => c !== car && Math.hypot(c.x-car.x,c.y-car.y) < 50)
    .map(c => ({ x:c.x,y:c.y,w:2.15,h:.95,yaw:c.yaw,bottom:c.height??0,top:(c.height??0)+1.55 }))];
  const discs = people.filter(p => visiblePerson(p,time) && Math.hypot(p.x-car.x,p.y-car.y) < 47)
    .map(p => ({ x:p.x,y:p.y,radius:.32,bottom:p.height??0,top:(p.height??0)+1.75 }));
  return Array.from({length:LIDAR_RAYS},(_,i) => {
    const angle = car.yaw - Math.PI + Math.PI*2*i/LIDAR_RAYS, dx=Math.cos(angle),dy=Math.sin(angle);
    let range=LIDAR_RANGE;
    for(const b of boxes) range=Math.min(range,rayBox(car.x,car.y,dx,dy,b,height));
    for(const p of discs) range=Math.min(range,rayDisc(car.x,car.y,dx,dy,p,height));
    return range;
  });
}
export function sensorLimit(car, time) {
  return car.sensor?.ready && time - car.sensor.stamp <= .6 && time >= car.sensor.stamp ? car.sensor.limit : 0;
}
export class FleetLidar {
  constructor(drive, colliders) {
    this.drive=drive; this.colliders=colliders; this.next=new Map(); this.samples=0; this.brakeSeconds=0; this.staleSeconds=0;
  }
  update(fleet, traffic, dt) {
    const code={clear:0,rain:1,snow:2}[fleet.weather];
    for(const [slot,taxi] of fleet.taxis.entries()) {
      const car=taxi.car; car.sensorDriven=true;
      if(!this.next.has(slot)) this.next.set(slot,fleet.time+(slot%8)*.025);
      if(fleet.time >= this.next.get(slot)) {
        this.next.set(slot,fleet.time+.2);
        const ranges=measureScan(car,traffic.cars,fleet.people,this.colliders,fleet.time);
        const p=this.drive, turn=turnIntent(car);
        let ok=p.fleet_lidar_begin(slot,fleet.time,code,fleet.weatherStrength);
        for(let i=0;ok&&i<ranges.length;i++) ok=p.fleet_lidar_ray(i,ranges[i]);
        if(ok) ok=p.fleet_lidar_finish(slot,Math.max(0,car.speed),turn.curvature,Math.min(45,turn.bend));
        const n=ok?p.fleet_lidar_read(slot,2):0,d=ok?p.fleet_lidar_read(slot,3):0;
        car.sensor={ready:Boolean(ok),stamp:ok?p.fleet_lidar_read(slot,0):fleet.time,limit:ok?p.fleet_lidar_read(slot,5):0,
          nearest:ok?p.fleet_lidar_read(slot,4):0,baseReturns:ok?p.fleet_lidar_read(slot,7):0,
          points:Array.from({length:n},(_,i)=>({x:p.fleet_lidar_point(slot,i,0),y:p.fleet_lidar_point(slot,i,1)})),
          detections:Array.from({length:d},(_,i)=>({x:p.fleet_lidar_detection(slot,i,0),y:p.fleet_lidar_detection(slot,i,1),radius:p.fleet_lidar_detection(slot,i,2)})),
          pose:{x:car.x,y:car.y,yaw:car.yaw,height:car.height??0}};
        this.samples++;
      }
      if(!car.sensor?.ready || fleet.time-car.sensor.stamp>.6) this.staleSeconds+=dt;
      else if(car.sensor.limit < car.speed-.25) this.brakeSeconds+=dt;
    }
  }
  snapshot(selected) {
    return {samples:this.samples,brakeSeconds:this.brakeSeconds,staleSeconds:this.staleSeconds,
      sensor:selected?.car.sensor??null,rayCount:LIDAR_RAYS,maxRange:LIDAR_RANGE,
      scope:'planar collision-prism instrument + Rust weather/perception/brake; arcade map/signal steering'};
  }
}

export function boxesOverlap(a,b) {
  for(const yaw of [a.yaw??0,b.yaw??0]) for(const extra of [0,Math.PI/2]) {
    const t=yaw+extra, x=Math.cos(t),y=Math.sin(t),delta=Math.abs((b.x-a.x)*x+(b.y-a.y)*y);
    const extent=q=>q.w*Math.abs(Math.cos(q.yaw??0)*x+Math.sin(q.yaw??0)*y)+q.h*Math.abs(-Math.sin(q.yaw??0)*x+Math.cos(q.yaw??0)*y);
    if(delta>=extent(a)+extent(b)) return false;
  }
  return (a.top??1.55)>(b.bottom??0)&&(b.top??1.55)>(a.bottom??0);
}
export class ContactEvaluator {
  constructor(){this.active=new Set();this.events=[];}
  update(fleet,traffic,colliders){
    const current=new Set(), controlled=new Set(fleet.taxis.map(t=>t.car));
    for(const taxi of fleet.taxis) {
      const car=taxi.car,a={x:car.x,y:car.y,yaw:car.yaw,w:2.15,h:.95,bottom:car.height??0,top:(car.height??0)+1.55};
      for(const other of traffic.cars) {
        if(other===car||(controlled.has(other)&&other.id<car.id)) continue;
        const b={x:other.x,y:other.y,yaw:other.yaw,w:2.15,h:.95,bottom:other.height??0,top:(other.height??0)+1.55};
        if(Math.hypot(a.x-b.x,a.y-b.y)<6&&boxesOverlap(a,b)) current.add('car:'+car.id+':'+other.id);
      }
      colliders.forEach((b,i)=>{if(!b.broken&&Math.abs(a.x-b.x)<b.w+3&&Math.abs(a.y-b.y)<b.h+3&&boxesOverlap(a,b))current.add('body:'+car.id+':'+i);});
      for(const p of fleet.people) if(visiblePerson(p,fleet.time)&&!(p.taxi===taxi.id&&['boarding','alighting'].includes(p.state)&&car.speed<.25)&&Math.abs((p.height??0)-a.bottom)<1.5) {
        const c=Math.cos(car.yaw),s=Math.sin(car.yaw),x=(p.x-car.x)*c+(p.y-car.y)*s,y=-(p.x-car.x)*s+(p.y-car.y)*c;
        if(Math.hypot(x-Math.max(-2.15,Math.min(2.15,x)),y-Math.max(-.95,Math.min(.95,y)))<.32)current.add('person:'+car.id+':'+p.id);
      }
    }
    for(const pair of current)if(!this.active.has(pair))this.events.push({time:fleet.time,pair});
    this.active=current;
  }
  snapshot(){return{episodes:this.events.length,active:this.active.size,events:this.events.map(e=>({...e})),scope:'body-proxy overlaps; no contact-force or safety qualification'};}
}
