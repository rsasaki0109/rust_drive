import * as THREE from 'three';
import { STREETS } from './city.js';

export function createStormVisuals(scene) {
  const count=1000, seeds=Array.from({length:count},(_,i)=>({x:((i*73.131)%101)-50,y:((i*31.913)%101)-50,z:((i*17.731)%37)}));
  const rainPositions=new Float32Array(count*6),snowPositions=new Float32Array(count*3);
  const rainGeometry=new THREE.BufferGeometry(),snowGeometry=new THREE.BufferGeometry();
  rainGeometry.setAttribute('position',new THREE.BufferAttribute(rainPositions,3).setUsage(THREE.DynamicDrawUsage));
  snowGeometry.setAttribute('position',new THREE.BufferAttribute(snowPositions,3).setUsage(THREE.DynamicDrawUsage));
  const rain=new THREE.LineSegments(rainGeometry,new THREE.LineBasicMaterial({color:0xa6d9f3,transparent:true,opacity:.6,depthWrite:false}));
  const snow=new THREE.Points(snowGeometry,new THREE.PointsMaterial({color:0xf5f8ff,size:.23,transparent:true,opacity:.9,depthWrite:false}));
  rain.frustumCulled=snow.frustumCulled=false;scene.add(rain,snow);
  const dustMaterial=new THREE.MeshStandardMaterial({color:0xe0eaf0,transparent:true,opacity:0,depthWrite:false,roughness:.95});
  for(const street of STREETS)for(const vertical of [false,true]) {
    const patch=new THREE.Mesh(new THREE.PlaneGeometry(vertical?18:380,vertical?380:18),dustMaterial);
    patch.rotation.x=-Math.PI/2;patch.position.set(vertical?street:0,.025,vertical?0:-street);scene.add(patch);
  }
  const cloudPositions=new Float32Array(360*3),cloudGeometry=new THREE.BufferGeometry();
  cloudGeometry.setAttribute('position',new THREE.BufferAttribute(cloudPositions,3).setUsage(THREE.DynamicDrawUsage));
  const cloud=new THREE.Points(cloudGeometry,new THREE.PointsMaterial({color:0x79ffcd,size:.28,depthTest:false,transparent:true,opacity:.95}));
  cloud.frustumCulled=false;cloud.renderOrder=5;scene.add(cloud);
  const rings=Array.from({length:120},()=>{const mesh=new THREE.Mesh(new THREE.RingGeometry(.97,1,32),new THREE.MeshBasicMaterial({color:0xffc76a,side:THREE.DoubleSide,depthTest:false,transparent:true,opacity:.8}));mesh.rotation.x=-Math.PI/2;mesh.visible=false;mesh.renderOrder=6;scene.add(mesh);return mesh;});
  return {
    update(weather,strength,time,car,sensor,show=true) {
      rain.visible=weather==='rain'&&strength>0;snow.visible=weather==='snow'&&strength>0;dustMaterial.opacity=weather==='snow'?strength*.36:0;
      rainGeometry.setDrawRange(0,Math.floor(count*strength)*2);snowGeometry.setDrawRange(0,Math.floor(count*strength));
      for(let i=0;i<count;i++){
        const p=seeds[i],x=car.x+p.x,y=-car.y+p.y,drop=((p.z-time*(weather==='rain'?18:1.6))%37+37)%37;
        rainPositions.set([x,drop,y,x-.15,drop+1.8,y+.15],i*6);
        snowPositions.set([x+Math.sin(time*.5+i)*.8,drop,y+Math.cos(time*.35+i)*.7],i*3);
      }
      rainGeometry.attributes.position.needsUpdate=true;snowGeometry.attributes.position.needsUpdate=true;
      cloud.visible=show&&Boolean(sensor?.ready);rings.forEach(r=>r.visible=false);
      if(!sensor?.ready){cloudGeometry.setDrawRange(0,0);return;}
      const pose=sensor.pose,c=Math.cos(pose.yaw),s=Math.sin(pose.yaw);
      sensor.points.forEach((p,i)=>cloudPositions.set([pose.x+p.x*c-p.y*s,pose.height+1.1,-pose.y-p.x*s-p.y*c],i*3));
      cloudGeometry.attributes.position.needsUpdate=true;cloudGeometry.setDrawRange(0,sensor.points.length);
      if(show)sensor.detections.slice(0,rings.length).forEach((d,i)=>{const ring=rings[i];ring.visible=true;ring.position.set(pose.x+d.x*c-d.y*s,pose.height+.12,-pose.y-d.x*s-d.y*c);ring.scale.setScalar(d.radius);});
    },
  };
}
