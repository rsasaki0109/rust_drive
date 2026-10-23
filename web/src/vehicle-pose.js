// Physics uses east/north metres and left-positive yaw/steering.
// Three.js uses X east, Y up, Z south; the model points along local +X.
export function applyVehiclePose(mesh, state) {
  const { x, y, yaw, height = 0, roll = 0, pitch = 0, steering = 0, distance = 0 } = state;
  mesh.position.set(x, height + .02, -y);
  mesh.rotation.set(roll, yaw, pitch, 'YXZ');
  for (const { root, wheel, front } of mesh.userData.wheels) {
    root.rotation.y = front ? steering : 0;
    // The cylinder's local Y axle becomes model Z after the fixed quarter turn.
    // Spin about that axle before the fixed orientation, retaining circular wheels.
    wheel.rotation.set(Math.PI / 2, -distance / .32, 0);
  }
}
