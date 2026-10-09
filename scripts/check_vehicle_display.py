"""Independent evaluated-mesh SI display audit (no sensing/collision claim).

blender --background --factory-startup --threads 2 --python-exit-code 1 --python
scripts/check_vehicle_display.py -- --output assets/vehicle-display-audit.json
SPDX-License-Identifier: Apache-2.0
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from blender_assets import car, material

# Independent acceptance specification, not imported from the constructor.
EXPECTED = {"hatchback": (4.25, 1.78, 1.48), "sedan": (4.65, 1.82, 1.46),
            "van": (5.0, 1.95, 2.05), "pickup": (5.35, 1.95, 1.82)}
RADII = (1.0, 2.2847319317591728)
ANGLES = (0.0, .19, .73, 1.57, 2.81)
TOLERANCE = 2e-5


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def bounds(points):
    require(bool(points), "Missing evaluated geometry")
    low = [min(point[axis] for point in points) for axis in range(3)]
    high = [max(point[axis] for point in points) for axis in range(3)]
    return {"minimum_m": low, "maximum_m": high,
            "extent_m": [b-a for a,b in zip(low, high)]}


def mesh_points(obj, reference, graph):
    evaluated = obj.evaluated_get(graph)
    mesh = evaluated.to_mesh()
    try:
        transform = reference.matrix_world.inverted() @ evaluated.matrix_world
        return [transform @ vertex.co for vertex in mesh.vertices]
    finally:
        evaluated.to_mesh_clear()


def inspect(root, variant, ego):
    graph = bpy.context.evaluated_depsgraph_get()
    all_points, body, mirrors, sensors = [], [], [], []
    wheels = [obj for obj in root.children if obj.get("rolling_wheel")]
    require(len(wheels) == 4, "Each model needs four independently circular tires")
    tire_radii = []
    for obj in root.children_recursive:
        if obj.type not in ("MESH", "CURVE"):
            continue
        points = mesh_points(obj, root, graph)
        all_points.extend(points)
        part = obj.get("vehicle_display_part")
        if part == "body":
            body.extend(points)
        elif part == "mirror":
            mirrors.extend(points)
        elif part == "sensor":
            sensors.extend(points)
        if obj.name.startswith("Tire"):
            require(obj.parent in wheels, "Tire does not belong to a rolling wheel")
            local = mesh_points(obj, obj.parent, graph)
            radial = [math.hypot(point.x, point.z) for point in local]
            require(max(radial)-min(radial) < TOLERANCE, "Display tire is stretched/elliptical")
            require(abs(radial[0]-root["wheel_radius"]) < TOLERANCE,
                    "Wheel animation radius differs from actual tire radius")
            tire_radii.append(radial[0])
    require(len(tire_radii) == 4, "Missing tire geometry")
    body_bounds, full_bounds = bounds(body), bounds(all_points)
    length, width, height = EXPECTED[variant]
    require(abs(body_bounds["extent_m"][0]-length) < TOLERANCE, "Incorrect SI body length")
    require(abs(body_bounds["extent_m"][1]-width) < TOLERANCE, "Incorrect SI body width")
    require(abs(body_bounds["maximum_m"][2]-height) < TOLERANCE,
            f"Incorrect {variant} roof height above datum: {body_bounds['maximum_m'][2]} vs {height}")
    require(full_bounds["minimum_m"][2] >= -TOLERANCE, "Wheel/geometry penetrates flat datum")
    require(max(math.hypot(point.x, point.y) for point in all_points) <= root["display_radius_m"]+TOLERANCE,
            "Actual mesh exceeds camera horizontal bound")
    require(full_bounds["maximum_m"][2] <= root["display_height_m"]+TOLERANCE,
            "Actual mesh exceeds camera height bound")
    require(bool(sensors) == ego, "Incorrect ego sensor presence")
    sensor_bounds = bounds(sensors) if sensors else None
    if sensors:
        require(sensor_bounds["minimum_m"][2] >= height-TOLERANCE,
                "Ego sensor intersects/buries below variant roof")
    return {"body": body_bounds, "mirrors": bounds(mirrors),
            "sensor": sensor_bounds, "full_display": full_bounds,
            "actual_tire_radii_m": tire_radii}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("assets/vehicle-display-audit.json"))
    args = parser.parse_args(sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else [])
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    materials = [material("Audit "+name, color) for name,color in
                 [("paint", (.1,.3,.6)), ("glass", (.1,.2,.3)),
                  ("tire", (.02,.02,.02)), ("headlight", (.8,.8,.7))]]
    cases = []
    for variant in EXPECTED:
        for ego in (False, True):
            reference_measurements = None
            for radius in RADII:
                root = car("SI audit "+variant, radius, *materials, ego=ego, variant=variant)
                root.location, root.rotation_euler = (17.0, -9.0, 0.0), (0.0, 0.0, .72)
                initial_pose = (tuple(root.location), tuple(root.rotation_euler), tuple(root.scale))
                expected = EXPECTED[variant]
                require(all(abs(root["display_dimensions_m"][key]-value) < TOLERANCE
                            for key,value in zip(("length", "width", "height"), expected)),
                        "Incorrect dimension metadata")
                require(root["declared_circle_radius"] == radius, "Physical radius metadata lost")
                measurements = []
                for angle in ANGLES:
                    for wheel in root.children:
                        if wheel.get("rolling_wheel"):
                            wheel.rotation_euler.y = angle
                    bpy.context.view_layer.update()
                    measurements.append(inspect(root, variant, ego))
                    require(initial_pose == (tuple(root.location), tuple(root.rotation_euler), tuple(root.scale)),
                            "Animation altered root pose/scale")
                if reference_measurements is None:
                    reference_measurements = measurements
                else:
                    for actual, reference in zip(measurements, reference_measurements):
                        for key in ("body", "mirrors", "sensor", "full_display"):
                            if actual[key] is None:
                                require(reference[key] is None, "Radius changed sensor presence")
                            else:
                                require(all(abs(a-b) < TOLERANCE for a,b in
                                            zip(actual[key]["minimum_m"]+actual[key]["maximum_m"],
                                                reference[key]["minimum_m"]+reference[key]["maximum_m"])),
                                        "Physical radius incorrectly scales display geometry")
                cases.append({"model": variant, "ego_sensor": ego, "physical_radius_m": radius,
                              "preset_dimensions_m": list(expected), "root_pose_unchanged": True,
                              "wheel_radius_m": root["wheel_radius"],
                              "camera_display_radius_m": root["display_radius_m"],
                              "camera_display_height_m": root["display_height_m"],
                              "wheel_angles_rad": list(ANGLES), "measurements": measurements})
    repository = Path(__file__).resolve().parent.parent
    report = {"schema": "rustdrive-vehicle-display-audit-v1", "passed": True,
              "display_only": True,
              "scope": "Original preset SI cosmetics. Independent evaluated mesh bounds, round tires, root pose and physical-radius invariance; not physical collision or sensor-body equivalence.",
              "dimension_scope": "Length and width include body handles/bumpers, exclude mirrors/wheels/sensor. Height is datum-to-roof, not body mesh bounding-box extent.",
              "blender_version": bpy.app.version_string, "units": "metres",
              "tolerance_m": TOLERANCE, "models": 4, "cases": len(cases),
              "evaluated_pose_cases": len(cases)*len(ANGLES),
              "source_sha256": {str(path.relative_to(repository)): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (Path(__file__).resolve(), repository/"scripts/blender_assets.py")},
              "results": cases}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("schema", "passed", "cases", "evaluated_pose_cases", "source_sha256")}))


if __name__ == "__main__":
    main()
