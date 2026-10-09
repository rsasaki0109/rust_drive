"""Render original VRU display meshes for inspection, not a driving record.

Usage: blender --background --factory-startup --threads 2 --python
       scripts/render_vru_models.py -- assets/vru-models.png
SPDX-License-Identifier: Apache-2.0
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
import blender_vru_assets as vru
from blender_assets import cube, material


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pose(root):
    return {"position_m": list(root.location),
            "rotation_euler_rad": list(root.rotation_euler), "scale": list(root.scale)}


def audit(root, speed):
    before = pose(root)
    maximum_height = maximum_radius = 0.0
    poses = [0.0, .18, .39, .62, 1.1]
    for time in poses:
        vru.animate(root, time, speed)
        bpy.context.view_layer.update()
        if pose(root) != before:
            raise RuntimeError("Cosmetic articulation changed the root pose")
        graph = bpy.context.evaluated_depsgraph_get()
        inverse = root.matrix_world.inverted()
        for obj in root.children_recursive:
            if obj.type not in ("MESH", "CURVE"):
                continue
            evaluated = obj.evaluated_get(graph)
            mesh = evaluated.to_mesh()
            try:
                transform = inverse @ evaluated.matrix_world
                for vertex in mesh.vertices:
                    point = transform @ vertex.co
                    maximum_height = max(maximum_height, point.z)
                    maximum_radius = max(maximum_radius, math.hypot(point.x, point.y))
            finally:
                evaluated.to_mesh_clear()
    if (maximum_height > root["display_height_m"] + 1e-6 or
            maximum_radius > root["display_radius_m"] + 1e-6):
        raise RuntimeError("Evaluated display geometry exceeds declared camera bounds")
    vru.animate(root, .39, speed)
    return {"kind": root["vru_kind"], "root_pose": before,
            "root_pose_unchanged": True, "checked_times_s": poses,
            "cosmetic_speed_mps": speed, "rendered_cosmetic_time_s": .39,
            "declared_simulation_circle_radius_m": root["declared_circle_radius"],
            "camera_display_height_m": root["display_height_m"],
            "camera_display_radius_m": root["display_radius_m"],
            "evaluated_max_height_m": maximum_height,
            "evaluated_max_horizontal_radius_m": maximum_radius}


def area_light(name, location, energy, size, target):
    data = bpy.data.lights.new(name, "AREA")
    data.energy, data.size = energy, size
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.location = location
    obj.rotation_euler = (Vector(target)-obj.location).to_track_quat("-Z", "Y").to_euler()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--samples", type=int, default=12)
    args = parser.parse_args(sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else [])
    if not 1 <= args.samples <= 128:
        parser.error("samples must be between 1 and 128")
    repository = Path(__file__).resolve().parent.parent
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    sources = [Path(__file__).resolve(), repository/"scripts/blender_vru_assets.py",
               repository/"scripts/blender_assets.py"]
    source_hashes = {str(path.relative_to(repository)): sha256(path) for path in sources}
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    pedestrian = vru.pedestrian("Original pedestrian", .4)
    cyclist = vru.cyclist("Original cyclist", .85)
    pedestrian.location = (-1., -.75, 0)
    cyclist.location = (1., .5, 0)
    audits = [audit(pedestrian, 1.3), audit(cyclist, 3.)]
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = args.samples
    scene.cycles.max_bounces = 3
    scene.cycles.use_denoising = False
    scene.render.resolution_x, scene.render.resolution_y = 1000, 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.world.use_nodes = True
    background = scene.world.node_tree.nodes["Background"]
    background.inputs[0].default_value = (.64, .76, .92, 1)
    background.inputs[1].default_value = .5
    cube("Showroom ground", (0, 0, -.055), (30, 30, .1),
         material("Showroom concrete", (.32, .35, .37)))
    area_light("Key", (2, -4, 6), 1400, 5, (0, 0, 1))
    data = bpy.data.cameras.new("Camera")
    camera = bpy.data.objects.new("Camera", data)
    bpy.context.collection.objects.link(camera)
    camera.location = (4.6, -6.5, 3.2)
    camera.rotation_euler = (Vector((0, 0, 1))-camera.location).to_track_quat("-Z", "Y").to_euler()
    data.type, data.ortho_scale = "ORTHO", 5.2
    scene.camera = camera
    scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)
    if source_hashes != {str(path.relative_to(repository)): sha256(path) for path in sources}:
        raise RuntimeError("Display source changed during the render")
    blend = repository/"artifacts/vru-models.blend"
    blend.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(blend), compress=True)
    metadata = {"schema": "rustdrive-vru-models-v1", "display_only": True,
                "scope": "Showroom model study, not a driving record. Cosmetic meshes and estimated limb poses do not enter sensing or collision; simulation actors retain declared circles.",
                "asset_license": "Apache-2.0; original procedural assets; no external meshes",
                "source_sha256": source_hashes,
                "renderer": {"blender": bpy.app.version_string, "engine": "CYCLES",
                             "device": "CPU", "samples": args.samples, "denoising": False,
                             "resolution_px": [1000, 720], "units": "metres",
                             "threads": scene.render.threads, "threads_mode": scene.render.threads_mode},
                "model_audits": audits,
                "wheel_spokes": [obj["spoke_count"] for obj in bpy.data.objects if obj.get("spoke_count")],
                "objects": len(bpy.data.objects),
                "mesh_vertices": sum(len(obj.data.vertices) for obj in bpy.data.objects if obj.type == "MESH"),
                "image": {"file": output.name, "bytes": output.stat().st_size, "sha256": sha256(output)},
                "editable_scene": {"file": str(blend.relative_to(repository)), "sha256": sha256(blend)}}
    output.with_suffix(".json").write_text(json.dumps(metadata, indent=2)+"\n", encoding="utf-8")
    print("VRU_MODELS_AUDIT " + json.dumps(metadata))


if __name__ == "__main__":
    main()
