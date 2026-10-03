"""Bake a raw CAD/Blender mesh into a light RViz mesh (run with Blender, headless).

    blender -b --factory-startup -P models/bake_mesh.py -- \
        <input.obj> <output.glb> <target_triangles> [yaw_deg] [up_axis]

What it does:
  1. joins all parts into one mesh (fewer draw calls in RViz)
  2. rotates so the robot stands on +Z with its front along +X:
     up_axis says which axis is "up" after import (Y or Z); yaw_deg then
     turns the robot about Z so its front points along +X
  3. puts its lowest point on z = 0; x/y keep the source origin, which should
     be the robot's center of rotation (not necessarily the bounding-box center)
  4. decimates to about target_triangles
  5. stores material colors as display (sRGB) values: RViz shows the numbers
     in a mesh as-is, while Blender keeps them linear
  6. writes a Z-up .glb, which RViz loads directly (no Y-up conversion)

Raw inputs live in models/source/ (git-ignored); only the baked .glb is committed.
"""
import math
import sys

import bpy
from mathutils import Euler, Vector

args = sys.argv[sys.argv.index('--') + 1:]
src, dst, target = args[0], args[1], int(args[2])
yaw = math.radians(float(args[3])) if len(args) > 3 else 0.0
up_axis = args[4].upper() if len(args) > 4 else 'Z'

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=src)

meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']
bpy.ops.object.select_all(action='DESELECT')
for o in meshes:
    o.select_set(True)
bpy.context.view_layer.objects.active = meshes[0]
bpy.ops.object.join()
obj = bpy.context.view_layer.objects.active
# The OBJ importer stores its axis conversion as the object transform; bake it in first.
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# 1-2: orientation. Y-up -> Z-up is +90 deg about X; then yaw about Z.
turn = Euler((0.0, 0.0, yaw)).to_matrix() @ Euler(
    (math.pi / 2 if up_axis == 'Y' else 0.0, 0.0, 0.0)).to_matrix()
obj.matrix_world = turn.to_4x4() @ obj.matrix_world
bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

# OBJ meshes are often unconnected triangles, which the decimator can barely
# reduce; weld coincident vertices and drop imported custom normals first.
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.remove_doubles(threshold=1e-5)
bpy.ops.object.mode_set(mode='OBJECT')
if obj.data.has_custom_normals:
    bpy.ops.mesh.customdata_custom_splitnormals_clear()
# Plain material colors only: UVs and color attributes are unused and confuse the exporter.
while obj.data.uv_layers:
    obj.data.uv_layers.remove(obj.data.uv_layers[0])
while obj.data.color_attributes:
    obj.data.color_attributes.remove(obj.data.color_attributes[0])

# 3: ground at z = 0
obj.location = (0.0, 0.0, -min((obj.matrix_world @ Vector(c)).z for c in obj.bound_box))
bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)

# 4: decimate
obj.data.calc_loop_triangles()
before = len(obj.data.loop_triangles)
mod = obj.modifiers.new('decimate', 'DECIMATE')
mod.decimate_type = 'COLLAPSE'
mod.ratio = min(1.0, target / before)
mod.use_collapse_triangulate = True
bpy.ops.object.modifier_apply(modifier=mod.name)
# Decimation leaves a few zero-area faces, which break the glTF exporter.
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.dissolve_degenerate(threshold=1e-6)
bpy.ops.mesh.delete_loose()
bpy.ops.mesh.select_all(action='SELECT')
bpy.ops.mesh.quads_convert_to_tris()
bpy.ops.object.mode_set(mode='OBJECT')
obj.data.validate()
obj.data.calc_loop_triangles()
after = len(obj.data.loop_triangles)
# Smooth shading with hard edges above 30 deg: low-poly curves look round, edges stay crisp.
bpy.ops.object.shade_smooth()
obj.data.use_auto_smooth = True
obj.data.auto_smooth_angle = math.radians(30)


# 5: linear -> sRGB base colors
def to_srgb(c):
    return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055


for mat in obj.data.materials:
    bsdf = mat.node_tree.nodes.get('Principled BSDF') if mat and mat.use_nodes else None
    if bsdf:
        col = bsdf.inputs['Base Color'].default_value
        bsdf.inputs['Base Color'].default_value = (*(to_srgb(c) for c in col[:3]), col[3])

# 6: export, keeping Blender's Z-up axes
bpy.ops.export_scene.gltf(filepath=dst, export_format='GLB', export_yup=False,
                          export_apply=True, use_selection=False,
                          export_cameras=False, export_lights=False)

corners = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
size = [round(max(c[i] for c in corners) - min(c[i] for c in corners), 4) for i in range(3)]
print(f'BAKED {dst}: {before} -> {after} triangles, size xyz {size} m, '
      f'materials {[m.name for m in obj.data.materials if m]}')
