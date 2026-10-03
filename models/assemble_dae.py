"""Combine COLLADA (.dae) parts into one OBJ+MTL, placed like URDF links.

    python3 assemble_dae.py <out.obj> <part.dae>:x,y,z,roll,pitch,yaw [...]

Each part is placed with URDF conventions (translation, then fixed-axis
roll/pitch/yaw), relative to the robot's ground frame. Faces with the same
diffuse color are merged into one material, so RViz draws the whole robot in
a few calls. The output (Y-up, the OBJ convention) feeds bake_mesh.py with
up_axis Z (Blender here can't read .dae).

Needs pycollada (`pip install pycollada`); it isn't in the docker image, so a
one-off container works. The scout (agilex_sitl meshes) was made with:

    docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp -v $PWD:/w -w /w \
        -v /home/khawaja/HERO/agilex_sitl/agilex_sitl/meshes:/meshes:ro icuas26_finals_image \
        bash -c 'pip install -q --user pycollada && python3 models/assemble_dae.py \
          models/source/scout/scout_mini.obj \
          /meshes/scout_mini_base_link.dae:0,0,0.237,1.5708,0,-1.5708 \
          /meshes/wheel.dae:0.226,0.205,0.16,-1.5708,0,0 \
          /meshes/wheel.dae:0.226,-0.205,0.16,1.5708,0,0 \
          /meshes/wheel.dae:-0.226,0.205,0.16,-1.5708,0,0 \
          /meshes/wheel.dae:-0.226,-0.205,0.16,1.5708,0,0'
    blender -b --factory-startup -P models/bake_mesh.py -- \
        models/source/scout/scout_mini.obj models/scout/scout.glb 12000 0 Z

The base .dae says Z-up but its geometry is Y-up with the length along Z, hence
roll 90 / yaw -90 (crash bar to the front, +X); it is already centered on its
suspension arms. Wheels follow the Scout Mini spec (wheelbase 452 mm, track
490 mm; agilex_sitl's xacro values put them 7 cm too far in for this body),
mirrored left/right so rims face out; this wheel mesh's tire sits 4 cm
outboard of its origin, hence y = 0.245 - 0.04. The bake grounds the wheels.
"""
import sys

import collada
import numpy as np


def urdf_transform(x, y, z, roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = (x, y, z)
    return t


def srgb_to_linear(c):
    # bake_mesh.py stores colors back as sRGB; MTL/Blender colors are linear.
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def triangles(path):
    """Yield (Nx3x3 triangle vertices in meters, Z-up; rgb) for every primitive."""
    dae = collada.Collada(path, ignore=[collada.common.DaeUnsupportedError,
                                        collada.common.DaeBrokenRefError])
    unit = dae.assetInfo.unitmeter or 1.0
    y_up = dae.assetInfo.upaxis == 'Y_UP'
    for geom in dae.scene.objects('geometry'):  # node transforms already applied
        for prim in geom.primitives():
            if not hasattr(prim, 'vertex_index'):
                continue  # lines etc.
            if not isinstance(prim, collada.triangleset.BoundTriangleSet):
                prim = prim.triangleset()
            v = prim.vertex[prim.vertex_index] * unit  # (T, 3, 3)
            if y_up:
                v = v[..., [0, 2, 1]] * np.array([1, -1, 1])
            diffuse = getattr(getattr(prim.material, 'effect', None), 'diffuse', None)
            rgb = tuple(float(c) for c in diffuse[:3]) if isinstance(diffuse, tuple) else (0.6,) * 3
            yield v, rgb


def main(out, parts):
    by_color = {}
    for spec in parts:
        path, pose = spec.rsplit(':', 1)
        t = urdf_transform(*(float(p) for p in pose.split(',')))
        for v, rgb in triangles(path):
            v = v @ t[:3, :3].T + t[:3, 3]
            by_color.setdefault(tuple(round(c, 3) for c in rgb), []).append(v)

    mtl = out.rsplit('.', 1)[0] + '.mtl'
    with open(mtl, 'w') as f:
        for i, rgb in enumerate(by_color):
            kd = ' '.join(f'{srgb_to_linear(c):.6f}' for c in rgb)
            f.write(f'newmtl color_{i}\nKd {kd}\n\n')
    n_tris, offset = 0, 1
    with open(out, 'w') as f:
        f.write(f'mtllib {mtl.rsplit("/", 1)[-1]}\n')
        for i, chunks in enumerate(by_color.values()):
            tris = np.concatenate(chunks)
            f.write(f'usemtl color_{i}\n')
            # OBJ convention is Y-up (-Z forward); Blender's importer maps it back to Z-up.
            f.writelines(f'v {x:.6f} {z:.6f} {-y:.6f}\n' for x, y, z in tris.reshape(-1, 3))
            f.writelines(f'f {offset + 3 * k} {offset + 3 * k + 1} {offset + 3 * k + 2}\n'
                         for k in range(len(tris)))
            offset += 3 * len(tris)
            n_tris += len(tris)
    print(f'ASSEMBLED {out}: {n_tris} triangles, {len(by_color)} colors')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2:])
