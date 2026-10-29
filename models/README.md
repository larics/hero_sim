# Robot meshes

RViz draws Crazyflies, and ground robots with a `model:`, with the meshes in `models/<name>/`. They are simplified for RViz (herover 309k → 20k triangles, scout 349k → 12k).

To add or re-make one, put the raw mesh in `models/source/` (not committed) and run:

```bash
blender -b --factory-startup -P models/bake_mesh.py -- \
    "models/source/herover/hero mini rover.obj" models/herover/herover.glb 6000 -90 Y
```

Robots made of several `.dae` parts (like the scout, from agilex_sitl's meshes) are first combined into one OBJ with `models/assemble_dae.py`; its header has the command. Then register the mesh in `MODEL_MESHES` in `hero_sim/config.py`.
