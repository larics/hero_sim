"""Import AWS RoboMaker's Small House world as hero_sim's `small_house` map.

    git clone --depth 1 --filter=blob:none --sparse -b ros2 \\
        https://github.com/aws-robotics/aws-robomaker-small-house-world.git /tmp/small_house
    git -C /tmp/small_house sparse-checkout set models worlds maps photos
    python3 maps/import_small_house.py /tmp/small_house

The world is a furnished one-floor house, about 19 x 11 m, with an open plan:
a bedroom, a living room with a gym corner, a dining room and kitchen, and a
sunroom. It writes maps/small_house/:

  small_house.yaml, .pgm  the world's own 2D map (a ROS map_server image map,
                          made with a TurtleBot3's lidar): what RViz shows
                          and what robots collide with (hero_sim/obstacles.py)
  small_house.sdf         the world's furniture and lights, which Gazebo's
                          cameras see (hero_sim/gazebo.py map_scenery)
  models/                 the furniture models, changed for Gazebo Harmonic:
                          - no <inertial>: every model is static, and some
                            have a mass of 0, which Harmonic refuses to load
                          - textures without transparency as JPEG, not PNG, a
                            fifth of the size
                          - the portraits' photos next to their model, at most
                            PHOTO_SIZE px, instead of in a photos/ folder
                            outside it
  LICENSE, README.md      AWS's license (MIT-0) and where this came from

Needs Pillow.
"""
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

from PIL import Image

HERE = Path(__file__).resolve().parent
NAME = 'small_house'
REPO = 'https://github.com/aws-robotics/aws-robomaker-small-house-world'
JPEG_QUALITY = 90
PHOTO_SIZE = 1024  # px, longest side of a portrait's photo


def _has_transparency(image):
    if image.mode not in ('RGBA', 'LA', 'P') and 'transparency' not in image.info:
        return False
    return image.convert('RGBA').getchannel('A').getextrema()[0] < 255


def import_model(src, dst, photos):
    """Copy one model from src to dst, changed for Harmonic (see the module docstring)."""
    dst.mkdir(parents=True)
    shutil.copy(src / 'model.config', dst / 'model.config')
    sdf = (src / 'model.sdf').read_text()
    (dst / 'model.sdf').write_text(re.sub(r'\s*<inertial>.*?</inertial>', '', sdf, flags=re.S))

    renames = {}  # texture file name in the .DAEs -> new name
    textures = src / 'materials' / 'textures'
    if textures.is_dir():
        (dst / 'materials' / 'textures').mkdir(parents=True)
        for tex in sorted(textures.iterdir()):
            out = dst / 'materials' / 'textures' / tex.name
            with Image.open(tex) as image:
                if tex.suffix.lower() == '.png' and not _has_transparency(image):
                    out = out.with_suffix('.jpg')
                    image.convert('RGB').save(out, quality=JPEG_QUALITY)
                    renames[tex.name] = out.name
                else:
                    shutil.copy(tex, out)

    (dst / 'meshes').mkdir()
    for mesh in sorted((src / 'meshes').iterdir()):
        text = mesh.read_text(errors='surrogateescape')
        for old, new in renames.items():
            text = text.replace(f'../materials/textures/{old}<', f'../materials/textures/{new}<')
        for photo in sorted(set(re.findall(r'\.\./\.\./\.\./\.\./photos/([\w.]+)<', text))):
            out = dst / 'materials' / 'textures' / photo
            out.parent.mkdir(parents=True, exist_ok=True)
            with Image.open(photos / photo) as image:
                image.thumbnail((PHOTO_SIZE, PHOTO_SIZE))
                image.convert('RGB').save(out, quality=JPEG_QUALITY)
            text = text.replace(f'../../../../photos/{photo}<', f'../materials/textures/{photo}<')
        (dst / 'meshes' / mesh.name).write_text(text, errors='surrogateescape')


def scenery(world_file):
    """The world's models and lights as an SDF document, for hero_sim/gazebo.py."""
    world = ET.parse(world_file).getroot().find('world')
    body = ''.join(f'    {ET.tostring(el, encoding="unicode").strip()}\n'
                   for el in world if el.tag in ('model', 'light'))
    return ('<?xml version="1.0"?>\n'
            f'<!-- The furniture and lights of AWS RoboMaker\'s Small House ({REPO}),\n'
            '     written by maps/import_small_house.py. hero_sim/gazebo.py puts them in the\n'
            '     Gazebo world; model:// URIs are found in models/ next to this file. -->\n'
            f'<sdf version="1.6">\n  <world name="{NAME}">\n{body}  </world>\n</sdf>\n')


def main(checkout, out_dir=HERE / NAME):
    checkout, out_dir = Path(checkout), Path(out_dir)
    if out_dir.exists():
        shutil.rmtree(out_dir)
    (out_dir / 'models').mkdir(parents=True)

    world_file = checkout / 'worlds' / 'small_house.world'
    text = scenery(world_file)
    (out_dir / f'{NAME}.sdf').write_text(text)
    used = sorted(set(re.findall(r'model://([\w]+)', text)))
    for model in used:
        import_model(checkout / 'models' / model, out_dir / 'models' / model,
                     checkout / 'photos')

    nav = checkout / 'maps' / 'turtlebot3_waffle_pi'
    shutil.copy(nav / 'map.pgm', out_dir / f'{NAME}.pgm')
    meta = (nav / 'map.yaml').read_text().replace('image: map.pgm', f'image: {NAME}.pgm')
    (out_dir / f'{NAME}.yaml').write_text(
        f'# The Small House\'s own 2D map, made with a TurtleBot3\'s lidar ({REPO}).\n' + meta)

    shutil.copy(checkout / 'LICENSE', out_dir / 'LICENSE')
    commit = subprocess.run(['git', '-C', str(checkout), 'rev-parse', 'HEAD'],
                            capture_output=True, text=True).stdout.strip() or 'unknown'
    (out_dir / 'README.md').write_text(
        f'# small_house\n\nAWS RoboMaker\'s Small House world, from {REPO} '
        f'(branch ros2, commit {commit}), under its MIT-0 license (LICENSE).\n\n'
        'Imported by `maps/import_small_house.py`, whose docstring lists what it changed.\n')
    size = sum(f.stat().st_size for f in out_dir.rglob('*') if f.is_file())
    print(f'{out_dir}: {len(used)} models, {size / 1e6:.1f} MB')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
