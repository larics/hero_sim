"""2D collisions: keep footprints out of the map, the pillars and each other. No ROS imports.

Two kinds of fixed obstacle, both in one ObstacleMap:

  map      the scenario's sim.map, one of
             octomap  a .bt, flattened once at startup: every occupied voxel
                      overlapping the height band sim.collision_z is an
                      obstacle, seen from above. The default band leaves out
                      the floor slab, ceilings and the tree canopies the
                      drones perch in, and keeps trunks, posts and walls.
             image    a ROS map_server .yaml and its image (read_image_map):
                      each dark pixel is an obstacle cell. For RViz,
                      write_image_map extrudes them into an octomap of walls
                      sim.wall_height tall.
  pillars  the scenario's top-level obstacles: list, circles (x, y, radius)
           kept exact rather than gridded. A scenario has these or a map, not
           both. For RViz, write_pillar_map turns them into an octomap.

Moving robots are circles too, passed to each move as `others`
(hero_sim/ground_robots.py does that for the ground robots).

Every robot is a disc of its footprint radius (config.FOOTPRINT_RADII), and so
is the tether object (hero_sim/tether.py). A move is checked along its
straight path, so fast robots can't tunnel through thin obstacles, and stops at
the last free point before the disc would touch one. Height is ignored: a
drone is blocked by a trunk or a pillar at any altitude.

Map lookups are O(1): ObstacleMap precomputes each cell's distance to the
nearest map obstacle, up to the largest radius it will be asked about.
"""
import math
from pathlib import Path

import numpy as np
import yaml

from hero_sim import config

_TREE_DEPTH = 16
_KEY_ORIGIN = 1 << (_TREE_DEPTH - 1)  # octomap key of coordinate 0
PILLAR_RES = 0.05  # m, cell size without a map; move() samples paths every half cell
MAP_WALL_HEIGHT = config.SIM_DEFAULTS['wall_height']  # m, default for an image map's walls


def read_bt(path):
    """Occupied leaves of an octomap .bt file.

    Returns (res, boxes): the voxel size in m, and an int array of rows
    (kx, ky, kz, size) giving each occupied leaf's min corner in octomap keys
    and its edge length in voxels (pruned leaves span several).
    """
    with open(path, 'rb') as f:
        if not f.readline().startswith(b'# Octomap OcTree binary file'):
            raise ValueError(f'{path}: not an octomap .bt file')
        header = {}
        for line in f:
            line = line.decode().strip()
            if line == 'data':
                break
            if line and not line.startswith('#'):
                key, value = line.split(None, 1)
                header[key] = value
        data = f.read()
    if header.get('id') != 'OcTree':
        raise ValueError(f'{path}: expected an OcTree, got {header.get("id")!r}')

    boxes = []
    pos = 0

    def read_node(kx, ky, kz, size):
        # 2 bits per child, children 0-7 in order: 01 free leaf, 10 occupied
        # leaf, 11 inner node (read next, depth first). Child bit 0 is +x,
        # bit 1 +y, bit 2 +z.
        nonlocal pos
        bits = data[pos] | data[pos + 1] << 8
        pos += 2
        half = size // 2
        inner = []
        for i in range(8):
            kind = bits >> 2 * i & 3
            if kind >= 2:
                child = (kx + half * (i & 1), ky + half * (i >> 1 & 1), kz + half * (i >> 2), half)
                (boxes if kind == 2 else inner).append(child)
        for child in inner:
            read_node(*child)

    read_node(0, 0, 0, 1 << _TREE_DEPTH)
    if pos != len(data):
        raise ValueError(f'{path}: {len(data) - pos} trailing bytes after the tree')
    return float(header['res']), np.array(boxes, dtype=np.int64).reshape(-1, 4)


def read_image_map(path):
    """An image map, a ROS map_server .yaml plus the image it names, as an occupancy grid.

    Returns (occupied, x0, y0, res): a bool array [row = y, col = x] whose
    cell (0, 0), the image's bottom-left pixel, has its corner at the .yaml's
    origin, and the pixel size `resolution` in m. As in map_server, a pixel
    is occupied when its darkness (255 - gray) / 255, or gray / 255 with
    negate, is over occupied_thresh; gray is the mean of the color channels.
    Everything else is free, including what map_server would call unknown,
    and transparent pixels. The image path is relative to the .yaml.
    """
    path = Path(path)
    with open(path) as f:
        meta = yaml.safe_load(f)
    if not isinstance(meta, dict):
        raise ValueError(f'{path}: not a map .yaml (map_server format)')
    missing = [k for k in ('image', 'resolution', 'origin') if k not in meta]
    if missing:
        raise ValueError(f'{path}: missing {", ".join(missing)}')
    res = float(meta['resolution'])
    if res <= 0.0:
        raise ValueError(f'{path}: resolution must be > 0, got {res}')
    origin = meta['origin']
    if not (isinstance(origin, (list, tuple)) and len(origin) in (2, 3)):
        raise ValueError(f'{path}: origin must be [x, y, yaw], got {origin!r}')
    if len(origin) == 3 and float(origin[2]) != 0.0:
        raise ValueError(f'{path}: a rotated map (origin yaw {origin[2]}) is not supported')
    mode = meta.get('mode', 'trinary')
    if mode not in ('trinary', 'scale'):
        raise ValueError(f'{path}: mode {mode!r} is not supported, only trinary or scale')
    threshold = float(meta.get('occupied_thresh', 0.65))
    from PIL import Image  # only image maps need Pillow
    with Image.open(path.parent / meta['image']) as im:
        rgba = np.asarray(im.convert('RGBA'), dtype=float)
    gray = rgba[..., :3].mean(axis=2)
    darkness = gray / 255.0 if meta.get('negate', 0) else (255.0 - gray) / 255.0
    occupied = (darkness > threshold) & (rgba[..., 3] >= 128)
    return occupied[::-1], float(origin[0]), float(origin[1]), res


class ObstacleMap:
    """Obstacle cells on a 2D grid plus circular pillars, with the clearance to the nearest one."""

    def __init__(self, occupied, x0, y0, res, max_radius, circles=()):
        """occupied: bool array [row = y, col = x] with cell (0, 0) at (x0, y0).

        circles: pillars (x, y, radius[, height]), in m. Height is ignored.
        """
        self.res = res
        self.max_radius = max_radius
        self.circles = tuple(tuple(float(v) for v in c[:3]) for c in circles)  # drop any height
        # Pad by the farthest reach so robots just outside the obstacles' extent are covered.
        n = math.ceil(max_radius / res + 0.5)
        self.x0, self.y0 = x0 - n * res, y0 - n * res
        rows, cols = np.nonzero(occupied)
        self._clearance = np.full((occupied.shape[0] + 2 * n, occupied.shape[1] + 2 * n), np.inf)
        # Distance from a cell's centre to the nearest point of an obstacle cell
        # (di, dj) cells away. Nearest last, so it wins where reaches overlap.
        reach = sorted(
            ((math.hypot(max(abs(di) - 0.5, 0.0), max(abs(dj) - 0.5, 0.0)) * res, di, dj)
             for di in range(-n, n + 1) for dj in range(-n, n + 1)), reverse=True)
        for d, di, dj in reach:
            if d < max_radius:
                self._clearance[rows + n + di, cols + n + dj] = d

    @classmethod
    def from_bt(cls, path, z_min, z_max, max_radius, circles=()):
        """Flatten the occupied voxels of a .bt that overlap z_min..z_max (m), plus `circles`."""
        res, boxes = read_bt(path)
        z_lo = (boxes[:, 2] - _KEY_ORIGIN) * res
        z_hi = z_lo + boxes[:, 3] * res
        # Extruded walls repeat the same footprint on every layer: keep one.
        boxes = np.unique(boxes[(z_hi > z_min) & (z_lo < z_max)][:, [0, 1, 3]], axis=0)
        if not len(boxes):
            return cls(np.zeros((0, 0), bool), 0.0, 0.0, res, max_radius, circles)
        kx0, ky0 = boxes[:, :2].min(axis=0)
        kx1, ky1 = (boxes[:, :2] + boxes[:, 2:]).max(axis=0)
        occupied = np.zeros((ky1 - ky0, kx1 - kx0), bool)
        for kx, ky, size in boxes - (kx0, ky0, 0):
            occupied[ky:ky + size, kx:kx + size] = True
        return cls(occupied, (kx0 - _KEY_ORIGIN) * res, (ky0 - _KEY_ORIGIN) * res, res,
                   max_radius, circles)

    @classmethod
    def from_image(cls, path, max_radius, circles=()):
        """The occupied pixels of an image map (read_image_map), plus `circles`."""
        occupied, x0, y0, res = read_image_map(path)
        return cls(occupied, x0, y0, res, max_radius, circles)

    def clearance(self, x, y, others=()):
        """Distance (m) from (x, y) to the nearest obstacle; inf if beyond max_radius.

        others: more circles (x, y, radius) to count, such as other robots.
        """
        best = math.inf
        row = math.floor((y - self.y0) / self.res)
        col = math.floor((x - self.x0) / self.res)
        if 0 <= row < self._clearance.shape[0] and 0 <= col < self._clearance.shape[1]:
            best = float(self._clearance[row, col])
        for cx, cy, r in (*self.circles, *others):
            best = min(best, max(math.hypot(x - cx, y - cy) - r, 0.0))
        return best if best < self.max_radius else math.inf

    def move(self, x0, y0, x1, y1, radius, others=()):
        """Where a disc of `radius` moving straight from (x0, y0) to (x1, y1) ends up.

        Samples the path every half cell and stops at the last sample before
        the disc would touch an obstacle or one of `others` (circles x, y,
        radius). A disc that already overlaps one (spawned or teleported
        there) may move, just not deeper into it. Radius 0 never collides.
        """
        if radius > self.max_radius:
            raise ValueError(f'radius {radius} m is over max_radius {self.max_radius} m')
        steps = int(math.hypot(x1 - x0, y1 - y0) / (0.5 * self.res)) + 1
        x, y = x0, y0
        clear = self.clearance(x, y, others)
        for i in range(1, steps + 1):
            nx, ny = x0 + (x1 - x0) * i / steps, y0 + (y1 - y0) * i / steps
            next_clear = self.clearance(nx, ny, others)
            if next_clear < radius and next_clear < clear:
                return x, y
            x, y, clear = nx, ny, next_clear
        return x1, y1


def pillar_voxels(pillars, res=PILLAR_RES):
    """Octomap keys (kx, ky, kz) of the voxels that fill config.Pillars.

    A voxel is in a pillar when its center is inside the pillar's circle, from
    the floor (z = 0, no ground plane) up to the pillar's height.
    """
    keys = set()
    for x, y, r, height in pillars:
        layers = range(max(1, round(height / res)))
        for kx in range(math.floor((x - r) / res), math.floor((x + r) / res) + 1):
            for ky in range(math.floor((y - r) / res), math.floor((y + r) / res) + 1):
                if math.hypot((kx + 0.5) * res - x, (ky + 0.5) * res - y) <= r:
                    keys.update((kx + _KEY_ORIGIN, ky + _KEY_ORIGIN, kz + _KEY_ORIGIN)
                                for kz in layers)
    return keys


def grid_voxels(occupied, x0, y0, res, height=MAP_WALL_HEIGHT):
    """Octomap keys (kx, ky, kz) of walls standing on an occupancy grid's cells.

    occupied, x0, y0, res as read_image_map returns them. Each cell becomes
    the column of voxels (size res) holding its center, from the floor up to
    `height`, so when the origin is a multiple of res they are the cells.
    """
    rows, cols = np.nonzero(occupied)
    kx = np.floor(x0 / res + cols + 0.5).astype(np.int64) + _KEY_ORIGIN
    ky = np.floor(y0 / res + rows + 0.5).astype(np.int64) + _KEY_ORIGIN
    layers = range(_KEY_ORIGIN, _KEY_ORIGIN + max(1, round(height / res)))
    return {(x, y, z) for x, y in zip(kx.tolist(), ky.tolist()) for z in layers}


def write_bt(path, voxels, res):
    """Write occupied voxels (octomap keys) as an octomap .bt file; read_bt reads it back.

    Every voxel is a leaf at full depth (nothing pruned), the rest is unknown.
    """
    if not voxels:
        raise ValueError('no voxels to write')
    data = bytearray()
    nodes = 1  # the root; the header's size must match what octomap reads

    def write_node(keys, kx, ky, kz, size):
        # The read_node encoding: 2 bits per child, 10 occupied leaf, 11 inner node.
        nonlocal nodes
        half = size // 2
        children = [[] for _ in range(8)]
        for key in keys:
            i = (key[0] >= kx + half) | (key[1] >= ky + half) << 1 | (key[2] >= kz + half) << 2
            children[i].append(key)
        kind = 2 if half == 1 else 3
        bits = sum(kind << 2 * i for i, child in enumerate(children) if child)
        data.extend((bits & 0xFF, bits >> 8))
        nodes += sum(1 for child in children if child)
        if kind == 3:
            for i, child in enumerate(children):
                if child:
                    write_node(child, kx + half * (i & 1), ky + half * (i >> 1 & 1),
                               kz + half * (i >> 2), half)

    write_node(sorted(voxels), 0, 0, 0, 1 << _TREE_DEPTH)
    with open(path, 'wb') as f:
        f.write(b'# Octomap OcTree binary file\n'
                b'# written by hero_sim from a scenario\'s obstacles\n'
                + f'id OcTree\nsize {nodes}\nres {res}\ndata\n'.encode() + bytes(data))


def write_pillar_map(path, pillars, res=PILLAR_RES):
    """Write config.Pillars as an octomap .bt at `path` (dirs made as needed). Returns path."""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    write_bt(path, pillar_voxels(pillars, res), res)
    return path


def write_image_map(path, map_file, height=MAP_WALL_HEIGHT):
    """Write an image map's occupied pixels as an octomap .bt of walls `height` tall.

    Dirs are made as needed. Returns path, or None when the image has nothing
    occupied (an octomap can't be empty).
    """
    occupied, x0, y0, res = read_image_map(map_file)
    if not occupied.any():
        return None
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    write_bt(path, grid_voxels(occupied, x0, y0, res, height), res)
    return path


def load(sim, radii, pillars=()):
    """The scenario's ObstacleMap for robots of these radii, or None when collisions are off.

    pillars: the scenario's obstacles (config.Pillar). None when
    sim.collision is off or there are no robots. Without a map or pillars the
    map is empty, but still checks the `others` passed to move().
    """
    if not sim['collision'] or not radii:
        return None
    map_file = config.resolve_map(sim['map'])
    if map_file is None:
        return ObstacleMap(np.zeros((0, 0), bool), 0.0, 0.0, PILLAR_RES, max(radii), pillars)
    if config.is_image_map(map_file):
        return ObstacleMap.from_image(map_file, max(radii), pillars)
    return ObstacleMap.from_bt(map_file, *sim['collision_z'], max_radius=max(radii),
                               circles=pillars)
