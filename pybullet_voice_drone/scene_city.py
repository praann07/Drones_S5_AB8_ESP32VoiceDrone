"""The city-plaza scene: a paved plaza with a helipad, a ring road with traffic, a sidewalk, tree
planters, benches, lamp posts, and a skyline of textured buildings. The obstacles become kiosk
towers. Everything is built from PyBullet primitives and generated textures (scene_assets.py).

Layout (metres, home pad at the origin; the drone's geofence is +-12):

    plaza            |x|,|y| <  16        stone slabs, helipad in the middle
    inner sidewalk   16 .. 18            lamp posts
    ring road        18 .. 28            two lanes, crosswalks, traffic
    outer sidewalk   28 .. 30
    buildings        31 and beyond       taller on the side away from the sun, so no building
                                         shades the plaza

This module only uses pybullet and scene_assets, so drone_core can import it freely.
"""

import math
import os
import random

import pybullet as p

import scene_assets as sa

PLAZA, WALK_IN, ROAD_IN, ROAD_OUT, WALK_OUT = 16.0, 18.0, 18.0, 28.0, 30.0
LANE_IN, LANE_OUT = 20.5, 25.5          # lane centre lines (inner: counter-clockwise, outer: clockwise)
CORNER_RADIUS = 3.0

SKY = (0.60, 0.76, 0.92)
HAZARD = (0.98, 0.80, 0.05)
WHITE = (0.95, 0.95, 0.93)
YELLOW_LINE = (0.95, 0.78, 0.12)

CAR_COLOURS = ((0.95, 0.78, 0.10), (0.15, 0.28, 0.70), (0.92, 0.92, 0.94),
               (0.75, 0.12, 0.10), (0.15, 0.50, 0.28), (0.12, 0.12, 0.14))

KIOSK_STYLES = (
    {"body": (0.93, 0.88, 0.74), "trim": (0.52, 0.47, 0.40), "glass": (0.45, 0.72, 0.90)},
    {"body": (0.20, 0.22, 0.27), "trim": (0.09, 0.10, 0.12), "glass": (0.55, 0.78, 0.95)},
    {"body": (0.80, 0.84, 0.90), "trim": (0.42, 0.48, 0.56), "glass": (0.35, 0.62, 0.85)},
)


def rotate(yaw, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def lane_pose(half, radius, s, ccw=True):
    """Position and heading of a car `s` metres along a square lane of half-size `half` with rounded
    corners. Counter-clockwise starts mid-way along the bottom edge heading +X; clockwise drives the
    same path backwards. Returns (x, y, heading)."""
    straight = 2.0 * (half - radius)
    arc = math.pi * radius / 2.0
    side = straight + arc
    total = 4.0 * side
    s %= total
    if not ccw:
        s = total - s
    edge, r = divmod(s, side)
    h = int(edge) * math.pi / 2.0
    a = half - radius
    if r < straight:
        t = r - straight / 2.0
        x = half * math.sin(h) + math.cos(h) * t
        y = -half * math.cos(h) + math.sin(h) * t
        heading = h
    else:
        u = (r - straight) / radius
        cx, cy = a * (math.cos(h) + math.sin(h)), a * (math.sin(h) - math.cos(h))
        ang = h - math.pi / 2.0 + u
        x, y, heading = cx + radius * math.cos(ang), cy + radius * math.sin(ang), h + u
    return x, y, heading if ccw else heading + math.pi


def lane_length(half, radius):
    return 4.0 * (2.0 * (half - radius) + math.pi * radius / 2.0)


class PartSet:
    """Visual-only parts collected in one local frame and emitted as multibodies of <= 100 links."""

    MAX_LINKS = 100

    def __init__(self, shapes):
        self.shapes = shapes                 # shared cache {key: visual shape id} (per physics client)
        self.parts = []

    def _shape(self, kind, dims, colour):
        rgba = tuple(colour) + ((1.0,) if len(colour) == 3 else ())
        key = (kind, tuple(dims), rgba)
        if key not in self.shapes:
            common = {"rgbaColor": list(rgba), "specularColor": [0, 0, 0]}
            if kind == "box":
                shape = p.createVisualShape(p.GEOM_BOX, halfExtents=list(dims), **common)
            elif kind == "cyl":
                shape = p.createVisualShape(p.GEOM_CYLINDER, radius=dims[0], length=dims[1], **common)
            else:
                shape = p.createVisualShape(p.GEOM_SPHERE, radius=dims[0], **common)
            self.shapes[key] = shape
        return self.shapes[key]

    def _add(self, kind, pos, dims, colour, euler):
        self.parts.append((self._shape(kind, dims, colour), list(pos), p.getQuaternionFromEuler(list(euler))))

    def box(self, pos, half, colour, euler=(0, 0, 0)):
        self._add("box", pos, half, colour, euler)

    def cyl(self, pos, radius, length, colour, euler=(0, 0, 0)):
        self._add("cyl", pos, (radius, length), colour, euler)

    def sphere(self, pos, radius, colour):
        self._add("sphere", pos, (radius,), colour, (0, 0, 0))

    def build(self, base=(0, 0, 0), base_yaw=0.0, base_collision=-1, base_visual=-1):
        """Emit the parts; returns the list of body ids (the first one carries the base shapes)."""
        bodies = []
        chunks = [self.parts[i:i + self.MAX_LINKS] for i in range(0, len(self.parts), self.MAX_LINKS)] or [[]]
        for index, chunk in enumerate(chunks):
            n = len(chunk)
            first = index == 0
            if n == 0 and (not first or (base_collision < 0 and base_visual < 0)):
                continue
            bodies.append(p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=base_collision if first else -1,
                baseVisualShapeIndex=base_visual if first else -1,
                basePosition=list(base), baseOrientation=p.getQuaternionFromEuler([0, 0, base_yaw]),
                linkMasses=[0] * n, linkCollisionShapeIndices=[-1] * n,
                linkVisualShapeIndices=[c[0] for c in chunk],
                linkPositions=[c[1] for c in chunk], linkOrientations=[c[2] for c in chunk],
                linkInertialFramePositions=[[0, 0, 0]] * n,
                linkInertialFrameOrientations=[[0, 0, 0, 1]] * n,
                linkParentIndices=[0] * n, linkJointTypes=[p.JOINT_FIXED] * n,
                linkJointAxis=[[0, 0, 1]] * n))
        return bodies


class CityScene:
    """Builds the environment. `set_glow` / `update` animate the kiosk halos and the traffic."""

    def __init__(self, floor, obstacles, home=(0.0, 0.0), sun=(-10.0, 8.0), asset_dir=sa.ASSET_DIR):
        self.asset_dir = asset_dir
        self.shapes = {}
        self.textures = {}
        paths = sa.ensure_textures(asset_dir)
        self._paths = paths
        self.sun = sun
        self.rng = random.Random(2026)

        # Ground: the collision plane is lowered and re-skinned as grass; every layer above it is
        # 1 cm higher than the one below so they never z-fight.
        p.resetBasePositionAndOrientation(floor, [0, 0, -0.03], [0, 0, 0, 1])
        p.changeVisualShape(floor, -1, textureUniqueId=self._texture("grass"),
                            rgbaColor=[0.92, 0.97, 0.9, 1], specularColor=[0, 0, 0])
        self._rings("asphalt", ROAD_IN, ROAD_OUT, -0.02)
        self._rings("sidewalk", PLAZA, WALK_IN, -0.01)
        self._rings("sidewalk", ROAD_OUT, WALK_OUT, -0.01)
        self._surface("plaza", -PLAZA, PLAZA, -PLAZA, PLAZA, 0.0)

        decor = PartSet(self.shapes)
        self._plaza_inlay(decor, home)
        self._road_markings(decor)
        self._trees(decor)
        self._benches(decor)
        self._lamps(decor)
        self._skyline_far(decor)
        decor.build()
        self._buildings()
        self._helipad(home)

        self.kiosks = [self._kiosk(ob, KIOSK_STYLES[i % len(KIOSK_STYLES)]) for i, ob in enumerate(obstacles)]
        self.cars, self.walkers = [], []
        self._moved_at = None
        self._traffic()
        self._pedestrians()

    # ------------------------------------------------------------ ground

    def _texture(self, name):
        if name not in self.textures:
            self.textures[name] = p.loadTexture(self._paths[name])
        return self.textures[name]

    def _surface(self, texture, x0, x1, y0, y1, z):
        key = "_".join(str(int(v)).replace("-", "n") for v in (x0, x1, y0, y1))
        obj = sa.mesh_path("floor_" + key, self.asset_dir)
        if not os.path.exists(obj):
            sa.write_floor_obj(obj, x0, x1, y0, y1)
        shape = p.createVisualShape(p.GEOM_MESH, fileName=obj, meshScale=[1, 1, 1],
                                    rgbaColor=[1, 1, 1, 1], specularColor=[0, 0, 0])
        body = p.createMultiBody(0, -1, shape, [0, 0, z])
        p.changeVisualShape(body, -1, textureUniqueId=self._texture(texture), rgbaColor=[1, 1, 1, 1],
                            specularColor=[0, 0, 0])
        return body

    def _rings(self, texture, inner, outer, z):
        """A square ring (four quads) between half-sizes `inner` and `outer`."""
        self._surface(texture, -outer, outer, -outer, -inner, z)
        self._surface(texture, -outer, outer, inner, outer, z)
        self._surface(texture, -outer, -inner, -inner, inner, z)
        self._surface(texture, inner, outer, -inner, inner, z)

    def _plaza_inlay(self, parts, home):
        """Square stone borders (a wide one at the plaza edge, a double one around the helipad) so the
        plaza is not just a flat grid. Thin boxes, not discs: wide discs a few millimetres above the
        floor flicker and render as ragged wedges."""
        stone, light = (0.78, 0.71, 0.58), (0.92, 0.88, 0.78)
        for half, width, colour in ((15.5, 0.3, stone), (5.4, 0.22, stone), (4.9, 0.08, light)):
            for k in range(4):
                yaw = k * math.pi / 2
                x, y = rotate(yaw, 0.0, -half)
                parts.box((home[0] + x, home[1] + y, 0.004), (half + width, width, 0.004), colour, (0, 0, yaw))

    # ------------------------------------------------------------ road markings

    def _road_markings(self, parts):
        z = -0.014
        for centre, colour, half_t in ((23.0 - 0.14, YELLOW_LINE, 0.06), (23.0 + 0.14, YELLOW_LINE, 0.06),
                                       (ROAD_IN + 0.5, WHITE, 0.09), (ROAD_OUT - 0.5, WHITE, 0.09)):
            length = centre
            for k in range(4):                       # one straight per side, rotated about the origin
                yaw = k * math.pi / 2
                x, y = rotate(yaw, 0.0, -centre)
                parts.box((x, y, z), (length + half_t, half_t, 0.004), colour, (0, 0, yaw))
        for k in range(4):                           # zebra crossings, mid-side
            yaw = k * math.pi / 2
            for i in range(9):
                across = ROAD_IN + 0.9 + i * 1.0
                x, y = rotate(yaw, 0.0, -across)
                # a little thicker than the centre lines so the stripes sit on top of them (no flicker)
                parts.box((x, y, z + 0.003), (2.0, 0.25, 0.004), WHITE, (0, 0, yaw))

    # ------------------------------------------------------------ props

    def _trees(self, parts):
        for side in range(4):
            yaw = side * math.pi / 2
            for along in (-12.0, -6.0, 6.0, 12.0):           # none at 0: that is where the crossing is
                x, y = rotate(yaw, along, -14.8)
                self._tree(parts, x, y)

    def _tree(self, parts, x, y):
        rng = self.rng
        parts.cyl((x, y, 0.25), 0.55, 0.5, (0.93, 0.93, 0.90))              # white planter
        parts.cyl((x, y, 0.52), 0.46, 0.04, (0.28, 0.20, 0.14))             # soil
        parts.cyl((x, y, 1.35), 0.11, 1.6, (0.38, 0.26, 0.16))              # trunk
        greens = ((0.16, 0.52, 0.20), (0.20, 0.60, 0.24), (0.13, 0.46, 0.18))
        parts.sphere((x, y, 2.75), 1.05 + rng.uniform(-0.1, 0.1), greens[0])
        parts.sphere((x + 0.6, y + 0.2, 2.35), 0.78, greens[1])
        parts.sphere((x - 0.5, y - 0.35, 2.45), 0.72, greens[2])
        parts.sphere((x - 0.1, y + 0.5, 3.35), 0.62, greens[1])

    def _benches(self, parts):
        for side in range(4):
            yaw = side * math.pi / 2
            for along in (-8.0, 8.0):
                x, y = rotate(yaw, along, -13.5)
                self._bench(parts, x, y, yaw)

    @staticmethod
    def _bench(parts, x, y, yaw):
        """Back to the plaza edge (local +Y is outward after the yaw): seat, backrest, two legs."""
        wood, iron = (0.62, 0.40, 0.22), (0.12, 0.12, 0.14)

        def at(lx, ly, z):
            dx, dy = rotate(yaw, lx, ly)
            return (x + dx, y + dy, z)

        e = (0, 0, yaw)
        parts.box(at(0, 0, 0.46), (0.9, 0.25, 0.03), wood, e)
        parts.box(at(0, -0.24, 0.72), (0.9, 0.03, 0.2), wood, e)
        for lx in (-0.75, 0.75):
            parts.box(at(lx, 0, 0.22), (0.04, 0.22, 0.22), iron, e)

    def _lamps(self, parts):
        spots = [(a, b) for a in (-17.0, 0.0, 17.0) for b in (-17.0, 0.0, 17.0) if (a, b) != (0.0, 0.0)]
        for x, y in spots:
            parts.cyl((x, y, 0.15), 0.2, 0.3, (0.16, 0.17, 0.19))
            parts.cyl((x, y, 3.0), 0.07, 6.0, (0.12, 0.13, 0.15))
            parts.box((x, y, 6.05), (0.4, 0.16, 0.05), (0.14, 0.15, 0.17))
            parts.box((x, y, 5.97), (0.34, 0.12, 0.03), (1.0, 0.97, 0.80))      # the lamp itself

    # ------------------------------------------------------------ buildings

    def _sun_side(self, x, y):
        """True if a building here would throw its shadow towards the plaza."""
        return x * self.sun[0] + y * self.sun[1] > 0.0

    def _height(self, x, y, near, far):
        low, high = near if self._sun_side(x, y) else far
        return self.rng.uniform(low, high)

    def _buildings(self):
        names = list(sa.FACADES)
        parts = PartSet(self.shapes)
        self.buildings = []                  # (x, y, width_x, width_y, height) of the near row
        lots = []
        for side in range(4):
            yaw = side * math.pi / 2
            for along in (-24.0, 0.0, 24.0):
                x, y = rotate(yaw, along, -39.0)
                lots.append((x, y, 20.0, 16.0, yaw))
        for sx in (-1, 1):
            for sy in (-1, 1):
                lots.append((sx * 41.0, sy * 41.0, 14.0, 14.0, 0.0))
        for i, (x, y, w, d, yaw) in enumerate(lots):
            h = self._height(x, y, (14.0, 22.0), (28.0, 58.0))
            sx, sy = (w, d) if abs(math.sin(yaw)) < 0.5 else (d, w)       # footprint after rotation
            obj = sa.mesh_path("walls_%dx%dx%d" % (sx, sy, round(h)), self.asset_dir)
            if not os.path.exists(obj):
                sa.write_walls_obj(obj, sx, sy, round(h))
            shape = p.createVisualShape(p.GEOM_MESH, fileName=obj, meshScale=[1, 1, 1],
                                        rgbaColor=[1, 1, 1, 1], specularColor=[0, 0, 0])
            body = p.createMultiBody(0, -1, shape, [x, y, 0.0])
            p.changeVisualShape(body, -1, textureUniqueId=self._texture(names[i % len(names)]),
                                rgbaColor=[1, 1, 1, 1], specularColor=[0, 0, 0])
            top = round(h)
            self.buildings.append((x, y, sx, sy, top))
            parts.box((x, y, top + 0.2), (sx / 2 + 0.15, sy / 2 + 0.15, 0.2), (0.26, 0.27, 0.30))   # roof slab
            for _ in range(2):                                                                       # rooftop plant
                ux, uy = self.rng.uniform(-sx / 4, sx / 4), self.rng.uniform(-sy / 4, sy / 4)
                parts.box((x + ux, y + uy, top + 0.4 + 0.6), (1.2, 0.9, 0.6), (0.62, 0.65, 0.68))
            parts.cyl((x, y, top + 0.4 + 2.5), 0.08, 5.0, (0.3, 0.3, 0.32))                         # antenna
            parts.sphere((x, y, top + 0.4 + 5.1), 0.18, (1.0, 0.2, 0.15))
        parts.build()

    def _skyline_far(self, parts):
        """A second row of plain towers, tinted towards the sky colour like distant haze."""
        for side in range(4):
            yaw = side * math.pi / 2
            for along in (-60.0, -36.0, -12.0, 12.0, 36.0, 60.0):
                x, y = rotate(yaw, along, -66.0)
                h = self._height(x, y, (30.0, 46.0), (46.0, 95.0))
                w = self.rng.uniform(14.0, 22.0)
                base = self.rng.choice(((0.45, 0.52, 0.62), (0.5, 0.55, 0.62), (0.55, 0.5, 0.52)))
                haze = tuple(b + (s - b) * 0.45 for b, s in zip(base, SKY))
                parts.box((x, y, h / 2), (w / 2, w / 2, h / 2), haze)

    # ------------------------------------------------------------ helipad

    def _helipad(self, home):
        parts = PartSet(self.shapes)
        x, y = home
        yellow = (0.98, 0.80, 0.05)
        parts.cyl((x, y, 0.004), 2.15, 0.008, yellow)                          # ring (lower)
        parts.cyl((x, y, 0.006), 1.92, 0.008, (0.15, 0.17, 0.20))              # asphalt disc: top at 0.010
        white = (0.96, 0.96, 0.96)
        for dy in (-0.62, 0.62):
            parts.box((x, y + dy, 0.0115), (0.8, 0.12, 0.0015), white)         # H uprights
        parts.box((x, y, 0.0115), (0.12, 0.62, 0.0015), white)                 # H bar
        for k in range(12):                                                   # edge lights
            a = k * math.pi / 6
            parts.sphere((x + 2.03 * math.cos(a), y + 2.03 * math.sin(a), 0.03), 0.06, (0.3, 1.0, 0.45))
        parts.build()

    # ------------------------------------------------------------ kiosks (the obstacles)

    def _kiosk(self, ob, style):
        hx, hy, hz = ob.half
        parts = PartSet(self.shapes)
        trim, glass = style["trim"], style["glass"]
        parts.box((0, 0, -hz + 0.12), (hx + 0.08, hy + 0.08, 0.12), trim)                      # plinth
        parts.box((0, 0, -hz + 0.55), (hx + 0.012, hy + 0.012, 0.09), HAZARD)                  # hazard band
        for z in (-0.15, 0.75):                                                                # windows
            for s in (-1, 1):
                parts.box((s * (hx + 0.01), 0, z), (0.012, hy * 0.6, 0.32), glass)
                parts.box((0, s * (hy + 0.01), z), (hx * 0.6, 0.012, 0.32), glass)
        parts.box((-(hx + 0.28), 0, 1.0), (0.3, hy * 0.75, 0.03), (0.78, 0.15, 0.12))          # awning
        parts.box((0, 0, hz + 0.07), (hx + 0.1, hy + 0.1, 0.07), (0.22, 0.23, 0.26))           # roof
        parts.box((0.25, 0.2, hz + 0.26), (0.22, 0.22, 0.12), (0.7, 0.72, 0.75))               # rooftop unit
        parts.cyl((-0.25, -0.2, hz + 0.54), 0.025, 0.8, (0.3, 0.3, 0.32))                      # antenna
        parts.sphere((-0.25, -0.2, hz + 0.98), 0.1, (1.0, 0.25, 0.1))                          # beacon
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(ob.half))
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=list(ob.half),
                                     rgbaColor=list(style["body"]) + [1.0], specularColor=[0, 0, 0])
        body = parts.build(base=ob.center, base_collision=collision, base_visual=visual)[0]
        halo = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx + 0.07, hy + 0.07, hz + 0.07],
                                   rgbaColor=[1.0, 0.15, 0.05, 0.0], specularColor=[0, 0, 0])
        halo_body = p.createMultiBody(0, -1, halo, list(ob.center))
        return {"obstacle": ob, "body": body, "halo": halo_body, "alpha": None}

    def set_glow(self, index, level):
        """level 0..1 -> how strongly the red halo around kiosk `index` shows."""
        kiosk = self.kiosks[index]
        alpha = round(level * 0.50, 2)
        if alpha != kiosk["alpha"]:
            kiosk["alpha"] = alpha
            p.changeVisualShape(kiosk["halo"], -1, rgbaColor=[1.0, 0.15, 0.05, alpha])

    # ------------------------------------------------------------ traffic

    def _car(self, colour):
        parts = PartSet(self.shapes)
        glass, dark = (0.12, 0.16, 0.22), (0.06, 0.06, 0.07)
        parts.box((0, 0, 0.58), (2.1, 0.9, 0.32), colour)                                   # body
        parts.box((-0.2, 0, 1.05), (1.15, 0.8, 0.28), glass)                                # cabin glass
        parts.box((-0.2, 0, 1.36), (1.1, 0.78, 0.04), colour)                               # roof
        parts.box((1.3, 0, 0.78), (0.8, 0.86, 0.04), colour)                                # bonnet
        parts.box((2.12, 0, 0.35), (0.06, 0.88, 0.1), dark)                                 # bumpers
        parts.box((-2.12, 0, 0.35), (0.06, 0.88, 0.1), dark)
        for sy in (-0.6, 0.6):
            parts.box((2.1, sy, 0.62), (0.03, 0.14, 0.07), (1.0, 0.97, 0.8))                # headlights
            parts.box((-2.1, sy, 0.62), (0.03, 0.14, 0.07), (0.85, 0.1, 0.1))               # tail lights
        for sx in (-1.3, 1.3):
            for sy in (-0.92, 0.92):
                parts.cyl((sx, sy, 0.36), 0.36, 0.26, (0.05, 0.05, 0.06), (math.pi / 2, 0, 0))
        return parts.build()[0]

    def _traffic(self):
        for lane, half, ccw, count in (("in", LANE_IN, True, 4), ("out", LANE_OUT, False, 5)):
            length = lane_length(half, CORNER_RADIUS)
            for i in range(count):
                colour = CAR_COLOURS[(len(self.cars) * 5 + 1) % len(CAR_COLOURS)]
                self.cars.append({
                    "body": self._car(colour), "half": half, "ccw": ccw, "length": length,
                    "speed": self.rng.uniform(5.0, 7.5),
                    "offset": (i + self.rng.uniform(0.0, 0.5)) * length / count})
        self.update(0.0)

    def _pedestrians(self):
        """People strolling round the inner sidewalk, half each way. The two paths are 1.1 m apart and
        straddle the lamp posts (which stand on the 17 m line) so nobody walks through one."""
        shirts = ((0.85, 0.25, 0.25), (0.20, 0.45, 0.80), (0.95, 0.80, 0.20), (0.30, 0.65, 0.40),
                  (0.70, 0.40, 0.80), (0.95, 0.55, 0.20))
        for i in range(10):
            parts = PartSet(self.shapes)
            parts.cyl((0, 0, 0.4), 0.17, 0.8, (0.14, 0.17, 0.27))                     # trousers
            parts.cyl((0, 0, 1.15), 0.23, 0.7, shirts[i % len(shirts)])               # torso
            parts.sphere((0, 0, 1.64), 0.15, (0.93, 0.76, 0.62))                      # head
            ccw = i % 2 == 0
            half = 16.45 if ccw else 17.55
            length = lane_length(half, 0.8)
            self.walkers.append({"body": parts.build()[0], "half": half, "ccw": ccw,
                                 "speed": self.rng.uniform(1.0, 1.6),
                                 "offset": (i // 2 + self.rng.uniform(0.0, 0.8)) * length / 5.0})
        self._moved_at = None                # force this first placement past the rate limit
        self.update(0.0)

    def update(self, clock):
        """Move the cars and pedestrians to where they are at simulation time `clock` (seconds).
        At most 60 times a second: each move is a call into the GUI and nothing here is fast enough
        for more to be visible."""
        if self._moved_at is not None and 0.0 <= clock - self._moved_at < 1.0 / 60.0:
            return
        self._moved_at = clock
        for car in self.cars:
            x, y, heading = lane_pose(car["half"], CORNER_RADIUS, car["offset"] + car["speed"] * clock,
                                      car["ccw"])
            p.resetBasePositionAndOrientation(car["body"], [x, y, -0.015],
                                              p.getQuaternionFromEuler([0, 0, heading]))
        for walker in self.walkers:
            x, y, heading = lane_pose(walker["half"], 0.8, walker["offset"] + walker["speed"] * clock,
                                      walker["ccw"])
            p.resetBasePositionAndOrientation(walker["body"], [x, y, -0.01],
                                              p.getQuaternionFromEuler([0, 0, heading]))
