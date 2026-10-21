"""Procedurally generated textures and meshes for the city scene.

Pure Python (no Pillow / NumPy). Everything is written into assets/ the first time it is needed,
so a fresh checkout still works, and the files are small enough to keep in git.

PyBullet's floor mesh repeats a texture every 2 m, and our own quads/walls are generated with UVs
that repeat per metre, so the textures here are sized as real-world tiles:
    grass / asphalt / plaza / sidewalk : 2 m x 2 m
    facade                             : one window bay, 2.5 m wide x 3.2 m tall
"""

import os
import random
import struct
import zlib

ASSET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

TILE = 2.0                      # metres covered by one floor texture tile
BAY_W, BAY_H = 2.5, 3.2         # metres covered by one facade texture tile


def _byte(value):
    return max(0, min(255, int(round(value))))


def write_png(path, width, height, rows):
    """Minimal RGB PNG writer. `rows` holds one bytes object of width*3 per row."""
    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + row for row in rows)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def _mix(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


# ---------------------------------------------------------------- floor textures

def make_noise_texture(path, size, base, jitter, seed, tint=(1.0, 1.0, 1.0)):
    """Speckled surface (grass, asphalt). Random noise tiles seamlessly by construction."""
    rng = random.Random(seed)
    rows = []
    for _ in range(size):
        row = bytearray()
        for _ in range(size):
            d = rng.uniform(-jitter, jitter)
            row.extend((_byte(base[0] + d * tint[0]), _byte(base[1] + d * tint[1]),
                        _byte(base[2] + d * tint[2])))
        rows.append(bytes(row))
    write_png(path, size, size, rows)


def make_slab_texture(path, size, base, grout, slabs, seed, grout_px=3):
    """Paving: `slabs` x `slabs` stone slabs per tile, each slightly different, with grout lines."""
    rng = random.Random(seed)
    cell = size // slabs
    shade = [[rng.uniform(-7, 7) for _ in range(slabs)] for _ in range(slabs)]
    rows = []
    for y in range(size):
        row = bytearray()
        dy = min(y % cell, cell - y % cell)
        for x in range(size):
            dx = min(x % cell, cell - x % cell)
            line = max(0.0, 1.0 - min(dx, dy) / grout_px)
            s = shade[y // cell][x // cell] + rng.uniform(-2.0, 2.0)
            colour = _mix((base[0] + s, base[1] + s, base[2] + s), grout, line)
            row.extend(_byte(c) for c in colour)
        rows.append(bytes(row))
    write_png(path, size, size, rows)


# ---------------------------------------------------------------- facade texture

def make_facade_texture(path, wall, glass_top, glass_bottom, trim, seed, w=100, h=128):
    """One window bay: wall colour, a glass window with a mullion, a frame and a sill."""
    rng = random.Random(seed)
    left, right = int(w * 0.18), int(w * 0.82)
    top, bottom = int(h * 0.16), int(h * 0.76)
    rows = []
    for y in range(h):
        row = bytearray()
        for x in range(w):
            colour = wall
            inside = left <= x < right and top <= y < bottom
            frame = (left - 3 <= x < right + 3 and top - 3 <= y < bottom + 3) and not inside
            if inside:
                colour = _mix(glass_top, glass_bottom, (y - top) / (bottom - top))
                if abs(x - (left + right) // 2) <= 1:
                    colour = trim                                   # mullion
            elif frame:
                colour = trim
            elif bottom + 3 <= y < bottom + 8 and left - 6 <= x < right + 6:
                colour = _mix(trim, (255, 255, 255), 0.25)          # sill
            n = rng.uniform(-3, 3)
            row.extend((_byte(colour[0] + n), _byte(colour[1] + n), _byte(colour[2] + n)))
        rows.append(bytes(row))
    write_png(path, w, h, rows)


FACADES = {
    # name: (wall, glass top, glass bottom, trim)
    "brick": ((150, 64, 50), (92, 142, 188), (46, 88, 128), (228, 226, 220)),
    "cream": ((224, 210, 182), (104, 158, 178), (62, 112, 134), (122, 112, 102)),
    "slate": ((98, 106, 118), (150, 198, 226), (92, 142, 182), (206, 210, 214)),
    "glass": ((44, 74, 104), (128, 186, 224), (62, 112, 164), (30, 50, 72)),
}
FLOORS = {
    "grass": lambda p, s: make_noise_texture(p, 128, (84, 140, 70), 16, s, tint=(0.55, 1.0, 0.45)),
    "asphalt": lambda p, s: make_noise_texture(p, 128, (56, 58, 63), 9, s),
    "plaza": lambda p, s: make_slab_texture(p, 256, (216, 203, 180), (170, 158, 138), 2, s),
    "sidewalk": lambda p, s: make_slab_texture(p, 128, (196, 196, 193), (158, 158, 156), 2, s, 2),
}


def texture_path(name, asset_dir=ASSET_DIR):
    return os.path.join(asset_dir, "city_%s.png" % name)


def ensure_textures(asset_dir=ASSET_DIR):
    """Create any missing city texture. Returns {name: path}."""
    paths = {}
    for i, (name, make) in enumerate(FLOORS.items()):
        paths[name] = texture_path(name, asset_dir)
        if not os.path.exists(paths[name]):
            make(paths[name], 100 + i)
    for i, (name, colours) in enumerate(FACADES.items()):
        paths[name] = texture_path(name, asset_dir)
        if not os.path.exists(paths[name]):
            make_facade_texture(paths[name], *colours, seed=200 + i)
    return paths


# ---------------------------------------------------------------- meshes

def _write_obj(path, faces):
    """faces: list of (normal, [(x, y, z, u, v) x4]) quads."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    lines = ["# generated by scene_assets.py"]
    for n, quad in faces:
        lines.append("vn %.4f %.4f %.4f" % n)
    for _, quad in faces:
        for x, y, z, u, v in quad:
            lines.append("v %.4f %.4f %.4f" % (x, y, z))
    for _, quad in faces:
        for x, y, z, u, v in quad:
            lines.append("vt %.4f %.4f" % (u, v))
    for i in range(len(faces)):
        a = 4 * i + 1
        lines.append("f %d/%d/%d %d/%d/%d %d/%d/%d" % (a, a, i + 1, a + 1, a + 1, i + 1, a + 2, a + 2, i + 1))
        lines.append("f %d/%d/%d %d/%d/%d %d/%d/%d" % (a, a, i + 1, a + 2, a + 2, i + 1, a + 3, a + 3, i + 1))
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def write_floor_obj(path, x0, x1, y0, y1, tile=TILE):
    """Flat quad on z = 0 whose texture repeats once per `tile` metres."""
    u, v = (x1 - x0) / tile, (y1 - y0) / tile
    _write_obj(path, [((0.0, 0.0, 1.0), [(x0, y0, 0, 0, 0), (x1, y0, 0, u, 0),
                                        (x1, y1, 0, u, v), (x0, y1, 0, 0, v)])])


def write_walls_obj(path, sx, sy, sz, bay_w=BAY_W, bay_h=BAY_H):
    """The four side walls of a box standing on z = 0, centred on the origin. The facade texture
    repeats a whole number of bays across and up so windows are never cut in half."""
    hx, hy = sx / 2.0, sy / 2.0
    rows = max(1, round(sz / bay_h))
    nx, ny = max(1, round(sx / bay_w)), max(1, round(sy / bay_w))
    faces = [
        ((1, 0, 0), [(hx, -hy, 0, 0, 0), (hx, hy, 0, ny, 0), (hx, hy, sz, ny, rows), (hx, -hy, sz, 0, rows)]),
        ((0, 1, 0), [(hx, hy, 0, 0, 0), (-hx, hy, 0, nx, 0), (-hx, hy, sz, nx, rows), (hx, hy, sz, 0, rows)]),
        ((-1, 0, 0), [(-hx, hy, 0, 0, 0), (-hx, -hy, 0, ny, 0), (-hx, -hy, sz, ny, rows), (-hx, hy, sz, 0, rows)]),
        ((0, -1, 0), [(-hx, -hy, 0, 0, 0), (hx, -hy, 0, nx, 0), (hx, -hy, sz, nx, rows), (-hx, -hy, sz, 0, rows)]),
    ]
    _write_obj(path, faces)


def mesh_path(name, asset_dir=ASSET_DIR):
    return os.path.join(asset_dir, "mesh", name + ".obj")
