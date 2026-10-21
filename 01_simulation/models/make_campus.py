"""Build a MuJoCo model of the Amrita Vishwa Vidyapeetham Coimbatore campus
from OpenStreetMap data (data/raw/osm/amrita_coimbatore.json, downloaded
via the Overpass API; map data (c) OpenStreetMap contributors, ODbL).

Outputs, all deterministic:
  campus/campus_world.json  buildings as convex prisms (the single source of
                            truth that world.py's CampusWorld reads)
  campus/campus.xml         one convex mesh geom per prism piece
  campus/campus_map.png     map-window image: floor (photo if fetched) + outlines + names
  campus/campus_floor.png   clean satellite ground for the 3D scene (only if fetched)
  scene_campus.xml          quad.xml + floor + lights + campus.xml

Scale: the campus is built at 1:SCALE (default 1:10). At real scale the
campus is ~1.3 km across, which 1.5 m voice steps can't cross; at 1:10 it is
~130 m with roofs at ~1.3-1.5 m, so the drone (real size, real physics)
flies between and over buildings and can land on any roof.

Heights: OSM has no height for almost every building here (3 of 86 carry a
floor count), so heights are ESTIMATED from building type (see
estimate_height_m). Footprints and names are real.

Collision: MuJoCo collides meshes by their convex hull, so each footprint
is split into convex pieces (ear-clipping triangulation, then
Hertel-Mehlhorn merging) and each piece is its own mesh. L/U-shaped
buildings therefore collide with their real shape, not a bounding box.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
OSM_PATH = ROOT / "data" / "raw" / "osm" / "amrita_coimbatore.json"
OUT_DIR = HERE / "campus"
SCALE = 10.0                  # world is 1:SCALE
LAT0, LON0 = 10.9006, 76.9026  # projection origin (the shared map link's centre)
SPAWN_NEAR = "Central Library"
SPAWN_CLEARANCE_REAL_M = 14.0  # open ground needed around the spawn point
TEXTURE_PX = 4096
MARGIN_M = 6.0                # floor beyond the outermost feature (scaled metres)

_ACADEMIC = (0.58, 0.36, 0.28)   # muted brick (reads as roof tile from the air)
_HOSTEL = (0.74, 0.69, 0.58)     # warm stone
_OTHER = (0.52, 0.51, 0.48)      # concrete grey
_ROAD_W_REAL = {"tertiary": 8.0, "unclassified": 6.0, "residential": 5.0,
                "service": 4.0, "footway": 2.0, "path": 1.5}


def project(lat: float, lon: float) -> tuple[float, float]:
    k = 111_320.0
    return (lon - LON0) * k * math.cos(math.radians(LAT0)), (lat - LAT0) * k


def estimate_height_m(tags: dict, area_m2: float) -> float:
    """Real-world height estimate in metres (tags first, then type)."""
    for key in ("height", "building:height"):
        try:
            return float(str(tags[key]).split()[0])
        except (KeyError, ValueError):
            pass
    try:
        return 3.5 * float(tags["building:levels"]) + 1.0
    except (KeyError, ValueError):
        pass
    kind = tags.get("building", "yes")
    name = tags.get("name", "").lower()
    if kind in ("residential", "apartments", "dormitory") or "bhavanam" in name or "hostel" in name:
        return 14.0
    if kind == "university" or any(w in name for w in ("block", "library", "auditorium", "school")):
        return 13.0
    if area_m2 < 150.0:
        return 4.0
    return 8.0


def colour(tags: dict) -> tuple[float, float, float]:
    kind = tags.get("building", "yes")
    name = tags.get("name", "").lower()
    if kind in ("residential", "apartments", "dormitory") or "bhavanam" in name or "hostel" in name:
        return _HOSTEL
    if kind == "university" or any(w in name for w in ("block", "library", "auditorium", "school")):
        return _ACADEMIC
    return _OTHER


# --- polygon helpers ---------------------------------------------------------

def _area2(poly: np.ndarray) -> float:
    x, y = poly[:, 0], poly[:, 1]
    return float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def _cross(o, a, b) -> float:
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def clean_polygon(pts: np.ndarray, eps: float = 1e-3) -> np.ndarray:
    """Drop the closing duplicate, repeated and collinear points; make CCW."""
    if len(pts) > 1 and np.allclose(pts[0], pts[-1]):
        pts = pts[:-1]
    keep = [p for i, p in enumerate(pts) if i == 0 or np.linalg.norm(p - pts[i - 1]) > eps]
    pts = np.array(keep)
    changed = True
    while changed and len(pts) > 3:
        changed = False
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            if abs(_cross(a, b, c)) < eps * max(np.linalg.norm(c - a), eps):
                pts = np.delete(pts, i, axis=0)
                changed = True
                break
    if _area2(pts) < 0:
        pts = pts[::-1]
    return pts


def _point_in_tri(p, a, b, c) -> bool:
    return _cross(a, b, p) >= 0 and _cross(b, c, p) >= 0 and _cross(c, a, p) >= 0


def triangulate(poly: np.ndarray) -> list[list[int]]:
    """Ear clipping on a simple CCW polygon -> list of index triangles."""
    idx = list(range(len(poly)))
    tris = []
    guard = 0
    while len(idx) > 3 and guard < 10_000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if _cross(a, b, c) <= 1e-9:
                continue                      # reflex or degenerate corner
            if any(_point_in_tri(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append([i0, i1, i2])
            idx.pop(k)
            break
        else:
            break                             # no ear found (self-touching input)
    if len(idx) == 3:
        tris.append(idx)
    return tris


def _is_convex(poly: np.ndarray) -> bool:
    n = len(poly)
    return all(_cross(poly[i - 1], poly[i], poly[(i + 1) % n]) >= -1e-9 for i in range(n))


def convex_parts(poly: np.ndarray) -> list[np.ndarray]:
    """Hertel-Mehlhorn: merge ear-clipped triangles across diagonals while the
    union stays convex. At most 4x the minimum number of pieces."""
    if _is_convex(poly):
        return [poly]
    pieces = triangulate(poly)
    merged = True
    while merged:
        merged = False
        for i in range(len(pieces)):
            for j in range(i + 1, len(pieces)):
                a, b = pieces[i], pieces[j]
                shared = None
                for k in range(len(a)):
                    u, v = a[k], a[(k + 1) % len(a)]
                    for m in range(len(b)):
                        if b[m] == v and b[(m + 1) % len(b)] == u:
                            shared = (k, m)
                            break
                    if shared:
                        break
                if not shared:
                    continue
                k, m = shared
                a_rot = a[k + 1:] + a[:k + 1]          # starts at v, ends at u
                b_rot = b[m + 1:] + b[:m + 1]          # starts at u, ends at v
                cand = a_rot + b_rot[1:-1]
                if _is_convex(poly[cand]):
                    pieces[i] = cand
                    pieces.pop(j)
                    merged = True
                    break
            if merged:
                break
    return [poly[p] for p in pieces]


def _dist_point_poly(p: np.ndarray, poly: np.ndarray) -> float:
    best = math.inf
    for i in range(len(poly)):
        a, b = poly[i - 1], poly[i]
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(p - (a + t * ab))))
    inside = False
    for i in range(len(poly)):
        a, b = poly[i - 1], poly[i]
        if (a[1] > p[1]) != (b[1] > p[1]):
            if p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]):
                inside = not inside
    return -best if inside else best


# --- build -------------------------------------------------------------------

def load_osm():
    data = json.loads(OSM_PATH.read_text(encoding="utf-8"))
    buildings, roads, campus = [], [], None
    for e in data["elements"]:
        tags = e.get("tags", {})
        geom = e.get("geometry")
        if not geom:
            continue
        pts = np.array([project(g["lat"], g["lon"]) for g in geom])
        if "building" in tags:
            buildings.append((e["id"], tags, pts))
        elif "highway" in tags:
            roads.append((tags, pts))
        elif tags.get("amenity") == "university":
            campus = pts
    return buildings, roads, campus


def choose_spawn(buildings) -> np.ndarray:
    """Open ground (SPAWN_CLEARANCE_REAL_M from every building) closest to
    SPAWN_NEAR, in real metres."""
    polys = [clean_polygon(p) for _, _, p in buildings]
    target = next(np.mean(clean_polygon(p), axis=0) for _, t, p in buildings
                  if t.get("name") == SPAWN_NEAR)
    best, best_d = None, math.inf
    for dx in np.arange(-150, 151, 3.0):
        for dy in np.arange(-150, 151, 3.0):
            c = target + np.array([dx, dy])
            if all(_dist_point_poly(c, q) >= SPAWN_CLEARANCE_REAL_M for q in polys):
                d = float(np.hypot(dx, dy))
                if d < best_d:
                    best, best_d = c, d
    return best


def build():
    buildings, roads, campus = load_osm()
    spawn = choose_spawn(buildings)

    def to_world(pts):
        return (np.asarray(pts) - spawn) / SCALE

    world = []
    for osm_id, tags, pts in buildings:
        poly_real = clean_polygon(pts)
        if len(poly_real) < 3:
            continue
        area = abs(_area2(poly_real)) / 2.0
        h = estimate_height_m(tags, area) / SCALE
        poly = to_world(poly_real)
        parts = [p for p in convex_parts(poly) if abs(_area2(p)) / 2.0 > 1e-4]
        world.append({"name": tags.get("name") or f"building_{osm_id}", "osm_id": osm_id,
                      "height": round(h, 4), "colour": colour(tags),
                      "footprint": poly.round(4).tolist(),
                      "parts": [p.round(4).tolist() for p in parts]})
    roads_w = [(t["highway"], to_world(p)) for t, p in roads]
    campus_w = to_world(campus) if campus is not None else None

    allpts = np.vstack([np.array(b["footprint"]) for b in world])   # roads run past the campus; crop to buildings
    lo = allpts.min(axis=0) - MARGIN_M
    hi = allpts.max(axis=0) + MARGIN_M
    # square floor centred on the features keeps the texture aspect simple
    centre = (lo + hi) / 2.0
    half = float(max(hi - lo) / 2.0)
    return world, roads_w, campus_w, centre, half, spawn


def write_texture(world, roads_w, campus_w, centre, half, path: Path, satellite: bool = False,
                  labels: bool = True):
    """satellite=True lays the real aerial photo (fetch_satellite.py) down as
    the ground and only outlines the footprints; otherwise the drawn map.
    labels=False leaves the ground clean for the 3D scene."""
    n = TEXTURE_PX
    if satellite:
        img = Image.open(OUT_DIR / "campus_satellite.png").convert("RGB").resize((n, n))
    else:
        img = Image.new("RGB", (n, n), (196, 205, 170))       # dry grass
    d = ImageDraw.Draw(img)

    def px(p):
        u = (p[0] - (centre[0] - half)) / (2 * half) * n
        v = (1.0 - (p[1] - (centre[1] - half)) / (2 * half)) * n
        return (u, v)

    m_to_px = n / (2 * half)
    if not satellite:
        if campus_w is not None:
            d.polygon([px(p) for p in campus_w], fill=(186, 200, 160), outline=(90, 130, 80))
        for kind, pts in sorted(roads_w, key=lambda r: _ROAD_W_REAL.get(r[0], 4.0)):
            w = max(2, int(_ROAD_W_REAL.get(kind, 4.0) / SCALE * m_to_px))
            col = (215, 200, 165) if kind in ("footway", "path") else (120, 120, 124)
            d.line([px(p) for p in pts], fill=col, width=w, joint="curve")
    for b in world:
        if satellite:
            if labels:                    # map window: show which footprint is which
                d.polygon([px(p) for p in b["footprint"]], outline=(0, 229, 255), width=3)
        else:
            r, g, bl = (int(255 * c * 0.75) for c in b["colour"])
            d.polygon([px(p) for p in b["footprint"]], fill=(r, g, bl), outline=(40, 40, 40))
    try:
        font = ImageFont.truetype("arial.ttf", 30)
    except OSError:
        font = ImageFont.load_default()
    for b in world:
        if not labels or b["name"].startswith("building_"):
            continue
        c = px(np.mean(np.array(b["footprint"]), axis=0))
        d.text((c[0], c[1] + 18), b["name"], fill=(20, 20, 20), font=font, anchor="mt",
               stroke_width=3, stroke_fill=(245, 245, 235))
    c = px((0.0, 0.0))                                         # home pad at the spawn point
    rr = 0.6 * m_to_px
    d.ellipse([c[0] - rr, c[1] - rr, c[0] + rr, c[1] + rr], fill=(60, 200, 240), outline=(20, 90, 120), width=4)
    d.text(c, "H", fill=(255, 255, 255), font=font, anchor="mm")
    img.save(path)


def write_xml(world, centre, half, out_dir: Path):
    floor_file = "campus_floor.png" if (out_dir / "campus_floor.png").exists() else "campus_map.png"
    assets, geoms = [], []
    for bi, b in enumerate(world):
        r, g, bl = b["colour"]
        for pi, part in enumerate(b["parts"]):
            mesh = f"b{bi}_{pi}"
            verts = []
            for z in (0.0, b["height"]):
                for x, y in part:
                    verts += [f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"]
            assets.append(f'    <mesh name="{mesh}" vertex="{" ".join(verts)}"/>')
            geoms.append(f'    <geom name="{mesh}" type="mesh" mesh="{mesh}" rgba="{r:.2f} {g:.2f} {bl:.2f} 1" '
                         f'contype="0" conaffinity="1"/>')
    (out_dir / "campus.xml").write_text(
        "<mujocoinclude>\n"
        "  <!-- Generated by make_campus.py from OpenStreetMap data - do not edit.\n"
        "       One convex mesh per building piece; conaffinity=1 so the drone's\n"
        "       feet and hull collide with them. Map data (c) OpenStreetMap contributors. -->\n"
        "  <asset>\n" + "\n".join(assets) + "\n  </asset>\n"
        "  <worldbody>\n" + "\n".join(geoms) + "\n  </worldbody>\n"
        "</mujocoinclude>\n", encoding="utf-8")
    (HERE / "scene_campus.xml").write_text(f"""<mujoco model="quad_scene_campus">
  <!-- GENERATED by make_campus.py - do not edit. quad.xml plus a 1:{SCALE:g}
       model of the Amrita Vishwa Vidyapeetham Coimbatore campus built from
       OpenStreetMap footprints (heights estimated). The drone spawns on open
       ground near the {SPAWN_NEAR}. -->
  <include file="quad.xml"/>
  <compiler meshdir="campus" texturedir="campus"/>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.5 0.5 0.5"/>
    <global azimuth="120" elevation="-20" offwidth="1920" offheight="1080"/>
    <quality shadowsize="4096"/>
    <map zfar="400" shadowclip="0.5" shadowscale="0.6"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.45 0.6 0.8" rgb2="0.05 0.08 0.15" width="512" height="3072"/>
    <texture type="2d" name="campus_map" file="{floor_file}"/>
    <material name="campus_map" texture="campus_map" texrepeat="1 1" reflectance="0" specular="0.05" shininess="0.02"/>
  </asset>
  <worldbody>
    <light name="sun" directional="true" pos="0 0 30" dir="-0.4 0.4 -0.8" diffuse="0.8 0.78 0.72" specular="0.1 0.1 0.1" castshadow="true"/>
    <light pos="0 0 20" dir="0 0 -1" diffuse="0.4 0.4 0.4" specular="0.1 0.1 0.1" castshadow="false"/>
    <geom name="floor" type="plane" pos="{centre[0]:.3f} {centre[1]:.3f} 0" size="{half:.3f} {half:.3f} 0.1"
          material="campus_map" contype="0" conaffinity="1"/>
  </worldbody>
  <include file="campus/campus.xml"/>
</mujoco>
""", encoding="utf-8")


def main():
    OUT_DIR.mkdir(exist_ok=True)
    world, roads_w, campus_w, centre, half, spawn = build()
    (OUT_DIR / "campus_world.json").write_text(json.dumps({
        "source": "OpenStreetMap via Overpass API, (c) OpenStreetMap contributors, ODbL",
        "scale": SCALE, "origin_latlon": [LAT0, LON0], "spawn_real_m": spawn.round(2).tolist(),
        "floor_centre": centre.round(4).tolist(), "floor_half": round(half, 4),
        "buildings": world}, indent=1), encoding="utf-8")
    sat = (OUT_DIR / "campus_satellite.png").exists()
    write_texture(world, roads_w, campus_w, centre, half, OUT_DIR / "campus_map.png", sat, True)
    if sat:                               # clean photo (no names) is the 3D ground
        write_texture(world, roads_w, campus_w, centre, half, OUT_DIR / "campus_floor.png", True, False)
    write_xml(world, centre, half, OUT_DIR)
    n_parts = sum(len(b["parts"]) for b in world)
    print(f"{len(world)} buildings -> {n_parts} convex pieces; floor {2 * half:.1f} m square "
          f"(1:{SCALE:g}); spawn near {SPAWN_NEAR}")


if __name__ == "__main__":
    main()
