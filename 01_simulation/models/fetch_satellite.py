"""Download real satellite imagery of the campus floor area and resample it
onto the floor plane, so the MuJoCo ground looks like the real campus.

Source: Esri World Imagery (Imagery (c) Esri, Maxar, Earthstar Geographics).
Tiles are cached in data/raw/satellite/z<zoom>/ and downloaded only once;
re-running with the cache present needs no internet.

The floor extent comes from campus/campus_world.json (written by
make_campus.py), and every output pixel is mapped back through the same
local projection make_campus.project() uses, then into Web Mercator, so the
photo lines up with the OpenStreetMap building footprints.

Run:  .venv\\Scripts\\python.exe 01_simulation\\models\\fetch_satellite.py
then: .venv\\Scripts\\python.exe 01_simulation\\models\\make_campus.py
"""
from __future__ import annotations

import json
import math
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import map_coordinates

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
META = HERE / "campus" / "campus_world.json"
OUT = HERE / "campus" / "campus_satellite.png"
ZOOM = 18                     # ~0.59 m per pixel here; zoom 19 is "not yet available" over most of the campus
TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")
CACHE = ROOT / "data" / "raw" / "satellite" / f"z{ZOOM}"
TEXTURE_PX = 4096
ATTRIBUTION = "Imagery (c) Esri, Maxar, Earthstar Geographics"
# The imagery sits this far (east, north) of the OpenStreetMap footprints,
# in real metres; measured by maximising photo edge strength along every
# footprint outline (score 30.4 unshifted -> 41.5, a single clear peak).
IMAGERY_OFFSET_REAL_M = (1.0, 5.5)
K = 111_320.0                 # metres per degree, as in make_campus.project()


def unproject(x_real, y_real, lat0, lon0):
    """Inverse of make_campus.project(): local metres -> (lat, lon)."""
    lat = lat0 + y_real / K
    lon = lon0 + x_real / (K * math.cos(math.radians(lat0)))
    return lat, lon


def mercator_px(lat, lon, z=ZOOM):
    """Global Web Mercator pixel coordinates (256 px tiles)."""
    n = 256 * 2 ** z
    x = (np.asarray(lon) + 180.0) / 360.0 * n
    s = np.sin(np.radians(lat))
    y = (0.5 - np.log((1 + s) / (1 - s)) / (4 * math.pi)) * n
    return x, y


def fetch_tile(tx: int, ty: int) -> Path:
    path = CACHE / f"{ty}_{tx}.jpg"
    if path.exists() and path.stat().st_size > 0:
        return path
    req = urllib.request.Request(TILE_URL.format(z=ZOOM, y=ty, x=tx),
                                 headers={"User-Agent": "drone-sim-course-project/1.0"})
    for attempt in range(4):
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                path.write_bytes(r.read())
            return path
        except Exception:
            if attempt == 3:
                raise
            time.sleep(1.5 * (attempt + 1))
    return path


def _is_placeholder(path: Path) -> bool:
    """Esri serves a flat grey 'Map data not yet available' tile where it has
    no imagery at this zoom."""
    a = np.asarray(Image.open(path).convert("L"), dtype=float)
    return a.std() < 6.0 and abs(a.mean() - 204) < 12


def main() -> None:
    meta = json.loads(META.read_text(encoding="utf-8"))
    scale = meta["scale"]
    lat0, lon0 = meta["origin_latlon"]
    spawn = np.array(meta["spawn_real_m"])
    cx, cy = meta["floor_centre"]
    half = meta["floor_half"]

    # output pixel centres -> world metres -> real metres -> lat/lon -> mercator px
    n = TEXTURE_PX
    s = (np.arange(n) + 0.5) / n
    xw = cx - half + s * 2 * half                      # columns, west -> east
    yw = cy + half - s * 2 * half                      # rows, north -> south (row 0 = top)
    ox, oy = IMAGERY_OFFSET_REAL_M
    lat_rows, _ = unproject(0.0, yw * scale + spawn[1] + oy, lat0, lon0)
    _, lon_cols = unproject(xw * scale + spawn[0] + ox, 0.0, lat0, lon0)
    mx, _ = mercator_px(lat0, lon_cols)
    _, my = mercator_px(lat_rows, lon0)

    tx0, tx1 = int(mx.min() // 256), int(mx.max() // 256)
    ty0, ty1 = int(my.min() // 256), int(my.max() // 256)
    tiles = [(tx, ty) for ty in range(ty0, ty1 + 1) for tx in range(tx0, tx1 + 1)]
    CACHE.mkdir(parents=True, exist_ok=True)
    cached = sum((CACHE / f"{ty}_{tx}.jpg").exists() for tx, ty in tiles)
    print(f"{len(tiles)} tiles at zoom {ZOOM} ({tx1 - tx0 + 1} x {ty1 - ty0 + 1}), "
          f"{cached} already cached")
    with ThreadPoolExecutor(max_workers=4) as pool:
        paths = list(pool.map(lambda t: fetch_tile(*t), tiles))
    total = sum(p.stat().st_size for p in paths)
    print(f"tiles on disk: {total / 1e6:.1f} MB in {CACHE}")

    blanks = [t for t, p in zip(tiles, paths) if _is_placeholder(p)]
    if blanks:
        raise SystemExit(f"{len(blanks)} of {len(tiles)} tiles are 'map data not yet available' "
                         f"placeholders at zoom {ZOOM}; lower ZOOM")
    mosaic = np.zeros(((ty1 - ty0 + 1) * 256, (tx1 - tx0 + 1) * 256, 3), dtype=np.uint8)
    for (tx, ty), p in zip(tiles, paths):
        tile = np.asarray(Image.open(p).convert("RGB").resize((256, 256)))
        mosaic[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = tile

    # separable mapping: column depends only on lon, row only on lat
    cols = np.broadcast_to(mx - tx0 * 256 - 0.5, (n, n))
    rows = np.broadcast_to((my - ty0 * 256 - 0.5)[:, None], (n, n))
    out = np.stack([map_coordinates(mosaic[..., c], [rows, cols], order=1, mode="nearest")
                    for c in range(3)], axis=-1)
    Image.fromarray(out).save(OUT)
    print(f"wrote {OUT} ({n} x {n} px, {2 * half * scale / n:.2f} real m per px). {ATTRIBUTION}")


if __name__ == "__main__":
    main()
