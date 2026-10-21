"""Measure how far each OpenStreetMap building outline sits from its roof in
the satellite photo, and save a per-building shift for make_campus.py.

Why per building: the aerial photo and the OSM outlines were captured
separately, so the error is not one constant offset. fetch_satellite.py
removes the average offset; what is left varies by a few metres from
building to building (outlines traced from other imagery, roofs leaning
with height in an off-nadir photo). Shifting each 3D building onto its own
roof makes the boxes sit on the photo.

A building is only moved when the evidence is clear: its outline must
cross much stronger photo edges at the new place than at the old one, the
best place must lie inside the search window, and the move must be small.
Everything else stays at its OpenStreetMap position.

Run (needs campus/campus_satellite.png, see fetch_satellite.py):
    .venv\\Scripts\\python.exe 01_simulation\\models\\align_buildings.py
then: .venv\\Scripts\\python.exe 01_simulation\\models\\make_campus.py
Delete campus/building_shifts.json to go back to the raw OSM positions.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

import make_campus as mc

HERE = Path(__file__).resolve().parent
META = HERE / "campus" / "campus_world.json"
SAT = HERE / "campus" / "campus_satellite.png"
OUT = HERE / "campus" / "building_shifts.json"
SEARCH_REAL_M = 5.0            # largest shift tried, each axis (real metres)
STEP_REAL_M = 0.5
MIN_SCORE = 30.0               # mean photo edge strength needed at the best spot
MIN_GAIN = 1.15                # best score must beat the unshifted score by this factor
MIN_EDGE_POINTS = 150          # tiny outlines give noisy scores


def main() -> None:
    meta = json.loads(META.read_text(encoding="utf-8"))
    scale = meta["scale"]
    cx, cy = meta["floor_centre"]
    half = meta["floor_half"]
    spawn = np.array(meta["spawn_real_m"])
    gray = ndimage.gaussian_filter(np.asarray(Image.open(SAT).convert("L"), dtype=float), 1.5)
    edges = np.hypot(ndimage.sobel(gray, 0), ndimage.sobel(gray, 1))
    n = gray.shape[0]
    ppm = n / (2 * half)                              # texture pixels per world metre

    shifts, kept, skipped = {}, [], []
    buildings, _, _ = mc.load_osm()
    for osm_id, tags, pts in buildings:
        poly = mc.clean_polygon(pts)
        if len(poly) < 3:
            continue
        world = (poly - spawn) / scale                # raw OSM position, world metres
        ring = np.vstack([world, world[:1]])
        edge_pts = np.vstack([a + (c - a) * np.linspace(0, 1, max(2, int(np.linalg.norm(c - a) * ppm)))[:, None]
                              for a, c in zip(ring[:-1], ring[1:])])
        if len(edge_pts) < MIN_EDGE_POINTS:
            continue

        def score(dx_real: float, dy_real: float) -> float:
            u = (edge_pts[:, 0] + dx_real / scale - (cx - half)) * ppm
            v = (cy + half - (edge_pts[:, 1] + dy_real / scale)) * ppm
            return float(ndimage.map_coordinates(edges, [v, u], order=1).mean())

        grid = np.arange(-SEARCH_REAL_M, SEARCH_REAL_M + 1e-9, STEP_REAL_M)
        s0 = score(0.0, 0.0)
        best = max((score(dx, dy), dx, dy) for dx in grid for dy in grid)
        name = tags.get("name") or f"building_{osm_id}"
        on_edge = abs(best[1]) >= SEARCH_REAL_M - 1e-9 or abs(best[2]) >= SEARCH_REAL_M - 1e-9
        if best[0] >= MIN_SCORE and best[0] >= MIN_GAIN * s0 and not on_edge:
            shifts[str(osm_id)] = [round(best[1], 2), round(best[2], 2)]
            kept.append((name, best[1], best[2], s0, best[0]))
        else:
            skipped.append(name)

    OUT.write_text(json.dumps({"unit": "real metres, added to the OSM outline (east, north)",
                               "shifts": shifts}, indent=1), encoding="utf-8")
    a = np.array([[k[1], k[2]] for k in kept])
    print(f"{len(kept)} buildings moved, {len(skipped)} left at the OSM position; "
          f"median move ({np.median(a[:, 0]):+.1f}, {np.median(a[:, 1]):+.1f}) m, "
          f"mean size {np.hypot(a[:, 0], a[:, 1]).mean():.1f} m")
    print(f"edge score at outlines: before {np.mean([k[3] for k in kept]):.1f} -> after {np.mean([k[4] for k in kept]):.1f}")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
