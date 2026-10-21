"""The live map window must put the drone where it really is."""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for p in ("05_voice_interface", "04_control", "01_simulation", "01_simulation/models"):
    sys.path.insert(0, str(ROOT / p))

from minimap import MiniMap, map_for, yaw_deg  # noqa: E402
from world import CAMPUS_SCENE  # noqa: E402
from sim_driver import SCENE_PATH  # noqa: E402


@pytest.mark.parametrize("name,scene", [("city", SCENE_PATH), ("campus", CAMPUS_SCENE)])
def test_map_extent_matches_scene_floor(name, scene):
    """The image covers exactly the floor plane MuJoCo draws it on."""
    _, (x0, x1, y0, y1), _ = map_for(name)
    m = mujoco.MjModel.from_xml_path(str(scene))
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    cx, cy = m.geom_pos[g][:2]
    hx, hy = m.geom_size[g][:2]
    assert np.allclose([x0, x1, y0, y1], [cx - hx, cx + hx, cy - hy, cy + hy], atol=1e-2)


def test_yaw_from_quaternion():
    for psi in (0.0, 45.0, 90.0, -135.0):
        r = np.radians(psi) / 2
        assert yaw_deg([np.cos(r), 0, 0, np.sin(r)]) == pytest.approx(psi, abs=1e-9)


@pytest.mark.parametrize("name", ["city", "campus"])
def test_drone_marker_trail_and_close_up_follow_the_drone(name):
    mm = MiniMap(name, show=False)
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]
    for x, y in path:
        mm.update([x, y, 1.0], [1, 0, 0, 0], target=np.array([3.0, 2.0, 1.0]))
    trail, tline, tmark, drone = mm.art[0]
    assert list(drone.get_xdata()) == [1.0] and list(drone.get_ydata()) == [2.0]
    assert list(zip(trail.get_xdata(), trail.get_ydata())) == path
    assert list(tmark.get_xdata()) == [3.0]
    # the close-up is drone-centred: the crop covers the whole fixed window
    ex0, ex1, ey0, ey1 = mm.img_zoom.get_extent()
    assert ex0 <= -mm.zoom and ex1 >= mm.zoom and ey0 <= -mm.zoom and ey1 >= mm.zoom
    assert mm.ax_zoom.get_xlim() == pytest.approx((-mm.zoom, mm.zoom))
    z_drone = mm.art[1][3]
    assert list(z_drone.get_xdata()) == [0.0] and list(z_drone.get_ydata()) == [0.0]
    # the map crop under the drone is the right part of the map: its pixel
    # at the drone matches the full map's pixel at (1, 2)
    x0, x1, y0, y1 = mm.extent
    col = int((1.0 - x0) * mm.px_per_m)
    row = int((y1 - 2.0) * mm.px_per_m)
    crop = mm.img_zoom.get_array()
    c = int((0.0 - ex0) * mm.px_per_m)
    r = int((ey1 - 0.0) * mm.px_per_m)
    assert np.array_equal(crop[r, c], mm.detail[row, col])
