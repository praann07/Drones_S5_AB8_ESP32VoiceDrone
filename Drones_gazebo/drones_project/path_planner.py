import json
import numpy as np
from scipy.interpolate import splprep, splev


class SplinePathPlanner:

  def __init__(self, obstacle_file="obstacles.json"):
    self.obstacles = self._load_obstacles(obstacle_file)

  def _load_obstacles(self, file_path):
    try:
      with open(file_path, "r") as f:
        data = json.load(f)

        if isinstance(data, dict):
          raw_list = data.get("buildings", [])
        elif isinstance(data, list):
          raw_list = data
        else:
          return []

        parsed_obstacles = []
        for b in raw_list:
          if not isinstance(b, dict):
            continue

          if "x_min" in b and "x_max" in b and "y_min" in b and "y_max" in b:
            x_center = (b["x_min"] + b["x_max"]) / 2.0
            y_center = (b["y_min"] + b["y_max"]) / 2.0
            width = abs(b["x_max"] - b["x_min"])
            depth = abs(b["y_max"] - b["y_min"])
            # Bounding circle radius covering full rectangular footprint
            radius = (np.sqrt(width**2 + depth**2) / 2.0) + 1.5

            parsed_obstacles.append({
                "name": b.get("name", "Building"),
                "x": x_center,
                "y": y_center,
                "radius": radius,
                "x_min": b["x_min"],
                "x_max": b["x_max"],
                "y_min": b["y_min"],
                "y_max": b["y_max"],
                "height": b.get("height", 15.0),
            })
          elif "x" in b and "y" in b:
            parsed_obstacles.append(b)

        return parsed_obstacles
    except Exception as e:
      print(f"   [Error loading obstacles.json: {e}]")
      return []

  def generate_bspline_path(
      self, start, goal, num_points=35, safety_margin=4.0
  ):
    start = np.array(start)
    goal = np.array(goal)

    waypoints = [start]
    dir_vec = goal[:2] - start[:2]
    total_dist = np.linalg.norm(dir_vec)

    if total_dist > 0:
      unit_dir = dir_vec / total_dist
      perp_dir = np.array([-unit_dir[1], unit_dir[0]])

      # Evaluate all obstacles lying between start and goal
      for obs in self.obstacles:
        obs_pos = np.array([obs["x"], obs["y"]])
        obs_r = obs["radius"] + safety_margin

        # Projection distance along path vector
        proj_length = np.dot(obs_pos - start[:2], unit_dir)

        if 0 < proj_length < total_dist:
          proj_pt = start[:2] + proj_length * unit_dir
          perp_dist = np.linalg.norm(obs_pos - proj_pt)

          # If the straight path intersects building radius, add a lateral waypoint
          if perp_dist < obs_r:
            # Determine shortest bypass side (left or right)
            cross_prod = np.cross(unit_dir, obs_pos - start[:2])
            bypass_side = 1.0 if cross_prod < 0 else -1.0

            bypass_2d = obs_pos + (bypass_side * perp_dir * (obs_r + 2.0))
            # Smooth altitude interpolation along path
            alpha = proj_length / total_dist
            bypass_alt = start[2] + alpha * (goal[2] - start[2])

            waypoints.append(
                np.array([bypass_2d[0], bypass_2d[1], bypass_alt])
            )

    waypoints.append(goal)

    # Sort waypoints sequentially from start to goal along path projection
    waypoints = sorted(
        waypoints, key=lambda p: np.dot(p[:2] - start[:2], dir_vec + 1e-6)
    )

    # Ensure unique waypoints to avoid B-spline fitting singularity
    unique_waypoints = [waypoints[0]]
    for wp in waypoints[1:]:
      if np.linalg.norm(wp[:2] - unique_waypoints[-1][:2]) > 0.5:
        unique_waypoints.append(wp)

    pts = np.array(unique_waypoints).T
    k = min(3, len(unique_waypoints) - 1)
    if k < 1:
      return [start, goal]

    # Generate dense 3D cubic B-spline curve
    tck, _ = splprep(pts, s=0, k=k)
    u_fine = np.linspace(0, 1, num_points)
    dense_path = np.array(splev(u_fine, tck)).T

    return dense_path
