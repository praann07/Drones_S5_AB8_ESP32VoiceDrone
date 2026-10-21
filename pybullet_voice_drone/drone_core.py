"""Shared core for the voice / keyboard controlled PyBullet quadcopter.

main.py, Voice_Drone_PyBullet.py and obstacles.py are thin launchers around
``main()`` in this module; they only differ in their default options.

Layers (top to bottom):
    parse_command()      spoken/typed text  -> Intent
    DroneController      intents + keys     -> smooth motion, flight modes, obstacle avoidance
    DroneModel/CameraRig controller state   -> PyBullet scene (single multibody from drone.urdf)

The controller and the avoider are plain Python (no PyBullet calls), so they can
be unit-tested headlessly -- see test_drone_core.py.

World frame: +X forward at yaw 0, +Y left, +Z up (metres, radians).
"""

import argparse
import csv
import math
import os
import queue
import re
import struct
import threading
import time
import zlib
from dataclasses import dataclass
from datetime import datetime
from enum import Enum

import pybullet as p
import pybullet_data

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "drone.urdf")
TELEMETRY_PATH = os.path.join(HERE, "drone_data.csv")
GROUND_TEXTURE = os.path.join(HERE, "assets", "ground.png")


# =========================================================
# SETTINGS
# =========================================================

DT = 1.0 / 120.0                        # target frame time

HORIZONTAL_SPEED = 1.5                  # m/s
VERTICAL_SPEED = 1.0                    # m/s
ROTATION_SPEED = math.radians(45)       # rad/s
ACCELERATION = 3.0                      # m/s^2, how quickly velocity follows the command
ROTATION_ACCEL = math.radians(180)      # rad/s^2

TAKEOFF_HEIGHT = 2.0
LANDED_HEIGHT = 0.31                    # body-centre height when parked: skids (z = -0.30) rest on the home pad (0.01 thick)
MIN_FLIGHT_HEIGHT = 0.5                 # "down" will not go lower than this; "land" will
MAX_HEIGHT = 8.0
WORLD_LIMIT = 12.0

# Obstacle avoidance
DRONE_RADIUS = 1.32                     # horizontal footprint incl. propeller tips (see drone.urdf)
DRONE_HALF_HEIGHT = 0.30                # skids to rotor discs
SAFETY_DISTANCE = 0.25                  # clearance kept between the footprint and an obstacle
DETECTION_DISTANCE = 2.0                # how far ahead the drone looks
PLANNING_HORIZON = 2 * DETECTION_DISTANCE
AVOID_SPEED = 2.0                       # sideways speed while going around an obstacle
CLEAR_HYSTERESIS = 0.1                  # extra clearance needed before resuming the original course
SIDE_BIAS = 0.5                         # prefer the side we already used if it is this much dearer
STOP_BUFFER = 0.4                       # the drone halts (or stops going forward) this far before the safety zone
BRAKE_DECEL = 2.0                       # m/s^2 used to plan the stop; gentler than ACCELERATION so it is achievable

PROP_SPIN = 40.0                        # rad/s at full throttle (visual only)
TILT_PER_MPS = math.radians(8.0)        # lean per m/s of speed: 1.5 m/s -> 12 degrees
MAX_TILT = math.radians(15.0)
TRAIL_SECONDS = 10.0                    # how long a piece of the flight trail stays visible
DISC_ALPHA = 0.30                       # opacity of the rotor discs at full speed
ALERT_RANGE = 3.0                       # an obstacle starts to glow when the drone is this close to its zone
SKY_COLOR = [0.60, 0.76, 0.92]
LIGHT_POSITION = [-10.0, 8.0, 12.0]     # where the sun is: low enough for long shadows, not so high it glares

CAMERA_DISTANCE = 8.0
CAMERA_YAW = 45.0
CAMERA_PITCH = -30.0


# =========================================================
# SMALL HELPERS
# =========================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def approach(value, target, max_delta):
    """Move value towards target by at most max_delta."""
    if abs(target - value) <= max_delta:
        return target
    return value + math.copysign(max_delta, target - value)


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# =========================================================
# COMMANDS
# =========================================================

class Intent(Enum):
    TAKEOFF = "TAKE OFF"
    LAND = "LAND"
    STOP = "STOP"
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    UP = "UP"
    DOWN = "DOWN"
    ROTATE_LEFT = "ROTATE LEFT"
    ROTATE_RIGHT = "ROTATE RIGHT"
    HOME = "RETURN HOME"
    CAMERA = "CAMERA"
    TRAIL = "TRAIL"
    ZONES = "ZONES"
    EXIT = "EXIT"


# (forward, left, up, yaw_rate) targets for every motion intent
MOTION = {
    Intent.FORWARD: (HORIZONTAL_SPEED, 0.0, 0.0, 0.0),
    Intent.BACKWARD: (-HORIZONTAL_SPEED, 0.0, 0.0, 0.0),
    Intent.LEFT: (0.0, HORIZONTAL_SPEED, 0.0, 0.0),
    Intent.RIGHT: (0.0, -HORIZONTAL_SPEED, 0.0, 0.0),
    Intent.UP: (0.0, 0.0, VERTICAL_SPEED, 0.0),
    Intent.DOWN: (0.0, 0.0, -VERTICAL_SPEED, 0.0),
    Intent.ROTATE_LEFT: (0.0, 0.0, 0.0, ROTATION_SPEED),
    Intent.ROTATE_RIGHT: (0.0, 0.0, 0.0, -ROTATION_SPEED),
}

# Speech recognisers often return a homophone for "right"; "back"/"forwards" are common variants.
_WORD_INTENTS = {
    "exit": Intent.EXIT, "quit": Intent.EXIT,
    "stop": Intent.STOP, "halt": Intent.STOP, "freeze": Intent.STOP,
    "hover": Intent.STOP, "hold": Intent.STOP,
    "takeoff": Intent.TAKEOFF, "launch": Intent.TAKEOFF,
    "land": Intent.LAND, "landing": Intent.LAND,
    "forward": Intent.FORWARD, "forwards": Intent.FORWARD, "ahead": Intent.FORWARD,
    "backward": Intent.BACKWARD, "backwards": Intent.BACKWARD,
    "back": Intent.BACKWARD, "reverse": Intent.BACKWARD,
    "left": Intent.LEFT,
    "right": Intent.RIGHT, "write": Intent.RIGHT, "rite": Intent.RIGHT,
    "up": Intent.UP, "rise": Intent.UP, "ascend": Intent.UP, "climb": Intent.UP,
    "down": Intent.DOWN, "descend": Intent.DOWN, "lower": Intent.DOWN,
    "camera": Intent.CAMERA,
    "trail": Intent.TRAIL,
    "zones": Intent.ZONES, "zone": Intent.ZONES,
}
_ROTATE_VERBS = {"rotate", "turn", "spin", "yaw", "pivot"}
_ROTATE_SIDES = {
    "left": Intent.ROTATE_LEFT,
    "right": Intent.ROTATE_RIGHT, "write": Intent.ROTATE_RIGHT, "rite": Intent.ROTATE_RIGHT,
}

_TOKEN = re.compile(r"\d+(?:\.\d+)?|[a-z]+")
_ONES = {"zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7,
         "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13,
         "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17, "eighteen": 18,
         "nineteen": 19}
_TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60, "seventy": 70,
         "eighty": 80, "ninety": 90}
_LENGTH_UNITS = {"foot": 0.3048, "feet": 0.3048, "ft": 0.3048}      # anything else is metres
MIN_MOVE, MAX_MOVE, MAX_TURN_DEG = 0.1, 50.0, 720.0


def _tokens(text):
    """Lower-case words and numbers; "take off" / "lift off" become the single word "takeoff"."""
    words = _TOKEN.findall(text.lower().replace("take-off", "takeoff"))
    merged = []
    i = 0
    while i < len(words):
        if words[i] in ("take", "lift") and i + 1 < len(words) and words[i + 1] == "off":
            merged.append("takeoff")
            i += 2
        else:
            merged.append(words[i])
            i += 1
    return merged


def _number_at(tokens, i):
    """Parse "3", "2.5", "twenty five", "one hundred eighty" starting at tokens[i]: (value, next_i)."""
    if tokens[i][0].isdigit():
        return float(tokens[i]), i + 1
    total, j, seen = 0, i, False
    while j < len(tokens):
        word = tokens[j]
        if word in _ONES:
            total += _ONES[word]
        elif word in _TENS:
            total += _TENS[word]
        elif word == "hundred" and seen:
            total *= 100
        else:
            break
        seen = True
        j += 1
    return (float(total), j) if seen else None


def _amount(tokens, intent):
    """The first number in the phrase: degrees for a rotation, metres for a translation."""
    for i in range(len(tokens)):
        parsed = _number_at(tokens, i)
        if parsed is None:
            continue
        value, after = parsed
        unit = tokens[after] if after < len(tokens) else None
        if intent in (Intent.ROTATE_LEFT, Intent.ROTATE_RIGHT):
            if unit in ("radian", "radians"):
                value = math.degrees(value)
            return clamp(value, 0.0, MAX_TURN_DEG) or None
        return clamp(value * _LENGTH_UNITS.get(unit, 1.0), 0.0, MAX_MOVE) or None
    return None


def parse_request(text):
    """Like parse_command, but also returns the distance/angle asked for: (intent, amount).

    "forward 3 meters" -> (FORWARD, 3.0), "rotate left ninety degrees" -> (ROTATE_LEFT, 90.0),
    "up ten feet" -> (UP, 3.048). amount is None for plain commands such as "forward".
    """
    tokens = _tokens(text)
    intent = _intent_from_tokens(tokens)
    if intent in MOTION:
        return intent, _amount(tokens, intent)
    return intent, None


def parse_command(text):
    """Turn a spoken/typed phrase into an Intent, or None if nothing matches.

    Whole words are matched, so "island" is not "land" and "go to the left"
    works. The first command word in the phrase wins ("home" beats everything,
    so "go back home" returns home instead of flying backwards).
    """
    return _intent_from_tokens(_tokens(text))


def _intent_from_tokens(merged):
    if merged == ["take"]:                      # the recogniser often drops "off"
        return Intent.TAKEOFF
    if "home" in merged:
        return Intent.HOME

    for i, word in enumerate(merged):
        if word in _ROTATE_VERBS:
            # "turn to the left", "rotate the drone left": any later side word means yaw,
            # never a sideways slide.
            for following in merged[i + 1:]:
                if following in _ROTATE_SIDES:
                    return _ROTATE_SIDES[following]
            continue
        if word in _WORD_INTENTS:
            return _WORD_INTENTS[word]
    return None


# =========================================================
# OBSTACLES AND AVOIDANCE
# =========================================================

@dataclass(frozen=True)
class Obstacle:
    name: str
    center: tuple
    half: tuple


DEFAULT_OBSTACLES = (
    Obstacle("Obstacle_1", (3.5, 0.0, 1.5), (0.7, 0.7, 1.5)),
    Obstacle("Obstacle_2", (7.0, 2.0, 1.5), (0.7, 0.7, 1.5)),
    Obstacle("Obstacle_3", (5.0, -3.0, 1.5), (0.7, 0.7, 1.5)),
)


def inflated_box(obstacle, extra):
    """Obstacle box grown by the drone's footprint plus `extra` clearance."""
    grow = (DRONE_RADIUS + extra, DRONE_RADIUS + extra, DRONE_HALF_HEIGHT + extra)
    low = tuple(c - h - g for c, h, g in zip(obstacle.center, obstacle.half, grow))
    high = tuple(c + h + g for c, h, g in zip(obstacle.center, obstacle.half, grow))
    return low, high


def inside_box(point, low, high):
    """Strictly inside: standing exactly on a face does not count."""
    return all(a < x < b for x, a, b in zip(point, low, high))


def ray_box_distance(origin, direction, low, high, max_dist):
    """Distance along the ray at which it enters the box (0 if it starts inside), else None.

    Slab method. Rays that only graze a face or edge count as misses.
    """
    t_near, t_far = 0.0, max_dist
    for o, d, a, b in zip(origin, direction, low, high):
        if abs(d) < 1e-9:
            if o <= a or o >= b:
                return None
        else:
            t1, t2 = (a - o) / d, (b - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            t_near = max(t_near, t1)
            t_far = min(t_far, t2)
            if t_near >= t_far:
                return None
    return t_near


class ObstacleAvoider:
    """Keeps a commanded world-frame velocity from running into box obstacles.

    Every frame a ray is cast along the commanded direction of travel; it is
    tested against each obstacle grown by the drone's footprint plus
    SAFETY_DISTANCE (the "safety zone"). Two strategies:

    "stop" (default)
        The drone brakes smoothly and halts STOP_BUFFER before the safety zone,
        then stays put. `stopped` is True while it is held there; the controller
        cancels the command. Moving away from the obstacle, or in any direction
        that is clear, works as normal.

    "steer"
        Once per encounter it picks left or right (whichever needs the smaller
        sideways shift to clear everything within the planning horizon, and is
        free and inside the world), otherwise up if the drone can climb over,
        otherwise it stops. It strafes (or climbs) while its forward speed fades,
        then resumes the original command as soon as the ray is clear again.
    """

    STRATEGIES = ("stop", "steer")

    def __init__(self, obstacles, log=print, strategy="stop"):
        if strategy not in self.STRATEGIES:
            raise ValueError("strategy must be one of %s, not %r" % (self.STRATEGIES, strategy))
        self.obstacles = tuple(obstacles)
        self.log = log
        self.strategy = strategy
        self.last_side = 1
        self.obstacle_name = None
        self.sensor = None          # (unit direction, distance to the safety zone) while something is ahead
        self.reset()

    def reset(self):
        self.active = False
        self.mode = None            # steer strategy: "left" | "right" | "up" | "blocked"
        self.stopped = False        # stop strategy: held at the safety zone right now
        self._announced = False

    def _finish(self):
        if self.active and self.strategy == "steer":
            self.log("PATH CLEAR - RESUMING COMMAND")
        self.reset()

    def _boxes(self, extra):
        return [(ob,) + inflated_box(ob, extra) for ob in self.obstacles]

    def first_hit(self, pos, direction, length, extra):
        best = None
        for ob, low, high in self._boxes(extra):
            t = ray_box_distance(pos, direction, low, high, length)
            if t is not None and (best is None or t < best[0]):
                best = (t, ob, low, high)
        return best

    def steer(self, pos, velocity):
        """The velocity to actually fly, given the one that was commanded."""
        self.sensor = None
        if self.strategy == "stop":
            return self._brake(pos, velocity)
        return self._steer_around(pos, velocity)

    def _brake(self, pos, velocity):
        self.stopped = False
        speed = math.sqrt(sum(v * v for v in velocity))
        if speed < 0.05:
            self._finish()
            return velocity

        d = tuple(v / speed for v in velocity)
        hit = self.first_hit(pos, d, DETECTION_DISTANCE, SAFETY_DISTANCE)
        if hit is None:
            self._finish()
            return velocity

        t_hit, obstacle, _, _ = hit
        self.sensor = (d, t_hit)
        if t_hit <= 0.0:
            # Already inside the zone (or on its edge): only moving away from the obstacle is allowed.
            if sum(di * (pi - ci) for di, pi, ci in zip(d, pos, obstacle.center)) > 0.0:
                return velocity
            allowed = 0.0
        else:
            # Fastest speed from which BRAKE_DECEL can still stop us STOP_BUFFER before the zone.
            allowed = math.sqrt(2.0 * BRAKE_DECEL * max(t_hit - STOP_BUFFER, 0.0))

        if not self.active:
            self.active = True
            self.log("OBSTACLE DETECTED: %s" % obstacle.name)
        if allowed >= speed:
            return velocity                 # still far enough away, no need to slow down yet
        if allowed < 0.1:
            self.stopped = True
            self.obstacle_name = obstacle.name
            if not self._announced:
                self._announced = True
                self.log("STOPPED: %s is in the way. Give another direction to move on." % obstacle.name)
            return (0.0, 0.0, 0.0)
        return tuple(v * allowed / speed for v in velocity)

    def _steer_around(self, pos, velocity):
        vx, vy, vz = velocity
        speed = math.hypot(vx, vy)
        if speed < 0.05:                    # hovering or moving only vertically
            self._finish()
            return velocity

        d = (vx / speed, vy / speed, 0.0)
        extra = SAFETY_DISTANCE + (CLEAR_HYSTERESIS if self.active else 0.0)
        hit = self.first_hit(pos, d, DETECTION_DISTANCE, extra)
        if hit is None:
            self._finish()
            return velocity

        t_hit, obstacle, _, high = hit
        self.sensor = (d, t_hit)
        if not self.active:
            self.log("OBSTACLE DETECTED: " + obstacle.name)
        if not self.active or not self._mode_is_valid(pos, d):
            self._choose(pos, d, high)
        self.active = True

        if self.mode == "blocked":
            return (0.0, 0.0, vz)

        slow = clamp((t_hit - STOP_BUFFER) / (DETECTION_DISTANCE - STOP_BUFFER), 0.0, 1.0)
        fx, fy = d[0] * speed * slow, d[1] * speed * slow
        if self.mode == "up":
            return (fx, fy, max(vz, VERTICAL_SPEED))
        side = 1.0 if self.mode == "left" else -1.0
        return (fx - d[1] * side * AVOID_SPEED, fy + d[0] * side * AVOID_SPEED, vz)

    def _mode_is_valid(self, pos, d):
        if self.mode == "up":
            return pos[2] < MAX_HEIGHT - 0.05
        if self.mode in ("left", "right"):
            side = 1.0 if self.mode == "left" else -1.0
            lateral = (-d[1] * side, d[0] * side, 0.0)
            return self.first_hit(pos, lateral, 1.0, SAFETY_DISTANCE) is None
        return False

    def _lateral_offsets(self, pos, d):
        """Smallest sideways shifts (left >= 0, right <= 0) that clear every obstacle ahead."""
        n = (-d[1], d[0])
        blocked = []
        for _, low, high in self._boxes(SAFETY_DISTANCE):
            if not low[2] < pos[2] < high[2]:
                continue                    # we fly above or below this one
            along, lateral = [], []
            for x in (low[0], high[0]):
                for y in (low[1], high[1]):
                    rx, ry = x - pos[0], y - pos[1]
                    along.append(rx * d[0] + ry * d[1])
                    lateral.append(rx * n[0] + ry * n[1])
            if max(along) <= 0.0 or min(along) >= PLANNING_HORIZON:
                continue                    # behind us or too far to matter yet
            blocked.append((min(lateral), max(lateral)))

        eps = 0.02
        left = 0.0
        moved = True
        while moved:
            moved = False
            for a, b in blocked:
                if a < left < b:
                    left, moved = b + eps, True
        right = 0.0
        moved = True
        while moved:
            moved = False
            for a, b in blocked:
                if a < right < b:
                    right, moved = a - eps, True
        return left, right

    def _choose(self, pos, d, obstacle_top):
        n = (-d[1], d[0])
        options = []
        for side, shift in zip((1.0, -1.0), self._lateral_offsets(pos, d)):
            target = (pos[0] + n[0] * shift, pos[1] + n[1] * shift)
            if max(abs(target[0]), abs(target[1])) > WORLD_LIMIT - 0.2:
                continue
            lateral = (n[0] * side, n[1] * side, 0.0)
            if self.first_hit(pos, lateral, abs(shift), SAFETY_DISTANCE) is not None:
                continue
            cost = abs(shift) - (SIDE_BIAS if side == self.last_side else 0.0)
            options.append((cost, side))

        if options:
            side = min(options)[1]
            self.last_side = int(side)
            mode = "left" if side > 0 else "right"
        elif pos[2] + (obstacle_top[2] - pos[2]) + 0.05 <= MAX_HEIGHT:
            mode = "up"
        else:
            mode = "blocked"

        if mode != self.mode:
            self.log({
                "left": "AUTO AVOIDANCE: MOVING LEFT",
                "right": "AUTO AVOIDANCE: MOVING RIGHT",
                "up": "AUTO AVOIDANCE: CLIMBING OVER",
                "blocked": "AUTO AVOIDANCE: NO SAFE ROUTE - STOPPING",
            }[mode])
        self.mode = mode


# =========================================================
# DRONE STATE AND MOTION
# =========================================================

class FlightMode(Enum):
    LANDED = "LANDED"
    TAKEOFF = "TAKEOFF"
    FLYING = "FLYING"
    LANDING = "LANDING"


class DroneController:
    """Kinematic flight model: commands in, smooth motion out. No PyBullet calls."""

    def __init__(self, obstacles=(), start=(0.0, 0.0), log=print, on_obstacle="stop"):
        self.log = log
        self.obstacles = tuple(obstacles)
        self.avoider = (ObstacleAvoider(self.obstacles, log, on_obstacle)
                        if self.obstacles else None)
        self.pos = [float(start[0]), float(start[1]), LANDED_HEIGHT]
        self.home = (float(start[0]), float(start[1]))
        self.maneuver = None                # measured move / measured turn / return-home in progress
        self.yaw = 0.0
        self.vel = [0.0, 0.0, 0.0]          # world frame, what the drone is actually doing
        self.yaw_rate = 0.0
        self.mode = FlightMode.LANDED
        self.label = "LANDED"
        self.cmd = [0.0, 0.0, 0.0]          # body frame target: forward, left, up
        self.cmd_yaw = 0.0
        self.exit_requested = False
        self.collisions_prevented = 0
        self._last_hint = (None, -1e9)

    # ---- commands -------------------------------------------------------

    def apply_intent(self, intent, amount=None):
        """Apply a parsed command. `amount` is metres (moves) or degrees (turns), if one was given.

        Returns False if the command was refused.
        """
        if intent is Intent.EXIT:
            self.exit_requested = True
            self.log(">> EXIT")
            return True
        if intent is Intent.STOP:
            self.stop()
            self.log(">> STOP")
            return True
        if intent is Intent.TAKEOFF:
            return self.take_off()
        if intent is Intent.LAND:
            return self.land()
        if intent is Intent.HOME:
            return self.go_home()
        if intent in MOTION:
            if amount is not None:
                if intent in (Intent.ROTATE_LEFT, Intent.ROTATE_RIGHT):
                    return self.turn_by(intent, amount)
                return self.move_by(intent, amount)
            accepted = self.set_manual(*MOTION[intent], label=intent.value)
            if accepted:
                self.log(">> " + intent.value)
            return accepted
        return False                         # CAMERA, TRAIL, ZONES are handled by the caller

    def _begin_maneuver(self, rising):
        """Get ready for an autonomous move. Refused on the ground unless the move is straight up."""
        if self.mode is FlightMode.LANDED:
            if not rising:
                self._hint("Drone is on the ground - say 'take off' (or press T) first.")
                return False
            self.mode = FlightMode.FLYING
        elif self.mode in (FlightMode.TAKEOFF, FlightMode.LANDING):
            self.mode = FlightMode.FLYING
        self.cmd = [0.0, 0.0, 0.0]
        self.cmd_yaw = 0.0
        return True

    def move_by(self, intent, distance):
        """Fly exactly `distance` metres in the direction of a motion intent, then hover."""
        forward, left, up, _ = MOTION[intent]
        if not self._begin_maneuver(rising=up > 0.0):
            return False
        speed = math.sqrt(forward * forward + left * left + up * up)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        direction = ((forward * c - left * s) / speed, (forward * s + left * c) / speed, up / speed)
        distance = clamp(distance, MIN_MOVE, MAX_MOVE)
        self.maneuver = {"kind": "move", "dir": direction, "remaining": distance, "speed": speed,
                         "stalled": 0.0, "time_left": 3.0 * distance / speed + 10.0}
        self.label = "%s %g m" % (intent.value, round(distance, 1))
        self.log(">> " + self.label)
        return True

    def turn_by(self, intent, degrees):
        """Rotate exactly `degrees` on the spot, then hover."""
        if not self._begin_maneuver(rising=False):
            return False
        degrees = clamp(degrees, 1.0, MAX_TURN_DEG)
        sign = 1.0 if intent is Intent.ROTATE_LEFT else -1.0
        self.maneuver = {"kind": "turn", "sign": sign, "remaining": math.radians(degrees),
                         "stalled": 0.0,
                         "time_left": 3.0 * math.radians(degrees) / ROTATION_SPEED + 10.0}
        self.label = "%s %g deg" % (intent.value, round(degrees, 1))
        self.log(">> " + self.label)
        return True

    def go_home(self):
        """Fly back over the take-off point, avoiding obstacles on the way, and land."""
        if self.mode is FlightMode.LANDED:
            self._hint("Drone is on the ground - it is already home.")
            return False
        distance = math.hypot(self.home[0] - self.pos[0], self.home[1] - self.pos[1])
        if distance < 0.15:
            self.log("Already above home.")
            return self.land()
        self._begin_maneuver(rising=True)
        self.maneuver = {"kind": "goto", "stalled": 0.0, "then_land": True,
                         "time_left": 3.0 * distance / HORIZONTAL_SPEED + 10.0}
        self.label = Intent.HOME.value
        self.log(">> " + self.label)
        return True

    def set_manual(self, forward, left, up, yaw_rate, label):
        """Set body-frame targets (used by voice intents and held keys)."""
        self.maneuver = None
        if self.mode is FlightMode.LANDED:
            if up <= 0.0:
                self._hint("Drone is on the ground - say 'take off' (or press T) first.")
                return False
            forward = left = yaw_rate = 0.0  # lifting straight off the ground
            self.mode = FlightMode.FLYING
        elif self.mode in (FlightMode.TAKEOFF, FlightMode.LANDING):
            self.mode = FlightMode.FLYING    # a manual command cancels the autopilot
        self.cmd = [forward, left, up]
        self.cmd_yaw = yaw_rate
        self.label = label
        return True

    def stop(self):
        self.maneuver = None
        self.cmd = [0.0, 0.0, 0.0]
        self.cmd_yaw = 0.0
        if self.mode in (FlightMode.TAKEOFF, FlightMode.LANDING):
            self.mode = FlightMode.FLYING
        self.label = "LANDED" if self.mode is FlightMode.LANDED else "STOP"

    def take_off(self):
        if self.mode is FlightMode.TAKEOFF:
            return True
        if self.mode is FlightMode.FLYING and self.pos[2] >= TAKEOFF_HEIGHT - 0.05:
            # Never snap a drone that is already higher back down to cruise height.
            self.log("Already airborne at %.1f m." % self.pos[2])
            return False
        self.maneuver = None
        self.cmd = [0.0, 0.0, 0.0]
        self.cmd_yaw = 0.0
        self.mode = FlightMode.TAKEOFF
        self.label = "TAKE OFF"
        self.log(">> TAKE OFF")
        return True

    def land(self):
        if self.mode is FlightMode.LANDED:
            self.log("Already on the ground.")
            return False
        self.maneuver = None
        self.cmd = [0.0, 0.0, 0.0]
        self.cmd_yaw = 0.0
        self.mode = FlightMode.LANDING
        self.label = "LAND"
        self.log(">> LAND")
        return True

    def _hint(self, message):
        """Log a hint, but not more than once every two seconds (held keys call this every frame)."""
        now = time.monotonic()
        if message != self._last_hint[0] or now - self._last_hint[1] > 2.0:
            self.log(message)
            self._last_hint = (message, now)

    # ---- simulation step -----------------------------------------------

    def step(self, dt):
        if dt <= 0.0:
            return
        fwd, left, up = self.cmd
        yaw_cmd = self.cmd_yaw
        if self.mode is FlightMode.LANDED:
            fwd = left = up = yaw_cmd = 0.0
        elif self.mode is FlightMode.TAKEOFF:
            fwd = left = yaw_cmd = 0.0
            up = clamp((TAKEOFF_HEIGHT - self.pos[2]) * 2.0, 0.1, VERTICAL_SPEED)
        elif self.mode is FlightMode.LANDING:
            fwd = left = yaw_cmd = 0.0
            up = -clamp((self.pos[2] - LANDED_HEIGHT) * 2.0, 0.15, VERTICAL_SPEED)

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        want = [fwd * c - left * s, fwd * s + left * c, up]
        if self.maneuver is not None and self.mode is FlightMode.FLYING:
            want, yaw_cmd = self._maneuver_targets()
        if self.avoider is not None:
            if self.mode is FlightMode.FLYING:
                want = list(self.avoider.steer(self.pos, want))
                if self.avoider.stopped:
                    self._halt_for_obstacle()
            else:
                self.avoider.reset()

        max_dv = ACCELERATION * dt
        self.vel = [approach(v, w, max_dv) for v, w in zip(self.vel, want)]
        self.yaw_rate = approach(self.yaw_rate, yaw_cmd, ROTATION_ACCEL * dt)
        old_pos, old_yaw = list(self.pos), self.yaw
        self.yaw = wrap_angle(self.yaw + self.yaw_rate * dt)

        blocked = self._move(dt)
        self._advance_maneuver(old_pos, old_yaw, dt)
        self._update_mode(blocked)

    # ---- measured moves / turns / return home --------------------------

    def _maneuver_targets(self):
        """World-frame velocity and yaw rate wanted right now; slows down as the goal gets close."""
        m = self.maneuver
        if m["kind"] == "move":
            speed = clamp(m["remaining"] * 2.0, 0.15, m["speed"])
            return [d * speed for d in m["dir"]], 0.0
        if m["kind"] == "turn":
            rate = clamp(m["remaining"] * 2.0, 0.15, ROTATION_SPEED)
            return [0.0, 0.0, 0.0], m["sign"] * rate
        dx, dy = self.home[0] - self.pos[0], self.home[1] - self.pos[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return [0.0, 0.0, 0.0], 0.0
        speed = clamp(distance * 1.5, 0.15, HORIZONTAL_SPEED)
        return [dx / distance * speed, dy / distance * speed, 0.0], 0.0

    def _advance_maneuver(self, old_pos, old_yaw, dt):
        m = self.maneuver
        if m is None or self.mode is not FlightMode.FLYING:
            return
        m["time_left"] -= dt
        if m["kind"] == "move":
            gained = sum((n - o) * d for n, o, d in zip(self.pos, old_pos, m["dir"]))
            m["remaining"] -= max(0.0, gained)
            done = m["remaining"] <= 0.02
        elif m["kind"] == "turn":
            m["remaining"] -= abs(wrap_angle(self.yaw - old_yaw))
            done = m["remaining"] <= 0.005
        else:
            done = math.hypot(self.home[0] - self.pos[0], self.home[1] - self.pos[1]) <= 0.1
        if done:
            self._end_maneuver("COMPLETE")
            return

        # Give up if the drone is pinned (world limit, ceiling, no route) or it simply takes too long.
        pinned = m["kind"] != "turn" and math.hypot(*self.vel) < 0.05
        m["stalled"] = m["stalled"] + dt if pinned else 0.0
        if m["stalled"] > 1.5:
            self._end_maneuver("BLOCKED - HOVERING")
        elif m["time_left"] <= 0.0:
            self._end_maneuver("TIMED OUT - HOVERING")

    def _halt_for_obstacle(self):
        """Held at an obstacle's safety zone: drop the command so it cannot restart on its own
        (e.g. a 'forward' that would resume the moment the drone is turned away)."""
        if self.maneuver is not None:
            self._end_maneuver("STOPPED - OBSTACLE AHEAD")
        self.cmd = [0.0, 0.0, 0.0]
        self.cmd_yaw = 0.0
        self.label = "OBSTACLE STOP"

    def _end_maneuver(self, outcome):
        m, self.maneuver = self.maneuver, None
        self.cmd = [0.0, 0.0, 0.0]
        self.cmd_yaw = 0.0
        self.label = "STOP"                    # becomes HOVER once the drone has settled
        name = {"move": "MOVE", "turn": "TURN", "goto": "RETURN HOME"}[m["kind"]]
        self.log(">> %s %s" % (name, outcome))
        if outcome == "COMPLETE" and m.get("then_land"):
            self.land()

    def _collides(self, point):
        return any(inside_box(point, *inflated_box(ob, 0.0)) for ob in self.obstacles)

    def _move(self, dt):
        """Integrate one axis at a time so the drone slides along obstacles instead of through them."""
        blocked = set()
        if self.mode in (FlightMode.LANDING, FlightMode.LANDED):
            z_low = LANDED_HEIGHT
        else:
            z_low = min(MIN_FLIGHT_HEIGHT, self.pos[2])    # never forces a low drone upwards
        limits = ((-WORLD_LIMIT, WORLD_LIMIT), (-WORLD_LIMIT, WORLD_LIMIT), (z_low, MAX_HEIGHT))

        for axis in range(3):
            wanted = self.pos[axis] + self.vel[axis] * dt
            low, high = limits[axis]
            value = clamp(wanted, low, high)
            if value != wanted:
                self.vel[axis] = 0.0
            trial = list(self.pos)
            trial[axis] = value
            if self._collides(trial) and not self._collides(self.pos):
                self.vel[axis] = 0.0
                self.collisions_prevented += 1
                blocked.add(axis)
                continue
            self.pos = trial
        return blocked

    def _update_mode(self, blocked):
        if self.mode is FlightMode.TAKEOFF:
            if self.pos[2] >= TAKEOFF_HEIGHT - 0.02:
                self.pos[2] = TAKEOFF_HEIGHT
                self._hover("TAKEOFF COMPLETE - HOVERING")
            elif 2 in blocked:
                self._hover("TAKEOFF BLOCKED - HOVERING")
        elif self.mode is FlightMode.LANDING:
            if self.pos[2] <= LANDED_HEIGHT + 0.01:
                self.pos[2] = LANDED_HEIGHT
                self.vel = [0.0, 0.0, 0.0]
                self.yaw_rate = 0.0
                self.mode = FlightMode.LANDED
                self.label = "LANDED"
                self.log(">> LANDING COMPLETE")
            elif 2 in blocked:
                self._hover("LANDING BLOCKED - HOVERING")
        elif self.mode is FlightMode.FLYING and self.label == "STOP":
            if math.hypot(*self.vel) < 0.02 and abs(self.yaw_rate) < 0.01:
                self.label = "HOVER"

    def _hover(self, message):
        self.vel[2] = 0.0
        self.cmd = [0.0, 0.0, 0.0]
        self.mode = FlightMode.FLYING
        self.label = "HOVER"
        self.log(">> " + message)

    def snapshot(self):
        return {
            "command": self.label,
            "pos": tuple(self.pos),
            "vel": tuple(self.vel),
            "yaw_deg": math.degrees(self.yaw),
        }


# =========================================================
# PYBULLET SCENE
# =========================================================

def _write_png(path, width, height, rows):
    """Minimal RGB PNG writer (no Pillow needed). `rows` holds one bytes object of width*3 per row."""
    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + row for row in rows)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def make_ground_texture(path=GROUND_TEXTURE, size=512):
    """Write the floor texture: PyBullet's floor repeats one tile every 2 m, so the tile is a soft
    two-tone 1 m checker with fine 1 m grid lines and a stronger line on the tile edge (every 2 m)."""
    half = size // 2
    light, dark = (206, 215, 226), (193, 203, 215)
    mid_line, edge_line = (160, 173, 190), (122, 138, 160)

    def mix(a, b, t):
        return tuple(round(x + (y - x) * t) for x, y in zip(a, b))

    edge = [clamp(1.0 - min(v, size - v) / 3.0, 0.0, 1.0) for v in range(size)]   # tile edge
    mid = [clamp(1.0 - abs(v - half) / 2.0, 0.0, 1.0) for v in range(size)]       # 1 m lines
    rows = []
    for y in range(size):
        row = bytearray()
        for x in range(size):
            colour = light if ((x >= half) + (y >= half)) % 2 == 0 else dark
            colour = mix(colour, mid_line, max(mid[x], mid[y]))
            colour = mix(colour, edge_line, max(edge[x], edge[y]))
            row.extend(colour)
        rows.append(bytes(row))
    _write_png(path, size, size, rows)


PILLAR_COLOR = (0.62, 0.16, 0.10)
PILLAR_ALERT = (1.00, 0.10, 0.05)
HAZARD_YELLOW = [0.98, 0.80, 0.05, 1.0]


def _lerp_color(a, b, t):
    return [x + (y - x) * t for x, y in zip(a, b)] + [1.0]


def _matte(body):
    """Remove specular highlights: under a strong sun they bleach colours to white and put a glare
    spot on the floor."""
    for link in range(-1, p.getNumJoints(body)):
        p.changeVisualShape(body, link, specularColor=[0.0, 0.0, 0.0])


class World:
    """Lighting, textured floor, the home pad and the obstacle pillars."""

    def __init__(self, obstacles, home=(0.0, 0.0)):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(DT)
        p.setRealTimeSimulation(False)

        floor = p.loadURDF("plane.urdf")
        if not os.path.exists(GROUND_TEXTURE):
            make_ground_texture()
        p.changeVisualShape(floor, -1, textureUniqueId=p.loadTexture(GROUND_TEXTURE),
                            rgbaColor=[1, 1, 1, 1])
        _matte(floor)

        self._pillars = []                  # [obstacle, body id, last glow level]
        for ob in obstacles:
            pillar = self._add_pillar(ob)
            _matte(pillar)
            self._pillars.append([ob, pillar, None])
        _matte(self._add_pad(home))

    @staticmethod
    def _add_pillar(ob):
        hx, hy, hz = ob.half
        body = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(ob.half))
        shaft = p.createVisualShape(p.GEOM_BOX, halfExtents=list(ob.half),
                                    rgbaColor=list(PILLAR_COLOR) + [1.0])
        band = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx + 0.012, hy + 0.012, 0.12],
                                   rgbaColor=HAZARD_YELLOW)
        cap = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx + 0.06, hy + 0.06, 0.05],
                                  rgbaColor=HAZARD_YELLOW)
        beacon = p.createVisualShape(p.GEOM_SPHERE, radius=0.13, rgbaColor=[1.0, 0.25, 0.1, 1.0])
        offsets = [[0, 0, -hz + 0.45], [0, 0, hz - 0.65], [0, 0, hz + 0.05], [0, 0, hz + 0.18]]
        return p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=body, baseVisualShapeIndex=shaft,
            basePosition=list(ob.center),
            linkMasses=[0] * 4, linkCollisionShapeIndices=[-1] * 4,
            linkVisualShapeIndices=[band, band, cap, beacon],
            linkPositions=offsets, linkOrientations=[[0, 0, 0, 1]] * 4,
            linkInertialFramePositions=[[0, 0, 0]] * 4,
            linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
            linkParentIndices=[0] * 4, linkJointTypes=[p.JOINT_FIXED] * 4,
            linkJointAxis=[[0, 0, 1]] * 4)

    @staticmethod
    def _add_pad(home):
        """Round landing pad with an H, just thick enough (0.01 m) for the skids to rest on."""
        disc = p.createVisualShape(p.GEOM_CYLINDER, radius=1.1, length=0.010,
                                   rgbaColor=[0.16, 0.18, 0.22, 1])
        ring = p.createVisualShape(p.GEOM_CYLINDER, radius=1.2, length=0.006,
                                   rgbaColor=[0.95, 0.95, 0.95, 1])
        stroke = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.45, 0.07, 0.0015],
                                     rgbaColor=[0.95, 0.95, 0.95, 1])
        bar = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.07, 0.30, 0.0015],
                                  rgbaColor=[0.95, 0.95, 0.95, 1])
        return p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=disc,
            basePosition=[home[0], home[1], 0.005],
            linkMasses=[0] * 4, linkCollisionShapeIndices=[-1] * 4,
            linkVisualShapeIndices=[ring, stroke, stroke, bar],
            linkPositions=[[0, 0, -0.002], [0, -0.30, 0.0065], [0, 0.30, 0.0065], [0, 0, 0.0065]],
            linkOrientations=[[0, 0, 0, 1]] * 4,
            linkInertialFramePositions=[[0, 0, 0]] * 4,
            linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
            linkParentIndices=[0] * 4, linkJointTypes=[p.JOINT_FIXED] * 4,
            linkJointAxis=[[0, 0, 1]] * 4)

    def update(self, controller, clock):
        """Make pillars glow as the drone closes in, and pulse the one that is holding it back."""
        blocker = controller.avoider.obstacle_name if controller.label == "OBSTACLE STOP" else None
        for entry in self._pillars:
            ob, body, last = entry
            low, high = inflated_box(ob, SAFETY_DISTANCE)
            gap = math.sqrt(sum(max(a - x, 0.0, x - b) ** 2
                                for x, a, b in zip(controller.pos, low, high)))
            level = clamp(1.0 - gap / ALERT_RANGE, 0.0, 1.0)
            if ob.name == blocker:
                level = 0.65 + 0.35 * (0.5 + 0.5 * math.sin(clock * 10.0))
            level = round(level * 20.0) / 20.0
            if level != last:
                p.changeVisualShape(body, -1, rgbaColor=_lerp_color(PILLAR_COLOR, PILLAR_ALERT, level))
                entry[2] = level


class DroneModel:
    """The drone as ONE multibody, so one reset moves every part at the same instant."""

    # (joint name, spin direction): diagonal rotors turn the same way, like a real quad
    PROPS = (("prop_fl_joint", 1.0), ("prop_fr_joint", -1.0),
             ("prop_rl_joint", -1.0), ("prop_rr_joint", 1.0))

    CORNERS = ("fl", "fr", "rl", "rr")
    # Navigation lights: red on the left, green on the right, white at the back.
    RED, RED_DIM = (1.0, 0.05, 0.05, 1.0), (0.35, 0.02, 0.02, 1.0)
    GREEN, GREEN_DIM = (0.1, 1.0, 0.2, 1.0), (0.02, 0.30, 0.05, 1.0)
    WHITE, WHITE_DIM = (1.0, 1.0, 1.0, 1.0), (0.35, 0.35, 0.35, 1.0)

    def __init__(self, controller):
        quat = p.getQuaternionFromEuler([0.0, 0.0, controller.yaw])
        self.body = p.loadURDF(URDF_PATH, controller.pos, quat, useFixedBase=True)
        joints, links = {}, {}
        for j in range(p.getNumJoints(self.body)):
            info = p.getJointInfo(self.body, j)
            joints[info[1].decode()] = j
            links[info[12].decode()] = j                # link index == index of the joint that carries it
            p.setJointMotorControl2(self.body, j, p.VELOCITY_CONTROL, force=0)
        _matte(self.body)
        self.props = [(joints[name], sign) for name, sign in self.PROPS]
        self.discs = [links["disc_" + tag] for tag in self.CORNERS]
        self.lights = {tag: links["light_" + tag] for tag in self.CORNERS}
        self._disc_alpha = None
        self._light_colors = {}
        self.clock = 0.0
        self.spin = 0.0
        self.angle = 0.0
        self.roll = 0.0
        self.pitch = 0.0

    def _update_effects(self, controller):
        # Rotor discs fade in as the rotors spin up (the blades themselves blur into a disc).
        alpha = round(clamp(self.spin / PROP_SPIN, 0.0, 1.0) * DISC_ALPHA, 2)
        if alpha != self._disc_alpha:
            self._disc_alpha = alpha
            for link in self.discs:
                p.changeVisualShape(self.body, link, rgbaColor=[0.85, 0.90, 0.95, alpha])

        airborne = controller.mode is not FlightMode.LANDED
        if controller.label == "OBSTACLE STOP":
            rear = self.RED if (self.clock * 4.0) % 1.0 < 0.5 else self.RED_DIM      # warning flash
        elif airborne and (self.clock * 1.6) % 1.0 < 0.12:
            rear = self.WHITE                                                      # strobe pulse
        else:
            rear = self.WHITE_DIM
        colors = {"fl": self.RED if airborne else self.RED_DIM,
                  "fr": self.GREEN if airborne else self.GREEN_DIM,
                  "rl": rear, "rr": rear}
        for tag, colour in colors.items():
            if self._light_colors.get(tag) != colour:
                self._light_colors[tag] = colour
                p.changeVisualShape(self.body, self.lights[tag], rgbaColor=list(colour))

    def update(self, controller, dt):
        self.clock += dt
        target = 0.0 if controller.mode is FlightMode.LANDED else PROP_SPIN
        self.spin = approach(self.spin, target, 60.0 * dt)
        self.angle = (self.angle + self.spin * dt) % (2.0 * math.pi)
        self._update_effects(controller)

        # Lean into the direction of travel like a real quadcopter (visual only).
        c, s = math.cos(controller.yaw), math.sin(controller.yaw)
        v_forward = controller.vel[0] * c + controller.vel[1] * s
        v_left = -controller.vel[0] * s + controller.vel[1] * c
        k = 1.0 - math.exp(-8.0 * dt)
        # +pitch tips the nose down; +roll lifts the left side, so leaning left needs -roll.
        self.pitch += (clamp(TILT_PER_MPS * v_forward, -MAX_TILT, MAX_TILT) - self.pitch) * k
        self.roll += (clamp(-TILT_PER_MPS * v_left, -MAX_TILT, MAX_TILT) - self.roll) * k

        p.resetBasePositionAndOrientation(
            self.body, controller.pos,
            p.getQuaternionFromEuler([self.roll, self.pitch, controller.yaw]))
        for joint, sign in self.props:
            p.resetJointState(self.body, joint, sign * self.angle)


class CameraRig:
    """Follow camera (keeps whatever angle/zoom you set with the mouse) or a fixed observer."""

    def __init__(self, mode="follow"):
        self.mode = mode
        self.target = [0.0, 0.0, 1.0]
        if mode == "fixed":
            p.resetDebugVisualizerCamera(14.0, CAMERA_YAW, -35.0, [3.0, 0.0, 1.0])
        else:
            p.resetDebugVisualizerCamera(CAMERA_DISTANCE, CAMERA_YAW, CAMERA_PITCH, self.target)

    def toggle(self):
        self.mode = "fixed" if self.mode == "follow" else "follow"
        return self.mode

    def update(self, pos, dt):
        if self.mode != "follow":
            return
        info = p.getDebugVisualizerCamera()
        yaw, pitch, distance = info[8], info[9], info[10]
        k = 1.0 - math.exp(-6.0 * dt)       # low-pass the target so the view does not jitter
        self.target = [t + (q - t) * k for t, q in zip(self.target, pos)]
        p.resetDebugVisualizerCamera(distance, yaw, pitch, self.target)


class Overlays:
    """Drawing on top of the scene.

    Always on: a marker on the floor under the drone joined to it by a thin line (so height is easy
    to judge), and a sensor ray that shows what the drone sees ahead, green (clear) to red (close).
    Toggleable: a flight trail that fades out and is coloured by speed (blue slow, orange fast),
    and the obstacles' safety zones.
    """

    ZONE_COLOR = [0.1, 0.8, 0.3]
    TRAIL_SLOW, TRAIL_FAST = (0.25, 0.65, 1.0), (1.0, 0.55, 0.1)
    RAY_FAR, RAY_MID, RAY_NEAR = (0.2, 1.0, 0.3), (1.0, 0.9, 0.1), (1.0, 0.1, 0.1)
    MARKER, MARKER_ALERT = [0.2, 0.9, 1.0, 0.55], [1.0, 0.2, 0.15, 0.7]

    def __init__(self, obstacles, trail=True, zones=False):
        self.obstacles = tuple(obstacles)
        self.trail = trail
        self._zone_ids = []
        self._last_point = None
        self._last_trail = 0.0
        self._last_lines = 0.0
        self._line_ids = {"drop": -1, "ray": -1}
        shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.45, length=0.004, rgbaColor=self.MARKER)
        self._marker = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                                         baseVisualShapeIndex=shape, basePosition=[0, 0, 0.016])
        self._marker_alert = False
        if zones:
            self.toggle_zones()

    def toggle_trail(self):
        self.trail = not self.trail
        self._last_point = None
        return self.trail

    def toggle_zones(self):
        """Returns True if the zones are now shown."""
        if self._zone_ids:
            for item in self._zone_ids:
                p.removeUserDebugItem(item)
            self._zone_ids = []
            return False
        for ob in self.obstacles:
            low, high = inflated_box(ob, SAFETY_DISTANCE)
            corners = [(x, y, z) for x in (low[0], high[0]) for y in (low[1], high[1])
                       for z in (low[2], high[2])]
            for a in corners:
                for b in corners:
                    # an edge joins corners that differ in exactly one coordinate (draw each once)
                    if a < b and sum(u != v for u, v in zip(a, b)) == 1:
                        self._zone_ids.append(p.addUserDebugLine(a, b, self.ZONE_COLOR, lineWidth=1))
        return bool(self._zone_ids)

    def _line(self, key, start, end, colour, width):
        """Draw (or move) one persistent line. Identical end points hide it."""
        replace = self._line_ids[key]
        kwargs = {"replaceItemUniqueId": replace} if replace >= 0 else {}
        self._line_ids[key] = p.addUserDebugLine(start, end, list(colour), lineWidth=width, **kwargs)

    def _ray_colour(self, t):
        """t = 1 far ... 0 touching the safety zone."""
        if t > 0.5:
            return _lerp_color(self.RAY_MID, self.RAY_FAR, (t - 0.5) * 2.0)[:3]
        return _lerp_color(self.RAY_NEAR, self.RAY_MID, t * 2.0)[:3]

    def update(self, controller, now):
        x, y, z = controller.pos
        p.resetBasePositionAndOrientation(self._marker, [x, y, 0.016], [0, 0, 0, 1])
        alert = controller.label == "OBSTACLE STOP"
        if alert != self._marker_alert:
            self._marker_alert = alert
            p.changeVisualShape(self._marker, -1, rgbaColor=self.MARKER_ALERT if alert else self.MARKER)

        if now - self._last_lines >= 0.05:                   # 20 Hz is plenty for thin lines
            self._last_lines = now
            bottom = z - 0.30                                # underside of the skids
            if bottom > 0.08:
                self._line("drop", [x, y, bottom], [x, y, 0.02], (0.55, 0.8, 1.0), 1)
            else:
                self._line("drop", [x, y, 0.02], [x, y, 0.02], (0, 0, 0), 1)

            sensor = controller.avoider.sensor if controller.avoider is not None else None
            if sensor is not None and sensor[1] is not None and controller.mode is FlightMode.FLYING:
                direction, distance = sensor
                end = [c + d * distance for c, d in zip(controller.pos, direction)]
                self._line("ray", list(controller.pos), end,
                           self._ray_colour(clamp(distance / DETECTION_DISTANCE, 0.0, 1.0)), 3)
            else:
                self._line("ray", list(controller.pos), list(controller.pos), (0, 0, 0), 1)

        if not self.trail or now - self._last_trail < 0.1:
            return
        self._last_trail = now
        point = list(controller.pos)
        if self._last_point is None or math.dist(point, self._last_point) > 0.03:
            if self._last_point is not None:
                speed = clamp(math.sqrt(sum(v * v for v in controller.vel)) / HORIZONTAL_SPEED, 0.0, 1.0)
                p.addUserDebugLine(self._last_point, point,
                                   _lerp_color(self.TRAIL_SLOW, self.TRAIL_FAST, speed)[:3],
                                   lineWidth=3, lifeTime=TRAIL_SECONDS)
            self._last_point = point


class Hud:
    """Status text floating above the drone, so you can watch the window instead of the console."""

    def __init__(self):
        self.item = None
        self._last = 0.0

    @staticmethod
    def _colour(controller):
        """Red = held at an obstacle, cyan = taking off / landing, green = hovering, white = parked,
        yellow = on the move."""
        if controller.label == "OBSTACLE STOP":
            return [1.0, 0.25, 0.2]
        if controller.mode in (FlightMode.TAKEOFF, FlightMode.LANDING):
            return [0.3, 0.9, 1.0]
        if controller.mode is FlightMode.LANDED:
            return [1.0, 1.0, 1.0]
        if controller.label in ("HOVER", "STOP"):
            return [0.4, 1.0, 0.5]
        return [1.0, 0.9, 0.2]

    def update(self, controller, heard, now):
        if now - self._last < 0.1:
            return
        self._last = now
        text = "%s | z %.1f m | %.1f m/s | hdg %d deg" % (
            controller.label, controller.pos[2], math.hypot(*controller.vel),
            round(math.degrees(controller.yaw)) % 360)
        if heard:
            text += " | heard: " + heard
        where = [controller.pos[0], controller.pos[1], controller.pos[2] + 0.9]
        options = dict(textColorRGB=self._colour(controller), textSize=1.6)
        if self.item is None:
            self.item = p.addUserDebugText(text, where, **options)
        else:
            p.addUserDebugText(text, where, replaceItemUniqueId=self.item, **options)


# =========================================================
# INPUT: KEYBOARD, VOICE, CONSOLE
# =========================================================

def _keys(chars):
    codes = {}
    for ch in chars:
        codes[ord(ch.lower())] = ch
        codes[ord(ch.upper())] = ch
    return codes


HELD_KEYS = {
    **{code: Intent.FORWARD for code in _keys("w")},
    **{code: Intent.BACKWARD for code in _keys("s")},
    **{code: Intent.LEFT for code in _keys("a")},
    **{code: Intent.RIGHT for code in _keys("d")},
    **{code: Intent.UP for code in _keys("q")},
    **{code: Intent.DOWN for code in _keys("e")},
    p.B3G_LEFT_ARROW: Intent.ROTATE_LEFT,
    p.B3G_RIGHT_ARROW: Intent.ROTATE_RIGHT,
}
TRIGGER_KEYS = {
    **{code: Intent.TAKEOFF for code in _keys("t")},
    **{code: Intent.LAND for code in _keys("l")},
    **{code: Intent.CAMERA for code in _keys("c")},
    **{code: Intent.HOME for code in _keys("h")},
    **{code: Intent.TRAIL for code in _keys("p")},
    **{code: Intent.ZONES for code in _keys("z")},
    **{code: Intent.EXIT for code in _keys("x")},
    ord(" "): Intent.STOP,
}


class KeyboardInput:
    """Held keys steer the drone; releasing every key makes it hover. With no key activity
    it does nothing, so a voice command keeps running."""

    def __init__(self):
        self.active = False

    def poll(self, controller):
        """Apply held keys to the controller; return one-shot intents (take off, land, ...)."""
        events = p.getKeyboardEvents()
        triggered = [TRIGGER_KEYS[k] for k, state in events.items()
                     if state & p.KEY_WAS_TRIGGERED and k in TRIGGER_KEYS]
        held = [HELD_KEYS[k] for k, state in events.items()
                if state & p.KEY_IS_DOWN and k in HELD_KEYS]

        if held:
            axes = [sum(MOTION[i][n] for i in held) for n in range(4)]
            axes[0] = clamp(axes[0], -HORIZONTAL_SPEED, HORIZONTAL_SPEED)
            axes[1] = clamp(axes[1], -HORIZONTAL_SPEED, HORIZONTAL_SPEED)
            controller.set_manual(*axes, label="+".join(i.value for i in held))
            self.active = True
        elif self.active:
            self.active = False
            controller.stop()
        return triggered


class VoiceListener(threading.Thread):
    """Background microphone loop: recognised phrases go onto a queue for the main loop."""

    def __init__(self, out, stop, log=print):
        super().__init__(daemon=True)
        self.out, self.stop_event, self.log = out, stop, log

    def run(self):
        try:
            import speech_recognition as sr
        except ImportError:
            self.log("Voice control unavailable: SpeechRecognition is not installed "
                     "(pip install SpeechRecognition). Keyboard control still works.")
            return

        recognizer = sr.Recognizer()
        recognizer.pause_threshold = 0.6
        try:
            microphone = sr.Microphone()
        except Exception as error:                      # no mic, or PyAudio missing
            self.log("Voice control unavailable: %s. Keyboard control still works "
                     "(or use --console to type commands)." % error)
            return

        try:
            with microphone as source:                  # open the stream once, not per phrase
                self.log("Calibrating microphone - stay quiet for a second...")
                recognizer.adjust_for_ambient_noise(source, duration=1)
                self.log("VOICE CONTROL READY - say: take off, forward, backward, left, right, up, "
                         "down, rotate left, rotate right, stop, go home, land, exit "
                         "(add a distance or angle: 'forward 3 meters', 'rotate right 90 degrees')")
                last_error = 0.0
                while not self.stop_event.is_set():
                    try:
                        audio = recognizer.listen(source, timeout=2, phrase_time_limit=4)
                    except sr.WaitTimeoutError:
                        continue
                    try:
                        text = recognizer.recognize_google(audio)
                    except sr.UnknownValueError:
                        continue
                    except sr.RequestError as error:
                        if time.monotonic() - last_error > 10.0:
                            self.log("Speech service unreachable (needs internet): %s" % error)
                            last_error = time.monotonic()
                        continue
                    self.out.put(("voice", text))
        except Exception as error:
            self.log("Voice input stopped: %s" % error)


class ConsoleListener(threading.Thread):
    """Type commands into the terminal instead of speaking them."""

    def __init__(self, out, stop):
        super().__init__(daemon=True)
        self.out, self.stop_event = out, stop

    def run(self):
        print("Type commands here and press Enter (e.g. take off, forward, stop, land, exit).")
        while not self.stop_event.is_set():
            try:
                line = input()
            except (EOFError, OSError):
                return
            if line.strip():
                self.out.put(("console", line))


# =========================================================
# TELEMETRY
# =========================================================

class TelemetryLogger:
    """Appends flight data to a CSV: one row per command plus one per second while airborne."""

    FIELDS = ["Time", "Voice_Input", "Command", "Velocity_X", "Velocity_Y", "Velocity_Z",
              "Drone_X", "Drone_Y", "Drone_Z", "Yaw"]

    def __init__(self, path=TELEMETRY_PATH, interval=1.0):
        self.interval = interval
        self._last = 0.0
        needs_header = not os.path.exists(path) or os.path.getsize(path) == 0
        if not needs_header:
            with open(path, "rb") as existing:
                existing.seek(-1, os.SEEK_END)
                needs_newline = existing.read(1) not in (b"\n", b"\r")
        self._file = open(path, "a", newline="", encoding="utf-8")
        if not needs_header and needs_newline:
            self._file.write("\n")
        self._writer = csv.writer(self._file)
        if needs_header:
            self._writer.writerow(self.FIELDS)

    def log(self, controller, heard=""):
        snap = controller.snapshot()
        self._writer.writerow(
            [datetime.now().strftime("%Y-%m-%d %H:%M:%S"), heard, snap["command"]]
            + ["%.3f" % (v + 0.0) for v in snap["vel"]]        # + 0.0 turns -0.0 into 0.0
            + ["%.3f" % (v + 0.0) for v in snap["pos"]]
            + ["%.1f" % snap["yaw_deg"]])
        self._file.flush()
        self._last = time.monotonic()

    def tick(self, controller):
        if (controller.mode is not FlightMode.LANDED
                and time.monotonic() - self._last >= self.interval):
            self.log(controller)

    def close(self):
        self._file.close()


# =========================================================
# MAIN LOOP
# =========================================================

def connect_gui():
    """Open the PyBullet window and set it up for flying. Returns the client id (< 0 on failure)."""
    client = p.connect(p.GUI, options="--width=1280 --height=800")
    if client < 0:
        return client
    # Without this, W/A/S/D/C/G... toggle PyBullet's own debug views while you fly.
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # Sharp shadows are what make height readable; the sky colour replaces the default grey.
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, lightPosition=LIGHT_POSITION,
                               shadowMapResolution=4096, shadowMapWorldSize=40,
                               rgbBackground=SKY_COLOR)
    return client


def run(voice=True, console=False, obstacles=True, camera="follow",
        log_path=TELEMETRY_PATH, duration=None, trail=True, show_zones=False,
        on_obstacle="stop"):
    client = connect_gui()
    if client < 0:
        print("ERROR: could not connect to the PyBullet GUI. Close any old PyBullet windows and retry.")
        return 1

    zones = DEFAULT_OBSTACLES if obstacles else ()
    controller = DroneController(obstacles=zones, on_obstacle=on_obstacle)
    world = World(zones, controller.home)
    model = DroneModel(controller)
    rig = CameraRig(camera)
    hud = Hud()
    overlays = Overlays(zones, trail, show_zones)
    keyboard = KeyboardInput()

    logger = None
    if log_path:
        try:
            logger = TelemetryLogger(log_path)
        except OSError as error:
            print("Telemetry logging disabled (%s)." % error)

    commands = queue.Queue()
    stop = threading.Event()
    if voice:
        VoiceListener(commands, stop).start()
    if console:
        ConsoleListener(commands, stop).start()

    print("\n======================================")
    print("   VOICE CONTROLLED DRONE - PYBULLET")
    print("======================================")
    if zones:
        print("Obstacle detection: ON - the drone will %s (%d obstacles)" % (
            "STOP before an obstacle" if on_obstacle == "stop" else "STEER AROUND obstacles",
            len(zones)))
        for ob in zones:
            print("  %s at x=%.1f y=%.1f" % (ob.name, ob.center[0], ob.center[1]))
    print("Keys (click the PyBullet window first): T take off, L land, H go home, W/A/S/D move,")
    print("      Q/E up/down, arrows rotate, SPACE stop, C camera, P trail, Z safety zones, X exit")
    print("Voice/typed commands accept a distance or angle: 'forward 3 meters', 'rotate left 90 degrees'.")
    print("The drone starts on the ground: take off first.\n")

    heard = ""
    start = last = time.perf_counter()
    try:
        while not controller.exit_requested and p.isConnected():
            now = time.perf_counter()
            dt = min(now - last, 0.05)
            last = now
            if duration is not None and now - start >= duration:
                break

            requests = [(intent, None, "") for intent in keyboard.poll(controller)]
            for source, text in _drain(commands):
                intent, amount = parse_request(text)
                print("\nYOU SAID:" if source == "voice" else "\nYOU TYPED:", text)
                if intent is None:
                    print(">> COMMAND NOT RECOGNIZED")
                    continue
                heard = text
                requests.append((intent, amount, text))
            for intent, amount, said in requests:
                if intent is Intent.CAMERA:
                    print(">> CAMERA:", rig.toggle().upper())
                elif intent is Intent.TRAIL:
                    print(">> TRAIL:", "ON" if overlays.toggle_trail() else "OFF")
                elif intent is Intent.ZONES:
                    if zones:
                        print(">> SAFETY ZONES:", "SHOWN" if overlays.toggle_zones() else "HIDDEN")
                    else:
                        print("No obstacles in this mode (try obstacles.py or --obstacles).")
                else:
                    controller.apply_intent(intent, amount)
                if logger:
                    logger.log(controller, said)

            controller.step(dt)
            model.update(controller, dt)
            rig.update(controller.pos, dt)
            world.update(controller, model.clock)
            overlays.update(controller, now)
            hud.update(controller, heard, now)
            if logger:
                logger.tick(controller)
            p.stepSimulation()

            time.sleep(max(0.0, DT - (time.perf_counter() - now)))
    except p.error:
        pass                                            # the window was closed mid-frame
    finally:
        stop.set()
        if logger:
            logger.close()
        if p.isConnected():
            p.disconnect()
    print("\nDrone simulation closed.")
    return 0


def _drain(q):
    items = []
    while True:
        try:
            items.append(q.get_nowait())
        except queue.Empty:
            return items


def main(description, voice, obstacles):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--voice", action=argparse.BooleanOptionalAction, default=voice,
                        help="listen to the microphone (default: %s)" % ("on" if voice else "off"))
    parser.add_argument("--obstacles", action=argparse.BooleanOptionalAction, default=obstacles,
                        help="spawn obstacles and detect them (default: %s)"
                        % ("on" if obstacles else "off"))
    parser.add_argument("--on-obstacle", choices=ObstacleAvoider.STRATEGIES, default="stop",
                        help="what the drone does when an obstacle is in its way: 'stop' before it "
                        "(default) or 'steer' around it")
    parser.add_argument("--console", action="store_true",
                        help="also accept typed commands in this terminal")
    parser.add_argument("--camera", choices=("follow", "fixed"), default="follow")
    parser.add_argument("--no-trail", action="store_true", help="start with the flight trail hidden")
    parser.add_argument("--zones", action="store_true",
                        help="start with the obstacle safety zones drawn")
    parser.add_argument("--telemetry-file", default=TELEMETRY_PATH,
                        help="CSV to append flight data to (default: drone_data.csv)")
    parser.add_argument("--no-log", action="store_true", help="do not write telemetry")
    parser.add_argument("--duration", type=float, default=None, metavar="SECONDS",
                        help="exit automatically after this many seconds")
    args = parser.parse_args()
    raise SystemExit(run(
        voice=args.voice, console=args.console, obstacles=args.obstacles, camera=args.camera,
        log_path=None if args.no_log else args.telemetry_file, duration=args.duration,
        trail=not args.no_trail, show_zones=args.zones, on_obstacle=args.on_obstacle))
