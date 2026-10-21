"""Headless tests: python -m unittest -v   (no window, microphone or network needed)."""

import csv
import math
import os
import queue
import struct
import sys
import tempfile
import threading
import unittest
import zlib
from unittest import mock
from xml.etree import ElementTree

import pybullet as p

import drone_core as dc
from drone_core import DroneController, FlightMode, Intent, parse_command

DT = 1.0 / 60.0


def simulate(controller, seconds, dt=DT):
    for _ in range(int(seconds / dt)):
        controller.step(dt)


def quiet_controller(obstacles=(), **kwargs):
    messages = []
    controller = DroneController(obstacles=obstacles, log=messages.append, **kwargs)
    controller.messages = messages
    return controller


def airborne(obstacles=(), **kwargs):
    controller = quiet_controller(obstacles, **kwargs)
    controller.apply_intent(Intent.TAKEOFF)
    simulate(controller, 5.0)
    assert controller.mode is FlightMode.FLYING
    return controller


def inside_any_obstacle(controller):
    return any(dc.inside_box(controller.pos, *dc.inflated_box(ob, 0.0))
               for ob in controller.obstacles)


class ParseCommandTests(unittest.TestCase):
    def check(self, phrases, intent):
        for phrase in phrases:
            with self.subTest(phrase=phrase):
                self.assertIs(parse_command(phrase), intent)

    def test_takeoff(self):
        self.check(["take off", "Take off.", "takeoff", "take-off", "lift off", "take"], Intent.TAKEOFF)

    def test_land(self):
        self.check(["land", "please land now", "landing"], Intent.LAND)

    def test_translation(self):
        self.check(["forward", "go forward", "move forwards"], Intent.FORWARD)
        self.check(["backward", "go back", "back", "reverse"], Intent.BACKWARD)
        # The old exact-match code ignored all of these:
        self.check(["left", "go left", "move to the left"], Intent.LEFT)
        self.check(["right", "go right", "write"], Intent.RIGHT)
        self.check(["up", "go up", "rise", "climb"], Intent.UP)
        self.check(["down", "go down", "descend"], Intent.DOWN)

    def test_rotation_is_not_translation(self):
        self.check(["rotate left", "turn left", "turn to the left",
                    "rotate the drone to the left"], Intent.ROTATE_LEFT)
        self.check(["rotate right", "turn right", "spin right"], Intent.ROTATE_RIGHT)

    def test_stop_and_exit(self):
        self.check(["stop", "halt", "freeze", "hover"], Intent.STOP)
        self.check(["exit", "quit"], Intent.EXIT)

    def test_whole_words_only(self):
        for phrase in ["island", "hello there", "", "rotate", "go", "bottom", "uphill"]:
            with self.subTest(phrase=phrase):
                self.assertIsNone(parse_command(phrase))


class ParseRequestTests(unittest.TestCase):
    def check(self, phrase, intent, amount):
        with self.subTest(phrase=phrase):
            got_intent, got_amount = dc.parse_request(phrase)
            self.assertIs(got_intent, intent)
            if amount is None:
                self.assertIsNone(got_amount)
            else:
                self.assertAlmostEqual(got_amount, amount, places=3)

    def test_distances(self):
        self.check("forward 3 meters", Intent.FORWARD, 3.0)
        self.check("go forward three meters", Intent.FORWARD, 3.0)
        self.check("move left 2.5 metres", Intent.LEFT, 2.5)
        self.check("back 4m", Intent.BACKWARD, 4.0)
        self.check("up 2", Intent.UP, 2.0)
        self.check("go down twenty five meters", Intent.DOWN, 25.0)
        self.check("up ten feet", Intent.UP, 3.048)

    def test_angles(self):
        self.check("rotate left 90 degrees", Intent.ROTATE_LEFT, 90.0)
        self.check("turn right ninety degrees", Intent.ROTATE_RIGHT, 90.0)
        self.check("turn to the right one hundred eighty", Intent.ROTATE_RIGHT, 180.0)
        self.check("spin left 100000 degrees", Intent.ROTATE_LEFT, dc.MAX_TURN_DEG)

    def test_no_amount_means_continuous(self):
        self.check("forward", Intent.FORWARD, None)
        self.check("rotate left", Intent.ROTATE_LEFT, None)
        self.check("land", Intent.LAND, None)
        self.check("forward 0 meters", Intent.FORWARD, None)

    def test_distance_is_capped(self):
        self.check("forward 1000 meters", Intent.FORWARD, dc.MAX_MOVE)

    def test_home_beats_back(self):
        for phrase in ["go home", "return home", "go back home", "return to home", "home"]:
            with self.subTest(phrase=phrase):
                self.assertIs(parse_command(phrase), Intent.HOME)

    def test_overlay_words(self):
        self.assertIs(parse_command("show safety zones"), Intent.ZONES)
        self.assertIs(parse_command("trail"), Intent.TRAIL)


class MeasuredMoveTests(unittest.TestCase):
    def test_moves_exactly_the_requested_distance_then_hovers(self):
        c = airborne()
        c.apply_intent(Intent.FORWARD, 3.0)
        simulate(c, 8.0)
        self.assertAlmostEqual(c.pos[0], 3.0, delta=0.05)
        self.assertAlmostEqual(c.pos[1], 0.0, delta=0.01)
        self.assertEqual(c.label, "HOVER")
        self.assertIsNone(c.maneuver)
        self.assertTrue(any("MOVE COMPLETE" in m for m in c.messages))

    def test_distance_follows_the_heading(self):
        c = airborne()
        c.yaw = math.pi / 2                                 # nose along +Y, so left is -X
        c.apply_intent(Intent.LEFT, 2.0)
        simulate(c, 8.0)
        self.assertAlmostEqual(c.pos[0], -2.0, delta=0.05)
        self.assertAlmostEqual(c.pos[1], 0.0, delta=0.01)

    def test_vertical_move_and_liftoff_from_the_ground(self):
        c = quiet_controller()
        c.apply_intent(Intent.UP, 2.0)                      # allowed on the ground: it is a lift-off
        simulate(c, 8.0)
        self.assertAlmostEqual(c.pos[2], dc.LANDED_HEIGHT + 2.0, delta=0.05)
        self.assertIs(c.mode, FlightMode.FLYING)

    def test_refused_on_the_ground_unless_straight_up(self):
        c = quiet_controller()
        self.assertFalse(c.apply_intent(Intent.FORWARD, 3.0))
        self.assertFalse(c.apply_intent(Intent.ROTATE_LEFT, 90.0))
        self.assertIsNone(c.maneuver)

    def test_measured_turn(self):
        for intent, expected in ((Intent.ROTATE_LEFT, math.radians(90)),
                                 (Intent.ROTATE_RIGHT, -math.radians(90))):
            with self.subTest(intent=intent):
                c = airborne()
                c.apply_intent(intent, 90.0)
                simulate(c, 6.0)
                self.assertAlmostEqual(c.yaw, expected, delta=math.radians(1.5))
                self.assertEqual((round(c.pos[0], 3), round(c.pos[1], 3)), (0.0, 0.0))
                self.assertIsNone(c.maneuver)

    def test_stop_and_manual_commands_cancel_a_measured_move(self):
        c = airborne()
        c.apply_intent(Intent.FORWARD, 10.0)
        simulate(c, 2.0)
        c.apply_intent(Intent.STOP)
        simulate(c, 2.0)
        self.assertLess(c.pos[0], 4.0)
        self.assertIsNone(c.maneuver)

        c = airborne()
        c.apply_intent(Intent.FORWARD, 10.0)
        simulate(c, 1.0)
        c.apply_intent(Intent.LEFT)                         # a new continuous command wins
        self.assertIsNone(c.maneuver)

    def test_gives_up_when_pinned_against_the_world_limit(self):
        c = airborne()
        c.pos[0] = dc.WORLD_LIMIT - 1.0
        c.apply_intent(Intent.FORWARD, 20.0)
        simulate(c, 6.0)
        self.assertEqual(c.pos[0], dc.WORLD_LIMIT)
        self.assertIsNone(c.maneuver)
        self.assertTrue(any("BLOCKED" in m for m in c.messages))

    def test_measured_move_through_obstacles_still_gets_there_safely_when_steering(self):
        c = airborne(dc.DEFAULT_OBSTACLES, on_obstacle="steer")
        c.apply_intent(Intent.FORWARD, 11.0)
        for _ in range(int(60.0 / DT)):
            c.step(DT)
            self.assertFalse(inside_any_obstacle(c))
            if c.maneuver is None:
                break
        self.assertTrue(any("MOVE COMPLETE" in m for m in c.messages), c.messages)
        self.assertGreater(c.pos[0], 9.5)


class ReturnHomeTests(unittest.TestCase):
    def test_returns_over_the_start_point_and_lands(self):
        c = airborne(start=(1.0, -2.0))
        c.apply_intent(Intent.FORWARD, 6.0)
        simulate(c, 8.0)
        c.apply_intent(Intent.LEFT, 4.0)
        simulate(c, 8.0)
        self.assertGreater(math.hypot(c.pos[0] - 1.0, c.pos[1] + 2.0), 5.0)
        c.apply_intent(Intent.HOME)
        simulate(c, 25.0)
        self.assertIs(c.mode, FlightMode.LANDED)
        self.assertAlmostEqual(c.pos[0], 1.0, delta=0.15)
        self.assertAlmostEqual(c.pos[1], -2.0, delta=0.15)
        self.assertTrue(any("RETURN HOME COMPLETE" in m for m in c.messages))

    def test_returns_home_through_the_obstacle_field_when_steering(self):
        c = airborne(dc.DEFAULT_OBSTACLES, on_obstacle="steer")
        c.apply_intent(Intent.FORWARD)
        simulate(c, 25.0)                                   # runs the gauntlet out to the wall
        self.assertEqual(c.pos[0], dc.WORLD_LIMIT)
        c.apply_intent(Intent.HOME)
        for _ in range(int(60.0 / DT)):
            c.step(DT)
            self.assertFalse(inside_any_obstacle(c))
            if c.mode is FlightMode.LANDED:
                break
        self.assertIs(c.mode, FlightMode.LANDED)
        self.assertLess(math.hypot(c.pos[0], c.pos[1]), 0.15)
        self.assertEqual(c.collisions_prevented, 0)

    def test_refused_on_the_ground_and_lands_straight_away_when_already_above_home(self):
        c = quiet_controller()
        self.assertFalse(c.apply_intent(Intent.HOME))
        c = airborne()
        self.assertTrue(c.apply_intent(Intent.HOME))
        self.assertIs(c.mode, FlightMode.LANDING)

    def test_stop_cancels_the_return(self):
        c = airborne()
        c.apply_intent(Intent.FORWARD, 8.0)
        simulate(c, 8.0)
        c.apply_intent(Intent.HOME)
        simulate(c, 2.0)
        c.apply_intent(Intent.STOP)
        simulate(c, 1.0)                                    # let it finish coasting
        x = c.pos[0]
        simulate(c, 3.0)
        self.assertIs(c.mode, FlightMode.FLYING)
        self.assertAlmostEqual(c.pos[0], x, delta=0.001)    # no longer heading home
        self.assertGreater(c.pos[0], 1.0)


class FlightModeTests(unittest.TestCase):
    def test_starts_on_the_ground_and_refuses_horizontal_commands(self):
        c = quiet_controller()
        self.assertIs(c.mode, FlightMode.LANDED)
        self.assertFalse(c.apply_intent(Intent.FORWARD))
        simulate(c, 2.0)
        self.assertEqual(c.pos, [0.0, 0.0, dc.LANDED_HEIGHT])

    def test_takeoff_reaches_cruise_height_and_hovers(self):
        c = airborne()
        self.assertAlmostEqual(c.pos[2], dc.TAKEOFF_HEIGHT, places=2)
        self.assertEqual(c.label, "HOVER")
        self.assertLess(max(abs(v) for v in c.vel), 1e-6)

    def test_takeoff_never_snaps_a_higher_drone_back_down(self):
        c = airborne()
        c.apply_intent(Intent.UP)
        simulate(c, 3.0)
        c.apply_intent(Intent.STOP)
        simulate(c, 1.0)                                    # let it finish coasting
        height = c.pos[2]
        self.assertGreater(height, dc.TAKEOFF_HEIGHT + 1)
        self.assertFalse(c.apply_intent(Intent.TAKEOFF))
        simulate(c, 1.0)
        self.assertAlmostEqual(c.pos[2], height, delta=0.01)

    def test_up_from_the_ground_lifts_off_without_sliding_sideways(self):
        c = quiet_controller()
        self.assertTrue(c.set_manual(1.0, 1.0, 1.0, 0.5, "TEST"))
        simulate(c, 1.0)
        self.assertGreater(c.pos[2], dc.LANDED_HEIGHT)
        self.assertEqual((c.pos[0], c.pos[1]), (0.0, 0.0))

    def test_land_touches_down_on_the_ground_not_half_a_metre_above_it(self):
        c = airborne()
        c.apply_intent(Intent.LAND)
        simulate(c, 6.0)
        self.assertIs(c.mode, FlightMode.LANDED)
        self.assertEqual(c.pos[2], dc.LANDED_HEIGHT)
        self.assertEqual(c.vel, [0.0, 0.0, 0.0])
        self.assertFalse(c.apply_intent(Intent.LEFT))      # and cannot taxi along the ground

    def test_down_stops_at_minimum_flight_height(self):
        c = airborne()
        c.apply_intent(Intent.DOWN)
        simulate(c, 6.0)
        self.assertAlmostEqual(c.pos[2], dc.MIN_FLIGHT_HEIGHT, places=2)
        self.assertIs(c.mode, FlightMode.FLYING)

    def test_manual_command_cancels_landing(self):
        c = airborne()
        c.apply_intent(Intent.LAND)
        simulate(c, 0.5)
        c.apply_intent(Intent.FORWARD)
        self.assertIs(c.mode, FlightMode.FLYING)

    def test_stop_brings_the_drone_to_rest_and_hover(self):
        c = airborne()
        c.apply_intent(Intent.FORWARD)
        simulate(c, 2.0)
        self.assertGreater(c.vel[0], 1.0)
        c.apply_intent(Intent.STOP)
        simulate(c, 1.5)
        self.assertLess(math.hypot(*c.vel), 1e-6)
        self.assertEqual(c.label, "HOVER")

    def test_motion_is_relative_to_heading(self):
        c = airborne()
        c.yaw = math.pi / 2                                 # nose along +Y
        c.apply_intent(Intent.FORWARD)
        simulate(c, 2.0)
        self.assertGreater(c.pos[1], 1.0)
        self.assertAlmostEqual(c.pos[0], 0.0, places=3)

    def test_rotation_wraps_and_matches_commanded_direction(self):
        c = airborne()
        c.apply_intent(Intent.ROTATE_LEFT)
        simulate(c, 6.0)                                    # 45 deg/s -> wraps past 180 deg
        self.assertTrue(-math.pi <= c.yaw <= math.pi)
        c.apply_intent(Intent.STOP)
        simulate(c, 1.0)
        c.yaw = 0.0
        c.apply_intent(Intent.ROTATE_RIGHT)
        simulate(c, 1.0)
        self.assertLess(c.yaw, 0.0)

    def test_world_limits(self):
        c = airborne()
        c.apply_intent(Intent.FORWARD)
        simulate(c, 20.0)
        self.assertEqual(c.pos[0], dc.WORLD_LIMIT)
        c.apply_intent(Intent.UP)
        simulate(c, 10.0)
        self.assertEqual(c.pos[2], dc.MAX_HEIGHT)

    def test_exit_flag(self):
        c = quiet_controller()
        c.apply_intent(Intent.EXIT)
        self.assertTrue(c.exit_requested)


class SteerAroundTests(unittest.TestCase):
    """The optional on_obstacle="steer" strategy: fly around obstacles and carry on."""

    OBSTACLES = dc.DEFAULT_OBSTACLES

    def fly(self, intent, seconds, yaw=0.0, start=(0.0, 0.0)):
        c = airborne(self.OBSTACLES, start=start, on_obstacle="steer")
        c.yaw = yaw
        c.apply_intent(intent)
        for _ in range(int(seconds / DT)):
            c.step(DT)
            self.assertFalse(inside_any_obstacle(c), "drone entered an obstacle at %s" % c.pos)
        return c

    def test_flies_around_obstacles_and_resumes_the_original_course(self):
        c = self.fly(Intent.FORWARD, 25.0)
        self.assertEqual(c.pos[0], dc.WORLD_LIMIT)          # got past all three obstacles
        self.assertEqual(c.collisions_prevented, 0)         # avoidance did the work, not the hard stop
        self.assertTrue(any("MOVING LEFT" in m for m in c.messages))
        self.assertTrue(any("RESUMING" in m for m in c.messages))
        self.assertEqual(c.vel[1], 0.0)                     # no leftover sideways drift (old bug)

    def test_avoidance_follows_the_direction_of_travel_not_the_heading(self):
        c = self.fly(Intent.BACKWARD, 25.0, yaw=math.pi)    # nose points away, travel is still +X
        self.assertEqual(c.pos[0], dc.WORLD_LIMIT)
        self.assertEqual(c.collisions_prevented, 0)

    def test_sideways_flight_into_open_space_triggers_nothing(self):
        c = self.fly(Intent.RIGHT, 12.0)
        self.assertEqual(c.pos[1], -dc.WORLD_LIMIT)
        self.assertFalse(any("OBSTACLE" in m for m in c.messages))

    def test_climbs_over_when_both_sides_are_walled_in(self):
        wall = tuple(dc.Obstacle("W%d" % i, (4.0, y, 1.5), (0.7, 0.7, 1.5))
                     for i, y in enumerate(range(-12, 13, 2)))
        c = airborne(wall, on_obstacle="steer")
        c.apply_intent(Intent.FORWARD)
        simulate(c, 30.0)
        self.assertTrue(any("CLIMBING OVER" in m for m in c.messages))
        self.assertGreater(c.pos[0], 6.0)
        self.assertEqual(c.collisions_prevented, 0)

    def test_stops_when_there_is_no_way_through(self):
        tall = tuple(dc.Obstacle("T%d" % i, (4.0, y, 4.5), (0.7, 0.7, 4.5))
                     for i, y in enumerate(range(-12, 13, 2)))
        c = airborne(tall, on_obstacle="steer")
        c.apply_intent(Intent.FORWARD)
        simulate(c, 20.0)
        self.assertTrue(any("NO SAFE ROUTE" in m for m in c.messages))
        self.assertLess(c.pos[0], 4.0)
        self.assertLess(math.hypot(*c.vel), 1e-6)

    def test_hard_stop_prevents_penetration_even_without_the_avoider(self):
        c = airborne(self.OBSTACLES)
        c.avoider = None
        c.apply_intent(Intent.FORWARD)
        for _ in range(int(10.0 / DT)):
            c.step(DT)
            self.assertFalse(inside_any_obstacle(c))
        self.assertGreater(c.collisions_prevented, 0)

    def test_hard_stop_also_covers_descending_onto_an_obstacle(self):
        c = airborne(self.OBSTACLES, start=(-10.0, -10.0), on_obstacle="steer")
        c.pos = [3.5, 0.0, 6.0]                             # high above Obstacle_1 (top at z = 3)
        c.apply_intent(Intent.DOWN)
        for _ in range(int(8.0 / DT)):
            c.step(DT)
            self.assertFalse(inside_any_obstacle(c))
        top = 3.0 + dc.DRONE_HALF_HEIGHT
        self.assertGreaterEqual(c.pos[2], top)
        self.assertLess(c.pos[2], top + 0.1)                # came down to the obstacle and stopped there


def zone_face_x(obstacle):
    """x of the near face of an obstacle's safety zone, seen from a drone approaching from -X."""
    return dc.inflated_box(obstacle, dc.SAFETY_DISTANCE)[0][0]


def in_safety_zone(controller):
    return any(dc.inside_box(controller.pos, *dc.inflated_box(ob, dc.SAFETY_DISTANCE))
               for ob in controller.obstacles)


class StopAtObstacleTests(unittest.TestCase):
    """The default strategy: brake smoothly and hold still in front of the obstacle."""

    OBSTACLES = dc.DEFAULT_OBSTACLES

    def approach(self, intent=Intent.FORWARD, seconds=12.0, **kwargs):
        c = airborne(self.OBSTACLES, **kwargs)
        c.apply_intent(intent)
        for _ in range(int(seconds / DT)):
            c.step(DT)
            self.assertFalse(in_safety_zone(c), "drone entered the safety zone at %s" % c.pos)
        return c

    def test_default_strategy_is_stop(self):
        self.assertEqual(airborne(self.OBSTACLES).avoider.strategy, "stop")

    def test_stops_in_front_of_the_obstacle_and_stays_there(self):
        c = self.approach()
        expected = zone_face_x(self.OBSTACLES[0]) - dc.STOP_BUFFER
        self.assertAlmostEqual(c.pos[0], expected, delta=0.05)
        self.assertEqual(c.pos[1], 0.0)                     # went straight, did not dodge
        self.assertLess(math.hypot(*c.vel), 1e-6)
        self.assertEqual(c.label, "OBSTACLE STOP")
        self.assertEqual(c.cmd, [0.0, 0.0, 0.0])            # the command was dropped
        self.assertEqual(c.collisions_prevented, 0)         # braked by itself, not the hard stop
        self.assertTrue(any("OBSTACLE DETECTED: Obstacle_1" in m for m in c.messages))
        self.assertEqual(sum("STOPPED" in m for m in c.messages), 1)

    def test_brakes_gradually_instead_of_slamming_to_a_halt(self):
        c = airborne(self.OBSTACLES)
        c.apply_intent(Intent.FORWARD)
        peak_deceleration = 0.0
        previous = 0.0
        for _ in range(int(12.0 / DT)):
            c.step(DT)
            peak_deceleration = max(peak_deceleration, (previous - c.vel[0]) / DT)
            previous = c.vel[0]
        self.assertLessEqual(peak_deceleration, dc.ACCELERATION + 1e-6)

    def test_pressing_forward_again_does_not_creep_closer(self):
        c = self.approach()
        x = c.pos[0]
        for _ in range(3):
            c.apply_intent(Intent.FORWARD)
            simulate(c, 2.0)
        self.assertAlmostEqual(c.pos[0], x, delta=0.01)
        self.assertEqual(c.label, "OBSTACLE STOP")

    def test_can_back_away_and_sidestep_after_stopping(self):
        c = self.approach()
        x = c.pos[0]
        c.apply_intent(Intent.BACKWARD)
        simulate(c, 2.0)
        self.assertLess(c.pos[0], x - 1.0)
        c = self.approach()
        c.apply_intent(Intent.RIGHT)                        # open space to the right of Obstacle_1
        simulate(c, 2.0)
        self.assertLess(c.pos[1], -1.0)

    def test_a_held_forward_command_does_not_spam_the_console(self):
        c = airborne(self.OBSTACLES)
        for _ in range(int(10.0 / DT)):
            c.set_manual(dc.HORIZONTAL_SPEED, 0.0, 0.0, 0.0, "FORWARD")   # a held W key, every frame
            c.step(DT)
        self.assertEqual(sum("STOPPED" in m for m in c.messages), 1)
        self.assertEqual(sum("DETECTED" in m for m in c.messages), 1)

    def test_a_stopped_drone_does_not_restart_after_turning_to_face_clear_space(self):
        c = self.approach()
        c.apply_intent(Intent.ROTATE_LEFT, 90.0)
        simulate(c, 6.0)
        self.assertAlmostEqual(c.yaw, math.pi / 2, delta=math.radians(2))
        x, y = c.pos[0], c.pos[1]
        simulate(c, 3.0)
        self.assertEqual((c.pos[0], c.pos[1]), (x, y))      # still hovering, no stale "forward"

    def test_measured_move_ends_when_the_obstacle_stops_it(self):
        c = airborne(self.OBSTACLES)
        c.apply_intent(Intent.FORWARD, 10.0)
        simulate(c, 12.0)
        self.assertIsNone(c.maneuver)
        self.assertTrue(any("MOVE STOPPED - OBSTACLE AHEAD" in m for m in c.messages))
        self.assertFalse(any("MOVE COMPLETE" in m for m in c.messages))
        self.assertAlmostEqual(c.pos[0], zone_face_x(self.OBSTACLES[0]) - dc.STOP_BUFFER, delta=0.05)

    def test_measured_move_that_stops_short_of_the_obstacle_completes_normally(self):
        c = airborne(self.OBSTACLES)
        c.apply_intent(Intent.FORWARD, 0.5)
        simulate(c, 4.0)
        self.assertTrue(any("MOVE COMPLETE" in m for m in c.messages))
        self.assertFalse(any("STOPPED" in m for m in c.messages))    # it saw the obstacle but never needed to brake
        self.assertEqual(c.label, "HOVER")

    def test_return_home_is_stopped_by_an_obstacle_and_does_not_land_on_it(self):
        c = airborne(self.OBSTACLES, start=(0.0, 0.0))
        c.pos = [10.0, 0.0, 2.0]                            # outside every zone; Obstacle_2 sits between here and home
        c.apply_intent(Intent.HOME)
        for _ in range(int(20.0 / DT)):
            c.step(DT)
            self.assertFalse(in_safety_zone(c))
        self.assertIs(c.mode, FlightMode.FLYING)             # did not carry on and land on top of it
        self.assertTrue(any("RETURN HOME STOPPED - OBSTACLE AHEAD" in m for m in c.messages))
        self.assertEqual(c.label, "OBSTACLE STOP")
        self.assertAlmostEqual(c.pos[0], dc.inflated_box(self.OBSTACLES[1], dc.SAFETY_DISTANCE)[1][0]
                               + dc.STOP_BUFFER, delta=0.05)

    def test_descending_onto_an_obstacle_brakes_above_it(self):
        c = airborne(self.OBSTACLES, start=(-10.0, -10.0))
        c.pos = [3.5, 0.0, 6.0]
        c.apply_intent(Intent.DOWN)
        for _ in range(int(10.0 / DT)):
            c.step(DT)
            self.assertFalse(in_safety_zone(c))
        zone_top = 3.0 + dc.DRONE_HALF_HEIGHT + dc.SAFETY_DISTANCE
        self.assertAlmostEqual(c.pos[2], zone_top + dc.STOP_BUFFER, delta=0.05)
        self.assertEqual(c.collisions_prevented, 0)

    def test_passing_beside_an_obstacle_is_not_mistaken_for_a_collision(self):
        c = airborne(self.OBSTACLES, start=(0.0, 3.5))      # y = 3.5 is clear of Obstacle_1's zone (|y| < 2.27)
        c.pos = [0.0, 2.5, 2.0]
        c.apply_intent(Intent.FORWARD, 2.0)
        simulate(c, 5.0)
        self.assertAlmostEqual(c.pos[0], 2.0, delta=0.05)
        self.assertFalse(any("DETECTED" in m for m in c.messages))

    def test_escapes_from_inside_a_zone_only_by_moving_away(self):
        c = airborne(self.OBSTACLES)
        c.pos = [zone_face_x(self.OBSTACLES[0]) + 0.1, 0.0, 2.0]    # just inside Obstacle_1's zone
        x = c.pos[0]
        c.apply_intent(Intent.FORWARD)
        simulate(c, 2.0)
        self.assertEqual(c.pos[0], x)                       # not allowed any deeper
        c.apply_intent(Intent.BACKWARD)
        simulate(c, 2.0)
        self.assertLess(c.pos[0], x - 1.0)                  # allowed out

    def test_unknown_strategy_is_rejected(self):
        with self.assertRaises(ValueError):
            dc.ObstacleAvoider(self.OBSTACLES, strategy="ram")


class UrdfFileTests(unittest.TestCase):
    def test_urdf_is_strictly_well_formed_xml(self):
        # PyBullet's parser forgives things (e.g. "--" inside comments) that ROS and other tools reject.
        ElementTree.parse(dc.URDF_PATH)


class ModelTests(unittest.TestCase):
    def setUp(self):
        self.client = p.connect(p.DIRECT)

    def tearDown(self):
        p.disconnect(self.client)

    def test_urdf_loads_cleanly_and_moves_as_one_body(self):
        controller = quiet_controller()
        controller.pos = [1.0, 2.0, 3.0]
        controller.yaw = math.pi / 2
        model = dc.DroneModel(controller)
        self.assertEqual(len(model.props), 4)
        model.update(controller, DT)

        base, _ = p.getBasePositionAndOrientation(model.body)
        self.assertEqual([round(v, 3) for v in base], [1.0, 2.0, 3.0])

        # Front marker sits 0.45 m ahead of the body centre: with yaw=90 deg that is +Y.
        names = {p.getJointInfo(model.body, j)[12].decode(): j
                 for j in range(p.getNumJoints(model.body))}
        link = p.getLinkState(model.body, names["front_marker"])[4]
        self.assertAlmostEqual(link[0], 1.0, places=2)
        self.assertAlmostEqual(link[1], 2.45, places=2)

    def test_props_spin_in_alternating_directions_only_when_flying(self):
        controller = quiet_controller()
        model = dc.DroneModel(controller)
        model.update(controller, DT)
        self.assertEqual(model.spin, 0.0)                   # landed: rotors still

        controller.apply_intent(Intent.TAKEOFF)
        for _ in range(60):
            model.update(controller, DT)
        self.assertGreater(model.spin, 0.0)
        angles = [p.getJointState(model.body, j)[0] for j, _ in model.props]
        self.assertNotEqual(angles[0], 0.0)
        self.assertAlmostEqual(angles[0], -angles[1], places=6)
        self.assertAlmostEqual(angles[0], angles[3], places=6)


class KeyboardTests(unittest.TestCase):
    DOWN = p.KEY_IS_DOWN
    TRIGGERED = p.KEY_WAS_TRIGGERED

    def poll(self, keyboard, controller, events):
        with mock.patch.object(p, "getKeyboardEvents", return_value=events):
            return keyboard.poll(controller)

    def test_held_keys_steer_and_release_hovers(self):
        c = airborne()
        kb = dc.KeyboardInput()
        self.poll(kb, c, {ord("w"): self.DOWN, ord("d"): self.DOWN})
        simulate(c, 1.0)
        self.assertGreater(c.vel[0], 1.0)                   # forward ...
        self.assertLess(c.vel[1], -1.0)                     # ... and right, together
        self.assertLessEqual(math.hypot(c.cmd[0], c.cmd[1]), dc.HORIZONTAL_SPEED * math.sqrt(2) + 1e-9)
        self.poll(kb, c, {})
        simulate(c, 2.0)
        self.assertLess(math.hypot(*c.vel), 1e-6)

    def test_opposing_keys_cancel(self):
        c = airborne()
        self.poll(dc.KeyboardInput(), c, {ord("w"): self.DOWN, ord("s"): self.DOWN})
        self.assertEqual(c.cmd[0], 0.0)

    def test_no_key_activity_leaves_a_voice_command_running(self):
        c = airborne()
        kb = dc.KeyboardInput()
        c.apply_intent(Intent.FORWARD)
        for _ in range(30):
            self.poll(kb, c, {})
        self.assertEqual(c.cmd[0], dc.HORIZONTAL_SPEED)

    def test_one_shot_keys_return_intents(self):
        c = quiet_controller()
        triggered = self.poll(dc.KeyboardInput(), c, {ord("t"): self.TRIGGERED, ord("C"): self.TRIGGERED})
        self.assertEqual(set(triggered), {Intent.TAKEOFF, Intent.CAMERA})
        # holding a one-shot key (no TRIGGERED flag) must not repeat it
        self.assertEqual(self.poll(dc.KeyboardInput(), c, {ord("t"): self.DOWN}), [])

    def test_holding_a_key_on_the_ground_does_not_spam_the_console(self):
        c = quiet_controller()
        kb = dc.KeyboardInput()
        for _ in range(120):
            self.poll(kb, c, {ord("w"): self.DOWN})
        self.assertEqual(len(c.messages), 1)
        self.assertEqual(c.pos, [0.0, 0.0, dc.LANDED_HEIGHT])


class FakeSpeechRecognition:
    """Stands in for the speech_recognition package: replays a scripted sequence of outcomes."""

    class WaitTimeoutError(Exception):
        pass

    class UnknownValueError(Exception):
        pass

    class RequestError(Exception):
        pass

    def __init__(self, script, stop_event, microphone_error=None):
        fake = self
        self.script = list(script)
        self.recognised = []

        class Recognizer:
            pause_threshold = 0

            def adjust_for_ambient_noise(self, source, duration=1):
                pass

            def listen(self, source, timeout=None, phrase_time_limit=None):
                if not fake.script:
                    stop_event.set()
                    raise fake.WaitTimeoutError()
                step = fake.script.pop(0)
                if step == "timeout":
                    raise fake.WaitTimeoutError()
                return step

            def recognize_google(self, audio):
                if audio == "mumble":
                    raise fake.UnknownValueError()
                if audio == "offline":
                    raise fake.RequestError("no internet")
                return audio

        class Microphone:
            def __init__(self):
                if microphone_error:
                    raise microphone_error

            def __enter__(self):
                return self

            def __exit__(self, *exc):
                return False

        self.Recognizer, self.Microphone = Recognizer, Microphone


class VoiceListenerTests(unittest.TestCase):
    def run_listener(self, script, microphone_error=None):
        stop = threading.Event()
        fake = FakeSpeechRecognition(script, stop, microphone_error)
        out, logs = queue.Queue(), []
        with mock.patch.dict(sys.modules, {"speech_recognition": fake}):
            listener = dc.VoiceListener(out, stop, log=logs.append)
            listener.start()
            listener.join(timeout=5)
        self.assertFalse(listener.is_alive())
        items = []
        while not out.empty():
            items.append(out.get_nowait())
        return items, logs

    def test_phrases_are_queued_and_recognition_errors_do_not_kill_the_loop(self):
        items, logs = self.run_listener(
            ["timeout", "mumble", "take off", "offline", "offline", "go forward"])
        self.assertEqual(items, [("voice", "take off"), ("voice", "go forward")])
        self.assertEqual(sum("unreachable" in line for line in logs), 1)   # warned once, not per phrase

    def test_missing_microphone_is_reported_not_raised(self):
        items, logs = self.run_listener([], microphone_error=OSError("No Default Input Device Available"))
        self.assertEqual(items, [])
        self.assertTrue(any("No Default Input Device" in line for line in logs))

    def test_missing_speech_recognition_package_is_reported_not_raised(self):
        stop = threading.Event()
        logs = []
        with mock.patch.dict(sys.modules, {"speech_recognition": None}):   # makes `import` fail
            listener = dc.VoiceListener(queue.Queue(), stop, log=logs.append)
            listener.start()
            listener.join(timeout=5)
        self.assertTrue(any("not installed" in line for line in logs))


class TiltAndOverlayTests(unittest.TestCase):
    def setUp(self):
        self.client = p.connect(p.DIRECT)

    def tearDown(self):
        p.disconnect(self.client)

    def nose_height_over_body(self, model):
        names = {p.getJointInfo(model.body, j)[12].decode(): j
                 for j in range(p.getNumJoints(model.body))}
        nose = p.getLinkState(model.body, names["front_marker"])[4]
        base = p.getBasePositionAndOrientation(model.body)[0]
        return nose[2] - base[2]

    def test_drone_leans_into_its_motion_and_levels_out_when_stopped(self):
        c = airborne()
        model = dc.DroneModel(c)
        level = self.nose_height_over_body(model)             # 0.10 m above the body centre
        c.apply_intent(Intent.FORWARD)
        for _ in range(120):
            c.step(DT)
            model.update(c, DT)
        self.assertGreater(model.pitch, math.radians(8))      # nose down ...
        self.assertLess(model.pitch, dc.MAX_TILT + 1e-9)
        self.assertLess(self.nose_height_over_body(model), level - 0.05)
        self.assertAlmostEqual(model.roll, 0.0, places=3)

        c.apply_intent(Intent.STOP)
        for _ in range(180):
            c.step(DT)
            model.update(c, DT)
        self.assertLess(abs(model.pitch), math.radians(0.5))

    def test_leans_towards_the_side_it_is_flying(self):
        c = airborne()
        model = dc.DroneModel(c)
        c.apply_intent(Intent.LEFT)
        for _ in range(120):
            c.step(DT)
            model.update(c, DT)
        self.assertLess(model.roll, -math.radians(8))         # left side down

    def test_tilt_is_relative_to_heading(self):
        c = airborne()
        c.yaw = math.pi / 2
        model = dc.DroneModel(c)
        c.apply_intent(Intent.FORWARD)                        # flying along +Y
        for _ in range(120):
            c.step(DT)
            model.update(c, DT)
        self.assertGreater(model.pitch, math.radians(8))
        self.assertAlmostEqual(model.roll, 0.0, places=3)

    def test_overlays_toggle(self):
        overlays = dc.Overlays(dc.DEFAULT_OBSTACLES, trail=True, zones=False)
        self.assertTrue(overlays.toggle_zones())
        self.assertEqual(len(overlays._zone_ids), 12 * len(dc.DEFAULT_OBSTACLES))   # 12 edges per box
        self.assertFalse(overlays.toggle_zones())
        self.assertEqual(overlays._zone_ids, [])
        self.assertFalse(overlays.toggle_trail())
        overlays.update(quiet_controller(), 1.0)               # nothing to draw while the trail is off


class SceneTests(unittest.TestCase):
    """The visual layer, checked through PyBullet's headless mode by reading colours back."""

    def setUp(self):
        self.client = p.connect(p.DIRECT)

    def tearDown(self):
        p.disconnect(self.client)

    @staticmethod
    def rgba(body, link):
        return next(v[7] for v in p.getVisualShapeData(body) if v[1] == link)

    def assert_rgba(self, actual, expected):
        """PyBullet keeps colours as float32, so compare with a tolerance."""
        self.assertEqual(len(actual), len(expected))
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=5)

    def test_ground_texture_is_a_valid_png_tile(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "nested", "ground.png")
            dc.make_ground_texture(path, size=64)
            with open(path, "rb") as f:
                data = f.read()
        self.assertEqual(data[:8], b"\x89PNG\r\n\x1a\n")
        width, height = struct.unpack(">II", data[16:24])
        self.assertEqual((width, height), (64, 64))
        idat = data.index(b"IDAT")
        length = struct.unpack(">I", data[idat - 4:idat])[0]
        self.assertEqual(len(zlib.decompress(data[idat + 4:idat + 4 + length])), 64 * (1 + 64 * 3))

    def test_shipped_ground_texture_loads_in_pybullet(self):
        dc.World((), (0.0, 0.0))                             # creates assets/ground.png if it is missing
        self.assertTrue(os.path.exists(dc.GROUND_TEXTURE))
        self.assertGreaterEqual(p.loadTexture(dc.GROUND_TEXTURE), 0)

    def test_world_contains_floor_pad_and_every_pillar(self):
        dc.World(dc.DEFAULT_OBSTACLES, (0.0, 0.0))
        self.assertEqual(p.getNumBodies(), 1 + len(dc.DEFAULT_OBSTACLES) + 1)   # floor, pillars, pad

    def test_pillars_keep_their_collision_shape_under_the_decoration(self):
        world = dc.World(dc.DEFAULT_OBSTACLES, (0.0, 0.0))
        body = world._pillars[0][1]
        shapes = p.getCollisionShapeData(body, -1)
        self.assertEqual(len(shapes), 1)                     # bands, cap and beacon are visual-only
        full_size = tuple(2 * h for h in dc.DEFAULT_OBSTACLES[0].half)       # PyBullet reports full extents
        for got, want in zip(shapes[0][3], full_size):
            self.assertAlmostEqual(got, want, places=5)

    def test_pillars_glow_as_the_drone_approaches_and_pulse_when_one_holds_it_back(self):
        world = dc.World(dc.DEFAULT_OBSTACLES, (0.0, 0.0))
        c = quiet_controller(dc.DEFAULT_OBSTACLES)
        shaft = world._pillars[0][1]

        c.pos = [-10.0, 0.0, 2.0]
        world.update(c, 0.0)
        far = self.rgba(shaft, -1)
        self.assertAlmostEqual(far[0], dc.PILLAR_COLOR[0], places=2)

        c.pos = [0.0, 0.0, 2.0]                              # 1.2 m from Obstacle_1's zone
        world.update(c, 0.0)
        near = self.rgba(shaft, -1)
        self.assertGreater(near[0], far[0] + 0.1)

        c.label, c.avoider.obstacle_name = "OBSTACLE STOP", "Obstacle_1"
        seen = set()
        for step in range(40):
            world.update(c, step * 0.05)
            seen.add(round(self.rgba(shaft, -1)[0], 2))
        self.assertGreater(len(seen), 3)                     # pulsing, not a fixed colour

    def test_rotor_discs_fade_in_with_rotor_speed(self):
        c = quiet_controller()
        model = dc.DroneModel(c)
        model.update(c, DT)
        self.assertEqual(self.rgba(model.body, model.discs[0])[3], 0.0)       # parked: no disc

        c.apply_intent(Intent.TAKEOFF)
        for _ in range(240):
            c.step(DT)
            model.update(c, DT)
        self.assertAlmostEqual(self.rgba(model.body, model.discs[0])[3], dc.DISC_ALPHA, places=2)

        c.apply_intent(Intent.LAND)
        for _ in range(60 * 8):
            c.step(DT)
            model.update(c, DT)
        self.assertEqual(self.rgba(model.body, model.discs[0])[3], 0.0)       # spun down again

    def test_navigation_lights_follow_the_flight_state(self):
        c = quiet_controller()
        model = dc.DroneModel(c)
        model.update(c, DT)
        self.assertEqual(model._light_colors["fl"], model.RED_DIM)             # parked: dim
        self.assertEqual(model._light_colors["fr"], model.GREEN_DIM)

        air = airborne()
        model = dc.DroneModel(air)
        model.update(air, DT)
        self.assertEqual(model._light_colors["fl"], model.RED)                 # red left, green right
        self.assertEqual(model._light_colors["fr"], model.GREEN)
        self.assert_rgba(self.rgba(model.body, model.lights["fl"]), model.RED)

        rear = set()                                                           # white strobe pulses
        for _ in range(240):
            model.update(air, DT)
            rear.add(model._light_colors["rl"])
        self.assertEqual(rear, {model.WHITE, model.WHITE_DIM})

        air.label = "OBSTACLE STOP"                                            # warning: rear flashes red
        rear = set()
        for _ in range(240):
            model.update(air, DT)
            rear.add(model._light_colors["rl"])
        self.assertEqual(rear, {model.RED, model.RED_DIM})

    def test_ground_marker_follows_the_drone_and_turns_red_on_an_obstacle_stop(self):
        overlays = dc.Overlays(dc.DEFAULT_OBSTACLES, trail=False)
        c = airborne(dc.DEFAULT_OBSTACLES)
        c.pos = [-4.0, 3.0, 2.0]
        overlays.update(c, 1.0)
        x, y, z = p.getBasePositionAndOrientation(overlays._marker)[0]
        self.assertEqual((round(x, 3), round(y, 3)), (-4.0, 3.0))
        self.assertLess(z, 0.05)                                               # on the floor, not in the air
        self.assert_rgba(self.rgba(overlays._marker, -1), overlays.MARKER)
        c.label = "OBSTACLE STOP"
        overlays.update(c, 2.0)
        self.assert_rgba(self.rgba(overlays._marker, -1), overlays.MARKER_ALERT)

    def test_status_text_colour_tells_the_state_at_a_glance(self):
        colour = dc.Hud._colour
        c = quiet_controller()
        self.assertEqual(colour(c), [1.0, 1.0, 1.0])                           # parked: white
        c.apply_intent(Intent.TAKEOFF)
        self.assertEqual(colour(c), [0.3, 0.9, 1.0])                           # taking off: cyan
        c = airborne()
        self.assertEqual(colour(c), [0.4, 1.0, 0.5])                           # hovering: green
        c.apply_intent(Intent.FORWARD)
        self.assertEqual(colour(c), [1.0, 0.9, 0.2])                           # moving: yellow
        c.label = "OBSTACLE STOP"
        self.assertEqual(colour(c), [1.0, 0.25, 0.2])                          # held at obstacle: red


class SensorTests(unittest.TestCase):
    def test_sensor_reports_the_distance_to_the_safety_zone_ahead(self):
        c = airborne(dc.DEFAULT_OBSTACLES)
        self.assertIsNone(c.avoider.sensor)                                    # hovering: nothing sensed
        c.apply_intent(Intent.FORWARD)
        simulate(c, 1.0)
        direction, distance = c.avoider.sensor
        self.assertEqual(direction, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(distance, zone_face_x(dc.DEFAULT_OBSTACLES[0]) - c.pos[0], delta=0.02)

    def test_sensor_is_empty_when_the_way_is_clear(self):
        c = airborne(dc.DEFAULT_OBSTACLES)
        c.apply_intent(Intent.RIGHT)                                           # open space
        simulate(c, 1.0)
        self.assertIsNone(c.avoider.sensor)

    def test_sensor_works_for_the_steer_strategy_too(self):
        c = airborne(dc.DEFAULT_OBSTACLES, on_obstacle="steer")
        c.apply_intent(Intent.FORWARD)
        simulate(c, 1.0)
        self.assertIsNotNone(c.avoider.sensor)


class TelemetryTests(unittest.TestCase):
    def test_header_rows_and_append(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "flight.csv")
            controller = airborne()
            logger = dc.TelemetryLogger(path)
            logger.log(controller, "take off")
            logger.close()
            logger = dc.TelemetryLogger(path)                # reopening must not repeat the header
            logger.log(controller, "forward")
            logger.close()
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 2)
        self.assertEqual(list(rows[0].keys()), dc.TelemetryLogger.FIELDS)
        self.assertEqual(rows[0]["Voice_Input"], "take off")
        self.assertEqual(rows[0]["Command"], "HOVER")
        self.assertAlmostEqual(float(rows[0]["Drone_Z"]), dc.TAKEOFF_HEIGHT, places=2)

    def test_appending_to_a_file_without_trailing_newline(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "flight.csv")
            with open(path, "w", newline="", encoding="utf-8") as f:
                f.write(",".join(dc.TelemetryLogger.FIELDS) + "\n1,a,b,0,0,0,0,0,0,0")
            logger = dc.TelemetryLogger(path)
            logger.log(quiet_controller(), "x")
            logger.close()
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 2)


if __name__ == "__main__":
    unittest.main()
