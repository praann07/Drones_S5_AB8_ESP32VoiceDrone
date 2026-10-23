"""Headless tests: python -m unittest -v   (no window, microphone or network needed)."""

import csv
import math
import os
import json
import queue
import socket
import struct
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
import zlib
from unittest import mock
from xml.etree import ElementTree

import pybullet as p
import pybullet_data

import drone_core as dc
import scene_assets as sa
import scene_city as sc
from dashboard import Dashboard
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

    def test_words_a_recogniser_splits_in_two(self):
        self.check(["for word", "four ward", "go for word", "fore word"], Intent.FORWARD)
        self.check(["back word", "go back ward"], Intent.BACKWARD)
        self.check(["take of", "lift off"], Intent.TAKEOFF)

    def test_common_mishearings(self):
        self.check(["foreword"], Intent.FORWARD)
        self.check(["stock"], Intent.STOP)
        self.check(["dawn"], Intent.DOWN)
        self.check(["lend"], Intent.LAND)

    def test_a_slip_of_a_letter_still_works(self):
        self.check(["forwad", "foward"], Intent.FORWARD)
        self.check(["backwad"], Intent.BACKWARD)
        self.check(["takeof"], Intent.TAKEOFF)
        self.check(["decend"], Intent.DOWN)
        self.check(["ascnd"], Intent.UP)
        self.check(["rotating left", "turning to the left", "spinning left"], Intent.ROTATE_LEFT)
        self.check(["rotating right", "turning right"], Intent.ROTATE_RIGHT)

    def test_repairs_do_not_invent_commands_out_of_other_speech(self):
        for phrase in ["wildlife", "sticker", "price", "hello", "sasaram", "yellow", "what a good",
                       "for this", "supply system work", "Kallu Reddy saree", "can I change to Bluetooth"]:
            with self.subTest(phrase=phrase):
                self.assertIsNone(parse_command(phrase))

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
        self.check("forwad 3 meters", Intent.FORWARD, 3.0)
        self.check("for word two meters", Intent.FORWARD, 2.0)

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
        self.languages = []
        self.gate = None

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

            def recognize_google(self, audio, language="en-US", show_all=False):
                fake.languages.append(language)
                if fake.gate is not None:
                    fake.gate.wait(5)                  # a slow network, until the test opens the gate
                if audio == "mumble":
                    raise fake.UnknownValueError()
                if audio == "offline":
                    raise fake.RequestError("no internet")
                guesses = list(audio) if isinstance(audio, tuple) else [audio]   # a tuple = ranked guesses
                if not show_all:
                    return guesses[0]
                return {"alternative": [{"transcript": g} for g in guesses], "final": True}

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
    def run_listener(self, script, microphone_error=None, **listener_options):
        stop = threading.Event()
        fake = self.fake = FakeSpeechRecognition(script, stop, microphone_error)
        out, logs = queue.Queue(), []
        with mock.patch.dict(sys.modules, {"speech_recognition": fake}):
            listener = dc.VoiceListener(out, stop, log=logs.append, **listener_options)
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

    def test_takes_the_first_guess_that_is_a_command(self):
        items, logs = self.run_listener([("for this", "forward", "four ward")])
        self.assertEqual(items, [("voice", "forward")])
        self.assertTrue(any('heard "for this"' in line and 'took "forward"' in line for line in logs))

    def test_a_command_in_the_top_spot_needs_no_explanation(self):
        items, logs = self.run_listener([("take off", "take of")])
        self.assertEqual(items, [("voice", "take off")])
        self.assertFalse(any("heard" in line and "took" in line for line in logs))

    def test_unrecognised_speech_is_passed_on_and_the_other_guesses_are_logged(self):
        items, logs = self.run_listener([("price 6 m", "prize 6 m", "rice 6 m")])
        self.assertEqual(items, [("voice", "price 6 m")])          # the main loop will say "not recognised"
        self.assertTrue(any("other guesses" in line and "prize 6 m" in line for line in logs))

    def test_only_the_top_three_guesses_are_considered(self):
        items, _ = self.run_listener([("a", "b", "c", "forward")])
        self.assertEqual(items, [("voice", "a")])

    def test_listens_in_indian_english_by_default_and_the_language_can_be_changed(self):
        self.run_listener(["stop"])
        self.assertEqual(self.fake.languages, ["en-IN"])
        self.run_listener(["stop"], language="en-US")
        self.assertEqual(self.fake.languages, ["en-US"])

    def test_the_microphone_keeps_listening_while_a_phrase_is_being_transcribed(self):
        stop = threading.Event()
        fake = FakeSpeechRecognition(["take off", "forward"], stop)
        fake.gate = threading.Event()                              # transcription blocks until we open this
        out = queue.Queue()
        with mock.patch.dict(sys.modules, {"speech_recognition": fake}):
            listener = dc.VoiceListener(out, stop, log=lambda line: None)
            listener.start()
            # `stop` is set when the listening loop has consumed both recordings. If listening waited
            # for the first transcription (the old behaviour) this would time out.
            self.assertTrue(stop.wait(timeout=5), "the microphone stalled behind the slow transcription")
            self.assertTrue(out.empty())                           # nothing transcribed yet...
            fake.gate.set()                                        # ...now the network answers
            listener.join(timeout=10)
        self.assertEqual([out.get_nowait() for _ in range(2)], [("voice", "take off"), ("voice", "forward")])

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
        dc.World((), (0.0, 0.0), "simple")                           # creates assets/ground.png if it is missing
        self.assertTrue(os.path.exists(dc.GROUND_TEXTURE))
        self.assertGreaterEqual(p.loadTexture(dc.GROUND_TEXTURE), 0)

    def test_world_contains_floor_pad_and_every_pillar(self):
        dc.World(dc.DEFAULT_OBSTACLES, (0.0, 0.0), "simple")
        self.assertEqual(p.getNumBodies(), 1 + len(dc.DEFAULT_OBSTACLES) + 1)   # floor, pillars, pad

    def test_pillars_keep_their_collision_shape_under_the_decoration(self):
        world = dc.World(dc.DEFAULT_OBSTACLES, (0.0, 0.0), "simple")
        body = world._pillars[0][1]
        shapes = p.getCollisionShapeData(body, -1)
        self.assertEqual(len(shapes), 1)                     # bands, cap and beacon are visual-only
        full_size = tuple(2 * h for h in dc.DEFAULT_OBSTACLES[0].half)       # PyBullet reports full extents
        for got, want in zip(shapes[0][3], full_size):
            self.assertAlmostEqual(got, want, places=5)

    def test_pillars_glow_as_the_drone_approaches_and_pulse_when_one_holds_it_back(self):
        world = dc.World(dc.DEFAULT_OBSTACLES, (0.0, 0.0), "simple")
        c = quiet_controller(dc.DEFAULT_OBSTACLES)
        shaft = world._pillars[0][1]

        c.pos = [-10.0, 0.0, 2.0]
        world.update(c, 0.0)
        far = self.rgba(shaft, -1)
        self.assertAlmostEqual(far[0], dc.PILLAR_COLOR[0], places=2)

        c.pos = [0.0, 0.0, 2.0]                              # 1.2 m from Obstacle_1's zone, but parked
        world.update(c, 0.0)
        self.assertAlmostEqual(self.rgba(shaft, -1)[0], far[0], places=5)      # no warning while landed

        c.mode = FlightMode.FLYING
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


class AssetGeneratorTests(unittest.TestCase):
    def test_every_city_texture_is_generated_as_a_valid_png(self):
        with tempfile.TemporaryDirectory() as folder:
            paths = sa.ensure_textures(folder)
            self.assertEqual(set(paths), {"grass", "asphalt", "plaza", "sidewalk", *sa.FACADES})
            for name, path in paths.items():
                with open(path, "rb") as f:
                    data = f.read()
                self.assertEqual(data[:8], b"\x89PNG\r\n\x1a\n", name)
                width, height = struct.unpack(">II", data[16:24])
                idat = data.index(b"IDAT")
                length = struct.unpack(">I", data[idat - 4:idat])[0]
                self.assertEqual(len(zlib.decompress(data[idat + 4:idat + 4 + length])),
                                 height * (1 + width * 3), name)
            before = os.path.getmtime(paths["grass"])
            sa.ensure_textures(folder)                        # second call must not regenerate
            self.assertEqual(os.path.getmtime(paths["grass"]), before)

    def test_floor_quad_texture_repeats_once_per_two_metres(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "floor.obj")
            sa.write_floor_obj(path, -16, 16, -8, 8)
            with open(path, encoding="utf-8") as f:
                uvs = [tuple(map(float, line.split()[1:])) for line in f if line.startswith("vt ")]
        self.assertEqual(max(u for u, _ in uvs), 16.0)       # 32 m / 2 m
        self.assertEqual(max(v for _, v in uvs), 8.0)        # 16 m / 2 m

    def test_wall_mesh_faces_point_outwards_and_windows_tile_whole_bays(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "walls.obj")
            sa.write_walls_obj(path, 20.0, 16.0, 30.0)
            with open(path, encoding="utf-8") as f:
                lines = f.read().splitlines()
        vertices = [tuple(map(float, l.split()[1:])) for l in lines if l.startswith("v ")]
        uvs = [tuple(map(float, l.split()[1:])) for l in lines if l.startswith("vt ")]
        faces = [[tuple(int(i) for i in t.split("/")) for t in l.split()[1:]] for l in lines if l.startswith("f ")]
        self.assertEqual(len(faces), 8)                      # 4 walls x 2 triangles
        for face in faces:
            a, b, c = (vertices[v - 1] for v, _, _ in face)
            ab = [y - x for x, y in zip(a, b)]
            ac = [z - x for x, z in zip(a, c)]
            normal = (ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0])
            centre = [sum(v[i] for v in (a, b, c)) / 3 for i in range(2)]
            self.assertGreater(normal[0] * centre[0] + normal[1] * centre[1], 0.0, "wall faces inward")
        self.assertEqual(max(u for u, _ in uvs), 8.0)        # 20 m / 2.5 m bays
        self.assertEqual(max(v for _, v in uvs), 9.0)        # 30 m / 3.2 m = 9.4 -> 9 whole floors


class LanePoseTests(unittest.TestCase):
    HALF, RADIUS = sc.LANE_IN, sc.CORNER_RADIUS

    def test_the_path_is_a_closed_rounded_square_that_stays_in_its_lane(self):
        length = sc.lane_length(self.HALF, self.RADIUS)
        for ccw in (True, False):
            previous = sc.lane_pose(self.HALF, self.RADIUS, 0.0, ccw)
            for i in range(1, 2001):
                s = length * i / 2000
                x, y, heading = sc.lane_pose(self.HALF, self.RADIUS, s, ccw)
                self.assertLessEqual(max(abs(x), abs(y)), self.HALF + 1e-6)
                self.assertGreaterEqual(max(abs(x), abs(y)), self.HALF - self.RADIUS - 1e-6)
                self.assertLess(math.hypot(x - previous[0], y - previous[1]), length / 2000 * 1.01)
                previous = (x, y, heading)
            end = sc.lane_pose(self.HALF, self.RADIUS, length, ccw)
            start = sc.lane_pose(self.HALF, self.RADIUS, 0.0, ccw)
            self.assertAlmostEqual(end[0], start[0], places=6)
            self.assertAlmostEqual(end[1], start[1], places=6)

    def test_cars_face_the_way_they_drive(self):
        for ccw in (True, False):
            for s in (3.0, 17.0, 31.0, 55.0, 90.0):
                x, y, heading = sc.lane_pose(self.HALF, self.RADIUS, s, ccw)
                x2, y2, _ = sc.lane_pose(self.HALF, self.RADIUS, s + 0.01, ccw)
                self.assertGreater(math.cos(heading) * (x2 - x) + math.sin(heading) * (y2 - y), 0.0099)

    def test_the_two_lanes_circulate_in_opposite_directions(self):
        def turning(ccw):                                    # signed area swept by the path
            pts = [sc.lane_pose(self.HALF, self.RADIUS, sc.lane_length(self.HALF, self.RADIUS) * i / 400, ccw)
                   for i in range(401)]
            return sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:]))
        self.assertGreater(turning(True), 0.0)
        self.assertLess(turning(False), 0.0)


class CitySceneTests(unittest.TestCase):
    def setUp(self):
        self.client = p.connect(p.DIRECT)
        self.addCleanup(p.disconnect, self.client)           # runs even if the rest of setUp raises
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.floor = p.loadURDF("plane.urdf")
        self.city = sc.CityScene(self.floor, dc.DEFAULT_OBSTACLES, (0.0, 0.0), sun=tuple(dc.LIGHT_POSITION[:2]))

    def test_has_the_expected_cast_of_characters(self):
        self.assertEqual(len(self.city.cars), 9)
        self.assertEqual(len(self.city.walkers), 10)
        self.assertEqual(len(self.city.kiosks), len(dc.DEFAULT_OBSTACLES))
        self.assertEqual(len(self.city.buildings), 16)
        self.assertGreater(p.getNumBodies(), 40)

    def test_kiosks_keep_the_exact_collision_box_the_obstacle_logic_uses(self):
        for kiosk in self.city.kiosks:
            ob = kiosk["obstacle"]
            shapes = p.getCollisionShapeData(kiosk["body"], -1)
            self.assertEqual(len(shapes), 1)
            for got, want in zip(shapes[0][3], (2 * h for h in ob.half)):
                self.assertAlmostEqual(got, want, places=5)
            position = p.getBasePositionAndOrientation(kiosk["body"])[0]
            for got, want in zip(position, ob.center):
                self.assertAlmostEqual(got, want, places=5)

    def test_traffic_moves_and_stays_on_the_road(self):
        start = [p.getBasePositionAndOrientation(c["body"])[0] for c in self.city.cars]
        for clock in (0.0, 3.7, 11.1, 29.9, 60.0, 123.4):
            self.city.update(clock)
            for car in self.city.cars:
                x, y, _ = p.getBasePositionAndOrientation(car["body"])[0]
                ring = max(abs(x), abs(y))
                self.assertGreaterEqual(ring, sc.ROAD_IN, "car left the road (inner)")
                self.assertLessEqual(ring, sc.ROAD_OUT, "car left the road (outer)")
        self.city.update(5.0)
        moved = [p.getBasePositionAndOrientation(c["body"])[0] for c in self.city.cars]
        self.assertTrue(all(math.dist(a, b) > 5.0 for a, b in zip(start, moved)))

    def test_pedestrians_stay_on_the_inner_sidewalk(self):
        for clock in (0.0, 9.0, 41.0, 97.0):
            self.city.update(clock)
            for walker in self.city.walkers:
                x, y, _ = p.getBasePositionAndOrientation(walker["body"])[0]
                self.assertGreaterEqual(max(abs(x), abs(y)), sc.PLAZA)
                self.assertLessEqual(max(abs(x), abs(y)), sc.WALK_IN)

    def test_pedestrians_never_walk_through_a_lamp_post(self):
        lamps = [(a, b) for a in (-17.0, 0.0, 17.0) for b in (-17.0, 0.0, 17.0) if (a, b) != (0.0, 0.0)]
        for clock in [i * 0.7 for i in range(400)]:
            self.city.update(clock)
            for walker in self.city.walkers:
                x, y, _ = p.getBasePositionAndOrientation(walker["body"])[0]
                for lx, ly in lamps:
                    self.assertGreater(math.hypot(x - lx, y - ly), 0.2 + 0.23, "walked through a lamp post")

    def test_no_building_throws_its_shadow_onto_the_plaza(self):
        sx, sy, sz = dc.LIGHT_POSITION
        for x, y, wx, wy, height in self.city.buildings:
            reach = (-sx / sz * height, -sy / sz * height)    # shadows point away from the sun
            for cx in (x - wx / 2, x + wx / 2):
                for cy in (y - wy / 2, y + wy / 2):
                    for t in [i / 20 for i in range(21)]:
                        px, py = cx + reach[0] * t, cy + reach[1] * t
                        self.assertFalse(abs(px) < sc.PLAZA and abs(py) < sc.PLAZA,
                                         "building at (%d,%d) h=%d shades the plaza" % (x, y, height))

    def test_buildings_are_taller_on_the_side_away_from_the_sun(self):
        sun = dc.LIGHT_POSITION
        toward = [h for x, y, _, _, h in self.city.buildings if x * sun[0] + y * sun[1] > 0]
        away = [h for x, y, _, _, h in self.city.buildings if x * sun[0] + y * sun[1] <= 0]
        self.assertTrue(toward and away)
        self.assertLess(max(toward), min(away))

    def test_halo_follows_the_requested_glow_level(self):
        halo = self.city.kiosks[0]["halo"]
        alpha = lambda: next(v[7] for v in p.getVisualShapeData(halo) if v[1] == -1)[3]
        self.city.set_glow(0, 0.0)
        self.assertEqual(alpha(), 0.0)
        self.city.set_glow(0, 1.0)
        self.assertAlmostEqual(alpha(), 0.5, places=5)
        self.city.set_glow(0, 0.5)
        self.assertAlmostEqual(alpha(), 0.25, places=5)

    def test_world_in_city_mode_drives_the_halo_and_the_traffic(self):
        p.resetSimulation()
        world = dc.World(dc.DEFAULT_OBSTACLES, (0.0, 0.0), "city")
        c = airborne(dc.DEFAULT_OBSTACLES)
        c.pos = [0.0, 0.0, 2.0]                              # 1.2 m from Obstacle_1's zone, flying
        world.update(c, 0.0)
        halo = world.city.kiosks[0]["halo"]
        self.assertGreater(next(v[7] for v in p.getVisualShapeData(halo) if v[1] == -1)[3], 0.1)
        car = world.city.cars[0]["body"]
        before = p.getBasePositionAndOrientation(car)[0]
        world.update(c, 4.0)
        self.assertGreater(math.dist(before, p.getBasePositionAndOrientation(car)[0]), 5.0)

    def test_unknown_scene_is_rejected(self):
        with self.assertRaises(ValueError):
            dc.World(dc.DEFAULT_OBSTACLES, (0.0, 0.0), "moon")


def fake_camera_info(yaw, pitch, distance, target, fov=60.0, aspect=1280 / 800):
    """What p.getDebugVisualizerCamera() returns, built from the same maths PyBullet uses."""
    view = p.computeViewMatrixFromYawPitchRoll(target, distance, yaw, pitch, 0, 2)
    proj = p.computeProjectionMatrixFOV(fov, aspect, 0.1, 500)
    forward = tuple(-view[c * 4 + 2] for c in range(3))
    up = tuple(view[c * 4 + 1] for c in range(3))
    return (1280, 800, view, proj, up, forward, (0, 0, 0), (0, 0, 0), yaw, pitch, distance, tuple(target))


class FakeDebugCamera:
    """Stands in for the GUI's camera: remembers what resetDebugVisualizerCamera was told."""

    def __init__(self, yaw=45.0, pitch=-30.0, distance=8.0, target=(0, 0, 1)):
        self.state = [yaw, pitch, distance, list(target)]
        self.resets = 0

    def get(self):
        yaw, pitch, distance, target = self.state
        return fake_camera_info(yaw, pitch, distance, target)

    def reset(self, distance, yaw, pitch, target):
        self.state = [yaw, pitch, distance, list(target)]
        self.resets += 1


class CameraTests(unittest.TestCase):
    def rig(self, mode, yaw=None, pitch=None, distance=None):
        """A rig on a fake camera. yaw/pitch/distance, if given, are applied AFTER the rig is created,
        like a user dragging or scrolling in the window."""
        camera = FakeDebugCamera()
        patches = [mock.patch.object(p, "getDebugVisualizerCamera", camera.get),
                   mock.patch.object(p, "resetDebugVisualizerCamera", camera.reset)]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        rig = dc.CameraRig(mode)
        for index, value in ((0, yaw), (1, pitch), (2, distance)):
            if value is not None:
                camera.state[index] = value
        return rig, camera

    def test_c_cycles_through_every_mode_and_wraps(self):
        rig, _ = self.rig("follow")
        self.assertEqual([rig.toggle() for _ in range(5)], ["chase", "top", "fixed", "follow", "chase"])

    def test_unknown_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            dc.CameraRig("orbit")

    def test_fixed_camera_never_moves_by_itself(self):
        rig, camera = self.rig("fixed")
        resets = camera.resets
        for _ in range(100):
            rig.update([5.0, 5.0, 2.0], 1 / 60, 1.0)
        self.assertEqual(camera.resets, resets)

    def test_follow_camera_tracks_the_drone_but_keeps_the_users_angle(self):
        rig, camera = self.rig("follow", yaw=200.0, pitch=-10.0, distance=15.0)
        for _ in range(600):
            rig.update([6.0, -4.0, 3.0], 1 / 60)
        yaw, pitch, distance, target = camera.state
        self.assertEqual((yaw, pitch, distance), (200.0, -10.0, 15.0))
        self.assertTrue(all(abs(t - w) < 0.01 for t, w in zip(target, (6.0, -4.0, 3.0))))

    def test_chase_camera_settles_behind_the_drone(self):
        rig, camera = self.rig("chase")
        for heading in (0.0, 90.0, 180.0, -90.0):
            for _ in range(600):
                rig.update([0.0, 0.0, 2.0], 1 / 60, math.radians(heading))
            forward = camera.get()[5]
            flat = math.hypot(forward[0], forward[1])                  # ignore the downward pitch
            self.assertGreater((forward[0] * math.cos(math.radians(heading))
                                + forward[1] * math.sin(math.radians(heading))) / flat, 0.999,
                               "camera is not looking along the heading %d" % heading)

    def test_chase_camera_turns_the_short_way_round(self):
        rig, camera = self.rig("chase", yaw=170.0)
        biggest = 0.0
        for _ in range(240):
            before = camera.state[0]
            rig.update([0.0, 0.0, 2.0], 1 / 60, math.radians(-100.0))   # wants yaw -190 == +170 ... via 180
            biggest = max(biggest, abs(dc.angle_difference(camera.state[0], before)))
        self.assertLess(biggest, 5.0)                                    # never a long swing

    def test_top_camera_looks_straight_down(self):
        rig, camera = self.rig("top")
        rig.update([1.0, 2.0, 3.0], 1 / 60)
        self.assertEqual(camera.state[1], -89.0)
        self.assertGreaterEqual(camera.state[2], 20.0)

    def test_angle_difference_takes_the_shortest_route(self):
        self.assertEqual(dc.angle_difference(10, 350), 20)
        self.assertEqual(dc.angle_difference(350, 10), -20)
        self.assertEqual(dc.angle_difference(90, 90), 0)
        self.assertAlmostEqual(abs(dc.angle_difference(180, 0)), 180)


class RecordingText:
    """Stands in for p.addUserDebugText and records every call."""

    def __init__(self):
        self.calls = []

    def __call__(self, text, position, **kwargs):
        self.calls.append((text, position, kwargs))
        return len(self.calls)

    def replaced(self):
        return [c for c in self.calls if "replaceItemUniqueId" in c[2]]


class HudTests(unittest.TestCase):
    BODY = 7

    def hud(self, **kwargs):
        return dc.Hud(self.BODY, dc.DEFAULT_OBSTACLES, **kwargs)

    def recording(self):
        recorder = RecordingText()
        patch = mock.patch.object(p, "addUserDebugText", recorder)
        patch.start()
        self.addCleanup(patch.stop)
        return recorder

    def test_four_labels_stacked_top_to_bottom(self):
        rows = self.hud().lines(quiet_controller(), "")
        self.assertEqual([r[0] for r in rows], ["mode", "data", "obstacle", "heard"])
        heights = [r[3] for r in rows]
        self.assertEqual(heights, sorted(heights, reverse=True))
        self.assertAlmostEqual(heights[2] - heights[3], dc.Hud.SPACING)

    def test_labels_show_state_position_speed_obstacle_and_heard(self):
        c = airborne(dc.DEFAULT_OBSTACLES)
        c.pos = [1.5, -2.25, 2.0]
        c.apply_intent(Intent.ROTATE_LEFT, 90.0)
        text = {k: t for k, t, *_ in self.hud().lines(c, "rotate left 90")}
        self.assertIn("ROTATE LEFT 90", text["mode"])
        self.assertIn("x +1.5", text["data"])
        self.assertIn("z 2.0", text["data"])
        self.assertIn("hdg 000", text["data"])
        self.assertIn("m/s", text["data"])
        self.assertIn("OBSTACLE", text["obstacle"])
        self.assertEqual(text["heard"], 'heard "rotate left 90"')

    def test_nothing_heard_shows_a_dash(self):
        text = {k: t for k, t, *_ in self.hud().lines(quiet_controller(), "")}
        self.assertEqual(text["heard"], "heard -")

    def test_obstacle_line_goes_green_amber_red_as_the_drone_closes_in(self):
        hud = self.hud()
        c = quiet_controller(dc.DEFAULT_OBSTACLES)
        results = {}
        for label, x in (("far", -10.0), ("near", -1.0), ("close", 0.5)):
            c.pos = [x, 0.0, 2.0]
            results[label] = hud.obstacle_line(c)
        self.assertEqual(results["far"][1], [0.4, 1.0, 0.5])
        self.assertEqual(results["near"][1], [1.0, 0.85, 0.2])
        self.assertEqual(results["close"][1], [1.0, 0.25, 0.2])
        self.assertIn(" m", results["close"][0])
        c.pos = [-30.0, 0.0, 2.0]
        self.assertEqual(hud.obstacle_line(c)[0], "OBSTACLE  clear")
        self.assertIn("none", dc.Hud(self.BODY, ()).obstacle_line(c)[0])

    def test_status_colour_tells_the_state_at_a_glance(self):
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

    def test_labels_are_attached_to_the_drone_so_the_window_moves_them_for_free(self):
        recorder = self.recording()
        self.hud().update(quiet_controller(), "", 1.0)
        self.assertEqual(len(recorder.calls), 4)
        for _, position, kwargs in recorder.calls:
            self.assertEqual(kwargs["parentObjectUniqueId"], self.BODY)
            self.assertEqual(kwargs["parentLinkIndex"], -1)
            self.assertEqual(position[:2], [0.0, 0.0])                          # straight above the drone

    def test_text_that_has_not_changed_is_never_rewritten(self):
        recorder = self.recording()
        hud, c = self.hud(), quiet_controller()
        hud.update(c, "", 1.0)
        for t in (1.1, 2.0, 5.0, 60.0):
            hud.update(c, "", t)
        self.assertEqual(len(recorder.calls), 4)                               # only the initial creation

    def test_rewrites_are_rate_limited_to_one_label_per_call(self):
        recorder = self.recording()
        hud = self.hud()
        c = airborne(dc.DEFAULT_OBSTACLES)
        hud.update(c, "", 1.0)
        self.assertEqual(len(recorder.calls), 4)
        c.apply_intent(Intent.FORWARD)               # mode, position/speed and obstacle distance all change
        for _ in range(30):
            c.step(1 / 60)
        hud.update(c, "", 1.1)
        self.assertEqual(len(recorder.replaced()), 0)                           # too soon after creation
        hud.update(c, "", 1.6)
        self.assertEqual(len(recorder.replaced()), 1)                           # one label per call...
        hud.update(c, "", 1.61)
        self.assertEqual(len(recorder.replaced()), 2)                           # ...the next on the next call
        hud.update(c, "", 1.62)
        self.assertEqual(len(recorder.replaced()), 3)                           # ...until all three have caught up
        hud.update(c, "", 1.63)
        self.assertEqual(len(recorder.replaced()), 3)                           # and then nothing more is written
        self.assertEqual(len({kw["replaceItemUniqueId"] for _, _, kw in recorder.replaced()}), 3)

    def test_a_label_is_not_rewritten_twice_within_half_a_second(self):
        recorder = self.recording()
        hud = self.hud()
        c = airborne(dc.DEFAULT_OBSTACLES)
        hud.update(c, "", 1.0)
        c.apply_intent(Intent.FORWARD)
        hud.update(c, "", 1.6)                                                  # a label is rewritten
        first = len(recorder.replaced())
        c.apply_intent(Intent.STOP)
        hud.update(c, "", 1.65)                                                 # changed again 50 ms later
        mode_rewrites = [call for call in recorder.replaced() if call[0] in ("FORWARD", "STOP")]
        self.assertEqual(len(mode_rewrites), 1)                                 # the mode label only once so far
        self.assertGreaterEqual(len(recorder.replaced()), first)

    def test_hud_can_be_hidden(self):
        recorder = self.recording()
        self.hud(enabled=False).update(quiet_controller(), "", 1.0)
        self.assertEqual(recorder.calls, [])

    def test_resizing_rescales_text_and_line_spacing_and_is_clamped(self):
        recorder = self.recording()
        hud, c = self.hud(), quiet_controller()
        hud.update(c, "", 1.0)
        base_size = recorder.calls[1][2]["textSize"]
        base_gap = recorder.calls[0][1][2] - recorder.calls[1][1][2]
        self.assertAlmostEqual(hud.resize(1.25), 1.25)
        for i in range(8):                                                      # one label per call, so a few calls
            hud.update(c, "", 10.0 + i * 0.6)
        latest = {}
        for text, position, kwargs in recorder.calls[4:]:
            latest[kwargs["replaceItemUniqueId"]] = (kwargs["textSize"], position[2])
        self.assertEqual(len(latest), 4)                                        # every label was redone
        data_size, data_height = latest[2]
        self.assertAlmostEqual(data_size, base_size * 1.25)
        self.assertAlmostEqual(latest[1][1] - data_height, base_gap * 1.25)
        for _ in range(30):
            hud.resize(0.5)
        self.assertEqual(hud.scale, 0.3)                                        # clamped, never vanishes
        for _ in range(30):
            hud.resize(2.0)
        self.assertEqual(hud.scale, 4.0)

    def test_bracket_keys_resize_the_hud(self):
        c = quiet_controller()
        with mock.patch.object(p, "getKeyboardEvents",
                               return_value={ord("["): p.KEY_WAS_TRIGGERED, ord("]"): p.KEY_WAS_TRIGGERED}):
            triggered = dc.KeyboardInput().poll(c)
        self.assertEqual(set(triggered), {Intent.HUD_SMALLER, Intent.HUD_BIGGER})

    def test_zone_gap_is_distance_to_the_safety_zone_surface(self):
        ob = dc.DEFAULT_OBSTACLES[0]
        face = zone_face_x(ob)
        self.assertAlmostEqual(dc.zone_gap([face - 2.0, 0.0, 2.0], ob), 2.0, places=6)
        self.assertEqual(dc.zone_gap([face + 0.5, 0.0, 2.0], ob), 0.0)         # inside the zone
        self.assertAlmostEqual(dc.zone_gap([face - 3.0, 4.0 + 2.27, 2.0], ob),
                               math.hypot(3.0, 4.0), places=1)


class DashboardTests(unittest.TestCase):
    CONFIG = {"obstacles": [], "geofence": 12.0}

    def start(self, **kwargs):
        dashboard = Dashboard(self.CONFIG, port=0, **kwargs)          # port 0: let the system pick a free one
        url = dashboard.start()
        self.addCleanup(dashboard.stop)
        return dashboard, url

    @staticmethod
    def get(url, path):
        with urllib.request.urlopen(url.rstrip("/") + path, timeout=5) as reply:
            return reply.status, reply.headers.get("Content-Type"), reply.read()

    def test_serves_the_page_and_the_scene_facts(self):
        _, url = self.start()
        status, kind, body = self.get(url, "/")
        self.assertEqual(status, 200)
        self.assertIn("text/html", kind)
        for needle in (b"<title>Drone live graphs</title>", b'id="c-alt"', b'id="c-speed"', b'id="c-gap"',
                       b'id="c-pos"', b'id="c-map"', b'id="c-strip"', b'id="log"'):
            self.assertIn(needle, body)
        self.assertEqual(self.get(url, "/?theme=dark")[2], body)        # a query string does not change what is served
        config = json.loads(self.get(url, "/api/config")[2])
        self.assertEqual(config["geofence"], 12.0)
        self.assertEqual(config["sample_hz"], Dashboard.SAMPLE_HZ)

    def test_it_only_listens_on_this_computer(self):
        dashboard, url = self.start()
        self.assertEqual(dashboard._server.server_address[0], "127.0.0.1")
        self.assertTrue(url.startswith("http://127.0.0.1:"))

    def test_samples_are_recorded_at_most_ten_times_a_second(self):
        dashboard = Dashboard(self.CONFIG)
        c = airborne()
        t = dashboard.start_time
        self.assertTrue(dashboard.record(c, now=t + 1.00))
        self.assertFalse(dashboard.record(c, now=t + 1.05))           # too soon
        self.assertTrue(dashboard.record(c, now=t + 1.11))
        self.assertEqual(len(dashboard.data()["samples"]), 2)

    def test_a_sample_describes_the_drone(self):
        dashboard = Dashboard(self.CONFIG, gap_fn=lambda controller: 1.2345)
        c = airborne()
        c.yaw = -math.pi / 2                                            # facing 270 degrees
        dashboard.record(c, now=dashboard.start_time + 3.0)
        sample = dashboard.data()["samples"][0]
        self.assertEqual((sample["t"], sample["z"], sample["label"], sample["mode"]), (3.0, 2.0, "HOVER", "FLYING"))
        self.assertEqual(sample["yaw"], 270.0)
        self.assertEqual(sample["gap"], 1.23)
        self.assertEqual((sample["hs"], sample["vz"]), (0.0, 0.0))
        no_obstacles = Dashboard(self.CONFIG)
        no_obstacles.record(c, now=no_obstacles.start_time + 1.0)
        self.assertIsNone(no_obstacles.data()["samples"][0]["gap"])

    def test_the_browser_only_gets_what_is_new(self):
        dashboard = Dashboard(self.CONFIG)
        c = airborne()
        for i in range(3):
            dashboard.record(c, now=dashboard.start_time + i)
        dashboard.event("command", "forward", "FORWARD", now=dashboard.start_time + 1.0)
        dashboard.event("rejected", "banana", "not recognised", now=dashboard.start_time + 2.0)
        first = dashboard.data(0, 0)
        self.assertEqual((len(first["samples"]), len(first["events"]), first["s"], first["e"]), (3, 2, 3, 2))
        again = dashboard.data(first["s"], first["e"])
        self.assertEqual((again["samples"], again["events"], again["s"], again["e"]), ([], [], 3, 2))
        dashboard.record(c, now=dashboard.start_time + 5)
        newest = dashboard.data(first["s"], first["e"])
        self.assertEqual([s["t"] for s in newest["samples"]], [5.0])
        self.assertEqual(newest["events"], [])

    def test_events_keep_what_was_said_and_what_it_meant(self):
        dashboard = Dashboard(self.CONFIG)
        dashboard.event("command", "go forward 3 meters", "FORWARD 3 m", now=dashboard.start_time + 4.0)
        self.assertEqual(dashboard.data()["events"],
                         [{"t": 4.0, "kind": "command", "text": "go forward 3 meters", "label": "FORWARD 3 m"}])

    def test_the_data_endpoint_returns_json_and_ignores_garbage_in_the_query(self):
        dashboard, url = self.start()
        dashboard.record(airborne(), now=dashboard.start_time + 1.0)
        status, kind, body = self.get(url, "/api/data?s=0&e=0")
        self.assertEqual((status, kind), (200, "application/json"))
        self.assertEqual(len(json.loads(body)["samples"]), 1)
        self.assertEqual(len(json.loads(self.get(url, "/api/data?s=abc&e=%00")[2])["samples"]), 1)   # treated as 0
        self.assertEqual(json.loads(self.get(url, "/api/data?s=1&e=0")[2])["samples"], [])

    def test_old_samples_are_dropped_but_the_counters_keep_counting(self):
        dashboard = Dashboard(self.CONFIG, history=20)
        c = airborne()
        for i in range(100):
            dashboard.record(c, now=dashboard.start_time + i)
            dashboard.event("command", "x", str(i), now=dashboard.start_time + i)
        everything = dashboard.data(0, 0)
        self.assertLessEqual(len(everything["samples"]), 20)
        self.assertEqual(everything["s"], 100)                          # still counts all 100
        self.assertEqual(everything["samples"][-1]["t"], 99.0)          # newest kept
        self.assertEqual([s["t"] for s in dashboard.data(95, 0)["samples"]], [95.0, 96.0, 97.0, 98.0, 99.0])

    def test_nothing_but_the_page_and_the_data_is_served(self):
        _, url = self.start()
        for path in ("/nope", "/../drone_core.py", "/dashboard.py", "/drone_data.csv", "/api"):
            with self.subTest(path=path):
                with self.assertRaises(urllib.error.HTTPError) as caught:
                    self.get(url, path)
                self.assertEqual(caught.exception.code, 404)

    def test_if_the_port_is_taken_the_next_free_one_is_used(self):
        taken = socket.socket()
        self.addCleanup(taken.close)
        taken.bind(("127.0.0.1", 0))
        taken.listen()
        busy = taken.getsockname()[1]
        dashboard = Dashboard(self.CONFIG, port=busy)
        url = dashboard.start()
        self.addCleanup(dashboard.stop)
        port = int(url.rstrip("/").rsplit(":", 1)[1])
        self.assertNotEqual(port, busy)
        self.assertLess(port - busy, Dashboard.MAX_PORT_TRIES)
        self.assertEqual(self.get(url, "/api/config")[0], 200)

    def test_stopping_closes_the_server(self):
        dashboard, url = self.start()
        self.assertEqual(self.get(url, "/api/config")[0], 200)
        dashboard.stop()
        with self.assertRaises(urllib.error.URLError):
            self.get(url, "/api/config")
        dashboard.stop()                                                # stopping twice is harmless

    def test_scene_facts_for_the_graphs_match_the_simulation(self):
        c = quiet_controller(dc.DEFAULT_OBSTACLES)
        config = dc.dashboard_config(dc.DEFAULT_OBSTACLES, c, "city", "stop")
        self.assertEqual([o["name"] for o in config["obstacles"]], [o.name for o in dc.DEFAULT_OBSTACLES])
        self.assertEqual(config["geofence"], dc.WORLD_LIMIT)
        self.assertEqual(config["takeoff_height"], dc.TAKEOFF_HEIGHT)
        self.assertEqual(config["stop_distance"], dc.STOP_BUFFER)
        self.assertAlmostEqual(config["zone_grow"], dc.DRONE_RADIUS + dc.SAFETY_DISTANCE)
        self.assertEqual(config["plaza"], sc.PLAZA)
        self.assertEqual(config["obstacle_strategy"], "stop")
        json.dumps(config)                                              # must be JSON-serialisable
        plain = dc.dashboard_config((), quiet_controller(), "simple", "stop")
        self.assertEqual((plain["obstacles"], plain["obstacle_strategy"], plain["plaza"]), ([], None, None))


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
