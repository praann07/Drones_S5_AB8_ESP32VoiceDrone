import os
import sys
import argparse
import math
import queue
import threading
import time

import config
from commands import parse as parse_command
from navigation import Mission, Waypoint, default_mission, to_local
from physics import Drone

import controller as ctrl
from controller import FlightController

try:
    from data_logger import TelemetryLogger
except ImportError:
    from .data_logger import TelemetryLogger


def _find_wp(waypoints, name):
    n = str(name).strip().upper()
    if not n:
        return None
    for w in waypoints:
        if w.name.upper() == n:
            return w
    for w in waypoints:
        if w.name.upper().startswith(n) or w.name.upper().startswith(f"{n}:"):
            return w
    if n.isdigit():
        idx = int(n) - 1
        if 0 <= idx < len(waypoints):
            return waypoints[idx]
    if n.startswith("P") and n[1:].isdigit():
        idx = int(n[1:]) - 1
        if 0 <= idx < len(waypoints):
            return waypoints[idx]
    for w in waypoints:
        if n in w.name.upper():
            return w
    return None


class GCS:
    def __init__(self, drone):
        self.drone = drone
        self.controller = FlightController()
        self.mission = Mission()
        self.waypoints = default_mission()
        self.home_pos = (0.0, 0.0)
        self.target_alt = 0.0
        self.log = []
        self.link_ok = True
        self.last_cmd_time = 0.0
        self.geofence_radius = config.GEOFENCE_DEFAULT_RADIUS
        self.speed_mult = 1.0
        self.rth_triggered = False
        self.trail = []
        self.trail_timer = 0.0
        self.voice_status = "voice active"
        self.manual_targets = {}
        self.failsafe_enabled = True
        self._wp_counter = 0
        self.last_cmd = "none"
        self.last_voice_text = ""
        self.last_voice_action = "STANDBY"
        self.voice_history = []
        
        # Advanced Autonomy: Orbit / POI & Wind Tracking
        self.orbit_active = False
        self.orbit_center = None
        self.orbit_radius = 10.0
        self.orbit_speed = 0.35  # rad/s
        self.orbit_angle = 0.0

        # High-Rate 250 Hz Flight Telemetry Logger (DDMS)
        self.logger = TelemetryLogger()
        self.is_analyzing = False

        try:
            from voice import TTSAnnouncer
            self.tts = TTSAnnouncer(enabled=True)
        except Exception:
            self.tts = None



    def log_msg(self, msg):
        stamp = "{:.0f}s".format(self.drone.t)
        line = "[{}] {}".format(stamp, msg)
        self.log.append(line)
        self.log = self.log[-40:]

    def log_voice(self, text, action):
        stamp = "{:.0f}s".format(self.drone.t)
        self.last_voice_text = text
        self.last_voice_action = action
        self.voice_history.append((stamp, text, action))
        self.voice_history = self.voice_history[-8:]


    def handle(self, action, args):
        self.last_cmd_time = self.drone.t
        self.link_ok = True
        self.rth_triggered = False
        self.last_cmd = action.replace("_", " ").upper()
        d = self.drone
        fc = self.controller
        if action == "quit":
            return False
        if action == "toggle_logging":
            if self.logger:
                self.logger.is_logging = not self.logger.is_logging
                st = "ACTIVE (250Hz)" if self.logger.is_logging else "PAUSED"
                self.log_msg(f"TELEMETRY LOGGING: {st}")
                if self.tts:
                    self.tts.speak(f"Telemetry recording {st.lower()}.")
        elif action == "run_analysis":
            self._trigger_data_analysis()
        elif action == "toggle_tts":
            if self.tts:
                self.tts.enabled = args.get("enabled", not self.tts.enabled)
            self.log_msg(f"TTS Audio: {'ENABLED' if (self.tts and self.tts.enabled) else 'DISABLED'}")
        elif action == "takeoff":
            alt = args.get("alt")
            if fc.mode == ctrl.STANDBY:
                d.arm()
                self.home_pos = (d.est_n, d.est_e)
                fc.takeoff(alt)
                self.target_alt = fc.target_alt
                self.log_msg("ARMED - takeoff to {:.1f} m".format(fc.target_alt))
                if self.tts:
                    self.tts.speak(f"Armed. Taking off to {fc.target_alt:.0f} meters.")
            else:
                self.log_msg("Already flying")
        elif action == "land":
            self.orbit_active = False
            if fc.mode != ctrl.STANDBY:
                fc.land()
                self.log_msg("LANDING")
                if self.tts:
                    self.tts.speak("Landing initiated.")
            else:
                self.log_msg("Already landed")
        elif action == "rth":
            self.orbit_active = False
            fc.return_home(self.home_pos)
            self.mission.paused = True
            self.log_msg("RETURNING HOME")
            if self.tts:
                self.tts.speak("Returning to home base.")
        elif action == "mission":
            self.orbit_active = False
            if not d.armed or fc.mode in (ctrl.STANDBY, ctrl.LANDING):
                d.arm()
                self.home_pos = (d.est_n, d.est_e)
                fc.takeoff(config.WAYPOINT_ALT)
            if not self.mission.active:
                if not self.mission.points:
                    self.mission.load(self.waypoints)
                else:
                    self.mission.active = True
                    self.mission.index = 0
            self.mission.paused = False
            wp = self.mission.current()
            if wp is not None:
                fc.goto(wp.n, wp.e, wp.alt)
                self.target_alt = fc.target_alt
                self.log_msg("MISSION STARTED - {} waypoints ahead".format(self.mission.remaining()))
                if self.tts:
                    self.tts.speak(f"Mission started. Navigating to Point {wp.name}.")
        elif action == "goto":
            self.orbit_active = False
            name = str(args.get("name", "")).strip().upper()
            wp = _find_wp(self.waypoints, name)
            if wp:
                if not d.armed or fc.mode in (ctrl.STANDBY, ctrl.LANDING):
                    d.arm()
                    self.home_pos = (d.est_n, d.est_e)
                fc.goto(wp.n, wp.e, wp.alt)
                self.target_alt = fc.target_alt
                self.mission.paused = True
                self.log_msg("GOING TO POINT {}".format(wp.name))
                if self.tts:
                    self.tts.speak(f"Navigating to Point {wp.name}.")
            else:
                self.log_msg("Point {} not found".format(name))

        elif action == "goto_latlon":
            self.orbit_active = False
            if not d.armed or fc.mode in (ctrl.STANDBY, ctrl.LANDING):
                d.arm()
                self.home_pos = (d.est_n, d.est_e)
            n, e = to_local(args["lat"], args["lon"])
            alt = max(config.WAYPOINT_ALT, d.est_alt)
            fc.goto(n, e, alt)
            self.target_alt = fc.target_alt
            self.mission.paused = True
            self.log_msg("GOING TO GPS {:.5f}, {:.5f}".format(args["lat"], args["lon"]))
            if self.tts:
                self.tts.speak("Navigating to target GPS coordinates.")
        elif action == "map_point":
            self.orbit_active = False
            if not d.armed or fc.mode in (ctrl.STANDBY, ctrl.LANDING):
                d.arm()
                self.home_pos = (d.est_n, d.est_e)
            self._wp_counter += 1
            name = "P{}".format(self._wp_counter)
            alt = args.get("alt")
            if alt is None:
                alt = max(config.WAYPOINT_ALT, d.est_alt)
            wp = Waypoint(name, args["n"], args["e"], alt)
            self.waypoints.append(wp)
            fc.goto(wp.n, wp.e, wp.alt)
            self.target_alt = fc.target_alt
            self.mission.paused = True
            self.log_msg("NEW WAYPOINT {} - going there".format(name))
            if self.tts:
                self.tts.speak(f"New waypoint {name} set. Flying there.")
        elif action == "pause":
            self.orbit_active = False
            fc.hold_position(d.est_n, d.est_e)
            self.mission.paused = True
            self.log_msg("HOLD POSITION")
            if self.tts:
                self.tts.speak("Holding position.")
        elif action == "resume":
            self.orbit_active = False
            if not d.armed or fc.mode in (ctrl.STANDBY, ctrl.LANDING):
                d.arm()
                self.home_pos = (d.est_n, d.est_e)
            self.mission.paused = False
            if self.mission.active:
                wp = self.mission.current()
                if wp is not None:
                    fc.goto(wp.n, wp.e, wp.alt)
                    self.log_msg("MISSION RESUMED")
                    if self.tts:
                        self.tts.speak(f"Mission resumed to Point {wp.name}.")
            else:
                fc.hover()
                self.log_msg("HOVERING")

        elif action == "abort":
            self.orbit_active = False
            fc.return_home(self.home_pos)
            self.mission.paused = True
            self.log_msg("ABORT - RETURN HOME")
            if self.tts:
                self.tts.speak("Emergency abort. Returning home.")
        elif action == "manual":
            self.orbit_active = False
            axis = args["axis"]
            val = args["value"] * self.speed_mult
            if axis == "vx":
                self.manual_targets["vx"] = val
            else:
                self.manual_targets["vy"] = val
            fc.set_manual(self.manual_targets.get("vx", 0.0), self.manual_targets.get("vy", 0.0))
            self.log_msg("MANUAL " + ("FORWARD" if (axis == "vx" and val > 0) else "BACK" if axis == "vx" else "LEFT" if val < 0 else "RIGHT"))
        elif action == "heading":
            cur = d.est_heading_deg
            target = (cur + args["deg"]) % 360.0
            fc.set_heading(target)
            fc.set_manual(0.0, 0.0)
            self.log_msg("TURN TO {:.0f} deg".format(target))
        elif action == "heading_abs":
            fc.set_heading(args["deg"] % 360.0)
            self.log_msg("HEADING TO {:.0f} deg".format(args["deg"] % 360.0))
        elif action == "altitude":
            fc.target_alt = max(0.0, fc.target_alt + args["delta"])
            self.target_alt = fc.target_alt
            self.log_msg("TARGET ALT {:.1f} m".format(fc.target_alt))
        elif action == "set_alt":
            fc.target_alt = max(0.0, args["alt"])
            self.target_alt = fc.target_alt
            self.log_msg("TARGET ALT {:.1f} m".format(fc.target_alt))
        elif action == "geofence":
            if args.get("radius"):
                self.geofence_radius = args["radius"]
                self.log_msg("GEOFENCE radius {:.0f} m".format(args["radius"]))
            else:
                self.log_msg("GEOFENCE {} m".format(self.geofence_radius))
        elif action == "link_loss":
            self.link_ok = False
            self.rth_triggered = False
            self.log_msg("COMMUNICATION LINK LOST - FAILSAFE")
            if self.tts:
                self.tts.speak("Warning. Communication link lost. Failsafe activated.")
        elif action == "wind":
            m = args.get("mode", "LIGHT").upper()
            d.set_wind(m)
            self.log_msg(f"WIND SIMULATION: {d.wind_mode} ({d.wind_speed:.1f} m/s)")
            if self.tts:
                self.tts.speak(f"Wind condition set to {d.wind_mode}.")
        elif action == "cycle_wind":
            m = d.cycle_wind()
            self.log_msg(f"WIND SIMULATION: {m} ({d.wind_speed:.1f} m/s)")
            if self.tts:
                self.tts.speak(f"Wind condition {m}.")
        elif action == "gust":
            st = args.get("strength", config.WIND_GUST_BURST)
            d.trigger_gust(st, duration=4.0)
            self.log_msg(f"WIND GUST INJECTED (+{st:.1f} m/s)")
            if self.tts:
                self.tts.speak("Wind gust alert.")
        elif action == "map_mode":
            mode = args.get("mode", "DARK").upper()
            try:
                from dashboard import set_map_mode
                set_map_mode(mode)
            except Exception:
                pass
            self.log_msg(f"MAP VIEW: {mode}")
            if self.tts:
                self.tts.speak(f"Map layer {mode.lower()}.")
        elif action == "cycle_map":
            try:
                from dashboard import cycle_map_mode
                m = cycle_map_mode()
                self.log_msg(f"MAP VIEW: {m}")
                if self.tts:
                    self.tts.speak(f"Map layer {m.lower()}.")
            except Exception:
                pass

        elif action == "orbit":
            name = str(args.get("name", "")).strip().upper()
            target_wp = None
            if name:
                target_wp = next((w for w in self.waypoints if w.name.upper() == name or w.name.upper() == f"P{name}"), None)
                if not target_wp and name.isdigit():
                    target_wp = next((w for w in self.waypoints if w.name.upper() == f"P{name}"), None)

            # If orbit is already active and no new specific target requested, toggle orbit OFF
            if self.orbit_active and not name:
                self.orbit_active = False
                fc.hold_position(d.est_n, d.est_e)
                self.log_msg("ORBIT STOPPED - HOLD POSITION")
                if self.tts:
                    self.tts.speak("Orbit mode cancelled. Holding position.")
                return True

            if target_wp:
                self.orbit_center = (target_wp.n, target_wp.e)
                lbl = target_wp.name
            else:
                self.orbit_center = (d.est_n, d.est_e)
                lbl = "Current Position"
            if not d.armed or fc.mode in (ctrl.STANDBY, ctrl.LANDING):
                d.arm()
                self.home_pos = (d.est_n, d.est_e)
                fc.takeoff(config.WAYPOINT_ALT)
            self.orbit_active = True
            self.mission.paused = True
            fc.mode = ctrl.GPS
            self.orbit_angle = math.atan2(d.est_e - self.orbit_center[1], d.est_n - self.orbit_center[0])
            self.log_msg(f"POI ORBIT STARTED around {lbl} (R={self.orbit_radius:.0f}m)")
            if self.tts:
                self.tts.speak(f"Orbiting around {lbl}.")

        elif action == "orbit_stop":
            self.orbit_active = False
            fc.hold_position(d.est_n, d.est_e)
            self.log_msg("ORBIT STOPPED - HOLD POSITION")
            if self.tts:
                self.tts.speak("Orbit mode stopped. Holding position.")

        elif action == "hover_at":
            name = str(args.get("name", "")).strip().upper()
            wp = _find_wp(self.waypoints, name)
            if wp:
                self.orbit_active = False
                if not d.armed or fc.mode in (ctrl.STANDBY, ctrl.LANDING):
                    d.arm()
                    self.home_pos = (d.est_n, d.est_e)
                fc.goto(wp.n, wp.e, wp.alt)
                self.target_alt = fc.target_alt
                self.mission.paused = True
                self.log_msg(f"FLYING TO HOVER AT {wp.name}")
                if self.tts:
                    self.tts.speak(f"Navigating to hover at Point {wp.name}.")
            else:
                self.log_msg(f"Point {name} not found")

        elif action == "delete_wp":
            name = str(args.get("name", "")).strip().upper()
            target_wp = _find_wp(self.waypoints, name)
            if target_wp:
                self.waypoints = [w for w in self.waypoints if w != target_wp]
                if self.mission.points:
                    self.mission.points = [w for w in self.mission.points if w != target_wp]
                self.log_msg(f"WAYPOINT {target_wp.name} REMOVED")
                if self.tts:
                    self.tts.speak(f"Waypoint {target_wp.name} removed.")
            else:
                self.log_msg(f"Waypoint {name} not found to delete")

        elif action == "clear_waypoints":
            custom_count = len([w for w in self.waypoints if w.name.startswith("P")])
            self.waypoints = [w for w in self.waypoints if not w.name.startswith("P")]
            if self.mission.points:
                self.mission.points = [w for w in self.mission.points if not w.name.startswith("P")]
            self.log_msg(f"CLEARED {custom_count} CUSTOM WAYPOINTS")
            if self.tts:
                self.tts.speak("Custom waypoints cleared.")
        elif action == "add_waypoint":
            self._wp_counter += 1
            name = "P{}".format(self._wp_counter)
            wp = Waypoint(name, d.est_n, d.est_e, self.target_alt)
            self.waypoints.append(wp)
            self.log_msg("WAYPOINT {} added at current position".format(name))
        elif action == "clear_trail":
            self.trail = []
        elif action == "status":
            self.log_msg("GPS {:.6f},{:.6f} alt {:.1f} batt {:.0f}% mode {} wind {}".format(
                d.gps_lat, d.gps_lon, d.est_alt, d.battery, fc.mode, d.wind_mode))
            if self.tts:
                self.tts.speak(f"Status report. Altitude {d.est_alt:.1f} meters. Battery {d.battery:.0f} percent. Wind {d.wind_mode}.")
        elif action == "text_cmd":
            raw_text = args.get("text", "")
            parsed = parse_command(raw_text)
            if parsed and parsed[0] != "unknown":
                act_name = parsed[0].replace("_", " ").upper()
                if parsed[0] == "goto":
                    act_name = f"GOTO POINT {parsed[1].get('name', '')}"
                elif parsed[0] == "takeoff":
                    act_name = f"TAKEOFF ({parsed[1].get('alt') or fc.target_alt or 3.0:.0f}m)"
                elif parsed[0] == "wind":
                    act_name = f"WIND {parsed[1].get('mode', '')}"
                elif parsed[0] == "orbit":
                    act_name = f"ORBIT {parsed[1].get('name', '')}"
                self.log_voice(raw_text, act_name)
                self.handle(parsed[0], parsed[1])
            else:
                self.log_voice(raw_text, "UNRECOGNIZED")
                self.log_msg("Did not understand: " + raw_text)
        elif action == "unknown":
            self.log_msg("Did not understand command")
        return True

    def _trigger_data_analysis(self):
        if self.is_analyzing:
            self.log_msg("Data analysis already running...")
            return
        saved_csv = self.logger.save_to_csv() if self.logger else None
        self.is_analyzing = True
        self.log_msg("STARTING SINDy & DMDc DATA ANALYSIS...")
        if self.tts:
            self.tts.speak("Running SINDy and DMD system identification.")

        def _worker():
            try:
                from data_driven_analysis import run_data_driven_pipeline
                out_dir = run_data_driven_pipeline(saved_csv)
                self.log_msg(f"ANALYSIS COMPLETE! Artifacts in data_driven_results/")
                if self.tts:
                    self.tts.speak("Data driven flight dynamics discovery complete.")
            except Exception as e:
                self.log_msg(f"ANALYSIS ERROR: {e}")
            finally:
                self.is_analyzing = False

        threading.Thread(target=_worker, daemon=True).start()

    def update(self, dt):
        d = self.drone
        fc = self.controller
        if self.logger:
            self.logger.record_step(d)
        if fc.mode != ctrl.STANDBY:
            if self.orbit_active and fc.mode == ctrl.GPS and self.orbit_center is not None:
                self.orbit_angle += self.orbit_speed * dt
                tgt_n = self.orbit_center[0] + self.orbit_radius * math.cos(self.orbit_angle)
                tgt_e = self.orbit_center[1] + self.orbit_radius * math.sin(self.orbit_angle)
                fc.target_pos = (tgt_n, tgt_e)
                # Face nose towards the POI orbit center
                center_bearing = math.degrees(math.atan2(self.orbit_center[1] - d.est_e, self.orbit_center[0] - d.est_n)) % 360.0
                fc.set_heading(center_bearing)
            elif fc.mode == ctrl.GPS and self.mission.active and not self.mission.paused:
                self.last_cmd_time = d.t
                wp = self.mission.current()
                if wp is not None:
                    fc.target_pos = (wp.n, wp.e)
                    fc.target_alt = wp.alt
                    self.target_alt = wp.alt
                    if wp.distance_to(d.est_n, d.est_e) < config.ACCEPT_RADIUS and abs(wp.alt - d.est_alt) < config.ALT_ACCEPT:
                        if self.mission.advance():
                            self.log_msg("REACHED point {}".format(wp.name))
                            nxt = self.mission.current()
                            if nxt is not None:
                                fc.target_pos = (nxt.n, nxt.e)
                                fc.target_alt = nxt.alt
                                self.target_alt = nxt.alt
                                if self.tts:
                                    self.tts.speak(f"Waypoint {wp.name} reached. Proceeding to {nxt.name}.")
                        else:
                            self.log_msg("MISSION COMPLETE - RETURNING HOME")
                            if self.tts:
                                self.tts.speak("Mission complete. Returning to home base.")
                            fc.return_home(self.home_pos)
            elif fc.mode == ctrl.RTH and fc.target_pos is not None:
                dist = math.hypot(fc.target_pos[0] - d.est_n, fc.target_pos[1] - d.est_e)
                if dist < config.ACCEPT_RADIUS and abs(fc.target_alt - d.est_alt) < config.ALT_ACCEPT:
                    self.log_msg("ARRIVED HOME - LANDING")
                    if self.tts:
                        self.tts.speak("Arrived at home base. Landing now.")
                    fc.land()

            dist_home = math.hypot(d.est_n - self.home_pos[0], d.est_e - self.home_pos[1])
            if fc.mode in (ctrl.GPS, ctrl.RTH):
                if dist_home > self.geofence_radius and not self.rth_triggered:
                    self.rth_triggered = True
                    fc.return_home(self.home_pos)
                    self.log_msg("GEOFENCE BREACH - RETURN HOME")
                    if self.tts:
                        self.tts.speak("Geofence boundary breach. Returning home.")
                if d.est_alt > config.GEOFENCE_MAX_ALT:
                    fc.target_alt = config.GEOFENCE_MAX_ALT
                    self.log_msg("ALTITUDE LIMIT - descending")
            if self.failsafe_enabled:
                if d.battery < config.BATTERY_LAND and fc.mode != ctrl.LANDING:
                    fc.land()
                    self.log_msg("CRITICAL BATTERY - LANDING")
                    if self.tts:
                        self.tts.speak("Critical battery level. Landing immediately.")
                elif d.battery < config.BATTERY_RTH and fc.mode in (ctrl.MANUAL, ctrl.GPS) and not self.rth_triggered:
                    self.rth_triggered = True
                    fc.return_home(self.home_pos)
                    self.log_msg("LOW BATTERY - RETURN HOME")
                    if self.tts:
                        self.tts.speak("Low battery warning. Returning home.")
                if fc.mode in (ctrl.GPS, ctrl.RTH):
                    if (not self.link_ok or d.t - self.last_cmd_time > config.COMMS_TIMEOUT) and not self.rth_triggered:
                        self.rth_triggered = True
                        fc.return_home(self.home_pos)
                        self.log_msg("LINK LOST - FAILSAFE RETURN HOME")
                        if self.tts:
                            self.tts.speak("Radio link lost. Failsafe return home initiated.")
                elif fc.mode == ctrl.MANUAL:
                    if not self.link_ok or d.t - self.last_cmd_time > config.COMMS_TIMEOUT:
                        fc.set_manual(0.0, 0.0)
                        self.manual_targets = {}
                        self.log_msg("NO COMMANDS - HOVERING")

            if fc.mode == ctrl.LANDING and d.alt < 0.4:
                d.disarm()
                fc.reset()
                self.log_msg("LANDED - DISARMED")
                self.rth_triggered = False
                if self.logger and self.logger.sample_count > 50:
                    saved_path = self.logger.save_to_csv()
                    if saved_path:
                        self.log_msg(f"SAVED {self.logger.sample_count} SAMPLES TO {os.path.basename(saved_path)}")

        self.trail_timer += dt
        if self.trail_timer > 0.12:
            self.trail_timer = 0.0
            self.trail.append((d.est_n, d.est_e))
            if len(self.trail) > 2000:
                self.trail = self.trail[-2000:]



    def gcs_data(self):
        nxt = self.mission.current()
        return {
            "mode": self.controller.mode,
            "target_alt": self.controller.target_alt,
            "target_pos": self.controller.target_pos,
            "home_pos": self.home_pos,
            "waypoints": self.waypoints,
            "geofence_radius": self.geofence_radius,
            "link_ok": self.link_ok,
            "log": self.log,
            "trail": self.trail,
            "voice_status": self.voice_status,
            "armed": self.drone.armed,
            "flight_time": self.drone.t,
            "motor": self.drone.motors,
            "mission_active": self.mission.active,
            "mission_paused": self.mission.paused,
            "mission_index": self.mission.index,
            "mission_total": len(self.mission.points) if self.mission.points else len(self.waypoints),
            "next_wp": nxt.name if nxt else "",
            "last_cmd": self.last_cmd,
            "last_voice_text": self.last_voice_text,
            "last_voice_action": self.last_voice_action,
            "voice_history": self.voice_history,
            "tts_enabled": self.tts.enabled if self.tts else False,
            "wind_mode": self.drone.wind_mode,
            "wind_speed": self.drone.wind_speed,
            "wind_heading": self.drone.wind_heading_deg,
            "current_wind_n": self.drone.current_wind_n,
            "current_wind_e": self.drone.current_wind_e,
            "orbit_active": self.orbit_active,
            "is_logging": self.logger.is_logging if self.logger else False,
            "log_samples": self.logger.sample_count if self.logger else 0,
            "is_analyzing": self.is_analyzing,
        }




def console_thread(q):
    while True:
        try:
            line = input("> ")
        except EOFError:
            return
        if line.strip():
            q.put(("text_cmd", {"text": line}))


def run_console_loop(drone, gcs, cmd_queue, steps_per_frame=3):
    target_phases = [
        ("TAKEOFF", lambda: gcs.handle("takeoff", {}), lambda d: d.est_alt > 3.5, 25.0),
        ("GO TO A", lambda: gcs.handle("goto", {"name": "A"}), lambda d: math.hypot(d.est_n - 20, d.est_e - 18) < 2.0, 60.0),
        ("GO TO B", lambda: gcs.handle("goto", {"name": "B"}), lambda d: math.hypot(d.est_n + 4, d.est_e - 42) < 2.0, 60.0),
        ("GO TO C", lambda: gcs.handle("goto", {"name": "C"}), lambda d: math.hypot(d.est_n + 26, d.est_e - 6) < 2.0, 60.0),
        ("GO TO D", lambda: gcs.handle("goto", {"name": "D"}), lambda d: math.hypot(d.est_n - 15, d.est_e + 25) < 2.0, 60.0),
        ("RETURN HOME", lambda: gcs.handle("rth", {}), lambda d: math.hypot(d.est_n, d.est_e) < 2.0, 60.0),
        ("LAND", lambda: gcs.handle("land", {}), lambda d: gcs.controller.mode == ctrl.STANDBY, 40.0),
    ]
    results = []
    phase_start = 0.0
    for name, fire, check, timeout in target_phases:
        fire()
        phase_start = drone.t
        ok = False
        while drone.t - phase_start < timeout:
            for _ in range(3):
                drone.step(config.DT)
                gcs.update(config.DT)
                gcs.controller.update(drone, config.DT)
            if not cmd_queue.empty():
                a, ar = cmd_queue.get()
                gcs.handle(a, ar)
            if check(drone):
                ok = True
                break
        results.append((name, ok, drone.t - phase_start))
        if not ok:
            break
    return results


def run_headless():
    drone = Drone()
    gcs = GCS(drone)
    gcs.failsafe_enabled = False
    gcs.tts = None  # Silent for automated testing
    drone.arm()
    gcs.home_pos = (0.0, 0.0)
    results = run_console_loop(drone, gcs, queue.Queue())
    all_ok = True
    for name, ok, dur in results:
        print("{:12s} {:5s}  {:.1f}s".format(name, "PASS" if ok else "FAIL", dur))
        all_ok = all_ok and ok
    print("FINAL POS n={:.2f} e={:.2f}  BATT {:.0f}%  SIM TIME {:.0f}s".format(
        drone.est_n, drone.est_e, drone.battery, drone.t))
    return 0 if all_ok else 1


def run_gui(drone, gcs, cmd_queue, use_voice, smoke=False, auto=False):
    try:
        from dashboard import Dashboard
    except Exception:
        print("pygame not available - falling back to console mode")
        return run_console()
    dash = Dashboard()
    running = True
    last = time.time()
    frame_count = 0
    auto_mission_at = None
    if auto:
        gcs.handle("takeoff", {})
        auto_mission_at = time.time() + 2.0
    if use_voice:
        v = None
        try:
            from voice import VoiceListener
            v = VoiceListener(cmd_queue, ready_cb=lambda: None)
            if v.start():
                gcs.voice_status = "voice listening..."
        except Exception:
            v = None
    while running:
        now = time.time()
        if auto_mission_at and now >= auto_mission_at:
            auto_mission_at = None
            gcs.handle("mission", {})
        frame_dt = min(0.033, max(0.001, now - last))
        last = now
        steps = min(6, max(1, int(frame_dt / config.DT)))
        for _ in range(steps):
            drone.step(config.DT)
            gcs.update(config.DT)
            gcs.controller.update(drone, config.DT)
        events = pygame_events()
        cmds = dash.handle_events(events)
        for c in cmds:
            if c[0] == "quit":
                running = False
            else:
                gcs.handle(c[0], c[1])
        while not cmd_queue.empty():
            kind, data = cmd_queue.get()
            if kind == "_voice":
                dash.trigger_voice_toast(data)
                gcs.handle("text_cmd", {"text": data})
            elif kind == "_voice_status":
                gcs.voice_status = data
                try:
                    from dashboard import set_voice_status
                    set_voice_status(data)
                except Exception:
                    pass
            elif kind == "text_cmd":
                gcs.handle(kind, data)
        gcs_data = gcs.gcs_data()
        gcs_data["voice_status"] = gcs.voice_status
        dash.draw(drone, gcs_data)
        dash.tick(60)
        frame_count += 1
        if smoke and frame_count >= 150:
            running = False
    if gcs.tts:
        gcs.tts.stop()
    pygame_quit()




def pygame_events():
    import pygame
    return pygame.event.get()


def pygame_quit():
    import pygame
    pygame.quit()


def run_console():
    drone = Drone()
    gcs = GCS(drone)
    q = queue.Queue()
    threading.Thread(target=console_thread, args=(q,), daemon=True).start()
    print("ESP32 GPS Waypoint Drone - console simulator")
    print("Commands: take off, land, return home, start mission, go to point a, forward, turn left, up, status, abort")
    last = time.time()
    while True:
        now = time.time()
        frame_dt = min(0.05, now - last)
        last = now
        steps = max(1, int(frame_dt / config.DT))
        for _ in range(steps):
            drone.step(config.DT)
            gcs.update(config.DT)
            gcs.controller.update(drone, config.DT)
        while not q.empty():
            kind, data = q.get()
            if kind == "text_cmd":
                gcs.handle("text_cmd", {"text": data})
        gcs_data = gcs.gcs_data()
        mode = gcs_data["mode"]
        line = "[{:.0f}s] mode={:<7s} pos=({:6.1f},{:6.1f}) alt={:5.1f} hdg={:5.1f} batt={:3.0f}%".format(
            drone.t, mode, drone.est_n, drone.est_e, drone.est_alt, drone.est_heading_deg, drone.battery)
        print(line, end="\r")
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description="ESP32 GPS Waypoint Drone - Python simulator")
    parser.add_argument("--headless", action="store_true", help="run automated mission test without GUI")
    parser.add_argument("--no-voice", action="store_true", help="disable voice recognition")
    parser.add_argument("--console", action="store_true", help="use console mode (no GUI)")
    parser.add_argument("--smoke", action="store_true", help="run GUI briefly for testing")
    parser.add_argument("--manual", action="store_true", help="start on the ground; do not auto-fly demo")
    args = parser.parse_args()

    if args.headless:
        rc = run_headless()
        raise SystemExit(rc)

    drone = Drone()
    gcs = GCS(drone)
    q = queue.Queue()

    if args.console:
        threading.Thread(target=console_thread, args=(q,), daemon=True).start()
        run_console()
        return

    run_gui(drone, gcs, q, not args.no_voice, smoke=args.smoke, auto=not args.manual)


if __name__ == "__main__":
    main()
