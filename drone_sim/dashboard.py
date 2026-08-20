import ctypes
import math
import random
import time
import pygame

import config
from map_tiles import MapTileManager

# Aeronautical Color Palette
COL_BG = (10, 13, 20)
COL_PANEL_DRONE = (18, 23, 35)
COL_PANEL_VOICE = (22, 28, 44)
COL_PANEL2 = (28, 36, 56)
COL_HEADER_VOICE = (38, 50, 78)
COL_HEADER_DRONE = (24, 38, 64)
COL_GRID = (36, 45, 68)
COL_TEXT = (235, 240, 250)
COL_DIM = (130, 142, 170)
COL_GREEN = (0, 230, 118)
COL_RED = (255, 23, 68)
COL_AMBER = (255, 214, 0)
COL_BLUE = (0, 229, 255)
COL_PURPLE = (179, 136, 255)
COL_HOME = (0, 230, 118)
COL_TRAIL = (64, 196, 255)
COL_SKY = (33, 150, 243)
COL_GROUND = (121, 85, 72)
COL_WIND = (180, 220, 255)

WINDOW_W = 1380
WINDOW_H = 780

_global_tile_mgr = None


def _dpi_aware():
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass


class Dashboard:
    def __init__(self):
        global _global_tile_mgr
        _dpi_aware()
        pygame.init()
        pygame.display.set_caption("ESP32-S3 LiteWing GPS Drone — Dual-Console GCS Simulator")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.clock = pygame.time.Clock()
        
        # Real-world Map Tile Engine (Satellite, Dark Tactical, OSM, Grid)
        self.tile_mgr = MapTileManager()
        _global_tile_mgr = self.tile_mgr
        
        # Fonts
        self.font_tiny = pygame.font.SysFont("consolas", 11)
        self.font = pygame.font.SysFont("consolas", 13)
        self.font_big = pygame.font.SysFont("consolas", 15, bold=True)
        self.font_btn = pygame.font.SysFont("segoeui", 11, bold=True)
        self.font_title = pygame.font.SysFont("segoeui", 13, bold=True)
        self.font_input = pygame.font.SysFont("consolas", 13)
        
        self.input_text = ""
        self.scale = 6.0
        
        # Layout Rectangles
        self.map_rect = pygame.Rect(12, 12, 850, 480)
        self.drone_panel_rect = pygame.Rect(874, 12, 494, 756)
        self.voice_panel_rect = pygame.Rect(12, 502, 850, 206)
        self.input_rect = pygame.Rect(12, 718, 850, 32)
        
        self.input_active = False
        self.tts_enabled = True
        self.last_voice_toast = ""
        self.voice_toast_time = 0.0
        
        # Interactive Selected Waypoint & Action Card
        self.selected_wp = None
        self.current_waypoints = []
        self.wp_card_buttons = []
        
        # Dynamic Wind Particles for Map Visualization
        self.wind_particles = [
            {
                "x": random.uniform(self.map_rect.left, self.map_rect.right),
                "y": random.uniform(self.map_rect.top, self.map_rect.bottom),
                "speed": random.uniform(0.7, 1.3),
                "len": random.randint(10, 24)
            }
            for _ in range(35)
        ]
        
        self.buttons = []
        self._build_buttons()

    def _build_buttons(self):
        bx, by = 470, 536
        cols, rows = 3, 4
        gap_x, gap_y = 6, 6
        bw = (380 - (cols - 1) * gap_x) / cols
        bh = (156 - (rows - 1) * gap_y) / rows
        
        labels = [
            ("TAKE OFF (T)", "takeoff"),
            ("LAND (L)", "land"),
            ("RETURN HOME (R)", "rth"),
            ("START MISSION (M)", "mission"),
            ("PAUSE/HOVER (P)", "pause"),
            ("ORBIT POI (O)", "orbit"),
            ("LOG 250Hz (U)", "toggle_logging"),
            ("SINDy/DMD (D)", "run_analysis"),
            ("MAP LAYER (K)", "cycle_map"),
            ("WIND: CALM", "cycle_wind"),
            ("WIND GUST (G)", "gust"),
            ("TTS AUDIO: ON", "toggle_tts"),
        ]
        self.buttons = []
        for i, (label, action) in enumerate(labels):
            col = i % cols
            row = i // cols
            rect = pygame.Rect(int(bx + col * (bw + gap_x)), int(by + row * (bh + gap_y)), int(bw), int(bh))
            self.buttons.append((rect, label, action))

    def _world_to_screen(self, n, e):
        cx = self.map_rect.x + self.map_rect.width / 2
        cy = self.map_rect.y + self.map_rect.height / 2
        x = cx + e * self.scale
        y = cy - n * self.scale
        return int(x), int(y)

    def _screen_to_world(self, x, y):
        cx = self.map_rect.x + self.map_rect.width / 2
        cy = self.map_rect.y + self.map_rect.height / 2
        e = (x - cx) / self.scale
        n = (cy - y) / self.scale
        return n, e

    def _draw_panels(self):
        # Map panel background
        pygame.draw.rect(self.screen, COL_PANEL_DRONE, self.map_rect, border_radius=6)
        
        # Section 2: Onboard Drone Panel
        pygame.draw.rect(self.screen, COL_PANEL_DRONE, self.drone_panel_rect, border_radius=6)
        pygame.draw.rect(self.screen, COL_GRID, self.drone_panel_rect, width=1, border_radius=6)
        
        # Section 1: Operator Voice Console Panel
        pygame.draw.rect(self.screen, COL_PANEL_VOICE, self.voice_panel_rect, border_radius=6)
        pygame.draw.rect(self.screen, COL_HEADER_VOICE, self.voice_panel_rect, width=1, border_radius=6)
        
        # Input bar
        pygame.draw.rect(self.screen, COL_PANEL2, self.input_rect, border_radius=6)

    def _draw_grid(self):
        step = 10.0
        while step * self.scale < 80.0:
            step *= 5.0
        base = int(step)
        span_n = self.map_rect.height / 2 / self.scale
        span_e = self.map_rect.width / 2 / self.scale
        n_start = -int(span_n / base) * base - base
        n_end = int(span_n / base) * base + base
        
        grid_col = (40, 52, 75) if self.tile_mgr.mode == "GRID" else (60, 75, 100)
        
        for n in range(n_start, n_end + 1, base):
            x0, y = self._world_to_screen(n, 0)
            pygame.draw.line(self.screen, grid_col, (self.map_rect.x, y), (self.map_rect.right, y), 1)
        
        e_start = -int(span_e / base) * base - base
        e_end = int(span_e / base) * base + base
        for e in range(e_start, e_end + 1, base):
            x, y0 = self._world_to_screen(0, e)
            pygame.draw.line(self.screen, grid_col, (x, self.map_rect.y), (x, self.map_rect.bottom), 1)
        
        self._text("▲ NORTH (+N)", (self.map_rect.x + 10, self.map_rect.y + 8), COL_DIM, self.font_tiny)
        self._text("EAST (+E) ►", (self.map_rect.right - 80, self.map_rect.y + 8), COL_DIM, self.font_tiny)
        
        for m in range(base, int(span_n) + base, base):
            x, y = self._world_to_screen(m, 0)
            self._text(f"{m}m", (x + 4, y - 6), COL_DIM, self.font_tiny)
        for m in range(base, int(span_e) + base, base):
            x, y = self._world_to_screen(0, m)
            self._text(f"{m}m", (x + 4, y + 4), COL_DIM, self.font_tiny)
        
        cx = self.map_rect.x + self.map_rect.width / 2
        cy = self.map_rect.y + self.map_rect.height / 2
        pygame.draw.circle(self.screen, COL_DIM, (int(cx), int(cy)), 3, 1)

    def _draw_wind_particles(self, gcs):
        wn = gcs.get("current_wind_n", 0.0)
        we = gcs.get("current_wind_e", 0.0)
        w_speed = math.hypot(wn, we)
        if w_speed < 0.5:
            return
        
        dx = we / w_speed
        dy = -wn / w_speed
        
        for p in self.wind_particles:
            step = w_speed * p["speed"] * 1.5
            p["x"] += dx * step
            p["y"] += dy * step
            
            if p["x"] < self.map_rect.left:
                p["x"] = self.map_rect.right
            elif p["x"] > self.map_rect.right:
                p["x"] = self.map_rect.left
            if p["y"] < self.map_rect.top:
                p["y"] = self.map_rect.bottom
            elif p["y"] > self.map_rect.bottom:
                p["y"] = self.map_rect.top
            
            p_len = p["len"] * min(2.0, max(0.6, w_speed / 5.0))
            x2 = p["x"] + dx * p_len
            y2 = p["y"] + dy * p_len
            
            col = (140, 190, 255)
            pygame.draw.line(self.screen, col, (int(p["x"]), int(p["y"])), (int(x2), int(y2)), 1)

    def _draw_wind_hud(self, gcs):
        wn = gcs.get("current_wind_n", 0.0)
        we = gcs.get("current_wind_e", 0.0)
        w_spd = math.hypot(wn, we)
        w_mode = gcs.get("wind_mode", "CALM")
        w_heading = (math.degrees(math.atan2(we, wn)) + 360.0) % 360.0
        
        bx = self.map_rect.right - 180
        by = self.map_rect.top + 34
        box = pygame.Rect(bx, by, 170, 32)
        pygame.draw.rect(self.screen, (12, 16, 26), box, border_radius=4)
        pygame.draw.rect(self.screen, COL_AMBER if w_mode in ("STRONG", "STORM") else COL_GRID, box, 1, border_radius=4)
        
        cx, cy = bx + 16, by + 16
        rad = math.radians(w_heading)
        nx = cx + 10 * math.sin(rad)
        ny = cy - 10 * math.cos(rad)
        pygame.draw.circle(self.screen, COL_PANEL2, (cx, cy), 12)
        pygame.draw.line(self.screen, COL_BLUE, (cx, cy), (int(nx), int(ny)), 2)
        
        col = COL_GREEN if w_mode == "CALM" else (COL_BLUE if w_mode == "LIGHT" else (COL_AMBER if w_mode == "STRONG" else COL_RED))
        self._text(f"WIND: {w_mode}", (bx + 34, by + 4), col, self.font_btn)
        self._text(f"{w_spd:.1f} m/s ({w_heading:.0f}°)", (bx + 34, by + 16), COL_DIM, self.font_tiny)

    def _draw_map_layer_badge(self, gcs=None):
        bx = self.map_rect.left + 10
        by = self.map_rect.bottom - 32
        box = pygame.Rect(bx, by, 130, 22)
        pygame.draw.rect(self.screen, (12, 16, 26), box, border_radius=4)
        pygame.draw.rect(self.screen, COL_GRID, box, 1, border_radius=4)
        self._text(f"🗺️ MAP: {self.tile_mgr.mode}", (bx + 6, by + 4), COL_BLUE, self.font_tiny)

        if gcs:
            is_log = gcs.get("is_logging", False)
            samples = gcs.get("log_samples", 0)
            is_analyzing = gcs.get("is_analyzing", False)

            # Telemetry 250 Hz Recording Badge
            if is_log:
                rec_bx = bx + 138
                rec_box = pygame.Rect(rec_bx, by, 156, 22)
                pygame.draw.rect(self.screen, (26, 12, 16), rec_box, border_radius=4)
                pygame.draw.rect(self.screen, COL_RED, rec_box, 1, border_radius=4)
                pulse = int(3 + 2 * math.sin(time.time() * 8))
                pygame.draw.circle(self.screen, COL_RED, (rec_bx + 10, by + 11), pulse)
                self._text(f"REC 250Hz: {samples:,} pts", (rec_bx + 18, by + 4), COL_RED, self.font_tiny)

            # Data-Driven Analysis in Progress Badge
            if is_analyzing:
                ana_bx = bx + (302 if is_log else 138)
                ana_box = pygame.Rect(ana_bx, by, 160, 22)
                pygame.draw.rect(self.screen, (28, 24, 10), ana_box, border_radius=4)
                pygame.draw.rect(self.screen, COL_AMBER, ana_box, 1, border_radius=4)
                self._text("⚙️ SINDy/DMDc FITTING...", (ana_bx + 6, by + 4), COL_AMBER, self.font_tiny)

    def _draw_trail(self, trail):
        if len(trail) < 2:
            return
        pts = [self._world_to_screen(n, e) for (n, e) in trail]
        pygame.draw.lines(self.screen, COL_TRAIL, False, pts, 2)

    def _draw_waypoints(self, gcs, drone=None):
        home = gcs.get("home_pos", (0.0, 0.0))
        hx, hy = self._world_to_screen(home[0], home[1])
        radius = gcs.get("geofence_radius", config.GEOFENCE_DEFAULT_RADIUS)
        
        # Geofence boundary circle
        pygame.draw.circle(self.screen, (70, 85, 120), (hx, hy), int(radius * self.scale), 1)
        pygame.draw.circle(self.screen, COL_HOME, (hx, hy), 8, 2)
        pygame.draw.circle(self.screen, COL_HOME, (hx, hy), 3)
        self._text("HOME BASE", (hx + 10, hy - 8), COL_HOME, self.font_tiny)
        
        wps = gcs.get("waypoints", [])
        self.current_waypoints = wps
        if len(wps) >= 1:
            route_pts = [self._world_to_screen(home[0], home[1])] + [self._world_to_screen(wp.n, wp.e) for wp in wps]
            for i in range(len(route_pts) - 1):
                p1, p2 = route_pts[i], route_pts[i+1]
                pygame.draw.line(self.screen, (90, 115, 160), p1, p2, 2)

        colors = {
            "A": (255, 64, 129),
            "B": (255, 214, 0),
            "C": (0, 229, 255),
            "HOME": COL_HOME,
        }
        for wp in wps:
            x, y = self._world_to_screen(wp.n, wp.e)
            col = colors.get(wp.name, (180, 180, 220))
            is_sel = (self.selected_wp == wp.name)
            
            if is_sel:
                # Pulsing selection ring & target reticle
                pulse = int(12 + 3 * math.sin(time.time() * 8))
                pygame.draw.circle(self.screen, COL_AMBER, (x, y), pulse, 2)
                pygame.draw.line(self.screen, COL_AMBER, (x - pulse - 4, y), (x - pulse + 4, y), 2)
                pygame.draw.line(self.screen, COL_AMBER, (x + pulse - 4, y), (x + pulse + 4, y), 2)
                pygame.draw.line(self.screen, COL_AMBER, (x, y - pulse - 4), (x, y - pulse + 4), 2)
                pygame.draw.line(self.screen, COL_AMBER, (x, y + pulse - 4), (x, y + pulse + 4), 2)

            pygame.draw.circle(self.screen, col, (x, y), 7 if is_sel else 6)
            pygame.draw.circle(self.screen, (15, 18, 26), (x, y), 3)
            self._text(f"{wp.name} ({wp.alt:.0f}m)", (x + 9, y - 9), COL_AMBER if is_sel else col, self.font_btn if is_sel else self.font_tiny)
        
        # Draw Orbit POI circle if active
        if gcs.get("orbit_active"):
            oc = gcs.get("orbit_center")
            if oc:
                ox, oy = self._world_to_screen(oc[0], oc[1])
                orb_r = int(15.0 * self.scale)
                pygame.draw.circle(self.screen, (170, 70, 230), (ox, oy), orb_r, 1)
                pygame.draw.circle(self.screen, (220, 100, 255), (ox, oy), 4)
                self._text("POI ORBIT CENTER", (ox + 8, oy + 6), (220, 100, 255), self.font_tiny)

        tp = gcs.get("target_pos")
        if tp:
            x, y = self._world_to_screen(tp[0], tp[1])
            pulse = int(5 + 2 * math.sin(time.time() * 6))
            pygame.draw.circle(self.screen, (255, 255, 255), (x, y), pulse, 2)
            pygame.draw.line(self.screen, (255, 255, 255), (x - 8, y), (x + 8, y), 1)
            pygame.draw.line(self.screen, (255, 255, 255), (x, y - 8), (x, y + 8), 1)

        # Floating Interactive Waypoint Card
        self.wp_card_buttons = []
        if self.selected_wp:
            target_wp = next((w for w in wps if w.name == self.selected_wp), None)
            if target_wp:
                dist = math.hypot(drone.est_n - target_wp.n, drone.est_e - target_wp.e) if drone else 0.0
                card_w, card_h = 430, 52
                card_x = self.map_rect.left + 14
                card_y = self.map_rect.top + 70
                card_rect = pygame.Rect(card_x, card_y, card_w, card_h)
                pygame.draw.rect(self.screen, (12, 16, 26), card_rect, border_radius=6)
                pygame.draw.rect(self.screen, COL_AMBER, card_rect, 1, border_radius=6)
                
                self._text(f"📍 POINT {target_wp.name} ({target_wp.alt:.0f}m)  |  DIST TO DRONE: {dist:.1f}m", (card_x + 10, card_y + 5), COL_AMBER, self.font_btn)
                
                bx = card_x + 8
                by = card_y + 24
                bh = 22
                gap = 6
                
                btns = [
                    ("✈️ FLY TO", "goto", target_wp.name, (20, 60, 110), 76),
                    ("🛑 HOVER", "hover_at", target_wp.name, (20, 90, 50), 74),
                    ("🔄 ORBIT", "orbit", target_wp.name, (70, 30, 90), 74),
                ]
                if target_wp.name.startswith("P"):
                    btns.append(("🗑️ DEL", "delete_wp", target_wp.name, (100, 30, 30), 62))
                    btns.append(("✖️", "close_wp", target_wp.name, (40, 50, 70), 32))
                else:
                    btns.append(("✖️ CLOSE", "close_wp", target_wp.name, (40, 50, 70), 74))
                    
                for label, act, name, col, bw in btns:
                    b_rect = pygame.Rect(bx, by, bw, bh)
                    self.wp_card_buttons.append((b_rect, act, name))
                    pygame.draw.rect(self.screen, col, b_rect, border_radius=3)
                    pygame.draw.rect(self.screen, COL_GRID, b_rect, 1, border_radius=3)
                    surf = self.font_tiny.render(label, True, (255, 255, 255))
                    self.screen.blit(surf, (b_rect.x + (b_rect.width - surf.get_width()) // 2, b_rect.y + 4))
                    bx += bw + gap

    def _draw_drone(self, drone):
        x, y = self._world_to_screen(drone.est_n, drone.est_e)
        heading = math.radians(drone.est_heading_deg)
        
        spd = math.hypot(drone.vel_n, drone.vel_e)
        if spd > 0.1:
            vx = x + math.sin(heading) * spd * 10
            vy = y - math.cos(heading) * spd * 10
            pygame.draw.line(self.screen, COL_AMBER, (x, y), (int(vx), int(vy)), 2)

        pts = []
        for lx, ly in ((15, 0), (-10, -9), (-5, 0), (-10, 9)):
            rx = lx * math.sin(heading) + ly * math.cos(heading)
            ry = -lx * math.cos(heading) + ly * math.sin(heading)
            pts.append((x + rx, y + ry))
        
        pygame.draw.polygon(self.screen, COL_BLUE, pts)
        pygame.draw.polygon(self.screen, (255, 255, 255), pts, 1)
        pygame.draw.circle(self.screen, (255, 255, 255), (x, y), 2)

    def _text(self, text, pos, color=COL_TEXT, font=None):
        f = font or self.font
        surf = f.render(text, True, color)
        self.screen.blit(surf, pos)

    def _draw_flight_banner(self, drone, gcs):
        mode = gcs.get("mode", "STANDBY")
        armed = gcs.get("armed", False)
        if not armed or mode == "STANDBY":
            text, col = "STANDBY (ON GROUND) — PRESS TAKE OFF", COL_AMBER
        elif mode == "LANDING":
            text, col = "AUTONOMOUS LANDING...", COL_AMBER
        elif mode == "RTH":
            text, col = "RETURN TO HOME (RTH) ACTIVE", COL_AMBER
        elif mode == "MANUAL":
            text, col = "MANUAL FLIGHT MODE", COL_GREEN
        elif mode == "GPS":
            if gcs.get("orbit_active", False):
                text, col = "POI ORBIT / LOITER MODE ACTIVE", COL_PURPLE
            else:
                text, col = "GPS WAYPOINT MISSION ACTIVE", COL_BLUE
        else:
            text, col = f"MODE: {mode}", COL_GREEN
        
        if not gcs.get("link_ok", True):
            text, col = "COMMUNICATION LOST — FAILSAFE ACTIVE", COL_RED
        
        dist_h = math.hypot(drone.est_n - gcs.get("home_pos", (0,0))[0], drone.est_e - gcs.get("home_pos", (0,0))[1])
        msg = f"{text}  |  ALT: {drone.est_alt:.1f}m  |  DIST HOME: {dist_h:.1f}m"
        surf = self.font_big.render(msg, True, col)
        rect = surf.get_rect(center=(self.map_rect.centerx - 80, self.map_rect.y + 22))
        box = rect.inflate(20, 10)
        pygame.draw.rect(self.screen, (10, 12, 18), box, border_radius=4)
        pygame.draw.rect(self.screen, col, box, 1, border_radius=4)
        self.screen.blit(surf, rect)

    def _draw_artificial_horizon(self, drone, cx, cy, radius=40):
        size = radius * 2
        pfd = pygame.Surface((size, size))
        pfd.fill(COL_PANEL2)
        
        roll = drone.roll
        pitch = drone.pitch
        
        pygame.draw.rect(pfd, COL_SKY, (0, 0, size, size))
        
        p_offset = max(-radius + 5, min(radius - 5, math.degrees(pitch) * 1.2))
        angle = -roll
        dx = radius * 1.5 * math.cos(angle)
        dy = radius * 1.5 * math.sin(angle)
        
        mx, my = radius, radius
        ground_poly = [
            (mx - dx, my - dy - p_offset),
            (mx + dx, my + dy - p_offset),
            (size + 20, size + 20),
            (-20, size + 20)
        ]
        pygame.draw.polygon(pfd, COL_GROUND, ground_poly)
        pygame.draw.line(pfd, (255, 255, 255), (mx - dx, my - dy - p_offset), (mx + dx, my + dy - p_offset), 2)
        
        mask = pygame.Surface((size, size), pygame.SRCALPHA)
        mask.fill((0, 0, 0, 255))
        pygame.draw.circle(mask, (0, 0, 0, 0), (radius, radius), radius)
        pfd.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_SUB)
        
        self.screen.blit(pfd, (cx - radius, cy - radius))
        pygame.draw.circle(self.screen, COL_DIM, (cx, cy), radius, 1)
        pygame.draw.line(self.screen, COL_AMBER, (cx - 14, cy), (cx - 4, cy), 2)
        pygame.draw.line(self.screen, COL_AMBER, (cx + 4, cy), (cx + 14, cy), 2)
        pygame.draw.circle(self.screen, COL_AMBER, (cx, cy), 2)
        self._text("ATTITUDE", (cx - 24, cy + radius + 3), COL_DIM, self.font_tiny)

    def _draw_compass_rose(self, drone, gcs, cx, cy, radius=40):
        pygame.draw.circle(self.screen, COL_PANEL2, (cx, cy), radius)
        pygame.draw.circle(self.screen, COL_GRID, (cx, cy), radius, 1)
        
        hdg_rad = math.radians(drone.est_heading_deg)
        cardinals = [("N", 0), ("E", 90), ("S", 180), ("W", 270)]
        for label, deg in cardinals:
            rad = math.radians(deg)
            tx = cx + (radius - 10) * math.sin(rad)
            ty = cy - (radius - 10) * math.cos(rad)
            col = COL_RED if label == "N" else COL_DIM
            self._text(label, (tx - 4, ty - 5), col, self.font_tiny)
        
        nx = cx + (radius - 5) * math.sin(hdg_rad)
        ny = cy - (radius - 5) * math.cos(hdg_rad)
        pygame.draw.line(self.screen, COL_BLUE, (cx, cy), (nx, ny), 2)
        pygame.draw.circle(self.screen, COL_BLUE, (int(nx), int(ny)), 3)
        
        tp = gcs.get("target_pos")
        if tp:
            bearing = math.degrees(math.atan2(tp[1] - drone.est_e, tp[0] - drone.est_n)) % 360.0
            b_rad = math.radians(bearing)
            bx = cx + (radius - 5) * math.sin(b_rad)
            by = cy - (radius - 5) * math.cos(b_rad)
            pygame.draw.line(self.screen, COL_AMBER, (cx, cy), (bx, by), 1)
            pygame.draw.circle(self.screen, COL_AMBER, (int(bx), int(by)), 4, 1)
        
        pygame.draw.circle(self.screen, COL_TEXT, (cx, cy), 2)
        self._text(f"{drone.est_heading_deg:.0f}°", (cx - 10, cy + radius + 3), COL_TEXT, self.font_tiny)

    # -------------------------------------------------------------
    # SECTION 2: ONBOARD DRONE FLIGHT & TELEMETRY SYSTEMS
    # -------------------------------------------------------------
    def _draw_drone_section(self, drone, gcs):
        px = self.drone_panel_rect.x + 14
        y = self.drone_panel_rect.y + 10
        
        # Section 2 Header
        header_rect = pygame.Rect(self.drone_panel_rect.x + 8, y - 2, self.drone_panel_rect.width - 16, 26)
        pygame.draw.rect(self.screen, COL_HEADER_DRONE, header_rect, border_radius=4)
        self._text("🚁 SECTION 2: ONBOARD DRONE FLIGHT & TELEMETRY", (px, y + 2), COL_BLUE, self.font_title)
        y += 34
        
        # PFD Instruments
        self._draw_artificial_horizon(drone, px + 95, y + 45, radius=40)
        self._draw_compass_rose(drone, gcs, px + 285, y + 45, radius=40)
        y += 105
        
        mode = gcs.get("mode", "STANDBY")
        link = gcs.get("link_ok", True)
        mode_col = COL_GREEN if mode in ("GPS", "RTH") else (COL_AMBER if mode == "MANUAL" else COL_DIM)
        
        mins, secs = divmod(int(gcs.get("flight_time", 0.0)), 60)
        
        # Relative Airspeed Calculation
        wn = gcs.get("current_wind_n", 0.0)
        we = gcs.get("current_wind_e", 0.0)
        v_air = math.hypot(drone.vel_n - wn, drone.vel_e - we)
        v_gnd = math.hypot(drone.vel_n, drone.vel_e)
        w_mode = gcs.get("wind_mode", "CALM")
        
        col1_items = [
            ("Flight Mode", mode, mode_col),
            ("Flight Time", f"{mins:02d}:{secs:02d}", COL_TEXT),
            ("RF Link", "CONNECTED" if link else "LOST", COL_GREEN if link else COL_RED),
            ("GPS Fix", f"{drone.sat} Sats (3D)", COL_GREEN if drone.gps_fix else COL_AMBER),
            ("Latitude", f"{drone.gps_lat:.6f}", COL_TEXT),
            ("Longitude", f"{drone.gps_lon:.6f}", COL_TEXT),
            ("Roll / Pitch", f"{math.degrees(drone.roll):.1f}° / {math.degrees(drone.pitch):.1f}°", COL_TEXT),
        ]
        
        tp = gcs.get("target_pos")
        if tp:
            dist = math.hypot(tp[0] - drone.est_n, tp[1] - drone.est_e)
            bearing = math.degrees(math.atan2(tp[1] - drone.est_e, tp[0] - drone.est_n)) % 360.0
            err = (bearing - drone.est_heading_deg + 180.0) % 360.0 - 180.0
            tgt_str = f"{dist:.1f} m"
            brg_str = f"{bearing:.0f}°"
            err_str = f"{err:+.0f}°"
        else:
            tgt_str, brg_str, err_str = "—", "—", "—"
            
        col2_items = [
            ("Altitude", f"{drone.est_alt:.2f} m", COL_TEXT),
            ("Target Alt", f"{gcs.get('target_alt', 0.0):.1f} m", COL_AMBER),
            ("Airspeed", f"{v_air:.2f} m/s", COL_TEXT),
            ("Ground Speed", f"{v_gnd:.2f} m/s", COL_TEXT),
            ("Wind Disturbance", f"{w_mode} ({math.hypot(wn,we):.1f}m/s)", COL_AMBER if w_mode in ("STRONG", "STORM") else COL_TEXT),
            ("Target Bearing", brg_str, COL_TEXT),
            ("Heading Error", err_str, COL_GREEN if err_str != "—" and abs(float(err_str.replace('°','').replace('+',''))) < 10 else COL_AMBER),
        ]
        
        start_y = y
        for label, val, c in col1_items:
            self._text(f"{label:<12}:", (px, y), COL_DIM, self.font_tiny)
            self._text(f"{val}", (px + 95, y), c, self.font)
            y += 17
            
        y = start_y
        for label, val, c in col2_items:
            self._text(f"{label:<16}:", (px + 225, y), COL_DIM, self.font_tiny)
            self._text(f"{val}", (px + 348, y), c, self.font)
            y += 17
            
        y += 12
        # Battery & Altitude Gauges
        self._draw_battery(drone, px, y)
        y += 38
        self._draw_altitude_bar(drone, gcs, px, y)
        y += 34
        self._draw_quad_motors(drone, gcs, px, y)
        y += 92
        self._draw_mission_progress(gcs, px, y)
        y += 36
        self._draw_log(gcs, px, y)

    def _draw_battery(self, drone, px, y):
        w, h = 230, 16
        pygame.draw.rect(self.screen, COL_PANEL2, (px, y, w, h), border_radius=3)
        frac = max(0.0, min(1.0, drone.battery / config.BATTERY_CAPACITY))
        col = COL_GREEN if frac > 0.4 else (COL_AMBER if frac > 0.2 else COL_RED)
        if frac > 0:
            pygame.draw.rect(self.screen, col, (px + 1, y + 1, int((w - 2) * frac), h - 2), border_radius=3)
        voltage = 3.3 + 0.9 * frac
        self._text(f"Battery: {drone.battery:.0f}% ({voltage:.2f}V)", (px + w + 10, y), col, self.font)

    def _draw_altitude_bar(self, drone, gcs, px, y):
        w, h = 230, 14
        pygame.draw.rect(self.screen, COL_PANEL2, (px, y, w, h), border_radius=3)
        max_alt = max(15.0, config.GEOFENCE_MAX_ALT)
        cur = int(min(1.0, max(0.0, drone.est_alt / max_alt)) * (w - 2))
        tgt = int(min(1.0, max(0.0, gcs.get("target_alt", 0.0) / max_alt)) * (w - 2))
        pygame.draw.rect(self.screen, COL_BLUE, (px + 1, y + 1, cur, h - 2), border_radius=2)
        if tgt > 0:
            pygame.draw.line(self.screen, COL_AMBER, (px + 1 + tgt, y - 2), (px + 1 + tgt, y + h + 2), 2)
        self._text(f"Alt: {drone.est_alt:.1f}m / {gcs.get('target_alt', 0.0):.1f}m", (px + w + 10, y), COL_TEXT, self.font)

    def _draw_quad_motors(self, drone, gcs, px, y):
        self._text("ONBOARD MOTOR MIXER (X-CONFIG)", (px, y), COL_BLUE, self.font_tiny)
        y += 16
        motors = gcs.get("motor", [0.0, 0.0, 0.0, 0.0])
        
        motor_names = [
            ("M1 (FL/CW)", motors[0], px, y),
            ("M2 (FR/CCW)", motors[1], px + 230, y),
            ("M4 (RL/CCW)", motors[3], px, y + 26),
            ("M3 (RR/CW)", motors[2], px + 230, y + 26),
        ]
        
        for name, val, mx, my in motor_names:
            col = COL_GREEN if val > 0.05 else COL_DIM
            w = 120
            pygame.draw.rect(self.screen, COL_PANEL2, (mx, my + 10, w, 8), border_radius=2)
            pygame.draw.rect(self.screen, col, (mx + 1, my + 11, int((w - 2) * max(0.0, min(1.0, val))), 6), border_radius=2)
            self._text(f"{name}: {val*100.0:.0f}%", (mx, my - 2), COL_TEXT, self.font_tiny)

    def _draw_mission_progress(self, gcs, px, y):
        mode = gcs.get("mode", "STANDBY")
        miss = gcs.get("mission_active", False)
        orbit = gcs.get("orbit_active", False)
        if mode == "RTH":
            self._text("MISSION: OVERRIDDEN BY RTH (RETURNING HOME)", (px, y), COL_AMBER, self.font_big)
        elif orbit:
            self._text("MISSION: POI ORBIT MODE (CIRCLING)", (px, y), COL_PURPLE, self.font_big)
        elif miss:
            idx = gcs.get("mission_index", 0) + 1
            total = gcs.get("mission_total", 0)
            nxt = gcs.get("next_wp", "")
            paused = gcs.get("mission_paused", False)
            status = "PAUSED (HOLD)" if paused else f"FLYING TO {nxt}"
            col = COL_AMBER if paused else COL_GREEN
            self._text(f"MISSION: WP {idx}/{total} — {status}", (px, y), col, self.font_big)
        else:
            self._text("MISSION: IDLE (STANDBY)", (px, y), COL_DIM, self.font)

    def _draw_log(self, gcs, px, y):
        self._text("DRONE SYSTEM & SENSOR EVENTS", (px, y), COL_BLUE, self.font_tiny)
        y += 16
        for line in gcs.get("log", [])[-5:]:
            self._text(line[:54], (px, y), COL_DIM, self.font_tiny)
            y += 14

    # -------------------------------------------------------------
    # SECTION 1: OPERATOR VOICE COMMAND CONSOLE (MY VOICE)
    # -------------------------------------------------------------
    def _draw_voice_section(self, gcs):
        px = self.voice_panel_rect.x + 14
        y = self.voice_panel_rect.y + 10
        
        # Section 1 Header Banner
        header_rect = pygame.Rect(self.voice_panel_rect.x + 8, y - 2, self.voice_panel_rect.width - 16, 26)
        pygame.draw.rect(self.screen, COL_HEADER_VOICE, header_rect, border_radius=4)
        self._text("🎙️ SECTION 1: OPERATOR VOICE CONSOLE (MY VOICE & GCS)", (px, y + 2), COL_AMBER, self.font_title)
        
        # Animated Mic Pulsing Indicator
        mic_pulse = int(4 + 3 * math.sin(time.time() * 8))
        pygame.draw.circle(self.screen, COL_GREEN, (self.voice_panel_rect.right - 28, y + 10), mic_pulse)
        pygame.draw.circle(self.screen, (255, 255, 255), (self.voice_panel_rect.right - 28, y + 10), 2)
        y += 34
        
        # Left Column: Voice recognition stream & Intent
        mic_status = gcs_voice_status()
        self._text(f"Microphone Status : {mic_status}", (px, y), COL_GREEN if "active" in mic_status or "listen" in mic_status else COL_AMBER, self.font_tiny)
        y += 18
        
        last_voice = gcs.get("last_voice_text", "")
        if not last_voice:
            last_voice = "Ready for voice command (Speak into mic)..."
        self._text("YOU SAID (RAW ASR) :", (px, y), COL_DIM, self.font_tiny)
        y += 14
        
        # Glowing Voice Box
        voice_box = pygame.Rect(px, y, 440, 24)
        pygame.draw.rect(self.screen, COL_PANEL2, voice_box, border_radius=3)
        pygame.draw.rect(self.screen, COL_AMBER if gcs.get("last_voice_text") else COL_GRID, voice_box, 1, border_radius=3)
        self._text(f"\"{last_voice}\"", (px + 8, y + 4), COL_AMBER if gcs.get("last_voice_text") else COL_DIM, self.font)
        y += 30
        
        # Parsed Intent
        act = gcs.get("last_voice_action", "STANDBY")
        self._text(f"FLIGHT ACTION     : [ {act} ]", (px, y), COL_GREEN if act != "UNRECOGNIZED" else COL_RED, self.font_big)
        y += 24
        
        # Recent Voice Command History
        self._text("VOICE COMMAND HISTORY:", (px, y), COL_BLUE, self.font_tiny)
        y += 14
        history = gcs.get("voice_history", [])[-2:]
        if history:
            for stamp, txt, a in history:
                self._text(f"[{stamp}] Spoke: \"{txt}\" -> {a}", (px, y), COL_TEXT, self.font_tiny)
                y += 14
        else:
            self._text("Say: 'take off', 'go to point a', 'satellite map', 'wind strong', 'orbit'", (px, y), COL_DIM, self.font_tiny)
        
        # Draw Quick Push Buttons on the right side of the Voice Console
        self._draw_buttons(gcs)

    def _draw_input(self):
        prompt = "GCS COMMAND > " + self.input_text
        surf = self.font_input.render(prompt + ("|" if self.input_active else ""), True, COL_TEXT)
        self.screen.blit(surf, (self.input_rect.x + 10, self.input_rect.y + 7))
        pygame.draw.rect(self.screen, COL_BLUE if self.input_active else COL_DIM, self.input_rect, 1, border_radius=4)
        
        status_line = (
            "Click Map: Add Waypoint  |  Hotkeys: [T]akeoff [L]and [R]TH [M]ission [O]rbit [K]Map Layer [W]ind [G]ust  |  "
            "Mic Active"
        )
        self._text(status_line, (14, WINDOW_H - 22), COL_DIM, self.font_tiny)

    def _draw_buttons(self, gcs):
        w_mode = gcs.get("wind_mode", "CALM")
        is_orbit = gcs.get("orbit_active", False)
        
        for rect, label, action in self.buttons:
            btn_label = label
            btn_col = COL_PANEL2
            border_col = COL_GRID
            
            if action == "toggle_tts":
                btn_label = "TTS AUDIO: ON" if self.tts_enabled else "TTS AUDIO: OFF"
            elif action == "cycle_wind":
                btn_label = f"WIND: {w_mode}"
                if w_mode in ("STRONG", "STORM"):
                    btn_col = (45, 30, 40)
            elif action == "cycle_map":
                btn_label = f"MAP: {self.tile_mgr.mode}"
                if self.tile_mgr.mode == "SATELLITE":
                    btn_col = (30, 45, 65)
            elif action == "orbit":
                if is_orbit:
                    btn_label = "STOP ORBIT (O)"
                    btn_col = (90, 25, 65)
                    border_col = COL_AMBER
                else:
                    if self.selected_wp:
                        btn_label = f"ORBIT {self.selected_wp} (O)"
                    else:
                        btn_label = "ORBIT POI (O)"
            elif action == "pause":
                if is_orbit:
                    btn_label = "STOP/HOVER (P)"
            elif action == "goto_a":
                if self.selected_wp:
                    btn_label = f"FLY TO {self.selected_wp}"
                    btn_col = (20, 50, 90)
                else:
                    btn_label = "GOTO POINT A"
            elif action == "goto_b":
                if self.selected_wp:
                    btn_label = f"HOVER {self.selected_wp}"
                    btn_col = (20, 75, 45)
                else:
                    btn_label = "GOTO POINT B"
            elif action == "goto_c":
                if self.selected_wp:
                    btn_label = f"DELETE {self.selected_wp}" if self.selected_wp.startswith("P") else "DESELECT"
                    btn_col = (85, 30, 30) if self.selected_wp.startswith("P") else (40, 50, 70)
                else:
                    btn_label = "GOTO POINT C"
                
            pygame.draw.rect(self.screen, btn_col, rect, border_radius=4)
            pygame.draw.rect(self.screen, border_col, rect, 1, border_radius=4)
            surf = self.font_btn.render(btn_label, True, COL_TEXT)
            self.screen.blit(surf, (rect.x + (rect.width - surf.get_width()) / 2, rect.y + (rect.height - surf.get_height()) / 2))

    def trigger_voice_toast(self, text):
        self.last_voice_toast = text
        self.voice_toast_time = time.time()

    def _draw_voice_toast(self):
        if self.last_voice_toast and time.time() - self.voice_toast_time < 3.5:
            msg = f"🎤 VOICE RECOGNIZED: \"{self.last_voice_toast}\""
            surf = self.font_big.render(msg, True, COL_AMBER)
            rect = surf.get_rect(center=(self.map_rect.centerx - 80, self.map_rect.bottom - 24))
            box = rect.inflate(24, 10)
            pygame.draw.rect(self.screen, (10, 14, 22), box, border_radius=4)
            pygame.draw.rect(self.screen, COL_AMBER, box, 1, border_radius=4)
            self.screen.blit(surf, rect)

    def handle_events(self, events):
        cmds = []
        for ev in events:
            if ev.type == pygame.QUIT:
                cmds.append(("quit", {}))
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == 1:
                    # 1. Check Floating Waypoint Action Card buttons first
                    clicked_card = False
                    for b_rect, act, name in getattr(self, "wp_card_buttons", []):
                        if b_rect.collidepoint(ev.pos):
                            clicked_card = True
                            if act == "goto":
                                cmds.append(("goto", {"name": name}))
                            elif act == "hover_at":
                                cmds.append(("hover_at", {"name": name}))
                            elif act == "orbit":
                                cmds.append(("orbit", {"name": name}))
                            elif act == "delete_wp":
                                cmds.append(("delete_wp", {"name": name}))
                                self.selected_wp = None
                            elif act == "close_wp":
                                self.selected_wp = None
                            break
                    
                    if not clicked_card:
                        if self.map_rect.collidepoint(ev.pos):
                            # Check if clicking on an existing waypoint
                            clicked_existing = None
                            for wp in getattr(self, "current_waypoints", []):
                                wx, wy = self._world_to_screen(wp.n, wp.e)
                                if math.hypot(ev.pos[0] - wx, ev.pos[1] - wy) < 18:
                                    clicked_existing = wp
                                    break
                            
                            if clicked_existing:
                                self.selected_wp = clicked_existing.name
                                self.trigger_voice_toast(f"Selected Point {clicked_existing.name}")
                            else:
                                n, e = self._screen_to_world(ev.pos[0], ev.pos[1])
                                cmds.append(("map_point", {"n": n, "e": e, "alt": None}))
                                new_num = len([w for w in self.current_waypoints if w.name.startswith("P")]) + 1
                                self.selected_wp = f"P{new_num}"
                        elif self.input_rect.collidepoint(ev.pos):
                            self.input_active = True
                        else:
                            self.input_active = False
                            for rect, label, action in self.buttons:
                                if rect.collidepoint(ev.pos):
                                    if action == "toggle_tts":
                                        self.tts_enabled = not self.tts_enabled
                                        cmds.append(("toggle_tts", {"enabled": self.tts_enabled}))
                                    elif action == "goto_a":
                                        if self.selected_wp:
                                            cmds.append(("goto", {"name": self.selected_wp}))
                                        else:
                                            cmds.append(("goto", {"name": "A"}))
                                    elif action == "goto_b":
                                        if self.selected_wp:
                                            cmds.append(("hover_at", {"name": self.selected_wp}))
                                        else:
                                            cmds.append(("goto", {"name": "B"}))
                                    elif action == "goto_c":
                                        if self.selected_wp:
                                            if self.selected_wp.startswith("P"):
                                                cmds.append(("delete_wp", {"name": self.selected_wp}))
                                            self.selected_wp = None
                                        else:
                                            cmds.append(("goto", {"name": "C"}))
                                    elif action == "cycle_wind":
                                        cmds.append(("cycle_wind", {}))
                                    elif action == "cycle_map":
                                        cmds.append(("cycle_map", {}))
                                    elif action == "orbit":
                                        if self.selected_wp:
                                            cmds.append(("orbit", {"name": self.selected_wp}))
                                        else:
                                            cmds.append(("orbit", {}))
                                    else:
                                        cmds.append((action, {}))
                elif ev.button == 4:  # Scroll Up (Zoom In)
                    self.scale = min(40.0, self.scale * 1.2)
                elif ev.button == 5:  # Scroll Down (Zoom Out)
                    self.scale = max(0.8, self.scale / 1.2)
            elif ev.type == pygame.KEYDOWN:
                if self.input_active:
                    if ev.key == pygame.K_RETURN:
                        if self.input_text.strip():
                            cmds.append(("text_cmd", {"text": self.input_text}))
                        self.input_text = ""
                    elif ev.key == pygame.K_BACKSPACE:
                        self.input_text = self.input_text[:-1]
                    elif ev.key == pygame.K_ESCAPE:
                        self.input_active = False
                    elif ev.unicode and ev.unicode.isprintable():
                        self.input_text += ev.unicode
                else:
                    if ev.key == pygame.K_t:
                        cmds.append(("takeoff", {}))
                    elif ev.key == pygame.K_l:
                        cmds.append(("land", {}))
                    elif ev.key == pygame.K_r:
                        cmds.append(("rth", {}))
                    elif ev.key == pygame.K_m:
                        cmds.append(("mission", {}))
                    elif ev.key == pygame.K_p:
                        cmds.append(("pause", {}))
                    elif ev.key == pygame.K_o:
                        cmds.append(("orbit", {"name": self.selected_wp} if self.selected_wp else {}))
                    elif ev.key == pygame.K_u:
                        cmds.append(("toggle_logging", {}))
                    elif ev.key == pygame.K_d:
                        cmds.append(("run_analysis", {}))
                    elif ev.key == pygame.K_k:
                        cmds.append(("cycle_map", {}))
                    elif ev.key == pygame.K_w:
                        cmds.append(("cycle_wind", {}))
                    elif ev.key == pygame.K_g:
                        cmds.append(("gust", {"strength": config.WIND_GUST_BURST}))
                    elif ev.key == pygame.K_ESCAPE:
                        self.selected_wp = None
                        cmds.append(("abort", {}))
                    elif ev.key in (pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.scale = min(40.0, self.scale * 1.2)
                    elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.scale = max(0.8, self.scale / 1.2)
        return cmds

    def draw(self, drone, gcs):
        self.screen.fill(COL_BG)
        self._draw_panels()
        
        # 1. Clip strictly to Map Rectangle
        self.screen.set_clip(self.map_rect)
        
        # Render real-world map tiles (Satellite / Dark Tactical / OSM / Grid)
        self.tile_mgr.draw_map(self.screen, self.map_rect, config.HOME_LAT, config.HOME_LON, self.scale)
        
        self._draw_grid()
        self._draw_wind_particles(gcs)
        self._draw_trail(gcs.get("trail", []))
        self._draw_waypoints(gcs, drone)
        self._draw_drone(drone)
        self._draw_map_layer_badge(gcs)
        self.screen.set_clip(None)
        
        # 2. Draw Top Flight Banner & Wind HUD
        self._draw_flight_banner(drone, gcs)
        self._draw_wind_hud(gcs)
        
        # 3. SECTION 1: Operator Voice Console (My Voice)
        self._draw_voice_section(gcs)
        
        # 4. SECTION 2: Onboard Drone Flight & Telemetry (The Drone)
        self._draw_drone_section(drone, gcs)
        
        # 5. Input Prompt & Toasts
        self._draw_input()
        self._draw_voice_toast()
        pygame.display.flip()

    def tick(self, fps=60):
        self.clock.tick(fps)


_voice_status_holder = {"text": "voice active"}


def set_voice_status(text):
    _voice_status_holder["text"] = text


def gcs_voice_status():
    return _voice_status_holder["text"]


def set_map_mode(mode):
    global _global_tile_mgr
    if _global_tile_mgr:
        _global_tile_mgr.set_mode(mode)


def cycle_map_mode():
    global _global_tile_mgr
    if _global_tile_mgr:
        return _global_tile_mgr.cycle_mode()
    return "DARK"
