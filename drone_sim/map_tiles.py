import io
import math
import os
import threading
import urllib.request
import pygame

import config

TILE_SIZE = 256
CACHE_DIR = os.path.join(os.path.dirname(__file__), "..", "tile_cache")

# Tile Server URLs
SERVERS = {
    "DARK": "https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png",
    "SATELLITE": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}.jpg",
    "STREET": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
}

USER_AGENT = "LiteWingDroneSim/1.0 (Python Pygame GCS)"


def latlon_to_xy(lat, lon, zoom):
    n = 2.0 ** zoom
    rad_lat = math.radians(lat)
    x = ((lon + 180.0) / 360.0) * n
    y = (1.0 - math.asinh(math.tan(rad_lat)) / math.pi) / 2.0 * n
    return x, y


def xy_to_latlon(x, y, zoom):
    n = 2.0 ** zoom
    lon_deg = x / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n)))
    lat_deg = math.degrees(lat_rad)
    return lat_deg, lon_deg


class MapTileManager:
    def __init__(self):
        self.mode = "GRID"  # "GRID", "DARK", "SATELLITE", "STREET"
        self.modes = ["GRID", "DARK", "SATELLITE", "STREET"]
        self.cache = {}  # (server, z, x, y) -> pygame.Surface
        self.pending = set()
        self.lock = threading.Lock()
        
        os.makedirs(CACHE_DIR, exist_ok=True)
        self.procedural_surface = self._create_procedural_terrain()

    def _create_procedural_terrain(self):
        surf = pygame.Surface((TILE_SIZE, TILE_SIZE))
        surf.fill((16, 22, 34))
        pygame.draw.rect(surf, (22, 30, 46), (0, 0, TILE_SIZE, TILE_SIZE), 1)
        return surf

    def cycle_mode(self):
        idx = (self.modes.index(self.mode) + 1) % len(self.modes)
        self.mode = self.modes[idx]
        return self.mode

    def set_mode(self, mode):
        m = mode.upper()
        if m in self.modes:
            self.mode = m

    def _fetch_tile_worker(self, server_key, z, x, y, url, file_path):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=4.0) as resp:
                data = resp.read()
                
            with open(file_path, "wb") as f:
                f.write(data)
                
            raw_surf = pygame.image.load(io.BytesIO(data))
            # Ensure surface is 24/32-bit for scaling compatibility
            surf = raw_surf.convert_alpha() if pygame.display.get_surface() else raw_surf
            with self.lock:
                self.cache[(server_key, z, x, y)] = surf
                self.pending.discard((server_key, z, x, y))
        except Exception:
            with self.lock:
                self.pending.discard((server_key, z, x, y))

    def get_tile(self, server_key, z, x, y):
        if (server_key, z, x, y) in self.cache:
            return self.cache[(server_key, z, x, y)]
            
        file_name = f"{server_key}_{z}_{x}_{y}.png"
        file_path = os.path.join(CACHE_DIR, file_name)
        
        if os.path.exists(file_path):
            try:
                raw_surf = pygame.image.load(file_path)
                surf = raw_surf.convert_alpha() if pygame.display.get_surface() else raw_surf
                self.cache[(server_key, z, x, y)] = surf
                return surf
            except Exception:
                pass
                
        if (server_key, z, x, y) not in self.pending:
            self.pending.add((server_key, z, x, y))
            url_pattern = SERVERS.get(server_key, SERVERS["DARK"])
            url = url_pattern.format(z=z, x=x, y=y)
            t = threading.Thread(
                target=self._fetch_tile_worker,
                args=(server_key, z, x, y, url, file_path),
                daemon=True
            )
            t.start()
            
        return None

    def draw_map(self, screen, map_rect, center_lat, center_lon, scale_px_per_m):
        if self.mode == "GRID":
            return  # Will just show the standard tactical grid
            
        # Calculate optimal zoom level based on pixel scale
        cos_lat = math.cos(math.radians(center_lat))
        z_float = math.log2(max(1.0, scale_px_per_m * 40075016.686 * cos_lat / 256.0))
        zoom = max(13, min(18, int(round(z_float))))
        
        # Center tile coordinate
        center_x, center_y = latlon_to_xy(center_lat, center_lon, zoom)
        
        # Screen center
        screen_cx = map_rect.x + map_rect.width / 2.0
        screen_cy = map_rect.y + map_rect.height / 2.0
        
        # Tile coordinates in pixels at this zoom level
        m_per_px_zoom = (156543.03 * cos_lat) / (2.0 ** zoom)
        scale_ratio = (1.0 / m_per_px_zoom) / scale_px_per_m  # stretch/shrink factor
        
        scaled_tile_size = int(round(TILE_SIZE * scale_ratio))
        if scaled_tile_size < 16:
            scaled_tile_size = 16
            
        # Visible tile bounds
        half_w = map_rect.width / 2.0
        half_h = map_rect.height / 2.0
        
        min_x = int(math.floor(center_x - (half_w / scaled_tile_size) - 1))
        max_x = int(math.ceil(center_x + (half_w / scaled_tile_size) + 1))
        min_y = int(math.floor(center_y - (half_h / scaled_tile_size) - 1))
        max_y = int(math.ceil(center_y + (half_h / scaled_tile_size) + 1))
        
        n_tiles = 2 ** zoom
        
        for tx in range(min_x, max_x + 1):
            wrapped_x = tx % n_tiles
            for ty in range(min_y, max_y + 1):
                if 0 <= ty < n_tiles:
                    # Screen position of top-left corner of this tile
                    px = screen_cx + (tx - center_x) * scaled_tile_size
                    py = screen_cy + (ty - center_y) * scaled_tile_size
                    
                    tile_surf = self.get_tile(self.mode, zoom, wrapped_x, ty)
                    if tile_surf:
                        if abs(scaled_tile_size - TILE_SIZE) > 4:
                            draw_surf = pygame.transform.scale(tile_surf, (scaled_tile_size, scaled_tile_size))
                        else:
                            draw_surf = tile_surf
                        screen.blit(draw_surf, (int(px), int(py)))
                    else:
                        proc_surf = pygame.transform.scale(self.procedural_surface, (scaled_tile_size, scaled_tile_size)) if scaled_tile_size != TILE_SIZE else self.procedural_surface
                        screen.blit(proc_surf, (int(px), int(py)))

