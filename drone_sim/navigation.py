import math

import config


def wrap180_deg(deg):
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def wrap360_deg(deg):
    return deg % 360.0


def wrap180_rad(rad):
    return math.radians(wrap180_deg(math.degrees(rad)))


def to_local(lat, lon):
    dn = (lat - config.HOME_LAT) * config.M_PER_DEG_LAT
    de = (lon - config.HOME_LON) * config.M_PER_DEG_LON
    return dn, de


def from_local(n, e):
    lat = config.HOME_LAT + n / config.M_PER_DEG_LAT
    lon = config.HOME_LON + e / config.M_PER_DEG_LON
    return lat, lon


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def bearing_deg(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return wrap360_deg(math.degrees(math.atan2(y, x)))


def distance_local(n1, e1, n2, e2):
    return math.hypot(n1 - n2, e1 - e2)


def bearing_local_deg(n1, e1, n2, e2):
    return wrap360_deg(math.degrees(math.atan2(e2 - e1, n2 - n1)))


class Waypoint:
    def __init__(self, name, n, e, alt):
        self.name = name
        self.n = n
        self.e = e
        self.alt = alt

    @property
    def lat(self):
        return config.HOME_LAT + self.n / config.M_PER_DEG_LAT

    @property
    def lon(self):
        return config.HOME_LON + self.e / config.M_PER_DEG_LON

    def distance_to(self, n, e):
        return distance_local(n, e, self.n, self.e)


class Mission:
    def __init__(self):
        self.points = []
        self.index = 0
        self.active = False
        self.paused = False

    def clear(self):
        self.points = []
        self.index = 0
        self.active = False
        self.paused = False

    def load(self, points):
        self.points = list(points)
        self.index = 0
        self.active = True
        self.paused = False

    def current(self):
        if self.active and self.index < len(self.points):
            return self.points[self.index]
        return None

    def advance(self):
        if not self.active:
            return False
        self.index += 1
        if self.index >= len(self.points):
            self.active = False
            return False
        return True

    def remaining(self):
        if not self.active or self.index >= len(self.points):
            return 0
        return len(self.points) - self.index


def default_mission():
    return [
        Waypoint("A", 20.0, 18.0, config.WAYPOINT_ALT),
        Waypoint("B", -4.0, 42.0, config.WAYPOINT_ALT + 1.0),
        Waypoint("C", -26.0, 6.0, config.WAYPOINT_ALT + 2.0),
        Waypoint("D", 15.0, -25.0, config.WAYPOINT_ALT + 1.0),
        Waypoint("HOME", 0.0, 0.0, config.WAYPOINT_ALT),
    ]
