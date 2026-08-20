import re

try:
    from navigation import default_mission
except ImportError:
    from .navigation import default_mission



def _num(text):
    nums = re.findall(r"[-+]?\d*\.?\d+", text)
    return [float(x) for x in nums]


def _norm(text):
    return " ".join(text.lower().replace(",", " ").replace(".", " ").split())


def parse(text):
    t = _norm(text)
    if not t:
        return None

    # Data-Driven Modeling & Simulation (DDMS) Analysis & Logging Commands
    if any(k in t for k in ("run analysis", "data analysis", "run sindy", "run dmd", "discover model", "system identification", "data driven", "analyze data")):
        return ("run_analysis", {})
    if any(k in t for k in ("start logging", "stop logging", "toggle logging", "record data", "log telemetry", "save flight data", "record telemetry")):
        return ("toggle_logging", {})
    if any(k in t for k in ("perimeter mission", "perimeter flight", "campus perimeter", "survey perimeter")):
        return ("mission", {})

    # Takeoff commands (including phonetic matches like "you cough", "they cough", "take of")
    if any(k in t for k in ("take off", "takeoff", "fly up", "launch", "ascend", "start flight", "lift off", "you cough", "they cough", "take of", "pick off")):
        nums = _num(t)
        alt = nums[0] if nums else None
        return ("takeoff", {"alt": alt})

    # Landing commands (including short phonetic words like "lab", "lang", "lend")
    if any(k in t for k in ("land", "touch down", "please land", "land now", "descend to ground", "set down", "lab", "lang", "lend")):
        return ("land", {})

    # Return to Home (RTH) commands
    if any(k in t for k in ("return home", "rth", "go home", "come back", "back to base", "fly home", "return whom")):
        return ("rth", {})

    # Mission commands
    if any(k in t for k in ("start mission", "begin mission", "run mission", "execute mission", "follow waypoints", "start")):
        return ("mission", {})

    # Stop / Cancel Orbit commands
    if any(k in t for k in ("stop orbit", "cancel orbit", "exit orbit", "undo orbit", "end orbit", "stop circling", "cancel circling")):
        return ("orbit_stop", {})

    # Hover at Waypoint ("hover at p1", "hover at point 1", "hover at point a", "hold at p1")
    if any(k in t for k in ("hover at", "hover on", "hold at", "stay at", "hover point")):
        m_p_hov = re.search(r"\bp\s*(\d+)\b", t)
        if m_p_hov:
            return ("hover_at", {"name": f"P{m_p_hov.group(1)}"})
        m_num_hov = re.search(r"(?:point|location|waypoint|at)?\s*(\d+)\b", t)
        if m_num_hov and m_num_hov.group(1):
            return ("hover_at", {"name": f"P{m_num_hov.group(1)}"})
        m_hov = re.search(r"(?:point|location|waypoint|at)?\s*([a-h]|p\d+)\b", t)
        if m_hov:
            return ("hover_at", {"name": m_hov.group(1).upper()})

    # Pause / Hover / Position Hold commands (general)
    if any(k in t for k in ("pause", "hover", "stop", "hold position", "stay here", "freeze", "halt", "hold")):
        return ("pause", {})

    # Resume commands
    if any(k in t for k in ("resume", "continue mission", "continue", "proceed")):
        return ("resume", {})

    # Abort / Emergency commands
    if any(k in t for k in ("abort", "emergency", "cancel mission", "disarm", "cut motors", "emergency stop", "kill")):
        return ("abort", {})

    # POI Orbit / Circle Mode ("orbit point a", "orbit p1", "circle point 1", "orbit")
    if any(k in t for k in ("orbit", "circle", "loiter around")):
        m_p_orb = re.search(r"\bp\s*(\d+)\b", t)
        if m_p_orb:
            return ("orbit", {"name": f"P{m_p_orb.group(1)}"})
        m_num_orb = re.search(r"(?:point|location|waypoint)?\s*(\d+)\b", t)
        if m_num_orb and m_num_orb.group(1):
            return ("orbit", {"name": f"P{m_num_orb.group(1)}"})
        m_orb = re.search(r"(?:point|location|waypoint|to)?\s*([a-h]|p\d+)\b", t)
        if m_orb:
            return ("orbit", {"name": m_orb.group(1).upper()})
        return ("orbit", {})

    # Delete / Remove Waypoints ("delete p1", "remove point 1", "clear waypoints")
    if any(k in t for k in ("clear all waypoints", "clear waypoints", "delete all waypoints", "reset waypoints")):
        return ("clear_waypoints", {})
    if any(k in t for k in ("delete", "remove", "clear point")):
        m_p_del = re.search(r"\bp\s*(\d+)\b", t)
        if m_p_del:
            return ("delete_wp", {"name": f"P{m_p_del.group(1)}"})
        m_num_del = re.search(r"(?:point|location|waypoint)?\s*(\d+)\b", t)
        if m_num_del and m_num_del.group(1):
            return ("delete_wp", {"name": f"P{m_num_del.group(1)}"})
        m_del = re.search(r"(?:point|location|waypoint)?\s*([a-h]|p\d+)\b", t)
        if m_del:
            return ("delete_wp", {"name": m_del.group(1).upper()})

    # Dynamic Wind & Gust Simulation Commands
    if any(k in t for k in ("wind calm", "calm wind", "no wind", "zero wind", "stop wind")):
        return ("wind", {"mode": "CALM"})
    if any(k in t for k in ("wind light", "light wind", "light breeze", "gentle breeze", "breeze")):
        return ("wind", {"mode": "LIGHT"})
    if any(k in t for k in ("wind strong", "strong wind", "high wind", "heavy wind", "gusty")):
        return ("wind", {"mode": "STRONG"})
    if any(k in t for k in ("wind storm", "storm", "turbulence", "turbulent wind", "severe wind")):
        return ("wind", {"mode": "STORM"})
    if any(k in t for k in ("gust", "wind gust", "simulate gust", "sudden gust", "blow gust")):
        return ("gust", {"strength": 8.0})
    if "wind" in t:
        if "calm" in t or "off" in t or "zero" in t:
            return ("wind", {"mode": "CALM"})
        if "light" in t or "low" in t:
            return ("wind", {"mode": "LIGHT"})
        if "strong" in t or "high" in t:
            return ("wind", {"mode": "STRONG"})
        if "storm" in t or "turb" in t:
            return ("wind", {"mode": "STORM"})
        return ("cycle_wind", {})

    # Map Tile View Modes ("satellite map", "dark map", "street map", "grid map")
    if any(k in t for k in ("satellite map", "satellite view", "sat map", "satellite")):
        return ("map_mode", {"mode": "SATELLITE"})
    if any(k in t for k in ("dark map", "dark mode", "tactical map", "night map")):
        return ("map_mode", {"mode": "DARK"})
    if any(k in t for k in ("street map", "osm map", "street view", "normal map", "open street")):
        return ("map_mode", {"mode": "STREET"})
    if any(k in t for k in ("grid map", "grid view", "simple map", "no map")):
        return ("map_mode", {"mode": "GRID"})
    if any(k in t for k in ("change map", "switch map", "toggle map", "next map")):
        return ("cycle_map", {})

    # Direct Custom Dropped Waypoint Patterns (e.g. "point p1", "point p 1", "go to p1", "p1", "p2")
    m_p = re.search(r"\bp\s*(\d+)\b", t)
    if m_p:
        return ("goto", {"name": f"P{m_p.group(1)}"})
    
    m_p_word = re.search(r"(?:point|location|waypoint|to)\s+p\s*(\d+)\b", t)
    if m_p_word:
        return ("goto", {"name": f"P{m_p_word.group(1)}"})

    # Specific phonetic waypoint aliases
    if any(k in t for k in ("point a", "point hey", "point ay", "location a", "plan a", "go to a", "fly to a", "to a")):
        return ("goto", {"name": "A"})
    if any(k in t for k in ("point b", "point be", "point bee", "to be", "worry to be", "plan b", "location b", "go to b", "fly to b", "to b")):
        return ("goto", {"name": "B"})
    if any(k in t for k in ("point c", "point see", "point sea", "plan c", "location c", "go to c", "fly to c", "to c")):
        return ("goto", {"name": "C"})

    # Numbered waypoints (e.g. "point 1", "point one", "point 2")
    if "point one" in t or "point 1" in t or "waypoint 1" in t:
        return ("goto", {"name": "P1"})
    if "point two" in t or "point 2" in t or "waypoint 2" in t:
        return ("goto", {"name": "P2"})
    if "point three" in t or "point 3" in t or "waypoint 3" in t:
        return ("goto", {"name": "P3"})

    # Named Waypoints general pattern (e.g., "point a", "go to b", "location c", "waypoint d", "p1")
    m = re.search(r"(?:point|location|waypoint|to)\s+([a-h]|p\d+|\d+)\b", t)
    if m:
        val = m.group(1).upper()
        if val.isdigit():
            val = f"P{val}"
        return ("goto", {"name": val})
    
    m = re.search(r"\b([a-h]|p\d+)\s+(?:point|location|waypoint)\b", t)
    if m:
        return ("goto", {"name": m.group(1).upper()})

    if t in ("a", "b", "c", "d", "e", "f", "home", "p1", "p2", "p3", "p4", "p5"):
        return ("goto", {"name": t.upper()})



    # Cardinal Headings ("face north", "turn east", etc.)
    if "north" in t:
        return ("heading_abs", {"deg": 0.0})
    if "east" in t:
        return ("heading_abs", {"deg": 90.0})
    if "south" in t:
        return ("heading_abs", {"deg": 180.0})
    if "west" in t:
        return ("heading_abs", {"deg": 270.0})

    # Relative Turns / Yaw rotations
    if any(k in t for k in ("turn", "rotate", "yaw", "spin", "face")):
        nums = _num(t)
        deg = nums[0] if nums else None
        if deg is not None:
            if any(k in t for k in ("left", "ccw", "counter", "anti")):
                deg = -abs(deg)
            elif any(k in t for k in ("right", "cw", "clockwise")):
                deg = abs(deg)
            return ("heading", {"deg": deg})
        if any(k in t for k in ("left", "ccw", "counter", "anti")):
            return ("heading", {"deg": -30.0})
        return ("heading", {"deg": 30.0})

    # Manual nudges / directional velocity
    if any(k in t for k in ("forward", "ahead", "straight", "front")):
        return ("manual", {"axis": "vx", "value": 1.0})
    if any(k in t for k in ("backward", "reverse", "back")):
        return ("manual", {"axis": "vx", "value": -1.0})
    if "left" in t:
        return ("manual", {"axis": "vy", "value": -1.0})
    if "right" in t:
        return ("manual", {"axis": "vy", "value": 1.0})

    # Absolute bearing / heading
    if any(k in t for k in ("bear", "to bearing", "heading to", "heading")):
        nums = _num(t)
        if nums:
            return ("heading_abs", {"deg": nums[0]})

    # Altitude control ("climb 5 meters", "go higher", "descend 2 meters", "go lower")
    if any(k in t for k in ("up", "climb", "higher", "increase altitude", "raise")):
        nums = _num(t)
        d = nums[0] if nums else 2.0
        return ("altitude", {"delta": abs(d)})
    if any(k in t for k in ("down", "descend", "lower", "decrease altitude", "drop")):
        nums = _num(t)
        d = nums[0] if nums else 2.0
        return ("altitude", {"delta": -abs(d)})

    # Set specific target altitude ("altitude 8")
    if "altitude" in t or "alt" in t:
        nums = _num(t)
        if nums:
            return ("set_alt", {"alt": abs(nums[0])})

    # Geofence settings
    if any(k in t for k in ("geofence", "boundary", "radius limit", "fence")):
        nums = _num(t)
        if nums:
            return ("geofence", {"radius": nums[0]})
        return ("geofence", {})

    # Simulation Failsafe trigger
    if any(k in t for k in ("link loss", "comms loss", "simulate loss", "lose link", "disconnect")):
        return ("link_loss", {})

    # Status / Telemetry enquiry
    if any(k in t for k in ("status", "where are you", "battery", "report", "telemetry", "how are you")):
        return ("status", {})

    # Waypoint & Trail management
    if any(k in t for k in ("add waypoint", "mark position", "set waypoint", "drop pin")):
        return ("add_waypoint", {})
    if any(k in t for k in ("clear trail", "clear path", "reset trail", "clear history")):
        return ("clear_trail", {})

    # Dynamic Wind & Gust Simulation Commands
    if any(k in t for k in ("wind calm", "calm wind", "no wind", "zero wind", "stop wind")):
        return ("wind", {"mode": "CALM"})
    if any(k in t for k in ("wind light", "light wind", "light breeze", "gentle breeze", "breeze")):
        return ("wind", {"mode": "LIGHT"})
    if any(k in t for k in ("wind strong", "strong wind", "high wind", "heavy wind", "gusty")):
        return ("wind", {"mode": "STRONG"})
    if any(k in t for k in ("wind storm", "storm", "turbulence", "turbulent wind", "severe wind")):
        return ("wind", {"mode": "STORM"})
    if any(k in t for k in ("gust", "wind gust", "simulate gust", "sudden gust", "blow gust")):
        return ("gust", {"strength": 8.0})
    if "wind" in t:
        if "calm" in t or "off" in t or "zero" in t:
            return ("wind", {"mode": "CALM"})
        if "light" in t or "low" in t:
            return ("wind", {"mode": "LIGHT"})
        if "strong" in t or "high" in t:
            return ("wind", {"mode": "STRONG"})
        if "storm" in t or "turb" in t:
            return ("wind", {"mode": "STORM"})
        return ("cycle_wind", {})

    # POI Orbit / Circle Mode ("orbit point a", "circle point b", "orbit")
    if any(k in t for k in ("orbit", "circle", "loiter around")):
        m = re.search(r"(?:point|location|waypoint|to)?\s*([a-h]|p\d+)\b", t)
        if m:
            return ("orbit", {"name": m.group(1).upper()})
        return ("orbit", {})

    # GPS Latitude/Longitude coordinates ("go to 19.051 75.052")
    nums = _num(t)
    if ("go to" in t or "fly to" in t or "coordinate" in t) and len(nums) >= 2:
        return ("goto_latlon", {"lat": nums[0], "lon": nums[1]})
    if len(nums) >= 2 and len(t.split()) >= 2:
        return ("goto_latlon", {"lat": nums[0], "lon": nums[1]})

    return ("unknown", {})


