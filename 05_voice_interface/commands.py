"""Stage E command vocabulary, world-frame waypoint map, and rule-based
classifier (docs/ENGINEERING_PLAN.md Stage E task 2).

Eight fixed commands, world-frame directions (docs/TDD.md section 7b):
forward=+X, back=-X, left=+Y, right=-Y, up=+Z, down=-Z, hover/stop = hold
position. Yaw is never commanded, so directions are world-fixed regardless
of current attitude. up/down were added for the Mission Control UI
(docs/CLAUDE.md Stage E "Mission Control" entry) - PositionController and
PointToPointTrajectory already operate on arbitrary R^3 offsets, so this
is a pure vocabulary/waypoint-map addition, not a controller change.
"""
from __future__ import annotations

import difflib

import numpy as np

COMMANDS = ("forward", "back", "left", "right", "up", "down", "hover", "stop")
# "land" is not a fixed-offset waypoint like the 8 above: it descends onto
# whatever is under the drone (ground, pad or a roof, see city.surface_under)
# and switches the motors off on touchdown (flight.py). Kept out of COMMANDS
# so the fixed-offset trial/gauntlet gates are unchanged.
LAND_COMMAND = "land"
ALL_COMMANDS = COMMANDS + (LAND_COMMAND,)
WAYPOINT_OFFSET_M = 1.5
GOTO_COMMAND = "goto"
# Vertical offset is deliberately smaller than the lateral one: hover
# altitude is ~1.0m (quad.xml body pos), so a 0.5m "down" is a visually
# obvious move that stays well above the floor. Repeated "down" commands
# stop at MIN_CLEARANCE_M above the surface below (flight.py); only "land"
# goes all the way down.
VERTICAL_OFFSET_M = 0.5

_COMMAND_OFFSET = {
    "forward": np.array([WAYPOINT_OFFSET_M, 0.0, 0.0]),
    "back": np.array([-WAYPOINT_OFFSET_M, 0.0, 0.0]),
    "left": np.array([0.0, WAYPOINT_OFFSET_M, 0.0]),
    "right": np.array([0.0, -WAYPOINT_OFFSET_M, 0.0]),
    "up": np.array([0.0, 0.0, VERTICAL_OFFSET_M]),
    "down": np.array([0.0, 0.0, -VERTICAL_OFFSET_M]),
    "hover": np.array([0.0, 0.0, 0.0]),
    "stop": np.array([0.0, 0.0, 0.0]),
}


def target_offset(command: str) -> np.ndarray:
    """World-frame position offset applied from the current position when
    `command` is dispatched."""
    return _COMMAND_OFFSET[command].copy()


def parse_goto(transcript: str) -> np.ndarray | None:
    """Parse 'goto x y z' or 'go to x y z' into a world-frame position.
    Returns None if the transcript is not a valid goto command."""
    import re
    match = re.search(r'goto\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)', transcript.lower())
    if not match:
        match = re.search(r'go\s+to\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)', transcript.lower())
    if not match:
        return None
    return np.array([float(match.group(1)), float(match.group(2)), float(match.group(3))])


def command_vocabulary() -> dict[str, list[str]]:
    """Every fixed command mapped to its recognized synonyms (may be an
    empty list). Built by inverting `_SYNONYMS` so a printed guide (see
    print_commands_guide.py) can never drift out of sync with the real
    vocabulary - there is exactly one place synonyms are defined."""
    out: dict[str, list[str]] = {cmd: [] for cmd in ALL_COMMANDS}
    for word, cmd in _SYNONYMS.items():
        out[cmd].append(word)
    return out


# Two-word phrases joined into one token before matching ("take off" ->
# "takeoff"), so each half can't be mistaken for something else ("off").
_PHRASES = {"take off": "takeoff", "touch down": "touchdown"}

# Filler words people say around a command ("go left", "move up please").
# They classify to nothing; they're only in the grammar so the recognizer
# doesn't force them onto a command word.
_FILLER_WORDS = ("go", "move", "turn", "fly", "drone", "please", "now", "a", "bit",
                 "little", "to", "the")


def asr_grammar() -> list[str]:
    """Word list for Vosk's restricted-grammar mode: every command, every
    synonym, a few filler words, and "[unk]" for anything else. With the
    open English vocabulary a noisy "left" can come back as "lift" (a
    synonym for up); restricted to this list the recognizer can only
    return words classify() understands, or [unk]."""
    words = list(ALL_COMMANDS) + list(_SYNONYMS) + list(_PHRASES) + list(_FILLER_WORDS)
    return list(dict.fromkeys(words)) + ["[unk]"]


# Natural synonyms that mean a command word but share no useful letter
# overlap with it (so fuzzy matching alone can't find them, and sometimes
# actively mismatches them - see the "straight" bug below). Checked as
# exact token matches before any fuzzy scoring.
_SYNONYMS = {
    "straight": "forward", "ahead": "forward", "forwards": "forward",
    "backward": "back", "backwards": "back", "reverse": "back",
    "halt": "stop", "freeze": "stop", "wait": "stop", "pause": "stop",
    "ascend": "up", "climb": "up", "rise": "up", "higher": "up", "lift": "up",
    "descend": "down", "lower": "down", "drop": "down",
    "landing": "land", "touchdown": "land", "lands": "land", "takeoff": "up",
    "stay": "hover", "hold": "hover",
}

# Below this length, a token is too short for difflib ratio to be a
# trustworthy signal - short common words coincidentally share enough
# letters with a real command to clear even a fairly high cutoff. Real
# commands this short ("up") are still matched at ratio 1.0 (exact) or
# near it, which clears this stricter bar fine; it only blocks WEAK
# coincidental matches from short filler words.
# 0.7 (not the original 0.6): measured on real live-mic transcripts,
# genuine matches (typos/short forms of the actual command word, e.g.
# "foward"->forward 0.923, "lef"->left 0.857) score far above coincidental
# collisions from unrelated English words (e.g. "laptop"->stop, "their"->
# hover, both exactly 0.6) - there's a clean gap between the two
# populations, and 0.7 sits safely in it.
_SHORT_TOKEN_LEN = 4
_SHORT_TOKEN_CUTOFF = 0.8
_CUTOFF = 0.7


def classify(transcript: str) -> str | None:
    """Map a raw ASR transcript to one of the fixed commands, or None if no
    command is confidently present. Rule-based: known synonyms first
    (exact word match, see _SYNONYMS), then exact/close word match via
    difflib, whole-transcript first, then the GLOBALLY best-scoring token
    in the transcript (not the first token to clear the cutoff).

    Two real bugs found from actual live-mic transcripts (not hypothetical):
    1. "first token above cutoff wins, left-to-right" (an earlier version of
       this function): in "five meters to left", the short filler word "to"
       fuzzy-matches "stop" at ratio 0.667 and got returned before the loop
       ever reached "left" (a perfect 1.0 match). Fixed by scoring every
       token and keeping the best match, not the first.
    2. Scoring every token was NOT enough on its own: in "go straight" (a
       completely natural way to say "forward"), "straight" fuzzy-matches
       "right" at a HIGH ratio purely from sharing the letters r-i-g-h-t as
       a near-contiguous run ("stRAIGHT" / "RIGHT"), and nothing in the
       sentence scores higher - so "straight" was still the best-scoring
       token, just for the WRONG command. No amount of "pick the best
       match" fixes a coincidental match that IS the best-scoring one -
       this needs an explicit synonym table (word-level, checked before
       fuzzy matching) plus a stricter cutoff for short tokens, since most
       of these coincidental collisions involve a short (<=3 char) token
       riding a lenient 0.6 cutoff against a longer command word.
    """
    if not transcript:
        return None
    text = transcript.lower().strip()
    if not text:
        return None

    for phrase, joined in _PHRASES.items():
        text = text.replace(phrase, joined)
    tokens = text.split()
    for tok in tokens:
        if tok in _SYNONYMS:
            return _SYNONYMS[tok]

    # Check for goto command
    if "goto" in tokens or "go" in tokens:
        coords = parse_goto(text)
        if coords is not None:
            return GOTO_COMMAND

    # "land" switches the motors off, so it is matched only as an exact word
    # (or a synonym above), never by fuzzy spelling: "and" scores 0.86 against it.
    if LAND_COMMAND in tokens:
        return LAND_COMMAND

    whole = difflib.get_close_matches(text, COMMANDS, n=1, cutoff=_CUTOFF)
    if whole:
        return whole[0]

    best_cmd, best_ratio = None, 0.0
    for tok in tokens:
        cutoff = _SHORT_TOKEN_CUTOFF if len(tok) < _SHORT_TOKEN_LEN else _CUTOFF
        hits = difflib.get_close_matches(tok, COMMANDS, n=1, cutoff=cutoff)
        if not hits:
            continue
        ratio = difflib.SequenceMatcher(None, tok, hits[0]).ratio()
        if ratio > best_ratio:
            best_ratio, best_cmd = ratio, hits[0]
    return best_cmd
