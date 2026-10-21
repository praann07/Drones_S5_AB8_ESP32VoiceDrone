"""Recognise "go to <place>" in a transcript: "go to AB1", "fly to the central
kitchen", "take me to the library".

Each place (world.places()) gets its own name plus spoken aliases. Speech
recognisers spell things their own way ("a b one", "ab 1"), so names and
transcripts are normalised the same way before matching: lower case,
number words to digits, single letters joined ("a b 1" -> "ab1").
"""
from __future__ import annotations

import difflib
import re

NAV_WORDS = ("go", "goto", "fly", "take", "head", "navigate", "reach", "visit", "to")

# Extra spoken names for the campus and city places (OpenStreetMap names
# are long, e.g. "Academic Block 1 - School of Engineering").
ALIASES = {
    "Academic Block 1 - School of Engineering": ["ab1", "academic block 1", "block 1",
                                                 "engineering block", "school of engineering"],
    "Academic Block 2": ["ab2", "academic block 2", "block 2"],
    "Academic Block 3 - International school of Law": ["ab3", "academic block 3", "block 3",
                                                       "law school", "school of law"],
    "Central Kitchen": ["kitchen", "central kitchen", "central kitchens"],
    "Central Library": ["library", "central library"],
    "Main Canteen": ["canteen", "main canteen"],
    "IT Canteen": ["it canteen", "i t canteen"],
    "MBA Canteen": ["mba canteen", "m b a canteen"],
    "Auditorium": ["auditorium"],
    "Amrita Guest House": ["guest house"],
    "Amrita School of Business": ["business school", "school of business"],
    "Amrita Boys Gym": ["gym", "boys gym"],
    "Aroyga Sadhanam - Boys Gym": ["sadhanam"],
    "Dhanlaxmi Bank": ["bank"],
    "Fabrication workshop": ["workshop", "fabrication workshop"],
    "Shrine": ["shrine", "temple"],
    "Amriteshwari Hall": ["hall", "amriteshwari hall"],
    "Amrita Automative Research & Testing Centre": ["research centre", "testing centre",
                                                     "research center", "testing center"],
    "Corporate and Industry Relations": ["corporate relations", "industry relations"],
    "obstacle_tower": ["tower", "obstacle tower", "orange tower"],
    "home_pad": ["home", "home pad", "pad", "base", "helipad"],
    "park_n": ["north park"],
    "park_s": ["south park"],
}

_NUMBERS = {"one": "1", "two": "2", "three": "3", "four": "4", "five": "5",
            "first": "1", "second": "2", "third": "3"}
_DROP = {"the", "a", "an", "me", "please", "now", "of"}
_COMMON = {"to", "go", "of", "me", "at", "in", "on", "no", "is", "it", "the", "and", "for", "fly"}


def normalise(text: str) -> list[str]:
    text = re.sub(r"[^a-z0-9 ]+", " ", text.lower().replace("&", " and "))
    toks = [_NUMBERS.get(t, t) for t in text.split()]
    out: list[str] = []
    spelling = False        # previous token was a single letter (or a run of them)
    for t in toks:
        letter = len(t) == 1 and t.isalpha()
        # join spelled-out letters and a trailing number: "a b 1" -> "ab1", "m b a" -> "mba"
        short_name = (out and out[-1].isalpha() and len(out[-1]) <= 3
                      and out[-1] not in _COMMON)        # "ab 3" -> "ab3"
        if (spelling and (letter or t.isdigit())) or (short_name and t.isdigit()):
            out[-1] += t
            spelling = letter
        else:
            out.append(t)
            spelling = letter
    return [t for t in out if t not in _DROP]


class PlaceBook:
    def __init__(self, world):
        self.polygons = world.places()
        self.aliases: list[tuple[list[str], str]] = []
        for name in self.polygons:
            spoken = {name, name.split(" - ")[0], name.replace("_", " ")}
            spoken.update(ALIASES.get(name, []))
            for a in spoken:
                toks = normalise(a)
                if toks:
                    self.aliases.append((toks, name))

    def names(self) -> list[str]:
        return sorted(self.polygons)

    def match(self, transcript: str) -> str | None:
        """The place a transcript asks to go to, or None. Needs a travel word
        ("go", "fly", "to", ...) unless the transcript is only the place name."""
        toks = normalise(transcript or "")
        if not toks:
            return None
        has_nav = any(t in NAV_WORDS for t in toks)
        words = [t for t in toks if t not in NAV_WORDS]
        best, best_score = None, 0.0
        for alias, name in self.aliases:
            score = _contains(words, alias)
            if score > best_score + 1e-9 or (abs(score - best_score) < 1e-9 and best
                                              and len(alias) > len(best[0])):
                best, best_score = (alias, name), score
        if best is None or best_score < 0.8:
            return None
        if not has_nav and len(words) != len(best[0]):
            return None
        return best[1]

    def grammar_words(self) -> list[str]:
        """Words a speech recogniser needs to hear any place name."""
        words = set(NAV_WORDS) | {"me", "the", "a", "b", "i", "t", "m"} | set(_NUMBERS)
        for name in self.polygons:
            for a in {name, *ALIASES.get(name, [])}:
                words.update(re.sub(r"[^a-z0-9 ]+", " ", a.lower()).split())
        num_words = {v: k for k, v in _NUMBERS.items() if k in ("one", "two", "three")}
        return sorted({num_words.get(w, w) for w in words if not w.isdigit() or w in num_words})


def _contains(words: list[str], alias: list[str]) -> float:
    """How well `alias` appears as a run of consecutive words (1.0 = exact).
    Each word may be slightly misspelt (difflib ratio), averaged."""
    n = len(alias)
    best = 0.0
    for i in range(len(words) - n + 1):
        r = [difflib.SequenceMatcher(None, w, a).ratio() for w, a in zip(words[i:i + n], alias)]
        if min(r) >= 0.75:
            best = max(best, sum(r) / n)
    return best
