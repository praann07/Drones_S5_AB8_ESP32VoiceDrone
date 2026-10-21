"""Prints a plain-English reference for the live demo: every voice command
(with its recognized synonyms), the dashboard buttons, and how the live
feedback works. Meant to be launched into its own window that stays open
for the whole demo (see RUN.bat) - this script just prints once
and exits; the launcher's `cmd /k` is what keeps the window up.

Vocabulary is imported from commands.py, never re-typed here, so this
guide can't silently drift out of sync with what the classifier actually
accepts.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from commands import ALL_COMMANDS, command_vocabulary  # noqa: E402


def main() -> None:
    vocab = command_vocabulary()
    width = 70
    print("=" * width)
    print("  VOICE-CONTROLLED QUADROTOR - COMMANDS GUIDE".center(width))
    print("=" * width)
    print()
    print("Say (or type) any of these words - exact word, a typo of it, or")
    print("one of its synonyms all work:")
    print()
    for cmd in ALL_COMMANDS:
        syns = vocab[cmd]
        line = f"  {cmd.upper():<8}"
        if syns:
            line += " - also: " + ", ".join(syns)
        print(line)
    print()
    print("-" * width)
    print("HOW MOVES WORK")
    print("-" * width)
    print("  - forward / back / left / right keep flying that way until a")
    print("    building, the edge of the map, or you say 'stop' / 'hover'.")
    print("  - 'stop' brakes smoothly (about 0.5 m) - it never turns back.")
    print("  - up / down move 0.5 m.  'land' lands on whatever is below.")
    print("  - GO TO A PLACE: 'go to AB1', 'go to the central kitchen',")
    print("    'fly to the library', 'go to the canteen', 'take me to the")
    print("    auditorium'. It plans a curved route around the buildings")
    print("    (dotted white line on the MAP window) and hovers beside it.")
    print("-" * width)
    print()
    print("-" * width)
    print("DASHBOARD WINDOW")
    print("-" * width)
    print("  - 9 click buttons (one per command above) - work even if your")
    print("    voice isn't recognized, or you're not using a mic at all.")
    print("  - TOGGLE CAMERA button - switches chase <-> bird's-eye view.")
    print("    Works by click regardless of which window has focus (the")
    print("    'B' keyboard shortcut only works if the 3D VIEWER window")
    print("    itself is focused).")
    print("  - LISTENING line - shows words appearing live as you speak.")
    print("  - HEARD LOG - every word you say shows up here, whether it")
    print("    was recognized or not - nothing is ever silently dropped.")
    print("  - 3 strip charts - attitude error, position error, control")
    print("    effort, updating live every control step.")
    print("-" * width)
    print()
    print("-" * width)
    print("AFTER YOU FLY")
    print("-" * width)
    print("  This window is just a reference - it stays open. When you're")
    print("  done flying, run RUN.bat and pick 2 for a fresh graph")
    print("  of exactly what you just did (not an old test run - every")
    print("  command you fly gets logged automatically).")


if __name__ == "__main__":
    main()
