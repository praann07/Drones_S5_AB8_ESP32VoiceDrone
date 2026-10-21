"""Keyboard-controlled drone (add --voice for the microphone, --obstacles for avoidance)."""

from drone_core import main

if __name__ == "__main__":
    main(description=__doc__, voice=False, obstacles=False)
