"""Voice-controlled drone (keyboard overrides work too; add --obstacles for avoidance)."""

from drone_core import main

if __name__ == "__main__":
    main(description=__doc__, voice=True, obstacles=False)
