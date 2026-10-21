"""Voice-controlled drone with obstacles: it stops before them (--on-obstacle steer to go around)."""

from drone_core import main

if __name__ == "__main__":
    main(description=__doc__, voice=True, obstacles=True)
