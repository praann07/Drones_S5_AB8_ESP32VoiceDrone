DRONE FLIGHT LAB — OBSTACLE STOP FIX
=====================================

This version keeps the seven requested commands:
TAKE OFF, TURN LEFT, TURN RIGHT, BACKWARD, HOVER, LAND, RETURN TO HOME.
LISTEN TO VOICE starts the voice listener and is not a flight command.

OBSTACLE CHANGE
---------------
The previous check considered the obstacle's height. That could allow the
simulated drone to pass over a configured low barrier. This version uses a
conservative horizontal no-fly safety footprint for configured barriers and
buildings while airborne. If the next motion segment enters a safety footprint,
the drone stops at the last safe position, zeros its simulated velocity, and
holds until the user gives a new command. It does not automatically detour.

INSTALLATION
------------
1. Extract this ZIP into a new folder inside Downloads named:
   DroneFlightLab_COMMANDS_7_OBSTACLE_STOP_FIXED
2. Keep DroneFlightLab_2026.m and listen_voice.ps1 together.
3. Close any old drone simulation windows.
4. In MATLAB Command Window, run one line at a time:

   clear functions
   close all force
   rehash
   cd(fullfile(getenv('USERPROFILE'),'Downloads','DroneFlightLab_COMMANDS_7_OBSTACLE_STOP_FIXED'))
   which DroneFlightLab_2026 -all
   DroneFlightLab_2026

Make sure `which DroneFlightLab_2026 -all` points to this new folder.

The city obstacles and safety regions are simplified approximations. This is a
software simulation, not a certified real-world collision-avoidance system.
Please test obstacle stopping in MATLAB before relying on the behaviour.
