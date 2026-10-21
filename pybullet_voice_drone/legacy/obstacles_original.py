import math
import time
import threading

import pybullet as p
import pybullet_data


# =========================================================
# SETTINGS
# =========================================================

DT = 1.0 / 120.0

HORIZONTAL_SPEED = 1.5
VERTICAL_SPEED = 1.0
ROTATION_SPEED = math.radians(45)

MIN_HEIGHT = 0.5
MAX_HEIGHT = 8.0

# How far the drone checks for obstacles
DETECTION_DISTANCE = 2.0

# Safety distance around drone
SAFETY_DISTANCE = 0.65

# How strongly the drone avoids an obstacle
AVOID_SPEED = 2.0

running = True


# =========================================================
# CONNECT TO PYBULLET
# =========================================================

physics_client = p.connect(p.GUI)

if physics_client < 0:
    print("ERROR: Could not connect to PyBullet.")
    raise SystemExit

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)

p.setTimeStep(DT)

p.setRealTimeSimulation(False)

p.loadURDF("plane.urdf")


# =========================================================
# DRONE STATE
# =========================================================

pos = [0.0, 0.0, 0.5]

yaw = 0.0

velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0

rotation_velocity = 0.0

# Current command
current_command = "STOP"

# Used for automatic obstacle avoidance
avoidance_active = False
avoidance_direction = 0

objects = {}

# Obstacles
obstacles = []


# =========================================================
# COORDINATE SYSTEM
# =========================================================

def world(local_x, local_y, local_z):

    c = math.cos(yaw)
    s = math.sin(yaw)

    x = pos[0] + local_x * c - local_y * s
    y = pos[1] + local_x * s + local_y * c
    z = pos[2] + local_z

    return [x, y, z]


def quat():

    return p.getQuaternionFromEuler(
        [0, 0, yaw]
    )


# =========================================================
# CREATE BOX
# =========================================================

def box(name, half, local, color):

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=color
    )

    objects[name] = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual,
        basePosition=world(*local),
        baseOrientation=quat()
    )


# =========================================================
# CREATE MOTOR
# =========================================================

def cylinder(name, radius, height, local, color):

    visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=radius,
        length=height,
        rgbaColor=color
    )

    objects[name] = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual,
        basePosition=world(*local),
        baseOrientation=quat()
    )


# =========================================================
# BUILD DRONE
# =========================================================

def build_drone():

    # Main body
    box(
        "body",
        [0.42, 0.30, 0.09],
        (0, 0, 0),
        [0.08, 0.08, 0.10, 1]
    )

    # Top
    box(
        "top",
        [0.28, 0.20, 0.035],
        (0, 0, 0.13),
        [0.05, 0.22, 0.70, 1]
    )

    # Front marker
    box(
        "front",
        [0.13, 0.055, 0.05],
        (0.45, 0, 0.10),
        [0.9, 0.08, 0.08, 1]
    )

    motors = [
        ("FL",  0.72,  0.72),
        ("FR",  0.72, -0.72),
        ("RL", -0.72,  0.72),
        ("RR", -0.72, -0.72)
    ]

    for name, x, y in motors:

        angle = math.atan2(y, x) + yaw

        length = math.sqrt(x*x + y*y)

        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[
                length / 2,
                0.035,
                0.035
            ],
            rgbaColor=[
                0.03,
                0.03,
                0.03,
                1
            ]
        )

        objects["arm_" + name] = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual,
            basePosition=world(
                x / 2,
                y / 2,
                0
            ),
            baseOrientation=
                p.getQuaternionFromEuler(
                    [0, 0, angle]
                )
        )

        cylinder(
            "motor_" + name,
            0.10,
            0.10,
            (x, y, 0.10),
            [0.03, 0.03, 0.03, 1]
        )

        box(
            "propA_" + name,
            [0.30, 0.018, 0.008],
            (x, y, 0.18),
            [0.85, 0.85, 0.85, 1]
        )

        box(
            "propB_" + name,
            [0.018, 0.30, 0.008],
            (x, y, 0.185),
            [0.65, 0.65, 0.65, 1]
        )


# =========================================================
# UPDATE DRONE
# =========================================================

def update_drone(prop_angle):

    motors = [
        ("FL",  0.72,  0.72),
        ("FR",  0.72, -0.72),
        ("RL", -0.72,  0.72),
        ("RR", -0.72, -0.72)
    ]

    p.resetBasePositionAndOrientation(
        objects["body"],
        world(0, 0, 0),
        quat()
    )

    p.resetBasePositionAndOrientation(
        objects["top"],
        world(0, 0, 0.13),
        quat()
    )

    p.resetBasePositionAndOrientation(
        objects["front"],
        world(0.45, 0, 0.10),
        quat()
    )

    for name, x, y in motors:

        angle = math.atan2(y, x) + yaw

        p.resetBasePositionAndOrientation(
            objects["arm_" + name],
            world(x / 2, y / 2, 0),
            p.getQuaternionFromEuler(
                [0, 0, angle]
            )
        )

        p.resetBasePositionAndOrientation(
            objects["motor_" + name],
            world(x, y, 0.10),
            quat()
        )

        a = yaw + prop_angle

        p.resetBasePositionAndOrientation(
            objects["propA_" + name],
            world(x, y, 0.18),
            p.getQuaternionFromEuler(
                [0, 0, a]
            )
        )

        p.resetBasePositionAndOrientation(
            objects["propB_" + name],
            world(x, y, 0.185),
            p.getQuaternionFromEuler(
                [0, 0, a + math.pi / 2]
            )
        )


# =========================================================
# OBSTACLES
# =========================================================

def create_obstacle(name, center, half_size):

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_size,
        rgbaColor=[0.85, 0.20, 0.10, 1]
    )

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_size
    )

    obstacle_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=center
    )

    obstacles.append({
        "id": obstacle_id,
        "center": center,
        "half": half_size,
        "name": name
    })


def build_obstacles():

    # Only a SMALL number of obstacles

    create_obstacle(
        "Obstacle_1",
        [3.5, 0.0, 1.5],
        [0.7, 0.7, 1.5]
    )

    create_obstacle(
        "Obstacle_2",
        [7.0, 2.0, 1.5],
        [0.7, 0.7, 1.5]
    )

    create_obstacle(
        "Obstacle_3",
        [5.0, -3.0, 1.5],
        [0.7, 0.7, 1.5]
    )

    print()
    print("======================================")
    print("          OBSTACLES CREATED")
    print("======================================")
    print("Obstacle 1 : x=3.5  y=0")
    print("Obstacle 2 : x=7.0  y=2")
    print("Obstacle 3 : x=5.0  y=-3")
    print("======================================")
    print()


# =========================================================
# CHECK WHETHER POINT IS INSIDE OBSTACLE
# =========================================================

def point_inside_obstacle(test_pos, obstacle, extra=0.0):

    cx, cy, cz = obstacle["center"]

    hx, hy, hz = obstacle["half"]

    x, y, z = test_pos

    if (
        cx - hx - extra <= x <= cx + hx + extra
        and
        cy - hy - extra <= y <= cy + hy + extra
        and
        cz - hz - extra <= z <= cz + hz + extra
    ):
        return True

    return False


# =========================================================
# OBSTACLE DETECTION
# =========================================================

def detect_obstacle():

    global avoidance_active
    global avoidance_direction

    # Direction of current movement
    move_x = velocity_x
    move_y = velocity_y
    move_z = velocity_z

    magnitude = math.sqrt(
        move_x * move_x +
        move_y * move_y +
        move_z * move_z
    )

    # If not moving, no forward obstacle detection
    if magnitude < 0.01:

        avoidance_active = False

        return None

    # Normalize movement direction
    dx = move_x / magnitude
    dy = move_y / magnitude
    dz = move_z / magnitude

    # Convert local movement into world coordinates
    c = math.cos(yaw)
    s = math.sin(yaw)

    world_dx = dx * c - dy * s
    world_dy = dx * s + dy * c
    world_dz = dz

    closest = None
    closest_distance = DETECTION_DISTANCE

    # Check points ahead of drone
    for step in range(1, 21):

        distance = (
            DETECTION_DISTANCE *
            step / 20.0
        )

        test_pos = [
            pos[0] + world_dx * distance,
            pos[1] + world_dy * distance,
            pos[2] + world_dz * distance
        ]

        for obstacle in obstacles:

            if point_inside_obstacle(
                test_pos,
                obstacle,
                SAFETY_DISTANCE
            ):

                if distance < closest_distance:

                    closest = obstacle

                    closest_distance = distance

                break

    return closest


# =========================================================
# FIND SAFE AVOIDANCE DIRECTION
# =========================================================

def choose_avoidance(obstacle):

    global avoidance_direction

    cx, cy, cz = obstacle["center"]

    # -----------------------------------------
    # Try LEFT
    # -----------------------------------------

    left_x = pos[0] - math.sin(yaw) * 2.0
    left_y = pos[1] + math.cos(yaw) * 2.0

    left_test = [
        left_x,
        left_y,
        pos[2]
    ]

    left_safe = True

    for obs in obstacles:

        if point_inside_obstacle(
            left_test,
            obs,
            SAFETY_DISTANCE
        ):

            left_safe = False
            break

    # -----------------------------------------
    # Try RIGHT
    # -----------------------------------------

    right_x = pos[0] + math.sin(yaw) * 2.0
    right_y = pos[1] - math.cos(yaw) * 2.0

    right_test = [
        right_x,
        right_y,
        pos[2]
    ]

    right_safe = True

    for obs in obstacles:

        if point_inside_obstacle(
            right_test,
            obs,
            SAFETY_DISTANCE
        ):

            right_safe = False
            break

    # -----------------------------------------
    # Choose direction
    # -----------------------------------------

    if left_safe:

        avoidance_direction = 1

        print("AUTO AVOIDANCE: MOVING LEFT")

    elif right_safe:

        avoidance_direction = -1

        print("AUTO AVOIDANCE: MOVING RIGHT")

    else:

        # If both sides are blocked,
        # go upward if possible.

        if pos[2] < MAX_HEIGHT - 1:

            avoidance_direction = 2

            print("AUTO AVOIDANCE: MOVING UP")

        else:

            avoidance_direction = -1

            print("AUTO AVOIDANCE: RIGHT")


# =========================================================
# AUTOMATIC OBSTACLE AVOIDANCE
# =========================================================

def automatic_avoidance():

    global velocity_x
    global velocity_y
    global velocity_z
    global avoidance_active

    obstacle = detect_obstacle()

    # -----------------------------------------
    # NO OBSTACLE
    # -----------------------------------------

    if obstacle is None:

        if avoidance_active:

            print("PATH CLEAR - RESUMING COMMAND")

        avoidance_active = False

        return False

    # -----------------------------------------
    # OBSTACLE DETECTED
    # -----------------------------------------

    if not avoidance_active:

        print()
        print("**************************************")
        print("OBSTACLE DETECTED!")
        print(
            "Obstacle:",
            obstacle["name"]
        )
        print("**************************************")

        choose_avoidance(obstacle)

        avoidance_active = True

    # -----------------------------------------
    # MOVE AROUND OBSTACLE
    # -----------------------------------------

    if avoidance_direction == 1:

        # LEFT
        velocity_x = 0
        velocity_y = AVOID_SPEED

    elif avoidance_direction == -1:

        # RIGHT
        velocity_x = 0
        velocity_y = -AVOID_SPEED

    elif avoidance_direction == 2:

        # UP
        velocity_x = 0
        velocity_y = 0
        velocity_z = VERTICAL_SPEED

    return True


# =========================================================
# COMMAND FUNCTION
# =========================================================

def command(text):

    global velocity_x
    global velocity_y
    global velocity_z
    global rotation_velocity
    global running
    global current_command

    text = text.lower().strip()

    print()
    print("======================================")
    print("YOU SAID:", text)
    print("======================================")

    # -----------------------------------------
    # EXIT
    # -----------------------------------------

    if "exit" in text or "quit" in text:

        running = False

        return

    # -----------------------------------------
    # STOP
    # -----------------------------------------

    if (
        text == "stop"
        or text == "halt"
        or text == "freeze"
    ):

        velocity_x = 0
        velocity_y = 0
        velocity_z = 0
        rotation_velocity = 0

        current_command = "STOP"

        print(">> STOP")

        return

    # -----------------------------------------
    # TAKE OFF
    # -----------------------------------------

    if (
        "take off" in text
        or "takeoff" in text
        or text == "take"
    ):

        velocity_x = 0
        velocity_y = 0
        velocity_z = VERTICAL_SPEED
        rotation_velocity = 0

        current_command = "TAKE OFF"

        print(">> TAKE OFF")

        return

    # -----------------------------------------
    # FORWARD
    # -----------------------------------------

    if (
        "forward" in text
        or "go forward" in text
    ):

        velocity_x = HORIZONTAL_SPEED
        velocity_y = 0
        velocity_z = 0
        rotation_velocity = 0

        current_command = "FORWARD"

        print(">> FORWARD")

        return

    # -----------------------------------------
    # BACKWARD
    # -----------------------------------------

    if (
        "backward" in text
        or text == "back"
        or "go back" in text
    ):

        velocity_x = -HORIZONTAL_SPEED
        velocity_y = 0
        velocity_z = 0
        rotation_velocity = 0

        current_command = "BACKWARD"

        print(">> BACKWARD")

        return

    # -----------------------------------------
    # LEFT
    # -----------------------------------------

    if (
        text == "left"
        or "go left" in text
    ):

        velocity_x = 0
        velocity_y = HORIZONTAL_SPEED
        velocity_z = 0
        rotation_velocity = 0

        current_command = "LEFT"

        print(">> LEFT")

        return

    # -----------------------------------------
    # RIGHT
    # -----------------------------------------

    if (
        text == "right"
        or "go right" in text
    ):

        velocity_x = 0
        velocity_y = -HORIZONTAL_SPEED
        velocity_z = 0
        rotation_velocity = 0

        current_command = "RIGHT"

        print(">> RIGHT")

        return

    # -----------------------------------------
    # UP
    # -----------------------------------------

    if (
        text == "up"
        or "go up" in text
        or "rise" in text
    ):

        velocity_x = 0
        velocity_y = 0
        velocity_z = VERTICAL_SPEED
        rotation_velocity = 0

        current_command = "UP"

        print(">> UP")

        return

    # -----------------------------------------
    # DOWN
    # -----------------------------------------

    if (
        text == "down"
        or "go down" in text
        or "descend" in text
    ):

        velocity_x = 0
        velocity_y = 0
        velocity_z = -VERTICAL_SPEED
        rotation_velocity = 0

        current_command = "DOWN"

        print(">> DOWN")

        return

    # -----------------------------------------
    # ROTATE LEFT
    # -----------------------------------------

    if (
        "rotate left" in text
        or "turn left" in text
    ):

        velocity_x = 0
        velocity_y = 0
        velocity_z = 0

        rotation_velocity = ROTATION_SPEED

        current_command = "ROTATE LEFT"

        print(">> ROTATE LEFT")

        return

    # -----------------------------------------
    # ROTATE RIGHT
    # -----------------------------------------

    if (
        "rotate right" in text
        or "turn right" in text
    ):

        velocity_x = 0
        velocity_y = 0
        velocity_z = 0

        rotation_velocity = -ROTATION_SPEED

        current_command = "ROTATE RIGHT"

        print(">> ROTATE RIGHT")

        return

    # -----------------------------------------
    # LAND
    # -----------------------------------------

    if "land" in text:

        velocity_x = 0
        velocity_y = 0
        velocity_z = -VERTICAL_SPEED
        rotation_velocity = 0

        current_command = "LAND"

        print(">> LAND")

        return

    # -----------------------------------------
    # UNKNOWN
    # -----------------------------------------

    print(">> COMMAND NOT RECOGNIZED")


# =========================================================
# VOICE CONTROL
# =========================================================

def voice_thread():

    try:

        import speech_recognition as sr

    except ImportError:

        print(
            "SpeechRecognition is not installed."
        )

        return

    recognizer = sr.Recognizer()

    try:

        microphone = sr.Microphone()

    except Exception as e:

        print(
            "Microphone error:",
            e
        )

        return

    with microphone as source:

        print(
            "Adjusting microphone..."
        )

        recognizer.adjust_for_ambient_noise(
            source,
            duration=1
        )

    print()
    print("======================================")
    print("        VOICE CONTROL READY")
    print("======================================")
    print()
    print("COMMANDS:")
    print()
    print("take off")
    print("forward")
    print("backward")
    print("left")
    print("right")
    print("up")
    print("down")
    print("rotate left")
    print("rotate right")
    print("land")
    print("stop")
    print("exit")
    print()
    print("======================================")

    while running:

        try:

            with microphone as source:

                audio = recognizer.listen(
                    source,
                    timeout=2,
                    phrase_time_limit=3
                )

            words = recognizer.recognize_google(
                audio
            )

            print(
                "YOU SAID:",
                words
            )

            command(words)

        except sr.WaitTimeoutError:

            pass

        except sr.UnknownValueError:

            print(
                "Could not understand voice"
            )

        except sr.RequestError:

            print(
                "Speech recognition needs internet."
            )

        except Exception as e:

            print(
                "Voice error:",
                e
            )


# =========================================================
# START
# =========================================================

build_drone()

build_obstacles()


# =========================================================
# CAMERA
# =========================================================

p.resetDebugVisualizerCamera(
    cameraDistance=7,
    cameraYaw=45,
    cameraPitch=-25,
    cameraTargetPosition=[
        pos[0],
        pos[1],
        pos[2]
    ]
)


# =========================================================
# START VOICE THREAD
# =========================================================

threading.Thread(
    target=voice_thread,
    daemon=True
).start()


print()
print("======================================")
print("       VOICE CONTROLLED DRONE")
print("======================================")
print()
print("Automatic obstacle avoidance: ON")
print("Camera following: ON")
print()
print("Waiting for voice command...")
print()


# =========================================================
# MAIN LOOP
# =========================================================

prop_angle = 0.0

while running and p.isConnected():

    # -----------------------------------------
    # AUTOMATIC OBSTACLE AVOIDANCE
    # -----------------------------------------

    avoided = automatic_avoidance()

    # -----------------------------------------
    # IF OBSTACLE IS NOT BEING AVOIDED
    # -----------------------------------------

    if not avoided:

        # Convert local movement
        # to world movement

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_vx = (
            velocity_x * cos_yaw
            -
            velocity_y * sin_yaw
        )

        world_vy = (
            velocity_x * sin_yaw
            +
            velocity_y * cos_yaw
        )

        # -------------------------------------
        # MOVE
        # -------------------------------------

        pos[0] += world_vx * DT

        pos[1] += world_vy * DT

        pos[2] += velocity_z * DT

    else:

        # -------------------------------------
        # AVOIDANCE MOVEMENT
        # -------------------------------------

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_vx = (
            velocity_x * cos_yaw
            -
            velocity_y * sin_yaw
        )

        world_vy = (
            velocity_x * sin_yaw
            +
            velocity_y * cos_yaw
        )

        pos[0] += world_vx * DT

        pos[1] += world_vy * DT

        pos[2] += velocity_z * DT


    # -----------------------------------------
    # ROTATION
    # -----------------------------------------

    yaw += rotation_velocity * DT


    # -----------------------------------------
    # TAKEOFF AUTO STOP
    # -----------------------------------------

    if current_command == "TAKE OFF":

        if pos[2] >= 2.0:

            pos[2] = 2.0

            velocity_z = 0

            current_command = "HOVER"

            print(">> TAKEOFF COMPLETE - HOVERING")


    # -----------------------------------------
    # LAND AUTO STOP
    # -----------------------------------------

    if current_command == "LAND":

        if pos[2] <= MIN_HEIGHT:

            pos[2] = MIN_HEIGHT

            velocity_z = 0

            current_command = "LANDED"

            print(">> LANDING COMPLETE")


    # -----------------------------------------
    # LIMIT POSITION
    # -----------------------------------------

    pos[0] = max(
        -12,
        min(12, pos[0])
    )

    pos[1] = max(
        -12,
        min(12, pos[1])
    )

    pos[2] = max(
        MIN_HEIGHT,
        min(MAX_HEIGHT, pos[2])
    )


    # -----------------------------------------
    # PROPELLERS
    # -----------------------------------------

    prop_angle += 20 * DT


    # -----------------------------------------
    # UPDATE DRONE
    # -----------------------------------------

    update_drone(prop_angle)


    # =====================================================
    # CAMERA FOLLOWS DRONE
    # =====================================================

    p.resetDebugVisualizerCamera(
        cameraDistance=7,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[
            pos[0],
            pos[1],
            pos[2]
        ]
    )


    # -----------------------------------------
    # PYBULLET
    # -----------------------------------------

    p.stepSimulation()

    time.sleep(DT)


# =========================================================
# CLOSE
# =========================================================

if p.isConnected():

    p.disconnect()

print()
print("Drone simulation closed.")