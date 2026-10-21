import math
import time
import threading

import pybullet as p
import pybullet_data


# =========================================================
# SETTINGS
# =========================================================

DT = 1.0 / 120.0

HORIZONTAL_SPEED = 1.0
VERTICAL_SPEED = 0.8
ROTATION_SPEED = math.radians(45)

TAKEOFF_HEIGHT = 2.0

MIN_HEIGHT = 0.5
MAX_HEIGHT = 8.0

CAMERA_DISTANCE = 7
CAMERA_YAW = 45
CAMERA_PITCH = -30

# Safety distance around the drone
DRONE_SAFETY_RADIUS = 0.75

running = True


# =========================================================
# DRONE STATE
# =========================================================

pos = [0.0, 0.0, 0.5]

yaw = 0.0

velocity_forward = 0.0
velocity_right = 0.0
velocity_vertical = 0.0

rotation_velocity = 0.0

auto_takeoff = False
auto_land = False

current_command = "STOP"

objects = {}


# =========================================================
# OBSTACLES
# =========================================================

# x, y, z, half_x, half_y, half_z

obstacle_data = [

    # Obstacle 1 - in front
    [4.0, 0.0, 1.0, 0.7, 0.7, 1.0],

    # Obstacle 2 - left side
    [1.5, 3.0, 1.0, 0.7, 0.7, 1.0],

    # Obstacle 3 - right side
    [-2.5, -3.0, 1.0, 0.7, 0.7, 1.0]
]

obstacles = []


# =========================================================
# CONNECT TO PYBULLET
# =========================================================

print()
print("======================================")
print("STARTING PYBULLET")
print("======================================")

client = p.connect(p.GUI)

if client < 0:

    print()
    print("ERROR: Could not connect to PyBullet GUI.")
    print("Close any old PyBullet windows and try again.")
    raise SystemExit

print(
    "PyBullet connected successfully."
)

print(
    "Client ID:",
    client
)

# ---------------------------------------------------------
# PYBULLET SETTINGS
# ---------------------------------------------------------

p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)

p.setGravity(
    0,
    0,
    -9.81
)

p.setTimeStep(
    DT
)

p.setRealTimeSimulation(
    False
)

# ---------------------------------------------------------
# LOAD GROUND
# ---------------------------------------------------------

p.loadURDF(
    "plane.urdf"
)

print(
    "Ground loaded successfully."
)

print(
    "======================================"
)


# =========================================================
# COORDINATE SYSTEM
# =========================================================

def world(local_x, local_y, local_z):

    c = math.cos(yaw)
    s = math.sin(yaw)

    x = (
        pos[0]
        + local_x * c
        - local_y * s
    )

    y = (
        pos[1]
        + local_x * s
        + local_y * c
    )

    z = (
        pos[2]
        + local_z
    )

    return [
        x,
        y,
        z
    ]


def quat():

    return p.getQuaternionFromEuler(
        [
            0,
            0,
            yaw
        ]
    )


# =========================================================
# CREATE BOX
# =========================================================

def box(
    name,
    half,
    local,
    color
):

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half,
        rgbaColor=color
    )

    objects[name] = p.createMultiBody(

        baseMass=0,

        baseCollisionShapeIndex=-1,

        baseVisualShapeIndex=visual,

        basePosition=world(
            *local
        ),

        baseOrientation=quat()
    )


# =========================================================
# CREATE CYLINDER
# =========================================================

def cylinder(
    name,
    radius,
    height,
    local,
    color
):

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

        basePosition=world(
            *local
        ),

        baseOrientation=quat()
    )


# =========================================================
# CREATE OBSTACLE
# =========================================================

def create_obstacle(data):

    x, y, z, hx, hy, hz = data

    collision = p.createCollisionShape(

        p.GEOM_BOX,

        halfExtents=[
            hx,
            hy,
            hz
        ]
    )

    visual = p.createVisualShape(

        p.GEOM_BOX,

        halfExtents=[
            hx,
            hy,
            hz
        ],

        rgbaColor=[
            0.9,
            0.15,
            0.15,
            1
        ]
    )

    obstacle_id = p.createMultiBody(

        baseMass=0,

        baseCollisionShapeIndex=collision,

        baseVisualShapeIndex=visual,

        basePosition=[
            x,
            y,
            z
        ]
    )

    obstacles.append(
        obstacle_id
    )


def create_obstacles():

    for data in obstacle_data:

        create_obstacle(
            data
        )

    print()
    print(
        "3 SMALL OBSTACLES CREATED"
    )
    print()


# =========================================================
# BUILD DRONE
# =========================================================

def build_drone():

    print(
        "Building drone..."
    )

    # -----------------------------------------------------
    # MAIN BODY
    # -----------------------------------------------------

    box(
        "body",

        [
            0.42,
            0.30,
            0.09
        ],

        (
            0,
            0,
            0
        ),

        [
            0.08,
            0.08,
            0.10,
            1
        ]
    )

    # -----------------------------------------------------
    # TOP
    # -----------------------------------------------------

    box(
        "top",

        [
            0.28,
            0.20,
            0.035
        ],

        (
            0,
            0,
            0.13
        ),

        [
            0.05,
            0.22,
            0.70,
            1
        ]
    )

    # -----------------------------------------------------
    # FRONT MARKER
    # -----------------------------------------------------

    box(
        "front",

        [
            0.13,
            0.055,
            0.05
        ],

        (
            0.45,
            0,
            0.10
        ),

        [
            0.9,
            0.08,
            0.08,
            1
        ]
    )

    motors = [

        (
            "FL",
            0.72,
            0.72
        ),

        (
            "FR",
            0.72,
            -0.72
        ),

        (
            "RL",
            -0.72,
            0.72
        ),

        (
            "RR",
            -0.72,
            -0.72
        )
    ]

    for name, x, y in motors:

        # -------------------------------------------------
        # ARM
        # -------------------------------------------------

        angle = math.atan2(
            y,
            x
        ) + yaw

        length = math.sqrt(
            x * x
            +
            y * y
        )

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

        objects[
            "arm_" + name
        ] = p.createMultiBody(

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
                [
                    0,
                    0,
                    angle
                ]
            )
        )

        # -------------------------------------------------
        # MOTOR
        # -------------------------------------------------

        cylinder(

            "motor_" + name,

            0.10,

            0.10,

            (
                x,
                y,
                0.10
            ),

            [
                0.03,
                0.03,
                0.03,
                1
            ]
        )

        # -------------------------------------------------
        # PROPELLER A
        # -------------------------------------------------

        box(

            "propA_" + name,

            [
                0.30,
                0.018,
                0.008
            ],

            (
                x,
                y,
                0.18
            ),

            [
                0.85,
                0.85,
                0.85,
                1
            ]
        )

        # -------------------------------------------------
        # PROPELLER B
        # -------------------------------------------------

        box(

            "propB_" + name,

            [
                0.018,
                0.30,
                0.008
            ],

            (
                x,
                y,
                0.185
            ),

            [
                0.65,
                0.65,
                0.65,
                1
            ]
        )

    print(
        "Drone built successfully."
    )


# =========================================================
# UPDATE DRONE
# =========================================================

def update_drone(prop_angle):

    motors = [

        (
            "FL",
            0.72,
            0.72
        ),

        (
            "FR",
            0.72,
            -0.72
        ),

        (
            "RL",
            -0.72,
            0.72
        ),

        (
            "RR",
            -0.72,
            -0.72
        )
    ]

    # -----------------------------------------------------
    # BODY
    # -----------------------------------------------------

    p.resetBasePositionAndOrientation(

        objects["body"],

        world(
            0,
            0,
            0
        ),

        quat()
    )

    # -----------------------------------------------------
    # TOP
    # -----------------------------------------------------

    p.resetBasePositionAndOrientation(

        objects["top"],

        world(
            0,
            0,
            0.13
        ),

        quat()
    )

    # -----------------------------------------------------
    # FRONT
    # -----------------------------------------------------

    p.resetBasePositionAndOrientation(

        objects["front"],

        world(
            0.45,
            0,
            0.10
        ),

        quat()
    )

    # -----------------------------------------------------
    # MOTORS
    # -----------------------------------------------------

    for name, x, y in motors:

        angle = math.atan2(
            y,
            x
        ) + yaw

        # Arm

        p.resetBasePositionAndOrientation(

            objects[
                "arm_" + name
            ],

            world(
                x / 2,
                y / 2,
                0
            ),

            p.getQuaternionFromEuler(
                [
                    0,
                    0,
                    angle
                ]
            )
        )

        # Motor

        p.resetBasePositionAndOrientation(

            objects[
                "motor_" + name
            ],

            world(
                x,
                y,
                0.10
            ),

            quat()
        )

        # Propeller

        a = (
            yaw
            +
            prop_angle
        )

        p.resetBasePositionAndOrientation(

            objects[
                "propA_" + name
            ],

            world(
                x,
                y,
                0.18
            ),

            p.getQuaternionFromEuler(
                [
                    0,
                    0,
                    a
                ]
            )
        )

        p.resetBasePositionAndOrientation(

            objects[
                "propB_" + name
            ],

            world(
                x,
                y,
                0.185
            ),

            p.getQuaternionFromEuler(
                [
                    0,
                    0,
                    a + math.pi / 2
                ]
            )
        )


# =========================================================
# OBSTACLE SAFETY CHECK
# =========================================================

def position_is_safe(
    test_x,
    test_y,
    test_z
):

    for obstacle in obstacle_data:

        ox, oy, oz, hx, hy, hz = obstacle

        # Expand obstacle
        # to create safety zone

        min_x = (
            ox
            - hx
            - DRONE_SAFETY_RADIUS
        )

        max_x = (
            ox
            + hx
            + DRONE_SAFETY_RADIUS
        )

        min_y = (
            oy
            - hy
            - DRONE_SAFETY_RADIUS
        )

        max_y = (
            oy
            + hy
            + DRONE_SAFETY_RADIUS
        )

        min_z = (
            oz
            - hz
            - DRONE_SAFETY_RADIUS
        )

        max_z = (
            oz
            + hz
            + DRONE_SAFETY_RADIUS
        )

        if (

            min_x <= test_x <= max_x

            and

            min_y <= test_y <= max_y

            and

            min_z <= test_z <= max_z

        ):

            return False

    return True


# =========================================================
# SAFE MOVEMENT
# =========================================================

def calculate_safe_velocity(
    vx,
    vy
):

    next_x = (
        pos[0]
        +
        vx * DT
    )

    next_y = (
        pos[1]
        +
        vy * DT
    )

    next_z = pos[2]

    # -----------------------------------------------------
    # PATH CLEAR
    # -----------------------------------------------------

    if position_is_safe(

        next_x,
        next_y,
        next_z

    ):

        return (
            vx,
            vy
        )

    # -----------------------------------------------------
    # OBSTACLE FOUND
    # -----------------------------------------------------

    print(
        "OBSTACLE DETECTED!"
    )

    speed = math.sqrt(

        vx * vx
        +
        vy * vy
    )

    if speed < 0.001:

        return (
            0,
            0
        )

    # Normalize

    dx = vx / speed
    dy = vy / speed

    # -----------------------------------------------------
    # TRY LEFT
    # -----------------------------------------------------

    left_x = -dy
    left_y = dx

    left_test_x = (

        pos[0]
        +
        left_x * speed * DT
        +
        dx * 0.3
    )

    left_test_y = (

        pos[1]
        +
        left_y * speed * DT
        +
        dy * 0.3
    )

    if position_is_safe(

        left_test_x,
        left_test_y,
        pos[2]

    ):

        print(
            "AVOIDING: LEFT"
        )

        return (

            left_x * speed,
            left_y * speed
        )

    # -----------------------------------------------------
    # TRY RIGHT
    # -----------------------------------------------------

    right_x = dy
    right_y = -dx

    right_test_x = (

        pos[0]
        +
        right_x * speed * DT
        +
        dx * 0.3
    )

    right_test_y = (

        pos[1]
        +
        right_y * speed * DT
        +
        dy * 0.3
    )

    if position_is_safe(

        right_test_x,
        right_test_y,
        pos[2]

    ):

        print(
            "AVOIDING: RIGHT"
        )

        return (

            right_x * speed,
            right_y * speed
        )

    # -----------------------------------------------------
    # BOTH SIDES BLOCKED
    # -----------------------------------------------------

    print(
        "OBSTACLE TOO CLOSE - STOPPING"
    )

    return (
        0,
        0
    )


# =========================================================
# STOP DRONE
# =========================================================

def stop_drone():

    global velocity_forward
    global velocity_right
    global velocity_vertical
    global rotation_velocity
    global auto_takeoff
    global auto_land

    velocity_forward = 0
    velocity_right = 0
    velocity_vertical = 0
    rotation_velocity = 0

    auto_takeoff = False
    auto_land = False


# =========================================================
# COMMAND PROCESSING
# =========================================================

def command(text):

    global velocity_forward
    global velocity_right
    global velocity_vertical
    global rotation_velocity

    global running
    global auto_takeoff
    global auto_land
    global current_command

    text = text.lower().strip()

    print()
    print(
        "YOU SAID:",
        text
    )

    # =====================================================
    # TAKE OFF
    # =====================================================

    if (

        "take off" in text

        or

        "takeoff" in text

        or

        text == "take"

    ):

        stop_drone()

        auto_takeoff = True

        current_command = "TAKE OFF"

        print(
            ">> TAKE OFF"
        )

        return

    # =====================================================
    # LAND
    # =====================================================

    if "land" in text:

        stop_drone()

        auto_land = True

        current_command = "LAND"

        print(
            ">> LAND"
        )

        return

    # =====================================================
    # STOP
    # =====================================================

    if (

        text == "stop"

        or

        text == "halt"

        or

        text == "freeze"

    ):

        stop_drone()

        current_command = "STOP"

        print(
            ">> STOP"
        )

        return

    # =====================================================
    # FORWARD
    # =====================================================

    if (

        "forward" in text

        or

        "go forward" in text

        or

        "move forward" in text

    ):

        stop_drone()

        velocity_forward = (
            HORIZONTAL_SPEED
        )

        current_command = "FORWARD"

        print(
            ">> FORWARD"
        )

        return

    # =====================================================
    # BACKWARD
    # =====================================================

    if (

        "backward" in text

        or

        "go backward" in text

        or

        "go back" in text

        or

        text == "back"

    ):

        stop_drone()

        velocity_forward = (
            -HORIZONTAL_SPEED
        )

        current_command = "BACKWARD"

        print(
            ">> BACKWARD"
        )

        return

    # =====================================================
    # LEFT
    # =====================================================

    if (

        text == "left"

        or

        "go left" in text

        or

        "move left" in text

    ):

        stop_drone()

        velocity_right = (
            -HORIZONTAL_SPEED
        )

        current_command = "LEFT"

        print(
            ">> LEFT"
        )

        return

    # =====================================================
    # RIGHT
    # =====================================================

    if (

        text == "right"

        or

        "go right" in text

        or

        "move right" in text

    ):

        stop_drone()

        velocity_right = (
            HORIZONTAL_SPEED
        )

        current_command = "RIGHT"

        print(
            ">> RIGHT"
        )

        return

    # =====================================================
    # UP
    # =====================================================

    if (

        text == "up"

        or

        "go up" in text

        or

        "move up" in text

        or

        "rise" in text

    ):

        stop_drone()

        velocity_vertical = (
            VERTICAL_SPEED
        )

        current_command = "UP"

        print(
            ">> UP"
        )

        return

    # =====================================================
    # DOWN
    # =====================================================

    if (

        text == "down"

        or

        "go down" in text

        or

        "move down" in text

        or

        "descend" in text

    ):

        stop_drone()

        velocity_vertical = (
            -VERTICAL_SPEED
        )

        current_command = "DOWN"

        print(
            ">> DOWN"
        )

        return

    # =====================================================
    # ROTATE LEFT
    # =====================================================

    if (

        "rotate left" in text

        or

        "turn left" in text

    ):

        stop_drone()

        rotation_velocity = (
            ROTATION_SPEED
        )

        current_command = (
            "ROTATE LEFT"
        )

        print(
            ">> ROTATE LEFT"
        )

        return

    # =====================================================
    # ROTATE RIGHT
    # =====================================================

    if (

        "rotate right" in text

        or

        "turn right" in text

    ):

        stop_drone()

        rotation_velocity = (
            -ROTATION_SPEED
        )

        current_command = (
            "ROTATE RIGHT"
        )

        print(
            ">> ROTATE RIGHT"
        )

        return

    # =====================================================
    # EXIT
    # =====================================================

    if (

        "exit" in text

        or

        "quit" in text

        or

        "close" in text

    ):

        running = False

        print(
            ">> EXIT"
        )

        return

    print(
        ">> COMMAND NOT RECOGNIZED"
    )


# =========================================================
# VOICE CONTROL
# =========================================================

def voice_thread():

    try:

        import speech_recognition as sr

    except ImportError:

        print()
        print(
            "SpeechRecognition is not installed."
        )

        print(
            "Run:"
        )

        print(
            "pip install SpeechRecognition"
        )

        return

    recognizer = sr.Recognizer()

    recognizer.energy_threshold = 300

    recognizer.dynamic_energy_threshold = True

    recognizer.pause_threshold = 0.6

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
    print(
        "======================================"
    )
    print(
        "VOICE CONTROL READY"
    )
    print(
        "======================================"
    )

    print(
        "take off"
    )

    print(
        "forward"
    )

    print(
        "backward"
    )

    print(
        "left"
    )

    print(
        "right"
    )

    print(
        "up"
    )

    print(
        "down"
    )

    print(
        "rotate left"
    )

    print(
        "rotate right"
    )

    print(
        "stop"
    )

    print(
        "land"
    )

    print(
        "exit"
    )

    print(
        "======================================"
    )

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

            command(
                words
            )

        except sr.WaitTimeoutError:

            pass

        except sr.UnknownValueError:

            pass

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
# BUILD WORLD
# =========================================================

# IMPORTANT:
# PyBullet is already connected above.

create_obstacles()

build_drone()


# =========================================================
# CAMERA
# =========================================================

p.resetDebugVisualizerCamera(

    cameraDistance=CAMERA_DISTANCE,

    cameraYaw=CAMERA_YAW,

    cameraPitch=CAMERA_PITCH,

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


# =========================================================
# START MESSAGE
# =========================================================

print()
print(
    "======================================"
)

print(
    "       VOICE CONTROLLED DRONE"
)

print(
    "       OBSTACLE AVOIDANCE"
)

print(
    "======================================"
)

print()
print(
    "Waiting for your voice command..."
)

print()


# =========================================================
# MAIN SIMULATION LOOP
# =========================================================

prop_angle = 0.0

last_data_time = time.time()


while (

    running

    and

    p.isConnected()

):

    # =====================================================
    # AUTOMATIC TAKEOFF
    # =====================================================

    if auto_takeoff:

        if pos[2] < TAKEOFF_HEIGHT:

            pos[2] += (

                VERTICAL_SPEED
                *
                DT
            )

        else:

            pos[2] = TAKEOFF_HEIGHT

            auto_takeoff = False

            current_command = "HOVER"

            print(
                ">> TAKEOFF COMPLETE"
            )

    # =====================================================
    # AUTOMATIC LANDING
    # =====================================================

    elif auto_land:

        if pos[2] > MIN_HEIGHT:

            pos[2] -= (

                VERTICAL_SPEED
                *
                DT
            )

        else:

            pos[2] = MIN_HEIGHT

            auto_land = False

            current_command = "LANDED"

            print(
                ">> LANDING COMPLETE"
            )

    # =====================================================
    # NORMAL MOVEMENT
    # =====================================================

    else:

        # -------------------------------------------------
        # ROTATION
        # -------------------------------------------------

        yaw += (

            rotation_velocity
            *
            DT
        )

        # -------------------------------------------------
        # LOCAL → WORLD
        # -------------------------------------------------

        cos_yaw = math.cos(
            yaw
        )

        sin_yaw = math.sin(
            yaw
        )

        world_vx = (

            velocity_forward
            *
            cos_yaw

            -

            velocity_right
            *
            sin_yaw
        )

        world_vy = (

            velocity_forward
            *
            sin_yaw

            +

            velocity_right
            *
            cos_yaw
        )

        # -------------------------------------------------
        # OBSTACLE AVOIDANCE
        # -------------------------------------------------

        safe_vx, safe_vy = (
            calculate_safe_velocity(
                world_vx,
                world_vy
            )
        )

        # -------------------------------------------------
        # MOVE
        # -------------------------------------------------

        pos[0] += (

            safe_vx
            *
            DT
        )

        pos[1] += (

            safe_vy
            *
            DT
        )

        # -------------------------------------------------
        # VERTICAL MOVEMENT
        # -------------------------------------------------

        pos[2] += (

            velocity_vertical
            *
            DT
        )

    # =====================================================
    # POSITION LIMITS
    # =====================================================

    pos[0] = max(
        -12,
        min(
            12,
            pos[0]
        )
    )

    pos[1] = max(
        -12,
        min(
            12,
            pos[1]
        )
    )

    pos[2] = max(
        MIN_HEIGHT,
        min(
            MAX_HEIGHT,
            pos[2]
        )
    )

    # =====================================================
    # PROPELLER ROTATION
    # =====================================================

    prop_angle += (

        30
        *
        DT
    )

    # =====================================================
    # UPDATE DRONE
    # =====================================================

    update_drone(
        prop_angle
    )

    # =====================================================
    # CAMERA FOLLOW
    # =====================================================

    p.resetDebugVisualizerCamera(

        cameraDistance=CAMERA_DISTANCE,

        cameraYaw=CAMERA_YAW,

        cameraPitch=CAMERA_PITCH,

        cameraTargetPosition=[

            pos[0],

            pos[1],

            pos[2]
        ]
    )

    # =====================================================
    # DATA-DRIVEN OUTPUT
    # =====================================================

    if (

        time.time()
        -
        last_data_time

        >

        2
    ):

        print(
            "DATA | "
            "Command:",
            current_command,
            "| X:",
            round(
                pos[0],
                2
            ),
            "| Y:",
            round(
                pos[1],
                2
            ),
            "| Height:",
            round(
                pos[2],
                2
            ),
            "| Yaw:",
            round(
                math.degrees(yaw),
                1
            )
        )

        last_data_time = time.time()

    # =====================================================
    # PYBULLET STEP
    # =====================================================

    p.stepSimulation()

    time.sleep(
        DT
    )


# =========================================================
# CLOSE
# =========================================================

if p.isConnected():

    p.disconnect()

print()
print(
    "Drone simulation closed."
)