import pybullet as p
import pybullet_data
import time
import math
import threading

# ============================================================
# CONNECT TO PYBULLET
# ============================================================

physics_client = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Ground
p.loadURDF("plane.urdf")

# ============================================================
# DRONE PARAMETERS
# ============================================================

DRONE_START = [0, 0, 1.5]

drone_x = 0.0
drone_y = 0.0
drone_z = 1.5

speed = 2.0
vertical_speed = 1.5

# ============================================================
# CREATE DRONE BODY
# ============================================================

body_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[0.35, 0.35, 0.12]
)

body_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.35, 0.35, 0.12],
    rgbaColor=[0.1, 0.1, 0.1, 1]
)

drone = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=body_collision,
    baseVisualShapeIndex=body_visual,
    basePosition=DRONE_START
)

# ============================================================
# CREATE ARMS
# ============================================================

arm_positions = [
    [0.75, 0, 1.5],
    [-0.75, 0, 1.5],
    [0, 0.75, 1.5],
    [0, -0.75, 1.5]
]

arm_ids = []

for pos in arm_positions:

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[0.45, 0.07, 0.05]
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.45, 0.07, 0.05],
        rgbaColor=[0.15, 0.15, 0.15, 1]
    )

    arm = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=pos
    )

    arm_ids.append(arm)

# ============================================================
# CREATE MOTOR MOUNTS
# ============================================================

motor_positions = [
    [0.85, 0, 1.58],
    [-0.85, 0, 1.58],
    [0, 0.85, 1.58],
    [0, -0.85, 1.58]
]

motor_ids = []

for pos in motor_positions:

    collision = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=0.09,
        height=0.12
    )

    visual = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=0.09,
        length=0.12,
        rgbaColor=[0.05, 0.05, 0.05, 1]
    )

    motor = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=pos
    )

    motor_ids.append(motor)

# ============================================================
# CREATE PROPELLERS
# ============================================================

prop_positions = [
    [0.85, 0, 1.67],
    [-0.85, 0, 1.67],
    [0, 0.85, 1.67],
    [0, -0.85, 1.67]
]

prop_ids = []

for pos in prop_positions:

    prop_collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[0.35, 0.025, 0.015]
    )

    prop_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[0.35, 0.025, 0.015],
        rgbaColor=[0.05, 0.05, 0.05, 1]
    )

    prop = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=prop_collision,
        baseVisualShapeIndex=prop_visual,
        basePosition=pos
    )

    prop_ids.append(prop)

# ============================================================
# CAMERA
# ============================================================

p.resetDebugVisualizerCamera(
    cameraDistance=5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=DRONE_START
)

# ============================================================
# COMMAND VARIABLE
# ============================================================

command = "hover"

# ============================================================
# DRONE MOVEMENT
# ============================================================

def move_drone():

    global drone_x, drone_y, drone_z

    if command == "forward":
        drone_y += speed * 0.02

    elif command == "backward":
        drone_y -= speed * 0.02

    elif command == "left":
        drone_x -= speed * 0.02

    elif command == "right":
        drone_x += speed * 0.02

    elif command == "up":
        drone_z += vertical_speed * 0.02

    elif command == "down":
        drone_z -= vertical_speed * 0.02

        if drone_z < 0.5:
            drone_z = 0.5

    elif command == "land":
        drone_z -= vertical_speed * 0.02

        if drone_z < 0.2:
            drone_z = 0.2

    elif command == "takeoff":

        if drone_z < 1.5:
            drone_z += vertical_speed * 0.02

    # Keep drone position stable
    p.resetBasePositionAndOrientation(
        drone,
        [drone_x, drone_y, drone_z],
        [0, 0, 0, 1]
    )

    # Move arms
    for arm, pos in zip(arm_ids, arm_positions):

        offset_x = pos[0]
        offset_y = pos[1]

        p.resetBasePositionAndOrientation(
            arm,
            [
                drone_x + offset_x,
                drone_y + offset_y,
                drone_z
            ],
            [0, 0, 0, 1]
        )

    # Move motors
    for motor, pos in zip(motor_ids, motor_positions):

        offset_x = pos[0]
        offset_y = pos[1]

        p.resetBasePositionAndOrientation(
            motor,
            [
                drone_x + offset_x,
                drone_y + offset_y,
                drone_z + 0.08
            ],
            [0, 0, 0, 1]
        )

# ============================================================
# PROPELLER ROTATION
# ============================================================

prop_angle = 0

def rotate_propellers():

    global prop_angle

    prop_angle += 0.5

    for i, prop in enumerate(prop_ids):

        x, y, z = prop_positions[i]

        # Alternate rotation directions
        if i % 2 == 0:
            angle = prop_angle
        else:
            angle = -prop_angle

        # Rotation around Z axis
        quat = p.getQuaternionFromEuler(
            [0, 0, angle]
        )

        p.resetBasePositionAndOrientation(
            prop,
            [
                drone_x + x,
                drone_y + y,
                drone_z + 0.17
            ],
            quat
        )

# ============================================================
# KEYBOARD CONTROL
# ============================================================

def keyboard_control():

    global command

    keys = p.getKeyboardEvents()

    if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
        command = "forward"

    elif ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        command = "backward"

    elif ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
        command = "left"

    elif ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        command = "right"

    elif ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
        command = "up"

    elif ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
        command = "down"

    else:
        command = "hover"

# ============================================================
# MAIN LOOP
# ============================================================

print("---------------------------------------")
print("     VOICE CONTROLLED DRONE")
print("---------------------------------------")
print("")
print("Keyboard controls:")
print("W = Forward")
print("S = Backward")
print("A = Left")
print("D = Right")
print("Q = Up")
print("E = Down")
print("")
print("Drone simulation started!")
print("---------------------------------------")

while p.isConnected():

    keyboard_control()

    move_drone()

    rotate_propellers()

    p.stepSimulation()

    time.sleep(1 / 60)

p.disconnect()
