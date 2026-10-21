
import math
import time
import threading
import pybullet as p
import pybullet_data

# -----------------------------
# SETTINGS
# -----------------------------
DT = 1.0 / 120.0
MOVE = 0.6
VERTICAL = 0.4
TURN = math.radians(15)

# -----------------------------
# CONNECT
# -----------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(DT)
p.setRealTimeSimulation(False)
p.loadURDF("plane.urdf")

# -----------------------------
# DRONE STATE
# -----------------------------
pos = [0.0, 0.0, 2.0]
yaw = 0.0
running = True

objects = {}

def quat():
    return p.getQuaternionFromEuler([0, 0, yaw])

def world(local_x, local_y, local_z):
    c = math.cos(yaw)
    s = math.sin(yaw)
    x = pos[0] + local_x*c - local_y*s
    y = pos[1] + local_x*s + local_y*c
    z = pos[2] + local_z
    return [x, y, z]

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

# -----------------------------
# BUILD A SIMPLE QUADCOPTER
# -----------------------------
def build_drone():
    # central body
    box("body", [0.42, 0.30, 0.09], (0,0,0), [0.08,0.08,0.10,1])

    # top plate
    box("top", [0.28,0.20,0.035], (0,0,0.13), [0.05,0.22,0.70,1])

    # front marker
    box("front", [0.13,0.055,0.05], (0.45,0,0.10), [0.9,0.08,0.08,1])

    motors = [
        ("FL",  0.72,  0.72),
        ("FR",  0.72, -0.72),
        ("RL", -0.72,  0.72),
        ("RR", -0.72, -0.72),
    ]

    for name, x, y in motors:
        # arm: long thin box rotated in XY plane
        angle = math.atan2(y, x) + yaw
        length = math.sqrt(x*x + y*y)
        visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[length/2, 0.035, 0.035],
            rgbaColor=[0.03,0.03,0.03,1]
        )
        objects["arm_"+name] = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual,
            basePosition=world(x/2,y/2,0),
            baseOrientation=p.getQuaternionFromEuler([0,0,angle])
        )

        # motor
        cylinder("motor_"+name, 0.10, 0.10, (x,y,0.10), [0.03,0.03,0.03,1])

        # propeller: two thin crossed blades
        box("propA_"+name, [0.30,0.018,0.008], (x,y,0.18), [0.85,0.85,0.85,1])
        box("propB_"+name, [0.018,0.30,0.008], (x,y,0.185), [0.65,0.65,0.65,1])

# -----------------------------
# UPDATE DRONE VISUALS
# -----------------------------
def update_drone(prop_angle):
    motors = [
        ("FL",  0.72,  0.72),
        ("FR",  0.72, -0.72),
        ("RL", -0.72,  0.72),
        ("RR", -0.72, -0.72),
    ]

    p.resetBasePositionAndOrientation(objects["body"], world(0,0,0), quat())
    p.resetBasePositionAndOrientation(objects["top"], world(0,0,0.13), quat())
    p.resetBasePositionAndOrientation(objects["front"], world(0.45,0,0.10), quat())

    for name,x,y in motors:
        angle = math.atan2(y,x) + yaw
        p.resetBasePositionAndOrientation(
            objects["arm_"+name],
            world(x/2,y/2,0),
            p.getQuaternionFromEuler([0,0,angle])
        )
        p.resetBasePositionAndOrientation(
            objects["motor_"+name],
            world(x,y,0.10),
            quat()
        )

        # Rotate the two blades around the vertical axis.
        a = yaw + prop_angle
        ca, sa = math.cos(a), math.sin(a)

        # blade A direction
        p.resetBasePositionAndOrientation(
            objects["propA_"+name],
            world(x,y,0.18),
            p.getQuaternionFromEuler([0,0,a])
        )

        # blade B is 90 degrees from A
        p.resetBasePositionAndOrientation(
            objects["propB_"+name],
            world(x,y,0.185),
            p.getQuaternionFromEuler([0,0,a + math.pi/2])
        )

# -----------------------------
# COMMANDS
# -----------------------------
def command(text):
    global yaw, running

    text = text.lower().strip()
    print("COMMAND:", text)

    if "take off" in text or "takeoff" in text:
        pos[2] = 2.0

    elif "land" in text:
        pos[2] = 0.35

    elif "forward" in text:
        pos[0] += MOVE * math.cos(yaw)
        pos[1] += MOVE * math.sin(yaw)

    elif "backward" in text or text == "back":
        pos[0] -= MOVE * math.cos(yaw)
        pos[1] -= MOVE * math.sin(yaw)

    elif text == "left":
        pos[0] += MOVE * math.cos(yaw + math.pi/2)
        pos[1] += MOVE * math.sin(yaw + math.pi/2)

    elif text == "right":
        pos[0] += MOVE * math.cos(yaw - math.pi/2)
        pos[1] += MOVE * math.sin(yaw - math.pi/2)

    elif text == "up" or "rise" in text:
        pos[2] += VERTICAL

    elif text == "down" or "descend" in text:
        pos[2] = max(0.35, pos[2] - VERTICAL)

    elif "rotate left" in text or "turn left" in text:
        yaw += TURN

    elif "rotate right" in text or "turn right" in text:
        yaw -= TURN

    elif "exit" in text or "quit" in text:
        running = False

    # keep drone in a safe area
    pos[0] = max(-12, min(12, pos[0]))
    pos[1] = max(-12, min(12, pos[1]))
    pos[2] = max(0.35, min(8, pos[2]))

# -----------------------------
# OPTIONAL VOICE CONTROL
# -----------------------------
def voice_thread():
    try:
        import speech_recognition as sr
    except ImportError:
        print("SpeechRecognition not installed. Keyboard control still works.")
        return

    r = sr.Recognizer()

    try:
        mic = sr.Microphone()
    except Exception as e:
        print("Microphone unavailable:", e)
        return

    with mic as source:
        print("Calibrating microphone...")
        r.adjust_for_ambient_noise(source, duration=1)

    print("VOICE CONTROL READY")
    print("Say: take off, forward, backward, left, right, up, down,")
    print("rotate left, rotate right, land, exit")

    while running:
        try:
            with mic as source:
                audio = r.listen(source, timeout=2, phrase_time_limit=3)
            words = r.recognize_google(audio)
            print("YOU SAID:", words)
            command(words)
        except sr.WaitTimeoutError:
            pass
        except sr.UnknownValueError:
            pass
        except sr.RequestError:
            print("Internet speech recognition unavailable.")
        except Exception as e:
            print("Voice error:", e)

# -----------------------------
# START
# -----------------------------
build_drone()

# IMPORTANT:
# Camera stays fixed instead of following the drone.
# This prevents the disappearing/glitching effect while moving.
p.resetDebugVisualizerCamera(
    cameraDistance=8,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0,0,1.5]
)

threading.Thread(target=voice_thread, daemon=True).start()

print()
print("======================================")
print("   VOICE CONTROLLED DRONE - PYBULLET")
print("======================================")
print("TAKE OFF")
print("FORWARD / BACKWARD")
print("LEFT / RIGHT")
print("UP / DOWN")
print("ROTATE LEFT / ROTATE RIGHT")
print("LAND")
print("EXIT")
print("======================================")
print()

prop_angle = 0.0

while running and p.isConnected():
    prop_angle += 0.35
    update_drone(prop_angle)

    # Keep the camera fixed. Do NOT reset it to the drone every frame.
    p.stepSimulation()
    time.sleep(DT)

if p.isConnected():
    p.disconnect()
