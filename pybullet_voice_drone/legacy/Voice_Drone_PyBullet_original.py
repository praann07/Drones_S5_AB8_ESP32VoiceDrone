
import math
import time
import threading
import pybullet as p
import pybullet_data

try:
    import speech_recognition as sr
    VOICE_AVAILABLE = True
except ImportError:
    VOICE_AVAILABLE = False

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 120.0)
p.setRealTimeSimulation(False)
p.loadURDF("plane.urdf")

drone_pos = [0.0, 0.0, 1.5]
yaw = 0.0
flying = True
running = True

STEP = 0.8
VERTICAL_STEP = 0.5
YAW_STEP = math.radians(20)

BODY_X, BODY_Y, BODY_Z = 1.0, 0.55, 0.18
ARM_RADIUS = 0.055
MOTOR_RADIUS, MOTOR_HEIGHT = 0.12, 0.08
PROP_RADIUS, PROP_THICKNESS = 0.38, 0.025

parts = {}

def make_box(half_extents, position, color):
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                 rgbaColor=color)
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=visual, basePosition=position)

def make_cylinder(radius, height, position, color):
    visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius,
                                 length=height, rgbaColor=color)
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1,
                             baseVisualShapeIndex=visual, basePosition=position)

def rotate_xy(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c*x - s*y, s*x + c*y

def local_to_world(x, y, z=0):
    xx, yy = rotate_xy(x, y, yaw)
    return [drone_pos[0] + xx, drone_pos[1] + yy, drone_pos[2] + z]

def make_prop(position, color):
    return make_box([PROP_RADIUS, PROP_THICKNESS, 0.012], position, color)

def build_drone():
    parts["body"] = make_box([BODY_X/2, BODY_Y/2, BODY_Z/2],
                             local_to_world(0,0,0), [0.12,0.12,0.16,1])
    parts["top"] = make_box([0.42,0.25,0.035],
                            local_to_world(0,0,0.15), [0.08,0.25,0.75,1])
    parts["front"] = make_box([0.22,0.07,0.07],
                              local_to_world(0.50,0,0.15), [0.9,0.1,0.1,1])

    motors = [("fl",.72,.72),("fr",.72,-.72),("rl",- .72,.72),("rr",- .72,- .72)]
    for name, x, y in motors:
        length = math.sqrt(x*x+y*y)
        angle = math.atan2(y,x)
        pos = local_to_world(x/2,y/2,0)
        visual = p.createVisualShape(p.GEOM_CYLINDER, radius=ARM_RADIUS,
                                     length=length*1.45,
                                     rgbaColor=[0.08,0.08,0.08,1])
        parts["arm_"+name] = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=visual,
            basePosition=pos,
            baseOrientation=p.getQuaternionFromEuler([0,math.pi/2,-angle]))
        parts["motor_"+name] = make_cylinder(
            MOTOR_RADIUS,MOTOR_HEIGHT,local_to_world(x,y,.10),
            [0.04,0.04,0.04,1])
        parts["prop1_"+name] = make_prop(local_to_world(x,y,.17),
                                         [0.85,0.85,0.85,.9])
        parts["prop2_"+name] = make_prop(local_to_world(x,y,.175),
                                         [0.85,0.85,0.85,.9])

def update_drone(prop_angle):
    motors = [("fl",.72,.72),("fr",.72,-.72),("rl",- .72,.72),("rr",- .72,- .72)]

    for key, xyz in [
        ("body",(0,0,0)), ("top",(0,0,.15)), ("front",(.50,0,.15))
    ]:
        p.resetBasePositionAndOrientation(
            parts[key], local_to_world(*xyz),
            p.getQuaternionFromEuler([0,0,yaw]))

    for name,x,y in motors:
        length = math.sqrt(x*x+y*y)
        angle = math.atan2(y,x)+yaw
        p.resetBasePositionAndOrientation(
            parts["arm_"+name], local_to_world(x/2,y/2,0),
            p.getQuaternionFromEuler([0,math.pi/2,-angle]))
        p.resetBasePositionAndOrientation(
            parts["motor_"+name], local_to_world(x,y,.10),
            p.getQuaternionFromEuler([0,0,yaw]))

        pos = local_to_world(x,y,.17)
        p.resetBasePositionAndOrientation(
            parts["prop1_"+name], pos,
            p.getQuaternionFromEuler([0,0,yaw+prop_angle]))
        p.resetBasePositionAndOrientation(
            parts["prop2_"+name], pos,
            p.getQuaternionFromEuler([0,0,yaw+prop_angle+math.pi/2]))

def execute_command(command):
    global drone_pos, yaw, flying, running
    command = command.lower().strip()
    print("COMMAND:", command)

    if "take off" in command or "takeoff" in command:
        flying = True
        drone_pos[2] = max(drone_pos[2],1.5)
    elif "land" in command:
        flying = False
        drone_pos[2] = 0.25
    elif "forward" in command:
        drone_pos[0] += STEP*math.cos(yaw)
        drone_pos[1] += STEP*math.sin(yaw)
    elif "backward" in command or command == "back":
        drone_pos[0] -= STEP*math.cos(yaw)
        drone_pos[1] -= STEP*math.sin(yaw)
    elif command == "left":
        drone_pos[0] += STEP*math.cos(yaw+math.pi/2)
        drone_pos[1] += STEP*math.sin(yaw+math.pi/2)
    elif command == "right":
        drone_pos[0] += STEP*math.cos(yaw-math.pi/2)
        drone_pos[1] += STEP*math.sin(yaw-math.pi/2)
    elif command == "up" or "rise" in command:
        drone_pos[2] += VERTICAL_STEP
        flying = True
    elif command == "down" or "descend" in command:
        drone_pos[2] = max(.25,drone_pos[2]-VERTICAL_STEP)
    elif "rotate left" in command or "turn left" in command:
        yaw += YAW_STEP
    elif "rotate right" in command or "turn right" in command:
        yaw -= YAW_STEP
    elif "exit" in command or "quit" in command:
        running = False

    drone_pos[0] = max(-15,min(15,drone_pos[0]))
    drone_pos[1] = max(-15,min(15,drone_pos[1]))
    drone_pos[2] = max(.25,min(8,drone_pos[2]))

def voice_loop():
    if not VOICE_AVAILABLE:
        print("SpeechRecognition not available.")
        return
    recognizer = sr.Recognizer()
    try:
        mic = sr.Microphone()
    except Exception as e:
        print("Microphone error:",e)
        return
    with mic as source:
        print("Calibrating microphone...")
        recognizer.adjust_for_ambient_noise(source,duration=1)
    print("VOICE CONTROL READY")
    while running:
        try:
            with mic as source:
                audio = recognizer.listen(source,timeout=2,phrase_time_limit=3)
            text = recognizer.recognize_google(audio)
            print("YOU SAID:",text)
            execute_command(text)
        except sr.WaitTimeoutError:
            pass
        except sr.UnknownValueError:
            pass
        except sr.RequestError as e:
            print("Speech service error:",e)
        except Exception as e:
            print("Voice error:",e)

build_drone()
p.resetDebugVisualizerCamera(6.5,45,-30,drone_pos)
threading.Thread(target=voice_loop,daemon=True).start()

print("\n==========================================")
print(" VOICE CONTROLLED DRONE - PYBULLET")
print("==========================================")
print("Say: TAKE OFF")
print("Then: FORWARD / BACKWARD / LEFT / RIGHT")
print("      UP / DOWN / ROTATE LEFT / ROTATE RIGHT")
print("Finally: LAND")
print("Say EXIT to close.")
print("==========================================\n")

prop_angle = 0.0
while running and p.isConnected():
    prop_angle += 0.45
    update_drone(prop_angle)
    p.resetDebugVisualizerCamera(6.5,45,-30,drone_pos)
    p.stepSimulation()
    time.sleep(1/120)

if p.isConnected():
    p.disconnect()
