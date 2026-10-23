"""How much does each kind of PyBullet GUI call cost, with the full city scene loaded?"""
import math, os, sys, time
CLONE = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))     # the project folder (three levels up from this script)
sys.path.insert(0, CLONE)
import pybullet as p
import drone_core as dc

assert dc.connect_gui() >= 0
c = dc.DroneController(dc.DEFAULT_OBSTACLES, log=lambda m: None)
world = dc.World(dc.DEFAULT_OBSTACLES, c.home, "city")
model = dc.DroneModel(c)
time.sleep(1.0)
info = p.getDebugVisualizerCamera()


def bench(name, fn, n=40):
    fn(0)                                    # warm up
    t = time.perf_counter()
    for i in range(1, n + 1):
        fn(i)
    ms = (time.perf_counter() - t) / n * 1000
    print("  %-46s %7.2f ms/call" % (name, ms))
    return ms


body = model.body
halo = world.city.kiosks[0]["halo"]
car = world.city.cars[0]["body"]
text_id = [p.addUserDebugText("x", [0, 0, 3])]
line_id = [p.addUserDebugLine([0, 0, 0], [1, 1, 1], [1, 0, 0])]

print("cost per call (full city scene loaded):")
bench("resetBasePositionAndOrientation (drone)", lambda i: p.resetBasePositionAndOrientation(body, [0, 0, 1 + i * 1e-4], [0, 0, 0, 1]))
bench("resetBasePositionAndOrientation (car)", lambda i: p.resetBasePositionAndOrientation(car, [20, i * 0.01, 0], [0, 0, 0, 1]))
bench("resetJointState (rotor)", lambda i: p.resetJointState(body, model.props[0][0], i * 0.1))
bench("stepSimulation", lambda i: p.stepSimulation())
bench("getDebugVisualizerCamera", lambda i: p.getDebugVisualizerCamera())
bench("resetDebugVisualizerCamera", lambda i: p.resetDebugVisualizerCamera(8, 45, -30, [i * 0.01, 0, 1]))
bench("addUserDebugText REPLACE (same position)", lambda i: p.addUserDebugText("t%d" % i, [0, 0, 3], replaceItemUniqueId=text_id[0]))
bench("addUserDebugText REPLACE (new position)", lambda i: p.addUserDebugText("t", [i * 0.01, 0, 3], replaceItemUniqueId=text_id[0]))
bench("addUserDebugLine REPLACE", lambda i: p.addUserDebugLine([0, 0, 0], [1, 1, i * 0.01], [1, 0, 0], replaceItemUniqueId=line_id[0]))
bench("changeVisualShape colour (halo box)", lambda i: p.changeVisualShape(halo, -1, rgbaColor=[1, 0.1, 0.05, (i % 10) / 20]))
bench("changeVisualShape colour (drone link: light)", lambda i: p.changeVisualShape(body, model.lights["fl"], rgbaColor=[1, (i % 10) / 10, 0, 1]))
bench("changeVisualShape colour (drone link: disc)", lambda i: p.changeVisualShape(body, model.discs[0], rgbaColor=[1, 1, 1, (i % 10) / 30]))
p.disconnect()
