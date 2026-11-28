"""Demo controller of the HaasST20 PROTO with its Haas automation options (robot controller of the lathe, supervisor
only for the demo part, the coolant / air jet fields and the optional screenshots).

Loop: auto door opens until the "door completely open" switch is made -> auto air jet (M12/M13) blows the chuck ->
a bar stub is clamped (hydraulic chuck clamped switch) -> automatic tool presetter arm swings down and each of the
3 tools touches the probe in X then in Z (touch detected from the axis positions: emulated probe trigger) -> arm
stowed, door closes, roughing pass with coolant -> part-off with the grooving tool while the parts catcher swings
up (M36), the cut piece falls onto the tray, rolls down the chute into the outer bin, catcher stowed (M37).

Positions are joint coordinates of the PROTO (see protos/HaasST20.proto): door 0 = closed .. 0.722 = open, jaws
0 = grip radius 0.0255 m (+ = opening), Z -0.533 .. 0 (- = towards the chuck), X -0.15 .. 0.065 (+ = away from the
spindle along the 55 deg slide), turret 12 stations every pi/6 rad, presetter 0 = stowed .. pi/2 = deployed, parts
catcher 0 = stowed .. 1.33 = catching.
    controllerArgs [ "--shots" "<dir>" ]   saves screenshots of the key moments into <dir> (rendering needed)
"""
import math
import os
import sys

from controller import Supervisor

robot = Supervisor()
dt = int(robot.getBasicTimeStep())
SHOTS = sys.argv[sys.argv.index("--shots") + 1] if "--shots" in sys.argv else None
ONCE = "--once" in sys.argv


def dev(name):
    m = robot.getDevice(name + " motor")
    s = robot.getDevice(name + " sensor")
    s.enable(dt)
    return m, s


door, door_s = dev("door")
spindle, spindle_s = dev("spindle")
jaws = [dev("jaw %d" % k)[0] for k in (1, 2, 3)]
zm, z_s = dev("z")
xm, x_s = dev("x")
turret, turret_s = dev("turret")
pre, pre_s = dev("presetter")
catcher, catcher_s = dev("parts catcher")
leds = {c: robot.getDevice("stack light " + c) for c in ("green", "orange", "red")}
door_led = robot.getDevice("door open switch led")
clamp_led = robot.getDevice("chuck clamped switch")
unclamp_led = robot.getDevice("chuck unclamped switch")
me = robot.getSelf()
coolant = me.getField("coolantTransparency")
air = me.getField("airJetTransparency")
piece = robot.getFromDef("CUT_PIECE")
stub = robot.getFromDef("BAR_STUB")
view = robot.getFromDef("VIEW")

door.setVelocity(0.36)                 # 2 s stroke, as in the cell PLC
for j in jaws:
    j.setVelocity(0.05)
zm.setVelocity(0.25)
xm.setVelocity(0.15)
turret.setVelocity(3.0)
pre.setVelocity(1.5)
catcher.setVelocity(2.0)
spindle.setPosition(float("inf"))      # velocity control
spindle.setVelocity(0.0)

# geometry of the lathe frame (tools/gen_world.py, protos/meshes/haas_st20/haas_st20.json); the demo lathe is at the
# world origin with yaw 0, so the lathe frame is the world frame
CX, CZ = -0.40, 1.10
TUR0_Y, TOOL_DY, TIP_R0 = 0.3151, -0.140, 0.4568 - 0.305       # tool tip at X = 0: r = 0.1518 from the spindle axis
X_DIR = (-0.5737, 0.0, 0.8191)
PROBE_Y, PROBE_R, PROBE_H = -0.1447, 0.0478, 0.007    # deployed stylus tip (y, along X from the axis), half size
JAW_OPEN, JAW_CLAMP = 0.0245, -0.0005  # grip radius 0.050 m (open) / 0.025 m (clamped on the bar)
CATCH_PIVOT, CATCH_STOW, CATCH_UP = (-0.22, -0.30, 0.79), -0.75, 1.33
CHUTE = ((-0.215, 0.785), (0.005, 0.565))
BIN_FLOOR = (0.085, -0.30, 0.404)
PIECE_R, PIECE_Y = 0.025, -0.3325      # cut piece: 45 mm long, in front of the jaws (y -0.355 .. -0.31)
TOOLS = (("roughing", 0), ("finishing", 11), ("grooving", 10))
station = 0
shot_k = 0


def wait(seconds, cond=None):
    end = robot.getTime() + seconds
    while robot.getTime() < end:
        if robot.step(dt) == -1:
            raise SystemExit
        switches()
        if cond and cond():
            return True
    return False


def switches():
    """Switch lamps: door open (DistanceSensor on the door lug), chuck clamped / unclamped (jaw positions)."""
    door_led.set(1 if door_open() else 0)
    j = jaw_pos()
    clamp_led.set(1 if abs(j - JAW_CLAMP) < 5e-4 else 0)
    unclamp_led.set(1 if j > JAW_OPEN - 5e-4 else 0)


def door_open():
    """"Door completely open" limit switch: made over the last 8 mm of the door stroke (lug on the roller)."""
    return door_s.getValue() >= 0.722 - 0.008


jaw_s = robot.getDevice("jaw 1 sensor")
jaw_s.enable(dt)


def jaw_pos():
    return jaw_s.getValue()


def light(c):
    for k, led in leds.items():
        led.set(1 if k == c else 0)


def set_jaws(p):
    for j in jaws:
        j.setPosition(p)


def tip(y, r):
    """Motor positions (z, x) that put the tool tip at axial position y and radius r."""
    return y - TUR0_Y - TOOL_DY, r - TIP_R0


def goto(y, r, wait_s=None):
    z, x = tip(y, r)
    zm.setPosition(z)
    xm.setPosition(x)
    t = wait_s if wait_s is not None else max(abs(z - z_s.getValue()) / 0.25, abs(x - x_s.getValue()) / 0.15) + 0.2
    wait(t)


def index(k):
    global station
    a = k * math.pi / 6
    d = (a - station * math.pi / 6) % (2 * math.pi)        # unlimited joint: always the same direction
    station_angle = turret_s.getValue() + d
    turret.setPosition(station_angle)
    wait(d / 3.0 + 0.3)
    station = k


def touch(axis, start, target):
    """Feed one axis slowly from start towards the probe face; the probe triggers when the tool tip reaches the face
    (computed from the axis positions); returns the trigger position (tool offset)."""
    motor, sensor = (zm, z_s) if axis == "z" else (xm, x_s)
    motor.setVelocity(0.01)
    motor.setPosition(target - 0.003)                      # 3 mm overtravel, stopped by the probe
    hit = wait(5.0, lambda: (sensor.getValue() <= target if target < start else sensor.getValue() >= target))
    p = sensor.getValue()
    motor.setPosition(p)
    motor.setVelocity(0.25 if axis == "z" else 0.15)
    return p if hit else None


def rot_y(a):
    return [0, 1, 0, a]


def piece_at(p, roll=0.0):
    """Cut piece (axis along y) at p, rolled by `roll` about its axis."""
    piece.getField("translation").setSFVec3f(list(p))
    c, s = math.cos(roll), math.sin(roll)
    # Rx(pi/2) (cylinder axis z -> y) then roll about y: axis-angle of Ry(roll) * Rx(pi/2)
    m = [[c, s, 0.0], [0.0, 0.0, -1.0], [-s, c, 0.0]]
    t = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2))
    a = math.acos(t)
    k = 2 * math.sin(a) if abs(math.sin(a)) > 1e-6 else 1.0
    piece.getField("rotation").setSFRotation([(m[2][1] - m[1][2]) / k, (m[0][2] - m[2][0]) / k, (m[1][0] - m[0][1]) / k, a])


def move_piece(path, speed):
    """Moves the piece along a polyline at `speed` (m/s), rolling without slipping."""
    roll = 0.0
    for a, b in zip(path, path[1:]):
        L = math.dist(a, b)
        n = max(1, int(L / speed / (dt / 1000.0)))
        for i in range(1, n + 1):
            p = [a[j] + (b[j] - a[j]) * i / n for j in range(3)]
            roll += L / n / PIECE_R
            piece_at(p, roll)
            if robot.step(dt) == -1:
                raise SystemExit
            switches()


def look(pos, target, name=None):
    """Moves the viewpoint (DEF VIEW) to pos looking at target and saves a screenshot when --shots is given."""
    global shot_k
    if view is None:
        return
    d = [target[i] - pos[i] for i in range(3)]
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    m = [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]]      # Rz(yaw) * Ry(pitch)
    t = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2))
    a = math.acos(t)
    s = 2 * math.sin(a)
    axis = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s] if abs(s) > 1e-6 else [0, 0, 1]
    view.getField("position").setSFVec3f(list(pos))
    view.getField("orientation").setSFRotation(axis + [a])
    if SHOTS and name:
        wait(0.3)
        shot_k += 1
        robot.exportImage(os.path.join(SHOTS, "%02d_%s.jpg" % (shot_k, name)), 90)
        wait(0.2)


WIDE = ((2.2, 1.6, 2.4), (-0.45, -0.25, 1.3))
INSIDE = ((0.55, 0.35, 1.45), (-0.38, -0.25, 1.05))
cycle = 0
while True:
    cycle += 1
    # ------------------------------------------------------------ reset: bar stub in the chuck, piece on the bar
    piece_at((CX, PIECE_Y, CZ))
    stub.getField("translation").setSFVec3f([CX, -0.3775, CZ])
    catcher.setPosition(0.0)
    pre.setPosition(0.0)
    goto(TUR0_Y + TOOL_DY, TIP_R0, 0.1)
    # ------------------------------------------------------------ auto door + "door completely open" switch
    light("orange")
    look(*WIDE)
    door.setPosition(0.722)            # open (slides towards the headstock)
    wait(3.0, door_open)
    print("[%5.1f s] auto door: door open switch made (door at %.3f m), robot entry enabled" % (robot.getTime(), door_s.getValue()))
    look((0.9, -0.2, 2.75), (-0.43, -0.62, 2.03), "door_open_switch")
    look(*INSIDE)
    # ------------------------------------------------------------ auto air jet (M12 / M13)
    air.setSFFloat(0.6)
    print("[%5.1f s] M12 auto air jet on (chuck)" % robot.getTime())
    wait(0.8)
    look((0.35, 0.15, 1.5), (-0.38, -0.40, 1.2), "air_jet")
    wait(0.7)
    air.setSFFloat(1.0)
    print("[%5.1f s] M13 auto air jet off" % robot.getTime())
    # ------------------------------------------------------------ clamp (hydraulic chuck switches)
    set_jaws(JAW_OPEN)
    wait(0.8)
    set_jaws(JAW_CLAMP)
    wait(0.8)
    print("[%5.1f s] chuck clamped switch %s" % (robot.getTime(), "ON" if clamp_led.get() else "off"))
    # ------------------------------------------------------------ automatic tool presetter (setup mode, door open)
    pre.setPosition(math.pi / 2)
    wait(1.4)
    print("[%5.1f s] tool presetter arm down (%.3f rad)" % (robot.getTime(), pre_s.getValue()))
    for name, k in TOOLS:
        goto(PROBE_Y + 0.05, PROBE_R + 0.08)               # clearance point in front of the probe
        index(k)
        goto(PROBE_Y, PROBE_R + 0.03)                       # above the probe (X+ side)
        zx = touch("x", x_s.getValue(), tip(PROBE_Y, PROBE_R + PROBE_H)[1])
        if cycle == 1 and name == "roughing":
            look((0.25, 0.25, 1.30), (-0.43, -0.15, 1.15), "presetter_touch_x")
        goto(PROBE_Y, PROBE_R + 0.03, 0.4)
        goto(PROBE_Y + 0.03, PROBE_R)                       # Z+ side of the probe
        zz = touch("z", z_s.getValue(), tip(PROBE_Y + PROBE_H, PROBE_R)[0])
        if cycle == 1 and name == "grooving":
            look((0.25, 0.25, 1.30), (-0.43, -0.15, 1.15), "presetter_touch_z")
        goto(PROBE_Y + 0.03, PROBE_R, 0.4)
        print("[%5.1f s] T%d %s: probe X trigger at %s, Z trigger at %s" % (
            robot.getTime(), k, name, "%.4f" % zx if zx is not None else "MISSED", "%.4f" % zz if zz is not None else "MISSED"))
    goto(PROBE_Y + 0.05, PROBE_R + 0.08)
    index(0)
    goto(TUR0_Y + TOOL_DY, TIP_R0)
    pre.setPosition(0.0)
    wait(1.4)
    print("[%5.1f s] tool presetter arm stowed" % robot.getTime())
    # ------------------------------------------------------------ machining (door closed, coolant)
    door.setPosition(0.0)
    wait(2.2)
    light("green")
    spindle.setVelocity(9.0)            # rad/s, same visual speed as the cell
    wait(1.0)
    goto(-0.30, 0.027)
    coolant.setSFFloat(0.45)
    zm.setVelocity(0.02)
    goto(-0.345, 0.027, 2.4)
    zm.setVelocity(0.25)
    coolant.setSFFloat(1.0)
    goto(-0.28, 0.06)
    # ------------------------------------------------------------ part-off (grooving tool) + parts catcher (M36)
    index(10)
    goto(-0.356, 0.035)
    catcher.setPosition(CATCH_UP)
    print("[%5.1f s] M36 parts catcher up" % robot.getTime())
    coolant.setSFFloat(0.45)
    xm.setVelocity(0.01)
    goto(-0.356, 0.003, 3.3)
    xm.setVelocity(0.15)
    coolant.setSFFloat(1.0)
    goto(-0.356, 0.06)
    spindle.setVelocity(0.0)
    e = CATCH_STOW + CATCH_UP
    d = (-math.cos(e), 0.0, math.sin(e))
    n = (math.sin(e), 0.0, math.cos(e))
    P = CATCH_PIVOT
    tray = lambda s: [P[0] + s * d[0] + (PIECE_R + 0.003) * n[0], P[1], P[2] + s * d[2] + (PIECE_R + 0.003) * n[2]]  # noqa
    s_fall = (P[0] - CX) / math.cos(e)
    (xa, za), (xb, zb) = CHUTE
    a = math.atan2(za - zb, xb - xa)
    n2 = (math.sin(a), 0.0, math.cos(a))
    c1 = [xa + 0.03 * math.cos(a) + (PIECE_R + 0.003) * n2[0], P[1], za - 0.03 * math.sin(a) + (PIECE_R + 0.003) * n2[2]]
    c2 = [xb + (PIECE_R + 0.003) * n2[0], P[1], zb + (PIECE_R + 0.003) * n2[2]]
    move_piece([[CX, PIECE_Y, CZ], [CX, P[1], CZ - 0.04], tray(s_fall)], 0.8)          # falls onto the tray
    if cycle == 1:
        look((-0.17, 0.18, 1.18), (-0.32, -0.30, 0.86), "parts_catcher_up")       # camera inside, door closed
    move_piece([tray(s_fall), tray(0.03)], 0.5)                                       # rolls down the tray
    catcher.setPosition(0.0)
    print("[%5.1f s] M37 parts catcher stowed" % robot.getTime())
    move_piece([tray(0.03), c1, c2, [BIN_FLOOR[0], P[1], BIN_FLOOR[2] + PIECE_R]], 0.6)   # chute -> outer bin
    light("orange")
    if cycle == 1:
        look((0.55, 0.05, 1.15), (0.06, -0.30, 0.50), "parts_bin")
        print("[%5.1f s] cut piece in the bin at %s" % (robot.getTime(), [round(v, 3) for v in piece.getPosition()]))
    wait(1.0)
    door.setPosition(0.722)
    wait(2.5)
    if cycle == 1:
        look(*WIDE, "overview")
        if ONCE:
            print("[%5.1f s] demo cycle done" % robot.getTime())
            robot.simulationQuit(0)
            wait(1.0)
    look(*WIDE)
    door.setPosition(0.0)
    wait(2.5)
