"""Cell PLC (supervisor) of the robotic turning cell: 2 CNC lathes facing each other, UR10e, crates.

Two clearly separated parts:
  * PLANT (simulated): lathe doors, chuck jaws, spindle rotation, turret, coolant, material removal (stock sleeve
    shortening as the tool advances), crate exchange by the operator (walking pedestrian).
  * CONTROL: lathe state machines, robot job scheduling (dual gripper: unload the finished part and load the
    blank in the same visit), robot/machine handshakes (chuck clamp/open), safety (laser scanners with speed
    and separation monitoring, emergency stop, door interlocks).
  * Haas automation options of the HaasST20 PROTO: auto door with its "door completely open" limit switch (the robot
    is only sent into a lathe while the switch is made, the chuck air jet is off and the tool presetter arm and the
    parts catcher are stowed), auto air jet (M12/M13) blowing the chuck while the door opens before every robot visit
    (no extra time), hydraulic chuck clamped / unclamped proximity switches (lamps), tool presetter and parts catcher
    kept stowed (the robot unloads the parts).

Keys (3D view focused): E = emergency stop (toggle), R = reset, P = operator walks in to swap the crates,
S = end-of-cycle stop. The same letters can be written into the customData field of the "automate cellule" robot.
"""
import json
import math
import os
import sys

from controller import DistanceSensor, Supervisor

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ur_tending"))
from dual_gripper import base_q  # noqa: E402

# ---------------------------------------------------------------- geometry (see tools/gen_world.py)
CX, CZ, JAW_FACE = -0.40, 1.10, -0.358
SEAT_FRONT = -0.40                 # part front face against the chuck body face (lathe frame)
PART_L, PART_R = 0.09, 0.025
GRIP_FROM_BACK = 0.01              # TCP 10 mm inside the part's back end
TABLE_Z = 0.70
CRATES = {"BRUTS": (-0.25, 0.85), "FINIES": (0.25, 0.85)}
CELL_X = (-0.12, 0.0, 0.12)
CELL_Y = (-0.195, -0.065, 0.065, 0.195)
PART_Z = TABLE_Z + 0.0063 + PART_L / 2
LATHE_X = 0.80
LATHES = {"A": (-LATHE_X, 0.0), "B": (LATHE_X, math.pi)}     # translation x, yaw
# Haas ST-20 moving groups (lathe frame, see protos/meshes/haas_st20/haas_st20.json and tools/gen_world.py)
DOOR_CLOSED = (-0.56, -0.582, 0.717)   # door group origin, door closed
DOOR_TRAVEL = 0.722                    # opening stroke along -y (towards the headstock)
JAW_DIRS = ((-0.0386, 0.9993), (-0.8459, -0.5333), (0.8888, -0.4583))   # radial directions of the jaws (x, z)
JAW_R0 = 0.0255                        # jaw gripping radius in the CAD pose (jaw translation 0)
TUR0 = (-0.6622, 0.3151, 1.474)        # carriage / turret origin in the CAD pose (turret axis, disc +y face)
X_DIR = (-0.5737, 0.0, 0.8191)         # X cross slide (55° slant): +X = away from the spindle
TUR_AXIS_R = 0.4568                    # turret axis to spindle axis distance in the CAD pose
TOOL_R, TOOL_DY = 0.305, -0.140        # tool tip: distance from the turret axis towards the spindle, axial offset
TUR_PARK = (0.0, 0.0)                  # rest position (carriage Z along +y, X slide) from the CAD pose
PARK_Y = TUR0[1] + TUR_PARK[0] + TOOL_DY          # tool tip at rest: axial position (lathe y) ...
PARK_R = TUR_AXIS_R - TOOL_R + TUR_PARK[1]        # ... and distance from the spindle axis
CUT_Y0 = SEAT_FRONT + PART_L                      # turning pass: from the part's free end ...
CUT_Y1 = CUT_Y0 - 0.045                           # ... to the end of the stock sleeve, 3 mm before the jaws
CUT_R = (0.0125, 0.0185)                          # finished radii: back step (y > CUT_STEP), front step
CUT_STEP = CUT_Y0 - 0.02                          # shoulder between the two steps
SPIN = 9.0                         # rad/s (visual spindle speed)
T_CUT = 16.0                       # s, machining time (rapids + roughing + finishing + grooving + tool changes)
# turret tools (12 stations every pi/6, station k in the working position at k * pi/6), see tools/gen_world.py
TOOL_STATIONS = (("ebauche", 0), ("finition", 11), ("gorge", 10))
TOOL = {name: k for name, k in TOOL_STATIONS}
T_INDEX = 0.5                      # s per station (turret indexing)
CLEAR_Y = CUT_Y0 + 0.05            # clearance / tool-change point: every tool 50 mm in front of the part's free end
CLEAR_R = PARK_R                   # ... and at the rest radius (turret axis 0.457 m from the spindle axis)
ROUGH_ALLOW = 0.0005               # radial stock left by the roughing pass for the finishing pass
GROOVE_Y = CUT_Y1 + 0.003          # relief groove (3 mm insert) at the end of the turned diameter, 6 mm from the jaws
GROOVE_R = 0.016                   # groove bottom radius (turned diameter radius CUT_R[1] = 0.0185)
COOLANT_ON = 0.45                  # coolant jet transparency (PROTO field coolantTransparency), 1 = off
AIR_ON = 0.6                       # air jet transparency (PROTO field airJetTransparency), 1 = off
AIR_START = 1.5                    # s, chuck blown at start-up (doors open, before the robot is ready)
DOOR_SW_PRETRAVEL = 0.008          # m, the "door completely open" switch is made over the last 8 mm of the door stroke
SCAN_PROT = (-0.8, 0.8, 1.40, 2.0)  # x limited to the lathe facade planes (lathe A coolant tank / conveyor at x -0.85)
SCAN_WARN = (-0.8, 0.8, 2.0, 2.8)
OPERATOR_HOME = (3.6, 3.55)
OPERATOR_TABLE = (0.0, 1.62)
GAIT = [
    [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74], [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],
    [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0], [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],
    [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38], [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],
    [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4], [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],
    [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07], [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],
    [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49], [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],
    [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]]
GAIT_FIELDS = ["leftArmAngle", "leftLowerArmAngle", "leftHandAngle", "rightArmAngle", "rightLowerArmAngle",
               "rightHandAngle", "leftLegAngle", "leftLowerLegAngle", "leftFootAngle", "rightLegAngle",
               "rightLowerLegAngle", "rightFootAngle", "headAngle"]
GAIT_DZ = [-0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03]


# orientation of the bent dual gripper in the lathe: the idle gripper points along LATHE_M (lathe frame),
# the flange and the wrist stay on the opposite side
LATHE_M = [-0.866, 0.0, 0.5]       # idle gripper towards the back of the machine, 30° up (150°)
OUT_DX, OUT_DZ = 0.45, 0.22        # retreat point in front of the door (lathe frame, from the façade / spindle height),
                                   # far enough for the idle gripper to clear the CNC pendant (x <= 0.064)
PRE_DY = 0.08                      # pre-insertion / extraction distance along the spindle axis (wrist 2 vs door jamb)
CRATE_UP = 0.28                    # clearance height above the crate cells for the vertical moves
RETREAT_HOME = False               # go back above the table before reporting "clear" (door closing)
CRATE_M_ANGLE = math.radians(-10)  # idle gripper at the crates: sideways (±x), away from the centre line

# joint seeds per lathe / crate and gripper, given with the cell axes and converted with the orientation of the robot
# base. Haas ST-20: the door opening ends at y = 0.197 (lathe frame) on the operator side, so the arm works in the
# lathes with the shoulder offset on the headstock side (wrist 1 on the -y side of wrist 2); with the other shoulder
# configuration the forearm crosses the façade behind the right door jamb. Crate angle chosen so that wrist 3 is near
# 0 / π when wrist 2 goes through π on the moves between crates and lathes (bracket clear of the forearm).
# Checked with tools/check_collisions.py. Both grippers share the flange pose: switching gripper = half a turn of wrist 3.
SEEDS = {("A", 1): [0.068, -2.503, -1.003, -2.285, -2.297, -0.045],
         ("A", 2): [0.068, -2.503, -1.003, -2.285, -2.297, 3.097],
         ("B", 1): [-3.074, -2.503, -1.003, -2.285, -2.297, -0.045],
         ("B", 2): [-3.074, -2.503, -1.003, -2.285, -2.297, 3.097]}
CRATE_SEEDS = {1: [-1.2, -1.806, -1.681, -1.033, 2.337, 0.274], 2: [-1.2, -1.806, -1.681, -1.033, 2.337, -2.868]}
SEEDS = {k: base_q(v) for k, v in SEEDS.items()}
CRATE_SEEDS = {k: base_q(v) for k, v in CRATE_SEEDS.items()}


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def tool_x(g, a, m):
    """Finger axis for approach a such that the flange normal is n = cos45 a + sin45 m (m ⊥ a): the bent bracket
    puts the flange (and the wrist) on the -m side, and the idle gripper points along m."""
    return cross(a, m) if g == 1 else cross(m, a)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def mmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def axis_angle(m):
    c = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2))
    a = math.acos(c)
    if a < 1e-7:
        return [0, 0, 1, 0]
    if abs(a - math.pi) < 1e-4:
        xx = math.sqrt(max(0, (m[0][0] + 1) / 2))
        yy = math.sqrt(max(0, (m[1][1] + 1) / 2))
        zz = math.sqrt(max(0, (m[2][2] + 1) / 2))
        if xx > 1e-3:
            yy, zz = math.copysign(yy, m[0][1]), math.copysign(zz, m[0][2])
        elif yy > 1e-3:
            zz = math.copysign(zz, m[1][2])
        return [xx, yy, zz, a]
    s = 2 * math.sin(a)
    return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, a]


def station_angle(k):
    """Turret joint angle that brings station k into the working position, in ]-pi, pi]."""
    a = k * math.pi / 6
    return a - 2 * math.pi if a > math.pi else a


def profile_r(y, allow=0.0):
    """Finished radius of the part at axial position y (two steps, 2 mm shoulder ramp), plus a radial allowance."""
    return CUT_R[0] + (CUT_R[1] - CUT_R[0]) * max(0.0, min(1.0, (CUT_STEP - y) / 0.002)) + allow


def machining_program():
    """Turret program of one part as (duration, segment) pairs, played by Lathe.machining once the door is locked;
    it fits in the former single-tool timing (spindle at speed until T_CUT + 3.5 s), so the lathe cycle and the robot
    windows are unchanged. Segments: ("move", (y0, r0), (y1, r1), station, coolant) linear move of the tool tip
    (axial position y, radius r), ("pass", station, allowance) turning pass along the profile from CUT_Y0 to CUT_Y1,
    ("index", k0, k1) turret indexing at the clearance point (T_INDEX per station, coolant off)."""
    clear = (CLEAR_Y, CLEAR_R)
    park = (PARK_Y, PARK_R)
    r0 = PART_R + 0.002
    t_rough, t_finish = 7.0, 4.0
    k_r, k_f, k_g = TOOL["ebauche"], TOOL["finition"], TOOL["gorge"]
    steps = lambda k0, k1: abs(round((station_angle(k1) - station_angle(k0)) / (math.pi / 6)))  # noqa: E731
    out1 = lambda r: (CUT_Y1, r + 0.006)  # noqa: E731   radial retract first, clear of the turned diameter
    return [
        (1.2, ("move", park, clear, k_r, False)),
        (0.3, ("move", clear, (CUT_Y0, r0), k_r, False)),
        (t_rough, ("pass", k_r, ROUGH_ALLOW)),                        # roughing (stock removal, coolant)
        (0.15, ("move", (CUT_Y1, profile_r(CUT_Y1, ROUGH_ALLOW)), out1(CUT_R[1]), k_r, False)),
        (0.25, ("move", out1(CUT_R[1]), clear, k_r, False)),
        (T_INDEX * steps(k_r, k_f), ("index", k_r, k_f)),              # roughing -> finishing tool
        (0.3, ("move", clear, (CUT_Y0 + 0.001, CUT_R[0]), k_f, False)),
        (t_finish, ("pass", k_f, 0.0)),                                # finishing pass on the profile
        (0.15, ("move", (CUT_Y1, CUT_R[1]), out1(CUT_R[1]), k_f, False)),
        (0.25, ("move", out1(CUT_R[1]), clear, k_f, False)),
        (T_INDEX * steps(k_f, k_g), ("index", k_f, k_g)),              # finishing -> grooving tool
        (0.3, ("move", clear, (GROOVE_Y, CUT_R[1] + 0.003), k_g, False)),
        (1.0, ("move", (GROOVE_Y, CUT_R[1] + 0.003), (GROOVE_Y, GROOVE_R), k_g, True)),   # plunge (coolant)
        (0.3, ("move", (GROOVE_Y, GROOVE_R), (GROOVE_Y, GROOVE_R), k_g, True)),           # dwell at the bottom
        (0.4, ("move", (GROOVE_Y, GROOVE_R), (GROOVE_Y, CUT_R[1] + 0.006), k_g, False)),
        (0.3, ("move", (GROOVE_Y, CUT_R[1] + 0.006), clear, k_g, False)),
        (T_INDEX * steps(k_g, k_r), ("index", k_g, k_r)),              # back to the roughing tool for the next part
        (1.5, ("move", clear, park, k_r, False)),
    ]


PROGRAM = machining_program()
ROUGH_SEG = 2                                               # index of the roughing pass in PROGRAM
assert sum(d for d, _ in PROGRAM) <= T_CUT + 3.5 + 1e-9     # turret at rest before the spindle stops / door opens


class Lathe:
    def __init__(self, plc, name):
        self.plc, self.name = plc, name
        # HaasST20 PROTO instance: the PLC drives its internal joints (internal DEFs, setJointPosition in joint
        # coordinates, see protos/HaasST20.proto), its stack light lamps and coolant jet through the PROTO fields
        self.node = plc.getFromDef("TOUR_" + name)
        pd = self.node.getFromProtoDef
        self.x0, self.yaw = LATHES[name]
        self.R = rz(self.yaw)
        self.door = pd("DOOR_JOINT")
        self.spindle = pd("SPINDLE_JOINT")
        self.jaws = [pd("JAW%d_JOINT" % i) for i in (1, 2, 3)]
        self.zaxis = pd("Z_JOINT")
        self.xaxis = pd("X_JOINT")
        self.turret = pd("TURRET_JOINT")
        self.coolant = self.node.getField("coolantTransparency")
        # Haas options: door (end point, read back by the "door open" limit switch), air jet, switch lamps, tool
        # presetter arm and parts catcher (None when the option is not fitted), kept stowed
        self.door_solid = pd("DOOR")
        self.air = self.node.getField("airJetTransparency")
        self.sw_lamps = {k: self.node.getField(f) for k, f in (("door", "doorOpenLight"), ("clamped", "chuckClampedLight"),
                                                               ("unclamped", "chuckUnclampedLight"))}
        self.sw_on = {}
        self.air_on = None
        self.stowed = []
        for jd, sd in (("PRESETTER_JOINT", "PRESETTER"), ("CATCHER_JOINT", "PARTS_CATCHER")):
            j = pd(jd)
            if j:
                j.setJointPosition(0.0)
                self.stowed.append((jd, pd(sd), None))
        self.lamps = {c: self.node.getField(f) for c, f in (("G", "greenLight"), ("O", "orangeLight"), ("R", "redLight"))}
        self.lamp_on = {}
        self.coolant_on = None
        self.tool = None                # station in the working position (None while indexing)
        self.state = "empty"           # empty | robot | closing | cutting | opening | done
        self.part = None               # DEF of the part in the chuck
        self.finished = False
        self.door_y = 1.0              # 0 = closed, 1 = open
        self.jaw_r = 0.05              # jaw inner radius (0.025 = clamped on the blank)
        self.jaw_target = 0.05
        self.angle = 0.0
        self.spin = 0.0
        self.t = 0.0
        self.cut = 0.0                 # turning progress 0..1
        self.cycles = 0
        self.set_door(1.0)
        self.set_jaws(0.05)
        self.set_tool(TOOL["ebauche"])
        self.set_turret(PARK_Y, PARK_R)

    # -- frames
    def world(self, p):
        v = mv(self.R, p)
        return [self.x0 + v[0], v[1], v[2]]

    def wdir(self, d):
        return mv(self.R, d)

    # -- actuators
    def set_door(self, s):
        """s = 0 closed, 1 open: the door slides along -y (towards the headstock)."""
        self.door.setJointPosition(DOOR_TRAVEL * s)

    def set_jaws(self, r):
        """Jaw inner radius r: each jaw slides along its radial direction from the CAD pose (radius JAW_R0)."""
        q = max(-0.0155, min(0.0245, r - JAW_R0))          # joint stops of the PROTO (radius 0.010 .. 0.050)
        for j in self.jaws:
            j.setJointPosition(q)

    def set_turret(self, y_tip, r_tip):
        """Tool tip at axial position y_tip (lathe y) and at r_tip from the spindle axis: the carriage moves along y
        (machine Z), the turret along the 55° cross slide (machine X), the turret axis staying on the X line through
        the spindle axis."""
        z = y_tip - TUR0[1] - TOOL_DY
        x = r_tip - (TUR_AXIS_R - TOOL_R)
        self.zaxis.setJointPosition(z)
        self.xaxis.setJointPosition(x)

    def set_index(self, angle):
        """Turret joint angle (station k in the working position at station_angle(k))."""
        self.turret.setJointPosition(angle)

    def set_air(self, on):
        """Auto air jet (M12 on / M13 off) aimed at the chuck."""
        if on != self.air_on:
            self.air_on = on
            self.air.setSFFloat(AIR_ON if on else 1.0)

    def sw_lamp(self, k, on, col):
        if self.sw_on.get(k) != on:
            self.sw_on[k] = on
            self.sw_lamps[k].setSFColor(col if on else [0, 0, 0])

    def door_open_switch(self):
        """"Door completely open" limit switch, actuated by the door lug over the last DOOR_SW_PRETRAVEL of the stroke:
        reads the actual door position (end point of DOOR_JOINT) in the lathe frame."""
        p = self.door_solid.getPosition()
        y = mv([[self.R[j][i] for j in range(3)] for i in range(3)], [p[0] - self.x0, p[1], p[2]])[1]
        return DOOR_CLOSED[1] - y >= DOOR_TRAVEL - DOOR_SW_PRETRAVEL

    def options_stowed(self):
        """Tool presetter arm and parts catcher in their stowed pose (orientation of their end points unchanged)."""
        ok = True
        for i, (jd, sol, r0) in enumerate(self.stowed):
            r = sol.getOrientation()
            if r0 is None:
                self.stowed[i] = (jd, sol, list(r))
            elif max(abs(a - b) for a, b in zip(r, r0)) > 1e-3:
                ok = False
        return ok

    def entry_permitted(self):
        """Robot entry interlock: door completely open (limit switch), air jet off, presetter and catcher stowed."""
        return self.door_open_switch() and not self.air_on and self.options_stowed()

    def chuck_switches(self):
        """Hydraulic chuck proximity switches (draw tube position): clamped on the part / fully unclamped."""
        clamped = abs(self.jaw_r - self.jaw_target) <= 1e-4 and self.jaw_target < 0.05
        self.sw_lamp("clamped", clamped, [0, 1, 0])
        self.sw_lamp("unclamped", self.jaw_r >= 0.05 - 1e-4, [1, 0.55, 0])
        return clamped

    def set_coolant(self, on):
        if on != self.coolant_on:
            self.coolant_on = on
            self.coolant.setSFFloat(COOLANT_ON if on else 1.0)

    def lamp(self, c, on):
        if self.lamp_on.get(c) != on:
            self.lamp_on[c] = on
            col = {"G": [0, 1, 0], "O": [1, 0.55, 0], "R": [1, 0, 0]}[c]
            self.lamps[c].setSFColor(col if on else [0, 0, 0])

    def part_pose(self):
        """World pose of a part seated in the chuck (part axis = lathe +y), spinning with the chuck."""
        center = self.world([CX, SEAT_FRONT + PART_L / 2, CZ])
        base = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]            # part z -> lathe +y
        spin = [[math.cos(self.angle), 0, math.sin(self.angle)], [0, 1, 0],
                [-math.sin(self.angle), 0, math.cos(self.angle)]]   # about lathe y
        return center, mmul(self.R, mmul(spin, base))

    def place_part(self):
        if self.part is None:
            return
        n = self.plc.getFromDef(self.part)
        c, R = self.part_pose()
        n.getField("translation").setSFVec3f(c)
        n.getField("rotation").setSFRotation(axis_angle(R))

    # -- robot targets (world frame)
    def approach(self):
        return self.wdir([0, -1, 0])

    def tcp_seat(self):
        return self.world([CX, SEAT_FRONT + PART_L - GRIP_FROM_BACK, CZ])

    def tcp_pre(self):
        return self.world([CX, SEAT_FRONT + PART_L - GRIP_FROM_BACK + PRE_DY, CZ])

    def tcp_out(self):
        return self.world([OUT_DX, SEAT_FRONT + PART_L - GRIP_FROM_BACK + PRE_DY, CZ + OUT_DZ])

    # -- cycle
    def start_cycle(self):
        self.state, self.t = "closing", 0.0
        self.plc.log("Lathe %s: door closing, cycle start" % self.name)

    def machining(self, t):
        """Play the turret program (machining_program) at time t after the door was locked: roughing (stock sleeve
        shortening), tool change, finishing, tool change, grooving, tool change back to roughing, rest position."""
        t0, i = 0.0, 0
        while i < len(PROGRAM) and t >= t0 + PROGRAM[i][0]:
            t0 += PROGRAM[i][0]
            i += 1
        if i > ROUGH_SEG and not self.finished and self.part:       # roughing done: finished outer diameter
            self.finished = True
            self.plc.getFromDef(self.part.replace("PART", "PBO")).getField("radius").setSFFloat(0.015)
        if i == len(PROGRAM):                                        # program over: rest position, roughing tool
            self.set_coolant(False)
            self.set_tool(TOOL["ebauche"])
            self.set_turret(PARK_Y, PARK_R)
            return
        d, seg = PROGRAM[i]
        s = (t - t0) / d
        kind = seg[0]
        if kind == "index":
            self.set_coolant(False)
            if self.tool is not None:
                self.plc.log("Lathe %s: turret indexing T%d -> T%d (%s)" % (
                    self.name, seg[1], seg[2], [n for n, k in TOOL_STATIONS if k == seg[2]][0]))
            self.tool = None
            a0, a1 = station_angle(seg[1]), station_angle(seg[2])
            self.set_index(a0 + (a1 - a0) * (0.5 - 0.5 * math.cos(math.pi * s)))
            return
        if kind == "move":
            _, (y0, r0), (y1, r1), k, cool = seg
            self.set_tool(k)
            self.set_turret(y0 + (y1 - y0) * s, r0 + (r1 - r0) * s)
            self.set_coolant(cool)
            return
        # turning pass along the profile from the part's free end towards the jaws
        _, k, allow = seg
        self.set_tool(k)
        y = CUT_Y0 + (CUT_Y1 - CUT_Y0) * s
        r_in = PART_R + 0.002 if k == TOOL["ebauche"] else CUT_R[0]
        self.set_turret(y, r_in + (profile_r(y, allow) - r_in) * min(1.0, s / 0.03))
        self.set_coolant(True)
        if k == TOOL["ebauche"]:
            self.cut = s
            sl = self.part and self.plc.getFromDef(self.part.replace("PART", "SLV"))
            if sl:
                s_ = max(0.001, 1.0 - s)
                sl.getField("scale").setSFVec3f([1, 1, s_])
                sl.getField("translation").setSFVec3f([0, 0, 0.0225 * s_])

    def set_tool(self, k):
        """Station k in the working position (end of an indexing, or initial state)."""
        if self.tool != k:
            self.tool = k
            self.set_index(station_angle(k))

    def update(self, dt, running):
        # jaws
        if abs(self.jaw_r - self.jaw_target) > 1e-4:
            self.jaw_r += max(-0.05 * dt, min(0.05 * dt, self.jaw_target - self.jaw_r))
            self.set_jaws(self.jaw_r)
        self.chuck_switches()
        self.sw_lamp("door", self.door_open_switch(), [0, 1, 0])
        if not running:
            self.set_coolant(False)
            self.set_air(False)
            return
        st = self.state
        if st == "empty" and self.air_on is not False:              # start-up: blow the open chuck once
            self.set_air(self.plc.getTime() < AIR_START)
        if st in ("closing", "cutting", "opening"):
            self.t += dt
        if st == "closing":
            self.door_y = max(0.0, 1.0 - self.t / 2.0)
            self.set_door(self.door_y)
            if self.door_y <= 0.0:
                self.state, self.t = "cutting", 0.0
                self.plc.log("Lathe %s: door locked, spindle start" % self.name)
        elif st == "cutting":
            t = self.t
            self.spin = min(SPIN, SPIN * t / 1.0) if t < T_CUT + 3.5 else max(0.0, SPIN * (T_CUT + 4.5 - t))
            self.machining(t)
            if t >= T_CUT + 4.5:
                self.spin = 0.0
                self.state, self.t = "opening", 0.0
                self.cycles += 1
                self.plc.log("Lathe %s: part turned, door opening" % self.name)
            self.angle = (self.angle + self.spin * dt) % (2 * math.pi)
            self.spindle.setJointPosition(self.angle)
            self.place_part()
        elif st == "opening":
            self.door_y = min(1.0, self.t / 2.0)
            self.set_door(self.door_y)
            self.set_air(True)                       # M12: blow chips / coolant off the chuck and the part while opening
            if self.door_y >= 1.0 and self.door_open_switch():
                self.set_air(False)                  # M13
                self.state = "done"
                self.plc.log("Lathe %s: chuck blown (air jet), door open switch made" % self.name, overlay=False)
        # stack light
        self.lamp("G", st in ("closing", "cutting", "opening"))
        self.lamp("O", st in ("done", "empty", "robot"))
        self.lamp("R", self.plc.fault is not None)


class CellPLC(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt_ms = int(self.getBasicTimeStep())
        self.dt = self.dt_ms / 1000.0
        self.ds = {}
        for i in range(self.getNumberOfDevices()):
            d = self.getDeviceByIndex(i)
            if isinstance(d, DistanceSensor):
                d.enable(self.dt_ms * 4)
                self.ds[d.getName()] = d
        self.scanners = []
        for nm, x in (("scrutateur_gauche", -0.45), ("scrutateur_droit", 0.45)):
            self.scanners.append([(self.ds["%s_%02d" % (nm, k)], x, 1.2, math.pi * k / 30) for k in range(31)])
        self.tx = self.getDevice("plc_tx")
        self.rx = self.getDevice("robot_rx")
        self.rx.enable(self.dt_ms)
        self.kb = self.getKeyboard()
        self.kb.enable(self.dt_ms)
        self.lathes = {n: Lathe(self, n) for n in ("A", "B")}
        # crate contents: cell index -> part DEF
        self.crate = {"BRUTS": [None] * 12, "FINIES": [None] * 12}
        for k in range(1, 7):
            self.crate["BRUTS"][5 + k] = "PART_%d" % k
        for k in range(7, 13):
            self.crate["FINIES"][k - 7] = "PART_%d" % k
        self.robot_ready = False
        self.robot_busy = False
        self.job_id = 0
        self.held = {1: None, 2: None}            # what the robot holds (part DEF)
        self.job_lathe = None
        self.fault = None
        self.estop = False
        self.cycle_stop = False
        self.scan_zone = "clear"                  # clear | warning | protective
        self.scan_clear_t = 0.0
        self.log_lines = []
        self.parts_done = 0
        self.operator = self.getFromDef("OPERATEUR")
        self.op = dict(state="away", x=OPERATOR_HOME[0], y=OPERATOR_HOME[1], h=math.pi, t=0.0, phase=0.0)
        self.swap_requested = False
        self.place_operator()
        self.init_collisions()

    # ------------------------------------------------------------------ utilities
    def log(self, msg, overlay=True):
        line = "[%6.1f s] %s" % (self.getTime(), msg)
        print(line)
        if not overlay:                         # option events: console only (the video overlays stay unchanged)
            return
        self.log_lines = (self.log_lines + [line])[-2:]
        if "--sans-ihm" in sys.argv:            # README videos: no overlay
            return
        for i, s in enumerate(self.log_lines):
            self.setLabel(i, s, 0.01, 0.83 + 0.035 * i, 0.06, 0xFFE080, 0.0, "Arial")

    def send(self, **m):
        self.tx.send(json.dumps(m))

    def cell_xyz(self, crate, i):
        cx, cy = CRATES[crate]
        return [cx + CELL_X[i % 3], cy + CELL_Y[i // 3], PART_Z]

    # ------------------------------------------------------------------ robot jobs (world frame)
    def crate_frame(self, g, c):
        a = [0, 0, -1]
        # idle gripper pointing sideways, away from the cell centre line: the jaw axis (long side of the EGK housing)
        # then runs along the 600 mm side of the crate, clear of the crate walls
        sx = 1.0 if c[0] > 0 else -1.0
        m = [sx * math.cos(CRATE_M_ANGLE), math.sin(CRATE_M_ANGLE), 0.0]
        return a, tool_x(g, a, m)

    def crate_steps_pick(self, g, crate, i, part):
        c = self.cell_xyz(crate, i)
        grip = [c[0], c[1], c[2] + PART_L / 2 - GRIP_FROM_BACK]
        above = [grip[0], grip[1], grip[2] + CRATE_UP]
        d, x = self.crate_frame(g, c)
        return [dict(t="open", g=g), dict(t="joint", g=g, p=above, a=d, x=x, seed=CRATE_SEEDS[g]),
                dict(t="lin", g=g, p=grip, a=d, x=x, v=0.25),
                dict(t="grip", g=g, part=part), dict(t="lin", g=g, p=above, a=d, x=x, v=0.3)]

    def crate_steps_place(self, g, crate, i):
        c = self.cell_xyz(crate, i)
        grip = [c[0], c[1], c[2] + PART_L / 2 - GRIP_FROM_BACK]
        start = [grip[0], grip[1], grip[2] + 0.02]
        above = [grip[0], grip[1], grip[2] + CRATE_UP]
        d, x = self.crate_frame(g, c)
        return [dict(t="joint", g=g, p=above, a=d, x=x, seed=CRATE_SEEDS[g]), dict(t="lin", g=g, p=start, a=d, x=x, v=0.25),
                dict(t="force", g=g, dir=d, contact=0.02, max=0.035, F=20, v=0.03),
                dict(t="open", g=g), dict(t="lin", g=g, p=above, a=d, x=x, v=0.3)]

    def lathe_steps(self, L, unload):
        lt = self.lathes[L]
        a = lt.approach()
        m = lt.wdir(LATHE_M)
        x1, x2 = tool_x(1, a, m), tool_x(2, a, m)
        out, pre, seat = lt.tcp_out(), lt.tcp_pre(), lt.tcp_seat()
        s = [dict(t="home")]                # same way in from every crate cell (above the table, crate seed pose)
        if unload:
            # reach the retreat point with the gripper-1 wrist orientation, then half a turn of wrist 3 in place:
            # the direct joint move with gripper 2 swings the bracket across the forearm
            s += [dict(t="open", g=2), dict(t="joint", g=1, p=out, a=a, x=x1, seed=SEEDS[(L, 1)]),
                  dict(t="joint", g=2, p=out, a=a, x=x2, seed=SEEDS[(L, 2)]),
                  dict(t="lin", g=2, p=pre, a=a, x=x2, v=0.4),
                  dict(t="lin", g=2, p=seat, a=a, x=x2, v=0.12), dict(t="grip", g=2, part=lt.part),
                  dict(t="req", what="chuck_open", lathe=L),
                  dict(t="lin", g=2, p=pre, a=a, x=x2, v=0.2), dict(t="lin", g=2, p=out, a=a, x=x2, v=0.4)]
        s += [dict(t="joint", g=1, p=out, a=a, x=x1, seed=SEEDS[(L, 1)]), dict(t="lin", g=1, p=pre, a=a, x=x1, v=0.4),
              dict(t="force", g=1, dir=a, contact=PRE_DY, max=PRE_DY + 0.015, F=40, v=0.05),
              dict(t="req", what="chuck_clamp", lathe=L), dict(t="open", g=1),
              dict(t="lin", g=1, p=pre, a=a, x=x1, v=0.2), dict(t="lin", g=1, p=out, a=a, x=x1, v=0.4)]
        # always leave through the home pose too (the idle gripper passes next to the CNC pendant): the way out does
        # not depend on the next crate cell
        if RETREAT_HOME:                    # door closes only once the arm is back above the table
            s += [dict(t="home"), dict(t="evt", name="clear", lathe=L)]
        else:
            s += [dict(t="evt", name="clear", lathe=L), dict(t="home")]
        return s

    def send_job(self, steps, what):
        self.job_id += 1
        self.robot_busy = True
        self.send(cmd="job", id=self.job_id, steps=steps)
        self.log("Robot job #%d: %s" % (self.job_id, what))

    def schedule(self):
        if not self.robot_ready or self.robot_busy or self.fault or self.op["state"] != "away":
            return
        if self.held[2]:                                          # finished part to put away
            free = [i for i, p in enumerate(self.crate["FINIES"]) if p is None]
            if free:
                self.pending = ("place", free[0], self.held[2])
                self.send_job(self.crate_steps_place(2, "FINIES", free[0]), "finished part -> output crate")
            else:
                self.request_swap("output crate full")
            return
        if not self.held[1]:
            if self.cycle_stop:
                return
            cells = [i for i, p in enumerate(self.crate["BRUTS"]) if p]
            if cells:
                i = cells[0]
                part = self.crate["BRUTS"][i]
                self.pending = ("pick", i, part)
                self.send_job(self.crate_steps_pick(1, "BRUTS", i, part), "pick a blank (cell %d)" % (i + 1))
            else:
                self.request_swap("blank crate empty")
            return
        # holding a blank: serve a lathe that is waiting (finished part first)
        ready = [lt for lt in self.lathes.values() if lt.state in ("done", "empty") and lt.entry_permitted()]
        if not ready:
            return
        ready.sort(key=lambda lt: 0 if lt.state == "done" else 1)
        lt = ready[0]
        lt.state = "robot"
        self.job_lathe = lt.name
        unload = lt.part is not None
        self.send_job(self.lathe_steps(lt.name, unload),
                      "lathe %s: %s" % (lt.name, "unload + load (dual gripper)" if unload else "load"))

    def robot_events(self):
        while self.rx.getQueueLength() > 0:
            m = json.loads(self.rx.getString())
            self.rx.nextPacket()
            e = m.get("evt")
            if e == "ready":
                self.robot_ready = True
                self.log("UR10e ready")
            elif e == "gripped":
                self.held[m["g"]] = m["part"]
                pend = getattr(self, "pending", None)
                if pend and pend[0] == "pick" and pend[2] == m["part"]:
                    self.crate["BRUTS"][pend[1]] = None
                for lt in self.lathes.values():
                    if lt.part == m["part"]:
                        lt.part = None
                self.log("Gripper %d: part gripped (%d N / %d N fingertip force)" % (m["g"], m["Fa"], m["Fb"]))
            elif e == "force_contact":
                self.log("F/T sensor: contact detected at %.0f N" % m["F"])
            elif e == "req":
                lt = self.lathes[m["lathe"]]
                if m["what"] == "chuck_open":
                    lt.jaw_target = 0.05
                    self.ack_t = 0.6
                elif m["what"] == "chuck_clamp":
                    lt.part = self.held[1]
                    lt.finished = False
                    lt.jaw_target = PART_R
                    self.ack_t = 0.6
                self.log("Lathe %s: %s" % (lt.name, "chuck open" if m["what"] == "chuck_open" else "chuck clamped"))
            elif e == "released":
                g, d = m["g"], m["part"]
                self.held[g] = None
                pend = getattr(self, "pending", None)
                if pend and pend[0] == "place" and pend[2] == d:
                    self.crate["FINIES"][pend[1]] = d
                    self.snap_in_crate(d, "FINIES", pend[1])
                    self.parts_done += 1
                for lt in self.lathes.values():
                    if lt.part == d:
                        lt.angle = 0.0
                        lt.place_part()
            elif e == "clear":
                self.lathes[m["lathe"]].start_cycle()
            elif e == "job_done":
                self.robot_busy = False
            elif e == "error":
                self.log("ROBOT: " + m.get("msg", ""))

    def snap_in_crate(self, d, crate, i):
        n = self.getFromDef(d)
        n.getField("translation").setSFVec3f(self.cell_xyz(crate, i))
        n.getField("rotation").setSFRotation([0, 0, 1, 0])

    # ------------------------------------------------------------------ operator (crate exchange)
    def request_swap(self, why):
        if not self.swap_requested and self.op["state"] == "away":
            self.swap_requested = True
            self.log("Crate exchange requested (%s)" % why)
            self.op["state"], self.op["t"] = "walk_in", 0.0

    def place_operator(self):
        o = self.op
        self.operator.getField("translation").setSFVec3f([o["x"], o["y"], 1.27])
        self.operator.getField("rotation").setSFRotation([0, 0, 1, o["h"]])

    def gait(self, moving):
        o = self.op
        if moving:
            o["phase"] = (o["phase"] + 0.9 * self.dt / 0.22) % 8
        k = int(o["phase"]) % 8
        r = o["phase"] - int(o["phase"])
        for name, tab in zip(GAIT_FIELDS, GAIT):
            v = tab[k] * (1 - r) + tab[(k + 1) % 8] * r if moving else 0.0
            self.operator.getField(name).setSFFloat(v)

    def plant_operator(self):
        o = self.op
        if o["state"] in ("walk_in", "walk_out"):
            tx, ty = OPERATOR_TABLE if o["state"] == "walk_in" else OPERATOR_HOME
            dx, dy = tx - o["x"], ty - o["y"]
            d = math.hypot(dx, dy)
            v = 0.9 * self.dt
            if d <= v:
                o["x"], o["y"] = tx, ty
                if o["state"] == "walk_in":
                    o["state"], o["t"] = "swap", 0.0
                    o["h"] = -math.pi / 2
                    self.log("Operator at the crate table: exchanging crates")
                else:
                    o["state"] = "away"
                    o["h"] = math.pi
                self.gait(False)
            else:
                o["x"] += dx / d * v
                o["y"] += dy / d * v
                o["h"] = math.atan2(dy, dx)
                self.gait(True)
            self.place_operator()
        elif o["state"] == "swap":
            o["t"] += self.dt
            if o["t"] > 4.0:
                self.do_swap()
                o["state"] = "walk_out"
                self.swap_requested = False

    def do_swap(self):
        """New blank crate / empty output crate: the finished parts are recycled as blanks (simulation pool)."""
        moved = 0
        for i, d in enumerate(self.crate["FINIES"]):
            if d is None:
                continue
            free = [j for j, p in enumerate(self.crate["BRUTS"]) if p is None]
            if not free:
                break
            j = free[-1]
            self.crate["BRUTS"][j] = d
            self.crate["FINIES"][i] = None
            self.snap_in_crate(d, "BRUTS", j)
            self.getFromDef(d.replace("PART", "SLV")).getField("scale").setSFVec3f([1, 1, 1])
            self.getFromDef(d.replace("PART", "SLV")).getField("translation").setSFVec3f([0, 0, 0.0225])
            self.getFromDef(d.replace("PART", "PBO")).getField("radius").setSFFloat(PART_R)
            moved += 1
        self.log("Crates exchanged: %d new blanks, empty output crate" % moved)

    # ------------------------------------------------------------------ collision monitor
    # Every contact of the robot (links, bent bracket, grippers, fingers) is read from the physics engine. The only
    # expected contact is a finger pad on the part being gripped; anything else is a collision: logged, shown by a red
    # marker at the contact point and by a red line in the 3D view.
    # In Webots: View > Optional Rendering > Show Contact Points / Show All Bounding Objects shows them live too.
    def init_collisions(self):
        self.ur = self.getFromDef("UR10E")
        self.coll_markers = [self.getFromDef("COLL_%d" % k) for k in range(8)]
        self.coll_k = 0
        self.coll_count = 0
        self.coll_last = {}
        self.coll_obstacles = []
        for d in ("TOUR_A", "TOUR_B", "TABLE", "CAISSE_BRUTS", "CAISSE_FINIES", "ALVEOLES_BRUTS", "ALVEOLES_FINIES", "SOCLE",
                  "OPERATEUR"):
            n = self.getFromDef(d)
            if n:
                self.coll_obstacles.append((d, n))
        self.parts = []
        for k in range(1, 17):
            n = self.getFromDef("PART_%d" % k)
            if n:
                self.parts.append(("PART_%d" % k, n))
                self.coll_obstacles.append(("PART_%d" % k, n))
        self.node_names = {}

    def node_name(self, nid, p=None):
        if nid not in self.node_names and self.getFromId(nid) is None:
            # body of the robot PROTO not exposed to the supervisor: name it from where the contact is
            tcps = [self.getFromDef("TCP1"), self.getFromDef("TCP2")]
            near = p is not None and any(t and math.dist(p, t.getPosition()) < 0.3 for t in tcps)
            return "dual gripper / wrist" if near else "UR10e arm"
        if nid not in self.node_names:
            n = self.getFromId(nid)
            name = "?"
            if n:
                f = n.getField("name") or (n.getProtoField("name") if hasattr(n, "getProtoField") else None)
                name = f.getSFString() if f else n.getTypeName()
            self.node_names[nid] = name
        return self.node_names[nid]

    @staticmethod
    def cp_fields(cp):
        p = getattr(cp, "point", None) or cp.getPoint()
        nid = getattr(cp, "node_id", None)
        return list(p), (cp.getNodeId() if nid is None else nid)

    def collisions(self):
        cps = self.ur.getContactPoints(True)
        if not cps:
            return
        hits = []
        for cp in cps:
            p, nid = self.cp_fields(cp)
            link = self.node_name(nid, p)
            if link.endswith("_effort"):          # finger pad: expected on the part being gripped
                if any(math.dist(p, n.getPosition()) < 0.07 for _, n in self.parts):
                    continue
            hits.append((link, p))
        if not hits:
            return
        t = self.getTime()
        for link, p in hits:
            other = None
            for d, n in self.coll_obstacles:
                for cp in n.getContactPoints(True) or []:
                    if math.dist(self.cp_fields(cp)[0], p) < 0.003:
                        other = d
                        break
                if other:
                    break
            if other is None:
                others = [l2 for l2, p2 in hits if l2 != link and math.dist(p2, p) < 0.003]
                other = ("UR10e " + others[0]) if others else "UR10e (self) / unknown"
            key = (link, other)
            if t - self.coll_last.get(key, -9.0) < 2.0:
                continue
            self.coll_last[key] = t
            self.coll_count += 1
            m = self.coll_markers[self.coll_k % len(self.coll_markers)]
            self.coll_k += 1
            if m:
                m.getField("translation").setSFVec3f(p)
            msg = "COLLISION #%d: %s <-> %s at (%.2f, %.2f, %.2f)" % (self.coll_count, link, other, p[0], p[1], p[2])
            print("[%6.1f s] %s" % (t, msg))
            if "--sans-ihm" not in sys.argv:
                self.setLabel(2, msg, 0.01, 0.80, 0.06, 0xFF3030, 0.0, "Arial")

    # ------------------------------------------------------------------ safety
    def scanner(self):
        prot = warn = 0
        for rays in self.scanners:
            for ds, px, py, a in rays:
                r = ds.getValue()
                if r >= 3.95:
                    continue
                x, y = px + r * math.cos(a), py + r * math.sin(a)
                if SCAN_PROT[0] <= x <= SCAN_PROT[1] and SCAN_PROT[2] <= y <= SCAN_PROT[3]:
                    prot += 1
                elif SCAN_WARN[0] <= x <= SCAN_WARN[1] and SCAN_WARN[2] <= y <= SCAN_WARN[3]:
                    warn += 1
        zone = "protective" if prot else ("warning" if warn else "clear")
        if zone == "clear" and self.scan_zone != "clear":
            self.scan_clear_t += 4 * self.dt
            if self.scan_clear_t < 1.0:
                return
        self.scan_clear_t = 0.0
        if zone != self.scan_zone:
            self.scan_zone = zone
            if zone == "protective":
                self.send(cmd="hold")
                self.log("Laser scanner: person in protective field -> robot protective stop")
            elif zone == "warning":
                self.send(cmd="release_hold")
                self.send(cmd="speed", value=0.25)
                self.log("Laser scanner: person in warning field -> reduced speed 25 %")
            else:
                self.send(cmd="release_hold")
                self.send(cmd="speed", value=1.0)
                self.log("Laser scanner: zone clear -> full speed")

    def keys(self):
        ks = []
        k = self.kb.getKey()
        while k != -1:
            ks.append(chr(k & 0xFF).upper() if (k & 0xFF) < 128 else "")
            k = self.kb.getKey()
        cd = self.getCustomData()
        if cd:
            ks += list(cd.upper())
            self.setCustomData("")
        for c in ks:
            if c == "E":
                self.estop = not self.estop
                if self.estop:
                    self.fault = "EMERGENCY STOP"
                    self.send(cmd="hold")
                self.log("Emergency stop %s" % ("PRESSED" if self.estop else "released"))
            elif c == "R":
                if self.fault and not self.estop:
                    self.fault = None
                    if self.scan_zone != "protective":
                        self.send(cmd="release_hold")
                    self.log("Reset OK - cycle resumed")
                elif self.estop:
                    self.log("Reset refused: emergency stop still pressed")
            elif c == "P":
                self.request_swap("operator request")
            elif c == "S":
                self.cycle_stop = not self.cycle_stop
                self.log("End-of-cycle stop requested" if self.cycle_stop else "Production restarted")

    # ------------------------------------------------------------------ main loop
    def run(self):
        self.step(self.dt_ms)
        self.ack_t = None
        self.log("Cell PLC started")
        frame = 0
        while self.step(self.dt_ms) != -1:
            frame += 1
            self.keys()
            if frame % 4 == 0:
                self.scanner()
            self.robot_events()
            self.collisions()
            if self.ack_t is not None:
                self.ack_t -= self.dt
                if self.ack_t <= 0:
                    self.ack_t = None
                    self.send(cmd="ack")
            running = self.fault is None
            for lt in self.lathes.values():
                lt.update(self.dt, running)
            self.plant_operator()
            if running:
                self.schedule()


if __name__ == "__main__":
    CellPLC().run()
