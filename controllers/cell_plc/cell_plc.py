"""Cell PLC (supervisor) of the robotic turning cell: 2 CNC lathes facing each other, UR10e, crates.

Two clearly separated parts:
  * PLANT (simulated): lathe doors, chuck jaws, spindle rotation, turret, coolant, material removal (stock sleeve
    shortening as the tool advances), crate exchange by the operator (walking pedestrian).
  * CONTROL: lathe state machines, robot job scheduling (dual gripper: unload the finished part and load the
    blank in the same visit), robot/machine handshakes (chuck clamp/open), safety (laser scanners with speed
    and separation monitoring, emergency stop, door interlocks).

Keys (3D view focused): E = emergency stop (toggle), R = reset, P = operator walks in to swap the crates,
S = end-of-cycle stop. The same letters can be written into the customData field of the "automate cellule" robot.
"""
import json
import math
import sys

from controller import DistanceSensor, Supervisor

# ---------------------------------------------------------------- geometry (see tools/gen_world.py)
CX, CZ, CHUCK_Y, JAW_FACE = -0.40, 1.10, -0.46, -0.355
SEAT_FRONT = -0.40                 # part front face against the chuck face (lathe frame)
PART_L, PART_R = 0.09, 0.025
GRIP_FROM_BACK = 0.01              # TCP 10 mm inside the part's back end
TABLE_Z = 0.70
CRATES = {"BRUTS": (-0.25, 0.85), "FINIES": (0.25, 0.85)}
CELL_X = (-0.12, 0.0, 0.12)
CELL_Y = (-0.195, -0.065, 0.065, 0.195)
PART_Z = TABLE_Z + 0.0063 + PART_L / 2
LATHES = {"A": (-0.65, 0.0), "B": (0.65, math.pi)}           # translation x, yaw
SPIN = 9.0                         # rad/s (visual spindle speed)
TUR_AX0 = 0.39                     # turret axial travel to the part's free end (tool tip y = 0.08 - s)
TUR_RAD = 0.111                    # turret radial travel to the cutting diameter
T_CUT = 16.0                       # s, turning pass
SCAN_PROT = (-0.9, 0.9, 1.40, 2.0)
SCAN_WARN = (-0.9, 0.9, 2.0, 2.8)
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


# joint seeds per lathe and gripper: base turned towards the lathe, elbow up, wrist 2 at ±90° (far from the
# wrist singularity). Both grippers share the flange pose, so switching gripper = half a turn of wrist 3.
SEEDS = {("A", 2): [2.285, -2.254, 2.314, -1.631, -1.571, -0.856], ("A", 1): [2.285, -2.254, 2.314, -1.631, -1.571, 2.285],
         ("B", 1): [-0.856, -2.254, 2.314, -1.631, -1.571, 2.285], ("B", 2): [-0.856, -2.254, 2.314, -1.631, -1.571, -0.856]}


CRATE_SEEDS = {1: [1.299, -1.478, 1.594, -0.116, 1.299, -1.571], 2: [1.299, -1.478, 1.594, -0.116, 1.299, 1.571]}


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def tool_x(g, a, n):
    """Finger axis giving flange normal n (gripper 1: n = -y_tool, gripper 2: n = +y_tool) for approach a."""
    y = [-c for c in n] if g == 1 else list(n)
    return cross(y, a)


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


class Lathe:
    def __init__(self, plc, name):
        self.plc, self.name = plc, name
        g = plc.getFromDef
        p = "TOUR_" + name
        self.x0, self.yaw = LATHES[name]
        self.R = rz(self.yaw)
        self.door = g(p + "_DOOR")
        self.chuck = g(p + "_CHUCK")
        self.jaws = [g("%s_JAW%d" % (p, i)) for i in (1, 2, 3)]
        self.turret = g(p + "_TURRET")
        self.coolant = g(p + "_COOLANT")
        self.lamps = {c: g("%s_L%s" % (p, c)) for c in "GOR"}
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

    # -- frames
    def world(self, p):
        v = mv(self.R, p)
        return [self.x0 + v[0], v[1], v[2]]

    def wdir(self, d):
        return mv(self.R, d)

    # -- actuators
    def set_door(self, s):
        self.door.getField("translation").setSFVec3f([0.04, -0.2 + 1.05 * s, 0.84])

    def set_jaws(self, r):
        for i, j in enumerate(self.jaws):
            a = 2 * math.pi * i / 3
            j.getField("translation").setSFVec3f([(r + 0.02) * math.cos(a), 0.0825, (r + 0.02) * math.sin(a)])

    def set_turret(self, s_axial, s_radial):
        # parked at (-0.72, 0.10, 1.42), tool tip 0.133 m from the spindle axis; the cross slide moves the tip
        # along the part (lathe -y) and towards the axis (45° slant bed)
        self.turret.getField("translation").setSFVec3f(
            [-0.72 + 0.707 * s_radial, 0.10 - s_axial, 1.42 - 0.707 * s_radial])

    def lamp(self, c, on):
        col = {"G": [0, 1, 0], "O": [1, 0.55, 0], "R": [1, 0, 0]}[c]
        self.lamps[c].getField("emissiveColor").setSFColor(col if on else [0, 0, 0])

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
        return self.world([CX, SEAT_FRONT + PART_L - GRIP_FROM_BACK + 0.10, CZ])

    def tcp_out(self):
        return self.world([0.32, SEAT_FRONT + PART_L - GRIP_FROM_BACK + 0.10, CZ + 0.12])

    # -- cycle
    def start_cycle(self):
        self.state, self.t = "closing", 0.0
        self.plc.log("Lathe %s: door closing, cycle start" % self.name)

    def update(self, dt, running):
        # jaws
        if abs(self.jaw_r - self.jaw_target) > 1e-4:
            self.jaw_r += max(-0.05 * dt, min(0.05 * dt, self.jaw_target - self.jaw_r))
            self.set_jaws(self.jaw_r)
        if not running:
            self.coolant.getField("scale").setSFVec3f([1, 1, 0.001])
            return
        st = self.state
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
            # turret: approach 1.5 s, roughing pass T_CUT, retract 1.5 s
            if t < 1.5:
                s = t / 1.5
                self.set_turret(TUR_AX0 * s, TUR_RAD * s)
            elif t < 1.5 + T_CUT:
                self.cut = (t - 1.5) / T_CUT
                self.set_turret(TUR_AX0 + 0.045 * self.cut, TUR_RAD + 0.003 * math.sin(self.cut * 3 * math.pi) ** 2)
                self.coolant.getField("scale").setSFVec3f([1, 1, 1])
                n = self.plc.getFromDef(self.part)
                sl = n and self.plc.getFromDef(self.part.replace("PART", "SLV"))
                if sl:
                    s_ = max(0.001, 1.0 - self.cut)
                    sl.getField("scale").setSFVec3f([1, 1, s_])
                    sl.getField("translation").setSFVec3f([0, 0, 0.0225 * s_])
            elif t < 3.0 + T_CUT:
                if not self.finished:
                    self.finished = True
                    self.plc.getFromDef(self.part.replace("PART", "PBO")).getField("radius").setSFFloat(0.015)
                self.coolant.getField("scale").setSFVec3f([1, 1, 0.001])
                s = 1.0 - (t - 1.5 - T_CUT) / 1.5
                self.set_turret((TUR_AX0 + 0.045) * s, TUR_RAD * s)
            elif t >= T_CUT + 4.5:
                self.spin = 0.0
                self.state, self.t = "opening", 0.0
                self.cycles += 1
                self.plc.log("Lathe %s: part turned, door opening" % self.name)
            self.angle = (self.angle + self.spin * dt) % (2 * math.pi)
            self.chuck.getField("rotation").setSFRotation([0, 1, 0, self.angle])
            self.place_part()
        elif st == "opening":
            self.door_y = min(1.0, self.t / 2.0)
            self.set_door(self.door_y)
            if self.door_y >= 1.0:
                self.state = "done"
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

    # ------------------------------------------------------------------ utilities
    def log(self, msg):
        line = "[%6.1f s] %s" % (self.getTime(), msg)
        print(line)
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
        n = [c[0], c[1], 0.0]                            # flange normal pointing away from the base: wrist on the robot side
        k = math.hypot(n[0], n[1])
        return a, tool_x(g, a, [n[0] / k, n[1] / k, 0.0])

    def crate_steps_pick(self, g, crate, i, part):
        c = self.cell_xyz(crate, i)
        grip = [c[0], c[1], c[2] + PART_L / 2 - GRIP_FROM_BACK]
        above = [grip[0], grip[1], grip[2] + 0.28]
        d, x = self.crate_frame(g, c)
        return [dict(t="open", g=g), dict(t="joint", g=g, p=above, a=d, x=x, seed=CRATE_SEEDS[g]),
                dict(t="lin", g=g, p=grip, a=d, x=x, v=0.25),
                dict(t="grip", g=g, part=part), dict(t="lin", g=g, p=above, a=d, x=x, v=0.3)]

    def crate_steps_place(self, g, crate, i):
        c = self.cell_xyz(crate, i)
        grip = [c[0], c[1], c[2] + PART_L / 2 - GRIP_FROM_BACK]
        start = [grip[0], grip[1], grip[2] + 0.02]
        above = [grip[0], grip[1], grip[2] + 0.28]
        d, x = self.crate_frame(g, c)
        return [dict(t="joint", g=g, p=above, a=d, x=x, seed=CRATE_SEEDS[g]), dict(t="lin", g=g, p=start, a=d, x=x, v=0.25),
                dict(t="force", g=g, dir=d, contact=0.02, max=0.035, F=20, v=0.03),
                dict(t="open", g=g), dict(t="lin", g=g, p=above, a=d, x=x, v=0.3)]

    def lathe_steps(self, L, unload):
        lt = self.lathes[L]
        a = lt.approach()
        x1, x2 = tool_x(1, a, [0, 0, -1]), tool_x(2, a, [0, 0, -1])     # flange facing down: wrist above
        out, pre, seat = lt.tcp_out(), lt.tcp_pre(), lt.tcp_seat()
        s = []
        if unload:
            s += [dict(t="open", g=2), dict(t="joint", g=2, p=out, a=a, x=x2, seed=SEEDS[(L, 2)]),
                  dict(t="lin", g=2, p=pre, a=a, x=x2, v=0.4),
                  dict(t="lin", g=2, p=seat, a=a, x=x2, v=0.12), dict(t="grip", g=2, part=lt.part),
                  dict(t="req", what="chuck_open", lathe=L),
                  dict(t="lin", g=2, p=pre, a=a, x=x2, v=0.2), dict(t="lin", g=2, p=out, a=a, x=x2, v=0.4)]
        s += [dict(t="joint", g=1, p=out, a=a, x=x1, seed=SEEDS[(L, 1)]), dict(t="lin", g=1, p=pre, a=a, x=x1, v=0.4),
              dict(t="force", g=1, dir=a, contact=0.10, max=0.115, F=40, v=0.05),
              dict(t="req", what="chuck_clamp", lathe=L), dict(t="open", g=1),
              dict(t="lin", g=1, p=pre, a=a, x=x1, v=0.2), dict(t="lin", g=1, p=out, a=a, x=x1, v=0.4),
              dict(t="evt", name="clear", lathe=L)]
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
        ready = [lt for lt in self.lathes.values() if lt.state in ("done", "empty")]
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


CellPLC().run()
