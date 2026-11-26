"""UR10e machine tending controller: dual SCHUNK gripper, fingertip force sensors, emulated 6-axis wrist F/T sensor.

The cell PLC (cell_plc) sends jobs as lists of steps (JSON, channel 1); the robot runs them and replies with events
(channel 2).

Steps:
  {"t": "joint", "g": 1|2, "p": [x,y,z], "a": [approach], "x": [finger axis], "v": scale}  joint move (IK, world frame)
  {"t": "lin",   "g": ..., "p": ..., "a": ..., "x": ..., "v": m/s}                     linear Cartesian move
  {"t": "home"}                                                                          joint move to Q_HOME
  {"t": "force", "g": ..., "dir": [..], "contact": s_c, "max": s_max, "F": N, "v": m/s}
        force-guarded move along `dir`: the wrist F/T sensor (emulated: spring k in contact beyond s_c) stops the
        move once |F| >= F (seating a part against the chuck stop, setting a part down in the crate)
  {"t": "grip", "g": 1|2, "part": "PART_k"}   close the fingers, check the fingertip forces, attach the part
  {"t": "open", "g": 1|2}                     open the fingers, release the part
  {"t": "req", "what": "...", "lathe": "A"}   handshake with the PLC (chuck clamp/open...), waits for {"cmd": "ack"}
  {"t": "evt", "name": "..."}                 send an event to the PLC
PLC -> robot: {"cmd": "hold"} / {"cmd": "release_hold"} (protective stop), {"cmd": "speed", "value": 0..1}
(collaborative speed reduction requested by the laser scanner).
"""
import json
import math

from controller import Supervisor

from dual_gripper import JAW_STROKE, base_q, tcp_matrix
from ur10e_kin import fk, ik, pose, wrap_near

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TCP = {1: tcp_matrix(1), 2: tcp_matrix(2)}      # bent dual gripper (dual_gripper.py)
Q_HOME = base_q([-1.2, -1.806, -1.681, -1.033, 2.337, 0.274])  # above the crate table (same as the crate seed of gripper 1)
# The arm starts directly in Q_HOME (retreat pose, grippers above the crates): the joints are placed there by the
# supervisor API before the first step, so the stretched-out zero pose never appears.


def start_path(pan0=None):
    return [list(Q_HOME)]


V_JOINT = 1.4           # rad/s (average, quintic profile)
K_CONTACT = 40000.0     # N/m: stiffness of the emulated contact seen by the F/T sensor
GRIP_MIN_F = 50.0       # N per finger to accept a grip
FINGER_CLOSED = JAW_STROKE       # EGK 40: 6 mm per jaw


def quintic(s):
    s = max(0.0, min(1.0, s))
    return s * s * s * (10 - 15 * s + 6 * s * s)


def m3(T):
    return [[T[i][j] for j in range(3)] for i in range(3)]


def mmul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def tr3(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


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


def rot_from_aa(ax, a):
    x, y, z = ax
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


class Tending(Supervisor):
    def __init__(self):
        super().__init__()
        self.dt_ms = int(self.getBasicTimeStep())
        self.dt = self.dt_ms / 1000.0
        self.motors = [self.getDevice(n) for n in JOINTS]
        self.sensors = [self.getDevice(n + "_sensor") for n in JOINTS]
        for s in self.sensors:
            s.enable(self.dt_ms)
        for m in self.motors:
            m.setVelocity(3.0)
        self.fingers = {}
        for g in (1, 2):
            for side in ("a", "b"):
                name = "g%d_doigt_%s" % (g, side)
                mot = self.getDevice(name)
                ts = self.getDevice(name + "_effort")
                ts.enable(self.dt_ms)
                ps = self.getDevice(name + "_pos")
                ps.enable(self.dt_ms)
                mot.setVelocity(0.03)
                mot.enableForceFeedback(self.dt_ms)
                self.fingers[(g, side)] = (mot, ts, ps)
        self.tx = self.getDevice("robot_tx")
        self.rx = self.getDevice("plc_rx")
        self.rx.enable(self.dt_ms)
        self.base = self.getSelf()
        self.held = {1: None, 2: None}          # g -> (node, rel pos, rel rot, DEF)
        self.saved_bo = {}
        self.hold = False
        self.override = 1.0
        self.job = None
        self.steps = []
        self.cur = None
        self.wait_ack = False
        self.ft = [0.0, 0.0, 0.0]               # emulated wrist force (world frame)

    # ------------------------------------------------------------------ helpers
    def send(self, **m):
        self.tx.send(json.dumps(m))

    def base_T(self):
        p = self.base.getPosition()
        o = self.base.getOrientation()
        return [[o[0], o[1], o[2], p[0]], [o[3], o[4], o[5], p[1]], [o[6], o[7], o[8], p[2]], [0, 0, 0, 1]]

    def world_to_base(self, Tw):
        B = self.base_T()
        R = tr3(m3(B))
        p = [Tw[i][3] - B[i][3] for i in range(3)]
        pb = [sum(R[i][k] * p[k] for k in range(3)) for i in range(3)]
        Rb = mmul3(R, m3(Tw))
        return [Rb[0] + [pb[0]], Rb[1] + [pb[1]], Rb[2] + [pb[2]], [0, 0, 0, 1]]

    def target(self, st):
        Tw = pose(st["p"], st["a"], st["x"])
        return self.world_to_base(Tw)

    def solve(self, Tb, g, seed, station=None):
        if station:                                     # preferred configuration family for this station
            q = ik(Tb, station, TCP[g])
            if q is not None:
                return wrap_near(q, seed)
        best = None
        for s in (seed, Q_HOME):
            q = ik(Tb, s, TCP[g])
            if q is None:
                continue
            q = wrap_near(q, seed)          # nearest 2π-equivalent within the joint limits (unwinds if needed)
            d = max(abs(a - b) for a, b in zip(q, seed))
            if best is None or d < best[0]:
                best = (d, q)
        return None if best is None else best[1]

    def tcp_world(self, g, q=None):
        q = q if q is not None else [s.getValue() for s in self.sensors]
        T = fk(q, TCP[g])
        B = self.base_T()
        R = mmul3(m3(B), m3(T))
        p = [B[i][3] + sum(B[i][k] * T[k][3] for k in range(3)) for i in range(3)]
        return p, R

    # ------------------------------------------------------------------ gripper
    def finger_forces(self, g):
        out = []
        for side in ("a", "b"):
            mot, ts, _ = self.fingers[(g, side)]
            # clamping force = finger drive force feedback, only counted when the fingertip sensor is in contact
            out.append(abs(mot.getForceFeedback()) if ts.getValue() > 0.5 else 0.0)
        return out

    def set_fingers(self, g, pos):
        for side in ("a", "b"):
            self.fingers[(g, side)][0].setPosition(pos)

    def attach(self, g, d):
        n = self.getFromDef(d)
        p, R = self.tcp_world(g)
        Rt = tr3(R)
        bp = n.getPosition()
        br = n.getOrientation()
        BR = [[br[0], br[1], br[2]], [br[3], br[4], br[5]], [br[6], br[7], br[8]]]
        rel_p = [sum(Rt[i][k] * (bp[k] - p[k]) for k in range(3)) for i in range(3)]
        self.held[g] = (n, rel_p, mmul3(Rt, BR), d)
        # while carried, the part must not collide with the arm (kinematically attached = immovable obstacle):
        # shrink its collision shapes, keep the fingers where they clamped
        k = d.split("_")[1]
        self.saved_bo[d] = []
        for pre in ("PBA_", "PBO_"):
            c = self.getFromDef(pre + k)
            if c:
                f = c.getField("radius")
                self.saved_bo[d].append((f, f.getSFFloat()))
                f.setSFFloat(0.0005)
        for side in ("a", "b"):
            mot, _, ps = self.fingers[(g, side)]
            mot.setPosition(ps.getValue())

    def carry(self):
        for g in (1, 2):
            h = self.held[g]
            if not h:
                continue
            n, rp, rr, _ = h
            p, R = self.tcp_world(g)
            wp = [p[i] + sum(R[i][k] * rp[k] for k in range(3)) for i in range(3)]
            n.getField("translation").setSFVec3f(wp)
            n.getField("rotation").setSFRotation(axis_angle(mmul3(R, rr)))

    # ------------------------------------------------------------------ step execution
    def start_step(self, st):
        t = st["t"]
        q0 = list(self.q_cmd)
        st["_t"] = 0.0
        if t in ("joint", "home"):
            q1 = list(Q_HOME) if t == "home" else self.solve(self.target(st), st["g"], q0, st.get("seed"))
            if q1 is None:
                self.send(evt="error", msg="IK failed (joint) %s" % st["p"])
                q1 = q0
            st["_q0"], st["_q1"] = q0, q1
            st["_T"] = max(0.5, max(abs(a - b) for a, b in zip(q0, q1)) / V_JOINT * 1.875 / 1.875)
        elif t in ("lin", "force"):
            p0, R0 = self.tcp_world(st["g"], q0)
            if t == "lin":
                Tb_goal = pose(st["p"], st["a"], st["x"])
                p1, R1 = [Tb_goal[i][3] for i in range(3)], m3(Tb_goal)
            else:
                d = st["dir"]
                p1 = [p0[i] + d[i] * st["max"] for i in range(3)]
                R1 = R0
            Rrel = mmul3(R1, tr3(R0))
            aa = axis_angle(Rrel)
            st["_p0"], st["_p1"], st["_R0"], st["_aa"] = p0, p1, R0, aa
            dist = math.dist(p0, p1)
            if t == "lin":
                st["_T"] = max(0.3, 1.875 * dist / st.get("v", 0.4), abs(aa[3]) / 1.2)
            else:
                st["_T"] = dist / st.get("v", 0.02)
                st["_s"] = 0.0
        elif t == "grip":
            self.set_fingers(st["g"], FINGER_CLOSED)
            st["_T"] = 1.2
        elif t == "open":
            if self.held[st["g"]]:
                n, _, _, d = self.held[st["g"]]
                self.held[st["g"]] = None
                for f, r in self.saved_bo.pop(d, []):
                    f.setSFFloat(r)
                p, R = n.getPosition(), n.getOrientation()
                self.send(evt="released", g=st["g"], part=d, p=p, R=R)
            self.set_fingers(st["g"], 0.0)
            st["_T"] = 0.5
        elif t == "req":
            self.send(evt="req", what=st["what"], lathe=st.get("lathe"))
            self.wait_ack = True
            st["_T"] = 0.0
        elif t == "evt":
            self.send(evt=st["name"], **{k: v for k, v in st.items() if k not in ("t", "name") and not k.startswith("_")})
            st["_T"] = 0.0
        self.cur = st

    def cart_q(self, st, s):
        p = [a + (b - a) * s for a, b in zip(st["_p0"], st["_p1"])]
        aa = st["_aa"]
        R = mmul3(rot_from_aa(aa[:3], aa[3] * s), st["_R0"])
        Tw = [R[0] + [p[0]], R[1] + [p[1]], R[2] + [p[2]], [0, 0, 0, 1]]
        Tb = self.world_to_base(Tw)
        q = ik(Tb, self.q_cmd, TCP[st["g"]], iters=15)
        if q is not None:
            q = wrap_near(q, self.q_cmd)
            if max(abs(a - b) for a, b in zip(q, self.q_cmd)) > 0.3:     # branch jump: not a linear move
                q = None
        if q is None:
            q = self.solve(Tb, st["g"], self.q_cmd)
            if q is not None and max(abs(a - b) for a, b in zip(q, self.q_cmd)) > 0.3:
                q = None
        if q is None:
            if not st.get("_ikerr"):
                st["_ikerr"] = True
                self.send(evt="error", msg="IK failed on linear move (%s)" % st["t"])
            return self.q_cmd
        return q

    def step_job(self):
        st = self.cur
        t = st["t"]
        if t == "req":
            return not self.wait_ack
        if t == "evt":
            return True
        k = self.override
        st["_t"] += self.dt * k
        if t in ("joint", "home"):
            s = quintic(st["_t"] / st["_T"])
            self.q_cmd = [a + (b - a) * s for a, b in zip(st["_q0"], st["_q1"])]
            return st["_t"] >= st["_T"]
        if t == "lin":
            self.q_cmd = self.cart_q(st, quintic(st["_t"] / st["_T"]))
            return st["_t"] >= st["_T"]
        if t == "force":
            st["_s"] = min(st["max"], st["_s"] + st.get("v", 0.02) * self.dt * k)
            frac = st["_s"] / st["max"]
            self.q_cmd = self.cart_q(st, frac)
            pen = max(0.0, st["_s"] - st["contact"])
            fmag = K_CONTACT * pen
            self.ft = [-c * fmag for c in st["dir"]]
            if fmag >= st["F"]:
                self.send(evt="force_contact", F=round(fmag, 1), s=round(st["_s"], 4))
                self.ft = [0.0, 0.0, 0.0]
                return True
            if st["_s"] >= st["max"]:
                self.send(evt="error", msg="no contact detected by the F/T sensor")
                return True
            return False
        if t == "grip":
            if st["_t"] < st["_T"]:
                return False
            fa, fb = self.finger_forces(st["g"])
            if min(fa, fb) >= GRIP_MIN_F:
                self.attach(st["g"], st["part"])
                self.send(evt="gripped", g=st["g"], part=st["part"], Fa=round(fa), Fb=round(fb))
            else:
                self.set_fingers(st["g"], 0.0)          # re-open after a missed grip
                pa = self.fingers[(st["g"], "a")][2].getValue()
                pb = self.fingers[(st["g"], "b")][2].getValue()
                qm = [s_.getValue() for s_ in self.sensors]
                pf, _ = self.tcp_world(st["g"], qm)
                pc, _ = self.tcp_world(st["g"], self.q_cmd)
                tn = self.getFromDef("TCP%d" % st["g"])
                pn = tn.getPosition() if tn else [0, 0, 0]
                self.send(evt="error", msg="grip failed g%d (%.0f N / %.0f N, stroke %.1f / %.1f mm) | dq(mrad)=%s "
                          "| fk(meas)-node=%s mm | cmd-meas=%s mm" % (
                              st["g"], fa, fb, pa * 1000, pb * 1000,
                              [round(1000 * (a - b)) for a, b in zip(self.q_cmd, qm)],
                              [round(1000 * (a - b)) for a, b in zip(pf, pn)],
                              [round(1000 * (a - b)) for a, b in zip(pc, pf)]))
            return True
        if t == "open":
            return st["_t"] >= st["_T"]
        return True

    def poll(self):
        while self.rx.getQueueLength() > 0:
            m = json.loads(self.rx.getString())
            self.rx.nextPacket()
            c = m.get("cmd")
            if c == "hold":
                self.hold = True
            elif c == "release_hold":
                self.hold = False
            elif c == "speed":
                self.override = max(0.05, min(1.0, float(m["value"])))
            elif c == "ack":
                self.wait_ack = False
            elif c == "job" and self.job is None:
                self.job = m["id"]
                self.steps = list(m["steps"])
                self.cur = None

    def place_at_home(self):
        """Puts every arm joint at Q_HOME before the first physics step (supervisor API on the HingeJoint holding
        each motor), so the robot appears in its retreat pose from t = 0."""
        for m, q in zip(self.motors, Q_HOME):
            self.getFromDevice(m._tag).getParentNode().setJointPosition(q)   # getFromDevice takes the device tag
            m.setPosition(q)

    def run(self):
        self.place_at_home()
        self.step(self.dt_ms)
        self.q_cmd = [s.getValue() for s in self.sensors]
        self.steps = [{"t": "_home"}]
        ready = False
        # start-up: already at Q_HOME (place_at_home); a short settle move absorbs any residual offset
        home_path = start_path()
        home_t, home_q0 = 0.0, list(self.q_cmd)
        T_home = max(1.0, max(abs(a - b) for a, b in zip(home_q0, home_path[0])) / 0.8)
        self.set_fingers(1, 0.0)
        self.set_fingers(2, 0.0)
        while self.step(self.dt_ms) != -1:
            self.poll()
            if not ready:
                home_t += self.dt
                s = quintic(home_t / T_home)
                self.q_cmd = [a + (b - a) * s for a, b in zip(home_q0, home_path[0])]
                if home_t >= T_home and len(home_path) > 1:
                    home_q0 = home_path.pop(0)
                    home_t = 0.0
                    T_home = max(1.0, max(abs(a - b) for a, b in zip(home_q0, home_path[0])) / 0.8)
                elif home_t >= T_home:
                    ready = True
                    self.steps = []
                    self.send(evt="ready")
            elif not self.hold and self.job is not None:
                if self.cur is None and self.steps:
                    self.start_step(self.steps.pop(0))
                if self.cur is not None and self.step_job():
                    self.cur = None
                    if not self.steps:
                        self.send(evt="job_done", id=self.job)
                        self.job = None
            for m, q in zip(self.motors, self.q_cmd):
                m.setPosition(q)
            self.carry()


if __name__ == "__main__":
    import sys
    import traceback
    try:
        Tending().run()
    except Exception:
        traceback.print_exc()
        with open("crash.log", "w") as fh:
            traceback.print_exc(file=fh)
        sys.stdout.flush()
        raise
