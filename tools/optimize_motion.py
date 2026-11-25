#!/usr/bin/env python3
"""Search of the robot station configurations that minimise the motion of the big joints (base, shoulder, elbow).

For every candidate orientation of the dual gripper (in the lathes: direction of the idle gripper, at the crates:
angle of the idle gripper) the inverse kinematics gives several arm configurations (elbow up / down, wrist flips...).
Each lathe and crate candidate is first checked alone (collision-free job, tools/check_collisions.py), then the
best combinations are replayed over the whole production cycle. The score is the joint travel of J1 + J2 + J3 (rad)
plus 0.15 × the travel of the wrist joints, with no collision and no IK error allowed.

    uv run --no-project --with numpy tools/optimize_motion.py        (a few minutes; prints the best settings)
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_collisions as C  # noqa: E402

PLC, UR, DG = C.PLC, C.UR, C.DG
from ur10e_kin import ik  # noqa: E402

W = [1, 1, 1, 0.15, 0.15, 0.15]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def replay(jobs, q0, start=None):
    """Replays jobs from q0: (n_collisions, min clearance, weighted travel, travel per joint, ik errors)."""
    rb = C.Replay()
    rb.q_cmd = list(q0)
    held = {1: False, 2: False}
    n, mn, tr = 0, 1.0, [0.0] * 6
    for label, steps, cell in jobs:
        served = label.split()[1] if label.startswith("lathe") else ""
        ignore = {tuple(cell)} if cell else set()
        for st in steps:
            if st["t"] == "grip":
                held[st["g"]] = True
            elif st["t"] == "open":
                held[st["g"]] = False
            prev = list(rb.q_cmd)
            for q in C.samples(rb, st):
                for j in range(6):
                    tr[j] += abs(q[j] - prev[j])
                prev = list(q)
                for d, a, b in C.check(q, held, label, ignore, served):
                    mn = min(mn, d)
                    n += d < 0
    return n, mn, sum(w * t for w, t in zip(W, tr)), tr, len(rb.errors)


def ik_family(Tb, g, n=150, seed=0):
    random.seed(seed)
    sols = []
    for _ in range(n):
        q = ik(Tb, [random.uniform(-math.pi, math.pi) for _ in range(6)], UR.TCP[g])
        if q is None:
            continue
        q = [wrap(a) for a in q]
        if all(max(abs(wrap(a - b)) for a, b in zip(q, s)) > 0.3 for s in sols):
            sols.append(q)
    return sols


def half_turn(q):
    q2 = list(q)
    q2[5] = wrap(q[5] + math.pi)
    return q2


def to_cell(q):
    return [round(wrap(q[0] + DG.ROBOT_YAW), 3)] + [round(v, 3) for v in q[1:]]


def main():
    plc = C.make_plc()
    rb = C.Replay()
    # ---- lathe candidates (lathe A; B is the point-symmetric image: base joint + pi)
    lathes = []
    for phi in range(0, 360, 30):
        PLC.LATHE_M = [math.cos(math.radians(phi)), 0.0, math.sin(math.radians(phi))]
        st = [s for s in plc.lathe_steps("A", True) if s["t"] == "lin" and s["g"] == 1][0]
        for q in ik_family(rb.target(st), 1, seed=phi):
            qb = [wrap(q[0] + math.pi)] + q[1:]
            PLC.SEEDS = {("A", 1): q, ("A", 2): half_turn(q), ("B", 1): qb, ("B", 2): half_turn(qb)}
            steps = plc.lathe_steps("A", True)
            q_out = rb.solve(rb.target(steps[1]), 2, half_turn(q), half_turn(q))      # in front of the door
            if q_out is None:
                continue
            n, mn, cost, tr, ie = replay([("lathe A unload + load", steps, None)], q_out)
            if n == 0 and ie == 0:
                lathes.append((cost, phi, q))
    lathes.sort()
    print("%d collision-free lathe configurations" % len(lathes))
    for c in lathes[:6]:
        print("   cost %.1f phi %d %s" % (c[0], c[1], [round(v, 2) for v in c[2]]))
    # ---- crate candidates (corner cells of both crates)
    crates = []
    for psi in range(-90, 91, 15):
        PLC.CRATE_M_ANGLE = math.radians(psi)
        st = plc.crate_steps_pick(1, "BRUTS", 4, "P")[1]
        for q in ik_family(rb.target(st), 1, seed=1000 + psi):
            PLC.CRATE_SEEDS = {1: q, 2: half_turn(q)}
            jobs = [("pick blank cell %d" % (i + 1), plc.crate_steps_pick(1, "BRUTS", i, "P"), ("BRUTS", i))
                    for i in (0, 2, 9, 11)]
            jobs += [("finished part -> cell %d" % (i + 1), plc.crate_steps_place(2, "FINIES", i), ("FINIES", i))
                     for i in (0, 2, 9, 11)]
            q_up = rb.solve(rb.target(jobs[0][1][1]), 1, q, q)                       # above the crate
            if q_up is None:
                continue
            n, mn, cost, tr, ie = replay(jobs, q_up)
            if n == 0 and ie == 0:
                crates.append((cost, psi, q))
    crates.sort()
    print("%d collision-free crate configurations" % len(crates))
    for c in crates[:6]:
        print("   cost %.1f psi %d %s" % (c[0], c[1], [round(v, 2) for v in c[2]]), flush=True)
    # ---- combinations over the full cycle
    results = []
    for lc in lathes[:6]:
        for cc in crates[:6]:
            for retreat in (False, True):
                _, phi, ql = lc
                _, psi, qc = cc
                PLC.LATHE_M = [math.cos(math.radians(phi)), 0.0, math.sin(math.radians(phi))]
                qlb = [wrap(ql[0] + math.pi)] + ql[1:]
                PLC.SEEDS = {("A", 1): ql, ("A", 2): half_turn(ql), ("B", 1): qlb, ("B", 2): half_turn(qlb)}
                PLC.CRATE_M_ANGLE = math.radians(psi)
                PLC.CRATE_SEEDS = {1: qc, 2: half_turn(qc)}
                PLC.RETREAT_HOME = retreat
                UR.Q_HOME[:] = qc
                n, mn, cost, tr, ie = replay(C.cycle(plc), qc)
                print("phi %3d psi %3d retreat %d: coll %d ik %d min %.3f cost %.1f travel %s" % (
                    phi, psi, retreat, n, ie, mn, cost, [round(t, 1) for t in tr]), flush=True)
                if n == 0 and ie == 0:
                    results.append((cost, phi, psi, retreat, ql, qc, mn, tr))
    results.sort(key=lambda r: r[0])
    print("\nBEST")
    for cost, phi, psi, retreat, ql, qc, mn, tr in results[:5]:
        print("cost %.1f  LATHE_M angle %d  CRATE_M_ANGLE %d  RETREAT_HOME %s  min %.3f" % (cost, phi, psi, retreat, mn))
        print("   lathe A seed (cell axes) %s\n   crate seed (cell axes) %s\n   travel %s" % (
            to_cell(ql), to_cell(qc), [round(t, 1) for t in tr]))


if __name__ == "__main__":
    main()
