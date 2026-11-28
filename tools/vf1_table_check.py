#!/usr/bin/env python3
"""Clearances of the equipment on the Haas VF-1 table (vices, parts, OTS) against the machine, over the X/Y travel.

    uv run --no-project --with numpy --with scipy python -I tools/vf1_table_check.py [--png out.png]

Equipment shapes (boxes / vertical cylinders, table frame) come from controllers/vf1_demo/vf1_layout.py (same numbers
as worlds/haas_vf1_demo.wbt). Checks:
  1. enclosure: equipment against the static meshes, the closed doors, the tool changer carriage and carousel
     (retracted and slid in) and the tools hanging in all 20 pockets (CAT40 envelopes of tools/haas_vf1_to_meshes.py),
     at every pose of an X/Y grid including the four travel corners. The meshes are sampled into points (spacing
     SAMPLE_M), so distances are precise to about +-SAMPLE_M / 2.
  2. head: the head casting, spindle nose, air gun nozzle and every tool of the generator (and the OMP40-2 probe)
     against the equipment over the X/Y grid with Z at the top (Haas home), and the lowest Z that clears all the
     equipment everywhere for each tool (a safe retract height over the whole table).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "controllers", "vf1_demo"))
import vf1_layout as L                                                # noqa: E402

SAMPLE_M = 0.004
MESH = os.path.join(HERE, "..", "protos", "meshes", "haas_vf1")


def load_obj(path, offset):
    V, F = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                V.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                F.append([int(t.split("/")[0]) - 1 for t in line.split()[1:4]])
    return np.array(V) + np.array(offset), np.array(F, dtype=int)


def sample(V, F, lo, hi):
    """Surface points of the triangles touching the box [lo, hi]."""
    T = V[F]
    keep = np.all(T.max(axis=1) >= lo, axis=1) & np.all(T.min(axis=1) <= hi, axis=1)
    T = T[keep]
    pts = [T.reshape(-1, 3)]
    for t in T:
        e1, e2 = t[1] - t[0], t[2] - t[0]
        n = int(max(np.linalg.norm(e1), np.linalg.norm(e2)) / SAMPLE_M)
        if n < 1:
            continue
        a, b = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        m = a + b <= n
        pts.append(t[0] + np.outer(a[m] / n, e1) + np.outer(b[m] / n, e2))
    P = np.concatenate(pts)
    return P[np.all((P >= lo) & (P <= hi), axis=1)]


def shape_point_dist(s, P):
    """Distances from points P (table frame) to shape s."""
    if s[0] == "box":
        lo, hi = np.array(s[2]), np.array(s[3])
        d = np.maximum(np.maximum(lo - P, 0), P - hi)
        return np.linalg.norm(d, axis=1)
    _, _, c, r, z0, z1 = s
    dh = np.maximum(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) - r, 0)
    dz = np.maximum(np.maximum(z0 - P[:, 2], 0), P[:, 2] - z1)
    return np.hypot(dh, dz)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--png", help="top view of the layout with the X/Y travel")
    args = ap.parse_args()
    meta = json.load(open(os.path.join(MESH, "haas_vf1.json")))
    G, kp = meta["groups"], meta["key_positions"]
    eq = L.shapes()
    top = max((s[3][2] if s[0] == "box" else s[5]) for s in eq)
    print("equipment: %d shapes, highest point %.1f mm above the table" % (len(eq), top * 1000))

    # ---- 1. enclosure
    region_lo = np.array([-1.0, -1.10, L.TABLE_TOP - 0.01])
    region_hi = np.array([1.0, 0.25, L.TABLE_TOP + top + 0.12])
    clouds = {}
    for grp in ("static", "door_left", "door_right"):
        o = G[grp]["origin"] if grp != "static" else (0, 0, 0)
        pts = [sample(*load_obj(os.path.join(MESH, fn), o), region_lo, region_hi) for fn in G[grp]["files"]]
        clouds[grp] = np.concatenate(pts)
    for slide in (0.0, G["changer"]["range"][1]):
        o = list(G["changer"]["origin"])
        o[0] += slide
        pts = [sample(*load_obj(os.path.join(MESH, fn), o), region_lo, region_hi)
               for fn in G["changer"]["files"] + G["carousel"]["files"]]
        clouds["changer s=%.3f" % slide] = np.concatenate(pts)
    from haas_vf1_to_meshes import PROBE, TOOL_TOP, TOOLS, tool_parts   # noqa: E402  (OCP not needed at import? see below)
    print("sampled points: " + ", ".join("%s %d" % (k, len(v)) for k, v in clouds.items()))
    poses = [(x, y) for x in np.linspace(*L.X_RANGE, 9) for y in np.linspace(*L.Y_RANGE, 9)]
    worst = {}
    for X, Y in poses:
        org = np.array([L.TABLE_TOP_XY0[0] + X, L.TABLE_TOP_XY0[1] + Y, L.TABLE_TOP])
        for name, P in clouds.items():
            if not len(P):
                continue
            Pt = P - org
            for s in eq:
                d = shape_point_dist(s, Pt).min()
                key = (s[1], name)
                if key not in worst or d < worst[key][0]:
                    worst[key] = (d, X, Y)
        # tools hanging in the pockets (all 20, carriage retracted and slid in)
        for slide in (0.0, G["changer"]["range"][1]):
            c = G["carousel"]["origin"]
            for k in range(20):
                a = -k * 2 * math.pi / 20
                px = c[0] + slide + G["carousel"]["pocket_radius"] * math.cos(a) - org[0]
                py = c[1] + G["carousel"]["pocket_radius"] * math.sin(a) - org[1]
                for n in list(range(1, len(TOOLS) + 1)) + ["probe"]:
                    parts = TOOL_TOP + PROBE["parts"] if n == "probe" else tool_parts(n)
                    for kind, r, z0, z1, _ in parts:
                        zc = c[2] - org[2]
                        tc = ("cyl", "pocket tool", (px, py), r, zc + min(z0, z1) - (r if kind == "sphere" else 0),
                              zc + max(z0, z1))
                        for s in eq:
                            d = L.dist(s, tc)
                            key = (s[1], "pocket tools")
                            if key not in worst or d < worst[key][0]:
                                worst[key] = (d, X, Y)
    by_eq = {}
    for (e, what), (d, X, Y) in worst.items():
        if e not in by_eq or d < by_eq[e][0]:
            by_eq[e] = (d, what, X, Y)
    print("\n1. equipment vs enclosure / doors / tool changer / pocket tools over the X/Y travel (9 x 9 poses):")
    for e, (d, what, X, Y) in sorted(by_eq.items(), key=lambda kv: kv[1][0]):
        print("  %-22s min %6.1f mm  (%s, X %+.3f Y %+.3f)" % (e, d * 1000, what, X, Y))
    print("  overall minimum: %.1f mm" % (min(v[0] for v in by_eq.values()) * 1000))

    # ---- 2. head and tools
    tools = {"T%d %s" % (i + 1, t["name"]): [(p[1], p[2] - (p[1] if p[0] == "sphere" else 0),
                                              p[3] if p[0] != "sphere" else p[2] + p[1])
                                             for p in tool_parts(i + 1)] for i, t in enumerate(TOOLS)}
    tools["OMP40-2 probe"] = [(p[1], p[2] - (p[1] if p[0] == "sphere" else 0), p[3] if p[0] != "sphere" else p[2] + p[1])
                              for p in TOOL_TOP + PROBE["parts"]]
    print("\n2. head + tool vs equipment, X/Y grid (9 x 9):")
    for name, cyls in tools.items():
        dmin = min(min(L.dist(h, s) for h in L.head_shapes(X, Y, 0.0, cyls) for s in eq) for X, Y in poses)
        # lowest Z clearing everything everywhere (bisection)
        lo, hi = L.Z_RANGE[0], 0.0
        for _ in range(30):
            mid = (lo + hi) / 2
            ok = all(min(L.dist(h, s) for h in L.head_shapes(X, Y, mid, cyls) for s in eq) > 0.0 for X, Y in poses)
            lo, hi = (lo, mid) if ok else (mid, hi)
        tip = min(c[1] for c in cyls)
        print("  %-22s Z = 0: min %6.1f mm;  lowest safe Z everywhere %+.4f m (tip %.1f mm above the highest point)"
              % (name, dmin * 1000, hi, (L.NOSE_Z_TOP + hi - L.GROOVE_TO_NOSE + tip - L.TABLE_TOP - top) * 1000))

    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.add_patch(Rectangle((-0.3295, -0.1834), 0.659, 0.3668, fc="#ddd", ec="k"))
        for y in (-0.125, 0, 0.125):
            ax.add_patch(Rectangle((-0.3295, y - 0.008), 0.659, 0.016, fc="#aaa", ec="none"))
        reach_x = sorted(L.spindle_in_table(X, 0)[0] for X in L.X_RANGE)
        reach_y = sorted(L.spindle_in_table(0, Y)[1] for Y in L.Y_RANGE)
        ax.add_patch(Rectangle((reach_x[0], reach_y[0]), reach_x[1] - reach_x[0], reach_y[1] - reach_y[0],
                               fc="none", ec="g", ls="--", label="spindle axis reach"))
        for s in eq:
            if s[0] == "box":
                ax.add_patch(Rectangle(s[2][:2], s[3][0] - s[2][0], s[3][1] - s[2][1], fc="none",
                                       ec="r" if "part" in s[1] else "b", lw=0.8))
            else:
                ax.add_patch(Circle(s[2], s[3], fc="none", ec="m"))
        ax.set_aspect("equal")
        ax.set_xlim(-0.36, 0.36)
        ax.set_ylim(-0.22, 0.24)
        ax.grid(True)
        ax.legend(loc="upper left")
        ax.set_title("VF-1 table (table frame, m): 2 x KSP3-F 100, OTS; green = spindle axis reach")
        plt.savefig(args.png, dpi=110)
        print("image:", args.png)


if __name__ == "__main__":
    main()
