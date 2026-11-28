#!/usr/bin/env python3
"""Clearance check of the Haas VF-1 umbrella tool change (meshes of protos/meshes/haas_vf1 + procedural tools).

    uv run --no-project --python 3.12 --with cadquery-ocp --with numpy --with scipy --with matplotlib \
        python -I tools/vf1_atc_check.py [protos/meshes/haas_vf1] [--png out_dir]

Every mesh is sampled into surface points (spacing STEP_M) and placed at the poses of the tool change sequence, read
from haas_vf1.json (group origins, slide travel, clamping height, pocket radius). Distances are nearest-point
distances between the sampled surfaces (precision about +-STEP_M / 2); a value near 0 means contact or crossing.
The tools are the CAT40 holders of tools/haas_vf1_to_meshes.py (TOOLS), every type at every tested position.
Poses: s = carriage slide (m, 0 = retracted), Z = head (m, 0 = top), carousel turned so that a pocket is at the
station (+x of the carousel axis).
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # python -I drops the script directory
from haas_vf1_to_meshes import POCKETS, TOOLS, tool_parts         # noqa: E402

STEP_M = 0.003
REGION_PAD_M = 0.10                       # static parts: only points near the swept changer envelope


def load_obj(path):
    V, F = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                V.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                F.append([int(t.split("/")[0]) - 1 for t in line.split()[1:4]])
    return np.array(V), np.array(F, dtype=int)


def sample(V, F, step=STEP_M, region=None):
    """Points on the triangles about `step` apart: points along every edge + area-proportional interior points
    (seeded, deterministic); only those inside the box `region` = (lo, hi), if given."""
    rng = np.random.default_rng(0)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    out = []
    for p, q in ((a, b), (b, c), (c, a)):                        # edges
        n = np.maximum(1, np.ceil(np.linalg.norm(q - p, axis=1) / step).astype(int))
        tri = np.repeat(np.arange(len(F)), n)
        t = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)) / np.repeat(n, n)
        out.append(p[tri] + t[:, None] * (q - p)[tri])
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n = np.floor(2.0 * area / step ** 2 + rng.random(len(F))).astype(int)   # interior
    tri = np.repeat(np.arange(len(F)), n)
    u, v = rng.random(len(tri)), rng.random(len(tri))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    out.append(a[tri] + u[:, None] * (b - a)[tri] + v[:, None] * (c - a)[tri])
    P = np.concatenate(out)
    if region is not None:
        P = P[np.all((P >= region[0]) & (P <= region[1]), axis=1)]
    return np.unique(np.round(P, 4), axis=0)


def tool_points(n, step=STEP_M):
    """Surface points of tool n (V-groove centre at the origin), cylinders / cones / spheres of TOOLS."""
    pts = []
    for kind, r, z0, z1, _ in tool_parts(n):
        if kind == "sphere":
            k = max(8, int(math.pi * r / step))
            th, ph = np.meshgrid(np.linspace(0, math.pi, k), np.linspace(0, 2 * math.pi, 2 * k))
            pts.append(np.stack([r * np.sin(th) * np.cos(ph), r * np.sin(th) * np.sin(ph),
                                 z0 + r * np.cos(th)], -1).reshape(-1, 3))
            continue
        na = max(12, int(2 * math.pi * r / step))
        nz = max(2, int((z1 - z0) / step) + 1)
        a = np.linspace(0, 2 * math.pi, na, endpoint=False)
        z = np.linspace(z0, z1, nz)
        A, Z = np.meshgrid(a, z)
        R = r * (Z - z0) / (z1 - z0) if kind == "cone" else np.full_like(Z, r)
        pts.append(np.stack([R * np.cos(A), R * np.sin(A), Z], -1).reshape(-1, 3))
        for zc, rc in ((z0, 0.0 if kind == "cone" else r), (z1, r)):          # caps
            for rr in np.arange(step, rc, step):
                k = max(6, int(2 * math.pi * rr / step))
                b = np.linspace(0, 2 * math.pi, k, endpoint=False)
                pts.append(np.stack([rr * np.cos(b), rr * np.sin(b), np.full(k, zc)], -1))
    return np.concatenate(pts)


def rotz(P, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.stack([c * P[:, 0] - s * P[:, 1], s * P[:, 0] + c * P[:, 1], P[:, 2]], -1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("mesh_dir", nargs="?", default="protos/meshes/haas_vf1")
    ap.add_argument("--png", help="write control images (key poses) into this directory")
    args = ap.parse_args()
    meta = json.load(open(os.path.join(args.mesh_dir, "haas_vf1.json")))
    G, tc = meta["groups"], meta["key_positions"]["tool_change"]
    T, ZC, R, pitch = tc["slide_travel"], tc["z_clamp"], tc["pocket_radius"], G["carousel"]["pitch_deg"]
    g2n = tc["groove_to_nose"]

    def group_pts(grp, region=None):
        out = []
        for fn in G[grp]["files"]:
            V, F = load_obj(os.path.join(args.mesh_dir, fn))
            if region is not None:                     # static: world frame, clipped to the region
                V = V + np.array(G[grp]["origin"])
                lo, hi = V[F].min(axis=1), V[F].max(axis=1)
                F = F[np.all((hi >= region[0]) & (lo <= region[1]), axis=1)]
            if len(F):
                out.append(sample(V, F, region=region))
        return np.concatenate(out) if out else np.zeros((0, 3))

    o_head, o_chg, o_car = (np.array(G[g]["origin"]) for g in ("head", "changer", "carousel"))
    head = np.concatenate([group_pts("head"), group_pts("spindle")])    # head frame (origin = nose at Z top)
    head = head[(head[:, 2] < 0.80) & (head[:, 0] + o_head[0] < 0.15)]   # lower part, changer side: the rest is far
    chg = group_pts("changer")
    car = group_pts("carousel")
    lo = np.minimum((chg + o_chg).min(0), (car + o_car).min(0)) - [REGION_PAD_M, REGION_PAD_M, 0.2]
    hi = np.maximum((chg + o_chg).max(0), (car + o_car).max(0)) + [T + REGION_PAD_M, REGION_PAD_M, REGION_PAD_M]
    static = group_pts("static", (lo, hi))        # fixed parts around the swept changer and its hanging tools
    tools = {n: tool_points(n) for n in range(1, len(TOOLS) + 1)}
    print("points: head %d, changer %d, carousel %d, static (near) %d" % (len(head), len(chg), len(car), len(static)))

    def P_head(z):
        return head + o_head + [0, 0, z]

    def P_changer(s, rot=0.0):
        return np.concatenate([chg + o_chg + [s, 0, 0], rotz(car, rot) + o_car + [s, 0, 0]])

    def P_pocket_tools(s, angles):
        """All tool types at each station-relative angle (deg)."""
        out = []
        for a in angles:
            c = o_car + [s + R * math.cos(math.radians(a)), R * math.sin(math.radians(a)), 0]
            out += [tools[n] + c for n in tools]
        return np.concatenate(out)

    def P_spindle_tools(z):
        return np.concatenate([tools[n] + o_head + [0, 0, z - g2n] for n in tools])

    trees = {}

    def dmin(A, B, key=None):
        """Nearest distance (mm) between point sets A and B, and where (point of A, mm, world)."""
        t = trees.get(key) if key else None
        if t is None:
            t = cKDTree(B)
            if key:
                trees[key] = t
        d, _ = t.query(A, k=1, distance_upper_bound=0.2, workers=-1)
        if not np.isfinite(d).any():
            return None
        i = int(np.argmin(d))
        return dict(mm=round(float(d[i]) * 1000, 1), at=[round(float(v) * 1000) for v in A[i]])

    def P_shroud(s):
        return chg + o_chg + [s, 0, 0]

    def P_plate(s, rot=0.0):
        return rotz(car, rot) + o_car + [s, 0, 0]


    def penetration(s, angles, z):
        """Deepest head point inside the cylinder envelope of any tool type at these pocket angles (mm, 0 = none)."""
        H = P_head(z)
        worst = 0.0
        for a in angles:
            c = o_car + [s + R * math.cos(math.radians(a)), R * math.sin(math.radians(a)), 0]
            rel = H - c
            rho = np.hypot(rel[:, 0], rel[:, 1])
            near = (rho < 0.035) & (rel[:, 2] > -0.2) & (rel[:, 2] < 0.11)
            rho, zz = rho[near], rel[near, 2]
            for n in tools:
                for kind, r, z0, z1, _ in tool_parts(n):
                    if kind == "sphere":
                        z0, z1 = z0 - r, z0 + r
                    m = (zz >= z0) & (zz <= z1) & (rho < r)
                    if m.any():
                        worst = max(worst, float((r - rho[m]).max()))
        return round(worst * 1000, 1)
    ring = [k * pitch for k in range(POCKETS)]
    rep = {}
    rows = []
    for s in np.linspace(0, T, 11):
        rows.append(dict(s=round(float(s), 4), Z=ZC,
                         carriage_vs_head=dmin(P_shroud(s), P_head(ZC), "hZC"),
                         carousel_vs_head=dmin(P_plate(s), P_head(ZC), "hZC"),
                         tools_pm1_vs_head=dmin(P_pocket_tools(s, [-pitch, pitch]), P_head(ZC), "hZC"),
                         tools_pm1_penetration=penetration(s, [-pitch, pitch], ZC),
                         tools_pm2_vs_head=dmin(P_pocket_tools(s, [-2 * pitch, 2 * pitch]), P_head(ZC), "hZC"),
                         tools_pm2_penetration=penetration(s, [-2 * pitch, 2 * pitch], ZC),
                         carriage_vs_spindle_tool=dmin(P_shroud(s), P_spindle_tools(ZC), "stZC"),
                         carousel_vs_spindle_tool=dmin(P_plate(s), P_spindle_tools(ZC), "stZC")))
    rep["A_slide_in_at_clamp_height"] = rows
    print("A done", flush=True)
    rows = []
    for z in np.linspace(ZC, 0.0, 8):
        rows.append(dict(s=T, Z=round(float(z), 4),
                         head_vs_carriage=dmin(P_head(z), P_shroud(T), "shT"),
                         head_vs_carousel=dmin(P_head(z), P_plate(T), "plT"),
                         head_vs_tools_pm1=dmin(P_head(z), P_pocket_tools(T, [-pitch, pitch]), "pm1T"),
                         head_vs_tools_pm2=dmin(P_head(z), P_pocket_tools(T, [-2 * pitch, 2 * pitch]), "pm2T"),
                         head_vs_station_tool=dmin(P_head(z), P_pocket_tools(T, [0.0]), "stT")))
    rep["B_head_up_down_changer_in"] = rows
    print("B done", flush=True)
    rep["C_carousel_turns_head_up"] = dict(
        s=T, Z=0.0, tools_vs_head=dmin(P_pocket_tools(T, list(np.arange(-36.0, 36.1, 3.0))), P_head(0.0), "h0"),
        carousel_vs_head=min((dmin(P_plate(T, r), P_head(0.0), "h0") for r in np.arange(0, pitch, 3.0)),
                             key=lambda v: v["mm"] if v else 1e9))
    rows = []
    for s in np.linspace(0, T, 6):
        rows.append(dict(s=round(float(s), 4),
                         carriage_vs_static=dmin(P_shroud(s), static, "static"),
                         carousel_vs_static=dmin(P_plate(s), static, "static"),
                         tools_vs_static=dmin(P_pocket_tools(s, ring), static, "static")))
    rep["D_changer_vs_fixed_parts"] = rows
    print("C, D done", flush=True)
    rows = []
    for z in np.arange(0.0, -0.5081, -0.02):
        rows.append(dict(s=0.0, Z=round(float(z), 3),
                         head_and_spindle_tool_vs_changer=dmin(np.concatenate([P_head(z), P_spindle_tools(z)]),
                                                               np.concatenate([P_changer(0.0),
                                                                               P_pocket_tools(0.0, ring)]), "out")))
    worst = min(rows, key=lambda r: r["head_and_spindle_tool_vs_changer"]["mm"]
                if r["head_and_spindle_tool_vs_changer"] else 1e9)
    rep["E_machining_changer_out"] = worst
    rep["F_tool_in_pocket_vs_carousel_plate"] = dmin(P_pocket_tools(0.0, [0.0]), P_plate(0.0))
    rep["note"] = ("mm, nearest sampled points (spacing %.0f mm): ~0 = contact. B.head_vs_station_tool: the taper "
                   "of the released tool is inside the spindle bore at the bottom of the stroke (expected small values "
                   "if the nose is modelled solid)" % (STEP_M * 1000))
    print(json.dumps(rep, indent=1))

    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        poses = [("retracted, Z top", 0.0, 0.0, 0.0), ("carriage in, Z clamp (transfer)", T, ZC, 0.0),
                 ("carriage in, Z top, carousel +45 deg", T, 0.0, 45.0), ("carriage out, Z -0.3", 0.0, -0.3, 0.0)]
        fig, axs = plt.subplots(2, 4, figsize=(28, 13))
        for j, (title, s, z, rot) in enumerate(poses):
            parts = [("static", static, (0.7, 0.7, 0.7), 0.02), ("head", P_head(z), (0.2, 0.2, 0.2), 0.05),
                     ("changer", P_changer(s, rot), (0.9, 0.4, 0.0), 0.15),
                     ("tools", np.concatenate([rotz(tools[(k % len(TOOLS)) + 1] + [R, 0, 0], k * pitch + rot)
                                               + o_car + [s, 0, 0] for k in range(1, 8)]), (0.1, 0.3, 0.9), 0.3),
                     ("spindle tool", tools[1] + o_head + [0, 0, z - g2n], (0.8, 0.1, 0.1), 0.3)]
            for i, (a, b) in enumerate(((0, 2), (0, 1))):
                ax = axs[i][j]
                for name, P, col, al in parts:
                    sel = P[:: max(1, len(P) // 60000)]
                    ax.scatter(sel[:, a], sel[:, b], s=0.3, color=col, alpha=al, label=name)
                ax.set_aspect("equal")
                ax.set_title("%s - %s view" % (title, "front" if i == 0 else "top"))
                ax.set_xlim(-1.0, 0.35)
                ax.set_ylim((1.1, 1.9) if i == 0 else (-0.7, 0.4))
                ax.grid(True)
        axs[0][0].legend(markerscale=20, loc="lower left")
        plt.tight_layout()
        os.makedirs(args.png, exist_ok=True)
        fn = os.path.join(args.png, "vf1_atc_poses.png")
        plt.savefig(fn, dpi=50)
        print("image:", fn)


if __name__ == "__main__":
    main()
