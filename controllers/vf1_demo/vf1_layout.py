"""Layout of the equipment on the Haas VF-1 table (worlds/haas_vf1_demo.wbt, HaasVF1.tableSlot) and simple
analytic collision shapes, shared by controllers/vf1_demo/vf1_demo.py and tools/vf1_table_check.py (no Webots import).

Frames (metres)
  table frame = HaasVF1.tableSlot frame: origin on the table top, centred on the top face; x = table length (machine
  X), y towards the column, z up. T-slots (16 mm) along x at y = -0.125, 0, +0.125.
  machine frame = HaasVF1 frame (floor, centre of the levelling pads).
Shapes: ("box", name, lo, hi) axis-aligned boxes and ("cyl", name, (cx, cy), r, z0, z1) vertical cylinders, table
frame. Keep the numbers in sync with the world file (the controller checks the vice / OTS translations at start).
"""
import json
import math
import os

# ---- machine (from protos/meshes/haas_vf1/haas_vf1.json and the table mesh; see tools/haas_vf1_to_meshes.py)
SPINDLE_XY = (-0.0004, -0.2893)          # spindle axis, machine frame
TABLE_TOP_XY0 = (-0.0015, -0.2968)       # table top centre at X = Y = 0 (saddle + table origins + top-face offset)
TABLE_TOP = 0.8655                       # table top height
NOSE_Z_TOP = 1.4788                      # spindle nose (lowest point) at Z = 0
GROOVE_TO_NOSE = 0.011
X_RANGE, Y_RANGE, Z_RANGE = (-0.254, 0.254), (-0.203, 0.203), (-0.508, 0.0)

# ---- equipment (table frame)
PLATE = 0.020                            # base plate under each vice (T-slot bolts in the centre slot)
PART = (0.060, 0.050, 0.040)             # raw part: x (table) x y (clamped, = clampWidth) x z, aluminium
VISES = [dict(name="vise 1", prefix="vise 1 ", x=-0.150, y=0.0, size="100"),
         dict(name="vise 2", prefix="vise 2 ", x=0.050, y=0.0, size="100")]
TOP_JAW = dict(height=0.030, thickness=0.020, support_h=0.008, support_w=0.004, clamp_at_ratio=0.6)
OTS = dict(x=0.225, y=0.125, disc_top=0.098, disc_r=0.0125, body_r=0.035, body_top=0.068)
VISE_YAW = math.pi / 2                   # vice x (jaw stroke) along table +y

_KSP3 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "protos", "meshes", "schunk_ksp3",
                     "schunk_ksp3.json")


def ksp3(size):
    with open(_KSP3) as fh:
        return json.load(fh)["sizes"][size]


def spindle_over(xt, yt):
    """Machine X, Y putting the spindle axis over table point (xt, yt)."""
    return SPINDLE_XY[0] - TABLE_TOP_XY0[0] - xt, SPINDLE_XY[1] - TABLE_TOP_XY0[1] - yt


def spindle_in_table(X, Y):
    """Table point under the spindle axis for machine X, Y."""
    return SPINDLE_XY[0] - TABLE_TOP_XY0[0] - X, SPINDLE_XY[1] - TABLE_TOP_XY0[1] - Y


def vise_data(v):
    """Derived numbers of vice v (table frame): stroke, clamp position, part box, heights."""
    k = ksp3(v["size"])
    kp = k["key_positions"]
    st = kp["stroke_per_jaw"]
    seat = PLATE + kp["top_jaw_seat_z"]
    part_z0 = seat + TOP_JAW["support_h"]
    return dict(stroke=st, clamp_at=TOP_JAW["clamp_at_ratio"] * st, seat=seat, part_z0=part_z0,
                part_top=part_z0 + PART[2], jaw_top=seat + TOP_JAW["height"], kp=kp, groups=k["groups"])


def _vbox(v, lo, hi):
    """Vice-frame box -> table-frame box (vice x -> table +y, vice y -> table -x)."""
    return ((v["x"] - hi[1], v["y"] + lo[0], lo[2]), (v["x"] - lo[1], v["y"] + hi[0], hi[2]))


def shapes(with_parts=True):
    out = []
    for v in VISES:
        d = vise_data(v)
        kp, g = d["kp"], d["groups"]
        L, W = kp["body_size_xy"]
        n = v["name"]
        out.append(("box", n + " plate", *_vbox(v, (-(L + 0.01) / 2, -(W + 0.08) / 2, 0), ((L + 0.01) / 2, (W + 0.08) / 2, PLATE))))
        for sy in (-1, 1):
            yb = sy * (W / 2 + 0.02)
            out.append(("box", n + " bolt", *_vbox(v, (-0.012, yb - 0.012, PLATE), (0.012, yb + 0.012, PLATE + 0.01))))
        blo, bhi = kp["body_bbox"]
        out.append(("box", n + " body", *_vbox(v, (blo[0], blo[1], PLATE), (bhi[0], bhi[1], PLATE + bhi[2]))))
        st = d["stroke"]
        for jaw, s in (("jaw_a", -1), ("jaw_b", 1)):
            lo, hi = g[jaw]["bbox_open"]
            out.append(("box", n + " base jaw", *_vbox(v, (min(lo[0], lo[0] - s * st), lo[1], PLATE + lo[2]),
                                                          (max(hi[0], hi[0] - s * st), hi[1], PLATE + hi[2]))))
            cw2 = PART[1] / 2
            xin = cw2 + d["clamp_at"] - st           # innermost face (jaw fully closed)
            xout = cw2 + d["clamp_at"] + TOP_JAW["thickness"]
            w = kp["base_jaw_width_y"]
            x0, x1 = (xin, xout) if s > 0 else (-xout, -xin)
            out.append(("box", n + " top jaw", *_vbox(v, (x0, -w / 2, d["seat"]), (x1, w / 2, d["jaw_top"]))))
        out.append(("box", n + " purge nozzle", *_vbox(v, (kp["jaw_gap_centre_x"] - 0.005, -W / 2 - 0.013, d["seat"]),
                                                         (kp["jaw_gap_centre_x"] + 0.005, -W / 2, d["seat"] + 0.011))))
        if with_parts:
            out.append(("box", n + " part", (v["x"] - PART[0] / 2, v["y"] - PART[1] / 2, d["part_z0"]),
                        (v["x"] + PART[0] / 2, v["y"] + PART[1] / 2, d["part_top"])))
    o = OTS
    out.append(("cyl", "OTS body", (o["x"], o["y"]), o["body_r"], 0.0, o["body_top"]))
    out.append(("cyl", "OTS cap", (o["x"], o["y"]), 0.020, o["body_top"], 0.073))
    out.append(("cyl", "OTS stem", (o["x"], o["y"]), 0.005, 0.073, o["disc_top"] - 0.005))
    out.append(("cyl", "OTS disc", (o["x"], o["y"]), o["disc_r"], o["disc_top"] - 0.005, o["disc_top"]))
    return out


# ---- distances between vertical cylinders and boxes (all axis aligned)
def _rect_dist(p, lo, hi):
    dx = max(lo[0] - p[0], 0.0, p[0] - hi[0])
    dy = max(lo[1] - p[1], 0.0, p[1] - hi[1])
    return math.hypot(dx, dy)


def _zgap(a0, a1, b0, b1):
    return max(b0 - a1, a0 - b1, 0.0)


def dist(a, b):
    """Distance between two shapes (0 = contact or overlap)."""
    if a[0] == "box" and b[0] == "cyl":
        a, b = b, a
    if a[0] == "cyl" and b[0] == "cyl":
        dh = max(0.0, math.hypot(a[2][0] - b[2][0], a[2][1] - b[2][1]) - a[3] - b[3])
        return math.hypot(dh, _zgap(a[4], a[5], b[4], b[5]))
    if a[0] == "cyl":                                   # cyl - box
        dh = max(0.0, _rect_dist(a[2], b[2], b[3]) - a[3])
        return math.hypot(dh, _zgap(a[4], a[5], b[2][2], b[3][2]))
    d = [max(a[2][i] - b[3][i], b[2][i] - a[3][i], 0.0) for i in range(3)]
    return math.sqrt(sum(x * x for x in d))


# ---- head + tool envelope (table frame) for machine X, Y, Z
HEAD_UNDERSIDE = 0.030                  # head casting bottom above the nose (spindle housing r 0.066 below it)
HEAD_BOX_XY = ((-0.205, -0.184), (0.205, 0.46))     # head casting footprint relative to the spindle axis
AIR_NOZZLE = ((0.068, -0.068), 0.006, 0.010)        # air gun nozzle: xy rel. spindle, radius, bottom above nose


def head_shapes(X, Y, Z, tool_cyls, air_gun=True):
    """tool_cyls: [(r, z0, z1)] of the tool, z relative to the V-groove (as TOOLS / PROBE of the generator)."""
    cx, cy = spindle_in_table(X, Y)
    nose = NOSE_Z_TOP + Z - TABLE_TOP
    groove = nose - GROOVE_TO_NOSE
    out = [("cyl", "spindle nose", (cx, cy), 0.066, nose, nose + HEAD_UNDERSIDE),
           ("box", "head", (cx + HEAD_BOX_XY[0][0], cy + HEAD_BOX_XY[0][1], nose + HEAD_UNDERSIDE),
            (cx + HEAD_BOX_XY[1][0], cy + HEAD_BOX_XY[1][1], nose + 0.6))]
    if air_gun:
        out.append(("cyl", "air nozzle", (cx + AIR_NOZZLE[0][0], cy + AIR_NOZZLE[0][1]), AIR_NOZZLE[1],
                    nose + AIR_NOZZLE[2], nose + HEAD_UNDERSIDE))
    for r, z0, z1 in tool_cyls:
        if z1 <= 0.0:                                   # parts below the V-flange (the rest is inside the spindle)
            out.append(("cyl", "tool", (cx, cy), r, groove + z0, groove + z1))
    return out
