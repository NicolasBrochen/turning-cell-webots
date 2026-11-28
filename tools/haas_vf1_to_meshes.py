#!/usr/bin/env python3
"""Haas VF-1 STEP -> grouped OBJ meshes (one rigid body per group, one file per group x colour), for Webots + three.js.

    uv run --no-project --python 3.12 --with cadquery-ocp python -I tools/haas_vf1_to_meshes.py \
        <VF-1_STEP_11_2021.STEP> protos/meshes/haas_vf1 --proto protos/HaasVF1.proto
    (PROTO only, from the existing meshes and JSON:  ... tools/haas_vf1_to_meshes.py - protos/meshes/haas_vf1 \
        --proto protos/HaasVF1.proto --from-json)

Same method as tools/haas_to_meshes.py (Haas ST-20), whose STEP helpers are reused: reads the official Haas STEP
(XCAF: names, colours, placements), keeps one option of each alternative (20-pocket umbrella tool changer, 55 gal
coolant tank; drops the 30-pocket SMTC, the 95 gal tank and the chip auger), drops tiny parts, tessellates coarsely,
maps everything into the machine frame below and writes, per group, OBJ files whose vertices are expressed in the
group's own frame (origin = pivot, axes parallel to the machine frame), plus haas_vf1.json describing files, pivots,
joint axes/ranges and key positions. With --proto, also writes the Webots PROTO that assembles the groups.

Frames
  STEP (after OCP read, mm; written in m): x = machine X (table length, towards the operator's right), y = up,
  z = towards the operator (front of the machine, pendant side).
  machine frame (m, z up):  x_m = x_h - cx,  y_m = -(z_h - cz),  z_m = y_h - floor   (rotation about x by +90 deg:
  proper rotation, triangle winding preserved).  (cx, cz) = centre of the levelling-pad footprint, floor = underside
  of the pads.  The front of the machine (doors, pendant) faces -y_m; +y_m points to the back (column).

Groups (CAD pose = X at its -x end, Y with the table fully forward, Z at the top):
  static; saddle (Y slide, prismatic along y_m); table (X slide on the saddle, prismatic along x_m); head (Z slide,
  prismatic along z_m); spindle (revolute about the spindle axis, child of head); door_left / door_right (sliding front
  doors, prismatic along -x_m / +x_m).  The two doors are fused with the front sheet metal into a single solid in the
  STEP: they are cut out with boolean operations (slab boxes parallel to the 5 deg inclined door skin, plus the roof
  return and the handles), and their sliding travel is measured by sweeping the left door against the enclosure.
  changer (umbrella tool changer carriage, prismatic along +x_m towards the spindle) and carousel (pocket plate + 20
  clips, revolute about the vertical carousel axis, child of changer). The changer body 37-2104 is also one fused
  solid: the carriage (shroud below its top plate + hub box above the carousel) is cut out of it, the top cover, guide
  plate and column bracket stay static. Carousel axis, pocket radius / angles, slide travel (station pocket onto the
  spindle axis) and the tool clamping height are measured on the CAD; the shroud is also cut where head parts sweep
  through it when the carriage slides in at the clamping height (see key_positions.tool_change). With --proto, the
  PROTO also gets procedural CAT40 tools (TOOLS) in the pockets and in the spindle. Clearances of the tool change
  sequence: tools/vf1_atc_check.py.
  Optional equipment in the PROTO (defaults: none): tableSlot (MFNode on the table top, centred; T-slots at
  y = -125 / 0 / +125 mm), probeTool (a tool number drawn as a procedural Renishaw OMP40-2, PROBE), probeReceiver
  (procedural OMM-2 under the roof, OMM_*), airGun (Haas auto air gun nozzle under the head, AIR_*); the live fields
  probeLedColor and airBlastTransparency are IS-linked only (never read by the template).
  The rear electrical cabinet door is modelled open at 90 deg, fused into an enclosure solid: it is cut out and
  turned closed about its vertical hinge (static, REAR_DOOR_*).
Files with more than SHADOW_MAX_TRIANGLES triangles are split (Webots only casts shadows of smaller meshes).
Deterministic: same input -> byte-identical output.
"""
import argparse
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # python -I drops the script directory

from OCP.BRepAdaptor import BRepAdaptor_Surface                  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse   # noqa: E402
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid, BRepBuilderAPI_Transform   # noqa: E402
from OCP.BRepGProp import BRepGProp                                # noqa: E402
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox                    # noqa: E402
from OCP.BRepTools import BRepTools                                # noqa: E402
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane            # noqa: E402
from OCP.GProp import GProp_GProps                                 # noqa: E402
from OCP.TopAbs import TopAbs_SHELL, TopAbs_WIRE                    # noqa: E402
from OCP.TopoDS import TopoDS     # noqa: E402
from OCP.TopExp import TopExp_Explorer                             # noqa: E402
from OCP.BRepExtrema import BRepExtrema_DistShapeShape             # noqa: E402
from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec   # noqa: E402

from haas_to_meshes import (bbox_mm, collect_items, face_area, faces_of, num, read_step, rnd,   # noqa: E402
                            tessellate, _unit)

# ----------------------------------------------------------------------------------------------- configuration
DEFLECTION_MM = 2.0                            # default linear deflection (mm, STEP units after read)
ANGLE_RAD = 0.5                                # default angular deflection
MIN_DIAG_MM = 25.0                             # drop solids/shells whose bbox diagonal is smaller
DIGITS = 5                                     # vertex precision in the OBJ (m)
FILE_PREFIX = "vf1_"                           # unique basenames: the HTML5 export flattens mesh URLs to meshes/<name>
SHADOW_MAX_TRIANGLES = 21845                   # Webots casts no shadow for meshes above this (65535 / 3)

# real VF-1 travels (m); the CAD pose sits at one end of each axis (checked in main)
TRAVEL_X, TRAVEL_Y, TRAVEL_Z = 0.508, 0.406, 0.508
DOOR_CLEARANCE_MM = 5.0                        # door travel = first collision - this
DOOR_SWEEP_STEP_MM = 25.0
DOOR_COLLISION_MM3 = 2000.0                    # interpenetration (above the CAD-pose baseline) counted as a collision

PALETTE = {
    "white":  (0.86, 0.87, 0.87),              # Haas sheet metal (enclosure, doors)
    "grey":   (0.50, 0.52, 0.54),              # castings, tool changer, way covers
    "steel":  (0.70, 0.71, 0.73),              # machined parts: table, spindle nose
    "dark":   (0.22, 0.23, 0.24),              # saddle, head cover, coolant tank, pendant body
    "black":  (0.05, 0.05, 0.05),              # pads, trims, screen
    "window": (0.72, 0.82, 0.86),              # door polycarbonate (generated, transparency 0.6)
}
TRANSPARENCY = {"window": 0.6}
DEFAULT_XCAF = [(0.591, 0.638, 0.855), (0.591, 0.638, 0.847), (0.597, 0.644, 0.855), (1.0, 1.0, 1.0)]
FACE_COLOUR_RE = r"^Enclosure_VF-1_11_2021 / "   # only these items honour per-face XCAF colours (trims, pendant)

ENC_BODY_RE = r"^Enclosure_VF-1_11_2021 / Enclosure_VF-1_11_2021$"
HEAD_RE = r"^Spindle Head_VF-1"
TABLE_RE = r"^Table_VF-1"
SADDLE_RE = r"^Saddle_VF-1"
PADS_RE = r"^Pads_VF-1"
CASTINGS_RE = r"^Castings_VF-1"
PENDANT_RE = r"Pendant_Common"
CHANGER_RE = r"^UMB_20_PCKT"

# Rules: first match on the item path ("A / B / C") wins.
# (regex, group or "drop", material or None = snapped XCAF colour, deflection_mm or None, angle or None)
RULES = [
    # alternatives / options we do not keep
    (r"^SMTC_30_PCKT_Option", "drop", None, None, None),          # 30+1 side-mount tool changer (option)
    (r"^95-Gal Coolant Tank", "drop", None, None, None),          # larger tank: the 55 gal one is kept
    (r"^Chip_Auger", "drop", None, None, None),                   # chip auger (option)
    # spindle head: parts mounted behind the column (lube/air fittings) stay on the column -> static (see main)
    (HEAD_RE + r".* / 37-0282.*", "head", None, 3.0, 0.8),         # head casting, cover, spindle cartridge
    (HEAD_RE + r".* / 37-0362", "head", "dark", None, None),       # inline motor fittings on the head
    (HEAD_RE, "head", None, None, None),
    (TABLE_RE, "table", "steel", 3.0, 0.8),
    (SADDLE_RE, "saddle", "dark", None, None),
    (PADS_RE, "static", "black", 8.0, 1.2),
    (CASTINGS_RE, "static", "grey", 6.0, 1.0),           # mostly hidden
    (CHANGER_RE + r".* / 37-2104_VF-1_DF_UMB_1", "static", None, 4.0, 1.0),
    (r"^55-Gal Coolant Tank", "static", "dark", 3.0, 0.8),
    (r"^Enclosure_VF-1_11_2021 / .*" + PENDANT_RE, "static", "dark", 3.0, 1.0),
    (r"^Enclosure_VF-1_11_2021", "static", "white", 3.0, 0.7),
    (r".*", "static", "grey", None, None),
]
# head: XCAF colours of 37-0282 are arbitrary (red / green / cyan bodies) -> by colour of the item
HEAD_COLOURS = [((0.799, 0.165, 0.002), "dark"), ((0.216, 1.0, 0.216), "dark"), ((0.141, 0.141, 0.141), "black"),
                ((0.474, 0.61, 0.651), "steel"), ((1.0, 1.0, 0.0), "steel"), ((0.521, 0.521, 0.521), "steel")]
CHANGER_COLOURS = [((0.376, 0.342, 0.296), "dark"), ((0.515, 0.515, 0.515), "grey"), ((1.0, 0.381, 0.136), "grey")]
FILE_NAMES = {}

SPINDLE_DEFL = (3.0, 1.0)                      # spindle: the drawbar is hidden in the nose
SPINDLE_R_MM = 75.0                            # spindle parts: within this radius of the spindle axis ...
SPINDLE_Y_MAX_MM = 1035.0                      # ... and reaching below this height (STEP y) = rotating nose
BEHIND_COLUMN_Z_MM = -300.0                    # head-assembly parts with z_h below this sit behind the column

# doors (STEP mm): split x between the two doors, outer x limits, slab depth behind / in front of the skin
DOOR_SPLIT_X = 7.0
DOOR_LEFT_X0, DOOR_RIGHT_X1 = -660.0, 573.5   # right door stops at its skin edge: the right front panel is a solid
DOOR_ROOF_X = (-660.0, 645.0)                 # roof returns of the doors (left / right outer x)
DOOR_SLAB_BACK, DOOR_SLAB_FRONT = 66.0, 1.5
DOOR_Y = (95.0, 1240.0)                        # slab height range at the skin (STEP y)
DOOR_ROOF = ((1140.0, 1240.0), (805.0, 1030.0))  # roof return box: y range, z range
# handle bars (in front of the skin, partly in the main shell, partly separate shells), per door: STEP mm box
DOOR_HANDLES = {"door_left": ((-60, 390, 1080), (-12, 760, 1215)), "door_right": ((26, 390, 1080), (74, 760, 1215))}
WINDOW_MARGIN_M = 0.015
WINDOW_BEHIND_M = 0.004                        # pane plane this far behind the outer skin

# rear electrical cabinet door: modelled open at 90 deg and fused into an enclosure solid -> cut out and closed (STEP mm)
REAR_DOOR_Y = (203.7, 1423.9)                  # door height range (204.2..1423.4) + 0.5 mm: no lamina left behind
REAR_DOOR_X0 = 450.5                           # inner flange plane of the open door (the cabinet back panel ends here)
REAR_DOOR_Z_HINGE = -726.0                     # hinge edge of the open door (the cabinet side wall ends here)
REAR_DOOR_Z_PANEL = -734.65                    # below this, everything with x >= REAR_DOOR_X0 - 1.5 is the door
REAR_DOOR_X_SIDE = 464.5                       # outer face of the cabinet side wall (hinge side), checked below
REAR_DOOR_GAP_MM = 1.0                         # closed: gap between the door flanges and the cabinet back

# umbrella tool changer (STEP mm). 37-2104 is one fused solid (top cover, guide plate, column bracket, hub, shroud):
# the carriage (shroud = everything below the shroud top + hub box above the carousel) is cut out of it; the carousel
# plate and its 20 pocket clips are separate solids.
CHANGER_BODY_RE = CHANGER_RE + r".* / 37-2104_VF-1_DF_UMB_1"
CHANGER_HUB_X = (-900.0, -436.0)               # hub box (moves with the carriage): x range ...
CHANGER_HUB_Z = (389.2, 700.0)                 # ... z_h range (y_m -508..-197.5: the hub's back face)
CHANGER_HUB_TOP_Y = 1229.7                     # ... from the shroud top up to the underside of the fixed top cover
CHANGER_HUB_CLEAR_MM = 3.0                     # hub skin removed (not kept static) at its top and back faces, so
                                               # that it slides 3 mm under the top cover / 5.5 mm before the block
POCKETS = 20
POCKET_ARC_R_MM = (30.0, 40.0)                 # clip / plate arcs around each pocket
KEYHOLE_R_MM = (80.0, 120.0)                   # notch of the shroud top at the change station
TOOL_GROOVE_TO_NOSE_MM = 11.0                  # CAT40: V-groove centre (clip plane) -> spindle nose face, tool clamped
CUT_MARGIN_MM = 6.0                            # clearance cut in the shroud around the head parts that sweep it
TOOL_PARK_M = 3.0                              # hidden tool copies are parked this far below their seat (PROTO)
# Renishaw OMP40-2 optical spindle probe (no CAD: procedural from the data sheet): body D40 x 50 mm on a CAT40 shank,
# IR window band (its emission = PROTO field probeLedColor), D6 ruby ball on a 50 mm stylus. Same frame as TOOLS.
PROBE = dict(name="Renishaw OMP40-2", colour=(0.8, 0.06, 0.1),
             parts=[("cyl", 0.0125, -0.020, -0.008, "holder"), ("cyl", 0.020, -0.040, -0.020, "probe"),
                    ("cyl", 0.0202, -0.052, -0.040, "window"), ("cyl", 0.020, -0.066, -0.052, "probe"),
                    ("cyl", 0.008, -0.070, -0.066, "probe"), ("cyl", 0.0022, -0.117, -0.070, "stylus"),
                    ("sphere", 0.003, -0.117, -0.117, "cutter")])
PROBE_TIP_BELOW_GROOVE = 0.120                 # ruby ball bottom
# Haas Auto Air Gun (M83 on / M84 off): nozzle under the head (head frame: origin = spindle nose, on the axis),
# aimed at a point on the spindle axis below the nose; jet = transparent cone (PROTO field airBlastTransparency)
AIR_NOZZLE_TIP = (0.068, -0.068, 0.014)
AIR_NOZZLE_PIPE_TOP = 0.066                    # pipe from the tip up to the underside of the head casting
AIR_JET_TARGET = (0.0, 0.0, -0.100)
AIR_JET_LENGTH = 0.17
# Renishaw OMM-2 optical receiver (procedural, ~ 84 x 64 x 45 mm housing) under the enclosure roof, aimed at the
# work zone (machine frame)
OMM_ROOF = (0.45, -0.45, 1.773)                # roof underside measured on the static meshes (ray cast)
OMM_DROP = 0.07                                # bracket length
OMM_AIM = (0.0, -0.29, 0.95)                   # point it looks at (table zone)

# CAT40 tool holders (machine frame, m; z = 0 at the V-groove centre, + towards the spindle). Common top (knob, taper,
# V-flange) + per-tool body: (kind, radius, z0, z1, material) with kind cyl | cone (apex down at z0) | sphere (at z0);
# material "holder" (steel), "body" (dark) or "cutter" (tool colour). Tool n lives in pocket n (fixed-pocket umbrella)
# and uses TOOLS[(n - 1) % len(TOOLS)].
TOOL_TOP = [("cyl", 0.0105, 0.076, 0.104, "holder"), ("cyl", 0.016, 0.042, 0.076, "holder"),
            ("cyl", 0.0205, 0.008, 0.042, "holder"), ("cyl", 0.03175, 0.003, 0.008, "holder"),
            ("cyl", 0.0275, -0.003, 0.003, "holder"), ("cyl", 0.03175, -0.008, -0.003, "holder")]
TOOLS = [
    dict(name="end mill D12", colour=(0.33, 0.27, 0.45),
         parts=[("cyl", 0.020, -0.058, -0.008, "holder"), ("cyl", 0.006, -0.118, -0.058, "cutter")]),
    dict(name="drill D8.5", colour=(0.85, 0.68, 0.22),
         parts=[("cyl", 0.025, -0.050, -0.008, "holder"), ("cyl", 0.015, -0.060, -0.050, "holder"),
                ("cyl", 0.00425, -0.160, -0.060, "cutter"), ("cone", 0.00425, -0.165, -0.160, "cutter")]),
    dict(name="face mill D63", colour=(0.85, 0.68, 0.22),
         parts=[("cyl", 0.020, -0.040, -0.008, "holder"), ("cyl", 0.0315, -0.082, -0.040, "body"),
                ("cyl", 0.0318, -0.090, -0.082, "cutter")]),
    dict(name="end mill D20", colour=(0.12, 0.12, 0.14),
         parts=[("cyl", 0.022, -0.062, -0.008, "holder"), ("cyl", 0.010, -0.107, -0.062, "cutter")]),
    dict(name="spot drill D10", colour=(0.75, 0.76, 0.78),
         parts=[("cyl", 0.018, -0.070, -0.008, "holder"), ("cyl", 0.005, -0.095, -0.070, "cutter"),
                ("cone", 0.005, -0.100, -0.095, "cutter")]),
    dict(name="tap M10", colour=(0.62, 0.45, 0.20),
         parts=[("cyl", 0.021, -0.075, -0.008, "body"), ("cyl", 0.005, -0.125, -0.075, "cutter"),
                ("cone", 0.005, -0.128, -0.125, "cutter")]),
    dict(name="ball mill D10", colour=(0.25, 0.35, 0.70),
         parts=[("cyl", 0.016, -0.065, -0.008, "holder"), ("cyl", 0.005, -0.110, -0.065, "cutter"),
                ("sphere", 0.005, -0.110, -0.110, "cutter")]),
    dict(name="chamfer mill D16", colour=(0.85, 0.68, 0.22),
         parts=[("cyl", 0.020, -0.055, -0.008, "holder"), ("cyl", 0.005, -0.080, -0.055, "cutter"),
                ("cone", 0.008, -0.088, -0.080, "cutter")]),
]


# ----------------------------------------------------------------------------------------------- geometry helpers
def volume(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    return g.Mass()


def translated(shape, dx, dy=0.0, dz=0.0):
    tr = gp_Trsf()
    tr.SetTranslation(gp_Vec(dx, dy, dz))
    return BRepBuilderAPI_Transform(shape, tr, True).Shape()


def abox(lo, hi):
    return BRepPrimAPI_MakeBox(gp_Pnt(*lo), gp_Pnt(*hi)).Shape()


def split_shells(solid):
    """Solid whose extra shells are separate closed bodies (door handles), not cavities -> (main solid, [others])."""
    shells = []
    e = TopExp_Explorer(solid, TopAbs_SHELL)
    while e.More():
        shells.append(TopoDS.Shell(e.Current()))
        e.Next()
    shells.sort(key=lambda sh: -sum(1 for _ in faces_of(sh)))
    solids = [BRepBuilderAPI_MakeSolid(sh).Solid() for sh in shells]
    return solids[0], solids[1:]


def door_skin(shape):
    """Largest planar face of the front solid with an inclined +z normal that spans the left door -> (n, w)."""
    best = None
    for f in faces_of(shape):
        a = BRepAdaptor_Surface(f)
        if a.GetType() != GeomAbs_Plane:
            continue
        d = a.Plane().Axis().Direction()
        lo, hi = bbox_mm(f)
        if d.Z() > 0.9 and abs(d.Y()) > 0.05 and -640 < lo[0] < -600 and hi[0] < DOOR_SPLIT_X:
            area = face_area(f)
            if best is None or area > best[0]:
                p = a.Plane().Location()
                best = (area, (d.X(), d.Y(), d.Z()), d.X() * p.X() + d.Y() * p.Y() + d.Z() * p.Z())
    return best[1], best[2]


def door_volume(x0, x1, roof_x, handle, n, w_skin):
    """Slab parallel to the inclined door skin + roof return box + handle box (STEP mm)."""
    yv = (0.0, n[2], -n[1])                                  # in-plane 'up' direction, normal x X
    def v_of(y, z):
        return y * yv[1] + z * yv[2]
    v0, v1 = v_of(DOOR_Y[0], 1150.0), v_of(DOOR_Y[1], 1050.0)
    w0, w1 = w_skin - DOOR_SLAB_BACK, w_skin + DOOR_SLAB_FRONT
    p = gp_Pnt(x0 + v0 * yv[0] + w0 * n[0], v0 * yv[1] + w0 * n[1], v0 * yv[2] + w0 * n[2])
    slab = BRepPrimAPI_MakeBox(gp_Ax2(p, gp_Dir(*n), gp_Dir(1, 0, 0)), x1 - x0, v1 - v0, w1 - w0).Shape()
    roof = abox((roof_x[0], DOOR_ROOF[0][0], DOOR_ROOF[1][0]), (roof_x[1], DOOR_ROOF[0][1], DOOR_ROOF[1][1]))
    return BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(slab, roof).Shape(), abox(*handle)).Shape()


class ColourMap:
    """Face colours of a solid carried through boolean operations (history of each coloured face)."""
    def __init__(self, shape, fcols):
        self.src = [(f, c) for f, c in zip(faces_of(shape), fcols) if c]
        self.ops = []

    def add_op(self, op):
        self.ops.append(op)

    def colours(self, result):
        table = {}
        for f, c in self.src:
            imgs = [f]
            for op in self.ops:
                lst = op.Modified(f)
                imgs += list(lst)
            for g in imgs:
                table.setdefault(hash(g), []).append((g, c))
        out = []
        for f in faces_of(result):
            c = None
            for g, cc in table.get(hash(f), []):
                if g.IsSame(f):
                    c = cc
                    break
            out.append(c)
        return out


def window_pane(door, n, w_skin):
    """Largest inner wire of the outer door skin = window cut-out -> pane corners (STEP m), both sides."""
    best = None
    for f in faces_of(door):
        a = BRepAdaptor_Surface(f)
        if a.GetType() != GeomAbs_Plane:
            continue
        d = a.Plane().Axis().Direction()
        if d.X() * n[0] + d.Y() * n[1] + d.Z() * n[2] < 0.999:
            continue
        outer = BRepTools.OuterWire_s(f)
        e = TopExp_Explorer(f, TopAbs_WIRE)
        while e.More():
            if not e.Current().IsSame(outer):
                lo, hi = bbox_mm(e.Current())
                key = (hi[0] - lo[0]) * (hi[1] - lo[1])
                if best is None or key > best[0]:
                    best = (key, lo, hi)
            e.Next()
    if best is None:
        return None
    _, lo, hi = best
    w = w_skin - WINDOW_BEHIND_M * 1000.0
    x0, x1 = lo[0] - WINDOW_MARGIN_M * 1000, hi[0] + WINDOW_MARGIN_M * 1000
    y0, y1 = lo[1] - WINDOW_MARGIN_M * 1000, hi[1] + WINDOW_MARGIN_M * 1000
    def p(x, y):
        return (x / 1000.0, y / 1000.0, (w - n[0] * x - n[1] * y) / n[2] / 1000.0)
    return [p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)]


def vertical_cylinders(shape):
    """Cylindrical faces whose axis is parallel to y_h: [(radius, x, z, y_min)] in mm."""
    out = []
    for f in faces_of(shape):
        a = BRepAdaptor_Surface(f)
        if a.GetType() == GeomAbs_Cylinder:
            ax = a.Cylinder().Axis()
            if abs(ax.Direction().Y()) > 0.999:
                p = ax.Location()
                out.append((a.Cylinder().Radius(), p.X(), p.Z(), bbox_mm(f)[0][1]))
    return out


def tool_parts(n):
    """All primitives of tool n (machine m, z = 0 at the V-groove)."""
    return TOOL_TOP + TOOLS[(n - 1) % len(TOOLS)]["parts"]


def box_gap(a, b):
    d = [max(a[0][i] - b[1][i], b[0][i] - a[1][i], 0.0) for i in range(3)]
    return math.sqrt(sum(v * v for v in d))


def close_rear_door(shape, face_cols):
    """Rear cabinet door, open at 90 deg in the STEP: cut it out of the enclosure solid, find the hinge that brings it
    flush with the cabinet (hinge edge on the cabinet side wall, flanges REAR_DOOR_GAP_MM off the back panel) and
    rotate it closed. Returns the parts with their face colours and the measured hinge / gap."""
    big = 1.0e4
    vol = BRepAlgoAPI_Fuse(abox((REAR_DOOR_X0 - 1.5, REAR_DOOR_Y[0], -big), (big, REAR_DOOR_Y[1], REAR_DOOR_Z_PANEL)),
                           abox((REAR_DOOR_X0, REAR_DOOR_Y[0], REAR_DOOR_Z_PANEL),
                                (big, REAR_DOOR_Y[1], REAR_DOOR_Z_HINGE))).Shape()
    op_d, op_r = BRepAlgoAPI_Common(shape, vol), BRepAlgoAPI_Cut(shape, vol)
    door, rest = op_d.Shape(), op_r.Shape()
    cm_d, cm_r = ColourMap(shape, face_cols), ColourMap(shape, face_cols)
    cm_d.add_op(op_d)
    cm_r.add_op(op_r)
    door_cols, rest_cols = cm_d.colours(door), cm_r.colours(rest)
    dlo, dhi = bbox_mm(door)
    assert dhi[2] - dlo[2] > 800 and dhi[0] - dlo[0] < 60, (dlo, dhi)       # a tall thin panel lying along z
    width = dhi[2] - dlo[2]
    # cabinet seen from the back, over the closed door footprint: side wall (x max) and outermost back (z min)
    side = [f for f in faces_of(rest) if BRepAdaptor_Surface(f).GetType() == GeomAbs_Plane
            and abs(bbox_mm(f)[0][0] - REAR_DOOR_X_SIDE) < 0.05 and abs(bbox_mm(f)[1][0] - REAR_DOOR_X_SIDE) < 0.05
            and BRepAdaptor_Surface(f).Plane().Axis().Direction().X() > 0.999]
    assert side, "cabinet side wall face not found"
    back = BRepAlgoAPI_Common(rest, abox((REAR_DOOR_X0 - width, REAR_DOOR_Y[0], REAR_DOOR_Z_HINGE - 100),
                                         (REAR_DOOR_X0, REAR_DOOR_Y[1], REAR_DOOR_Z_HINGE + 30))).Shape()
    x_side, z_back = REAR_DOOR_X_SIDE, bbox_mm(back)[0][2]
    # rotation +90 deg about +y through P maps (x, z) -> (Px + z - Pz, Pz - x + Px): hinge edge (z = dhi) -> x_side,
    # inner flanges (x = dlo) -> z_back - gap
    s1, s2 = x_side - dhi[2], z_back - REAR_DOOR_GAP_MM + dlo[0]
    px, pz = (s1 + s2) / 2, (s2 - s1) / 2
    tr = gp_Trsf()
    tr.SetRotation(gp_Ax1(gp_Pnt(px, 0.0, pz), gp_Dir(0, 1, 0)), math.radians(90.0))
    closed = BRepBuilderAPI_Transform(door, tr, True).Shape()
    d = BRepExtrema_DistShapeShape(closed, rest)
    d.Perform()
    overlap = volume(BRepAlgoAPI_Common(closed, rest).Shape())
    clo, chi = bbox_mm(closed)
    assert overlap < 1.0 and d.Value() > 0.5, (overlap, d.Value(), px, pz, z_back)

    return dict(door=closed, door_cols=door_cols, rest=rest, rest_cols=rest_cols, hinge=(px, pz), width=width,
                gap=d.Value(), overlap=overlap, x_side=x_side, z_back=z_back, open_bbox=(dlo, dhi),
                closed_bbox=(clo, chi), door_volume=volume(door), rest_volume=volume(rest), volume=volume(shape))


def changer_setup(items, head_items, axis_x, axis_z, nose_y):
    """Umbrella tool changer: carousel axis, pockets, slide travel, tool-change height, carriage split (STEP mm)."""
    chg = [it for it in items if re.search(CHANGER_BODY_RE, it["path"])]
    body = max(chg, key=lambda it: sum(1 for _ in faces_of(it["shape"])))
    big = body["shape"]
    big_cyl = []
    for f in faces_of(big):
        a = BRepAdaptor_Surface(f)
        if a.GetType() == GeomAbs_Cylinder and abs(a.Cylinder().Axis().Direction().Y()) > 0.999:
            p = a.Cylinder().Axis().Location()
            big_cyl.append((a.Cylinder().Radius(), p.X(), p.Z(), bbox_mm(f)))
    r_shroud, cx, cz, _ = max(big_cyl, key=lambda c: (round(c[0], 3), -c[3][0][1]))
    shroud_lo_y = min(c[3][0][1] for c in big_cyl if abs(c[0] - r_shroud) < 3)
    key = [c for c in big_cyl if KEYHOLE_R_MM[0] < c[0] < KEYHOLE_R_MM[1]]
    assert key, "shroud keyhole not found"
    key_r, key_x, key_z = key[0][0], key[0][1], key[0][2]
    top_y = max(c[3][1][1] for c in key)                         # shroud top (keyhole plate)

    def ctr_r(it):
        lo, hi = bbox_mm(it["shape"])
        return math.hypot((lo[0] + hi[0]) / 2 - cx, (lo[2] + hi[2]) / 2 - cz), lo, hi
    carousel = [it for it in chg if it is not body and bbox_mm(it["shape"])[1][1] < shroud_lo_y + 20
                and ctr_r(it)[0] < r_shroud]
    centres, clip_y = [], []
    for it in carousel:                                          # pocket clips: thin plates with a fork arc each
        lo, hi = bbox_mm(it["shape"])
        if hi[1] - lo[1] >= 10:
            continue
        clip_y.append((lo[1] + hi[1]) / 2)
        for f in faces_of(it["shape"]):
            a = BRepAdaptor_Surface(f)
            if a.GetType() == GeomAbs_Cylinder and POCKET_ARC_R_MM[0] < a.Cylinder().Radius() < POCKET_ARC_R_MM[1]:
                p = a.Cylinder().Axis().Location()
                if all(math.hypot(p.X() - x, p.Z() - z) > 3.0 for x, z in centres):
                    centres.append((p.X(), p.Z()))
    pockets = centres
    assert len(pockets) == POCKETS, "found %d pockets" % len(pockets)
    radius = sum(math.hypot(x - cx, z - cz) for x, z in pockets) / POCKETS
    angles = sorted(-math.degrees(math.atan2(z - cz, x - cx)) for x, z in pockets)   # machine frame (z up)
    a0 = min(angles, key=abs)
    pitch = [(b - a) for a, b in zip(angles, angles[1:])]
    assert all(abs(p - 360.0 / POCKETS) < 0.5 for p in pitch), pitch
    groove_y = sum(clip_y) / len(clip_y)
    travel = axis_x - (cx + radius)
    ztc = nose_y - (groove_y + TOOL_GROOVE_TO_NOSE_MM)           # head drop from Z top to the clamping height

    # carriage = shroud slab + hub box; the rest of the fused solid stays fixed
    blo, bhi = bbox_mm(big)
    slab = abox((blo[0] - 10, blo[1] - 10, blo[2] - 10), (bhi[0] + 10, top_y + 0.5, bhi[2] + 10))
    hub = abox((CHANGER_HUB_X[0], top_y + 0.5, CHANGER_HUB_Z[0]), (CHANGER_HUB_X[1], CHANGER_HUB_TOP_Y, CHANGER_HUB_Z[1]))
    c = CHANGER_HUB_CLEAR_MM
    hub_in = abox((CHANGER_HUB_X[0], top_y + 0.5, CHANGER_HUB_Z[0] + c),
                  (CHANGER_HUB_X[1], CHANGER_HUB_TOP_Y - c, CHANGER_HUB_Z[1]))
    carriage = BRepAlgoAPI_Common(big, BRepAlgoAPI_Fuse(slab, hub_in).Shape()).Shape()
    rest = BRepAlgoAPI_Cut(big, BRepAlgoAPI_Fuse(slab, hub).Shape()).Shape()

    # head parts that sweep through the shroud while it slides in with the head at the clamping height (the spindle
    # cartridge passes through the keyhole by design): cut their swept boxes out of the shroud
    clo, chi = bbox_mm(carriage)
    env = abox((clo[0] - 1, clo[1] - 1, clo[2] - 1), (chi[0] + travel + 1, top_y + CUT_MARGIN_MM, chi[2] + 1))
    cuts, through = [], []
    for it in head_items:
        lo, hi = bbox_mm(it["shape"])
        hx, hz = (lo[0] + hi[0]) / 2 - axis_x, (lo[2] + hi[2]) / 2 - axis_z
        if (math.hypot(hx, hz) < 6.0 and max(hi[0] - lo[0], hi[2] - lo[2]) / 2 <= key_r) or \
                all(math.hypot(x - axis_x, z - axis_z) <= key_r for x in (lo[0], hi[0]) for z in (lo[2], hi[2])):
            through.append(it)
            continue
        hm = translated(it["shape"], 0.0, -ztc)
        if box_gap(bbox_mm(hm), bbox_mm(env)) > 0:
            continue
        c = BRepAlgoAPI_Common(hm, env).Shape()
        if volume(c) < 1.0:
            continue
        lo, hi = bbox_mm(c)
        cuts.append(((lo[0] - travel - CUT_MARGIN_MM, lo[1] - CUT_MARGIN_MM, lo[2] - CUT_MARGIN_MM),
                     (hi[0] + CUT_MARGIN_MM, top_y + 1.0, hi[2] + CUT_MARGIN_MM), it["path"]))
    removed = 0.0
    if cuts:
        tool = abox(*cuts[0][:2])
        for c in cuts[1:]:
            tool = BRepAlgoAPI_Fuse(tool, abox(*c[:2])).Shape()
        v0 = volume(carriage)
        carriage = BRepAlgoAPI_Cut(carriage, tool).Shape()
        removed = v0 - volume(carriage)
    return dict(body=body, carriage=carriage, rest=rest, carousel=carousel, cx=cx, cz=cz, r_shroud=r_shroud,
                key=(key_r, key_x, key_z), top_y=top_y, radius=radius, a0=a0, pitch=360.0 / POCKETS,
                groove_y=groove_y, travel=travel, ztc=ztc, y_misalign=axis_z - cz, cuts=cuts, removed=removed,
                through=through)


# ----------------------------------------------------------------------------------------------- main
def classify(path):
    for rx, grp, mat, defl, ang in RULES:
        if re.search(rx, path):
            return grp, mat, defl, ang
    raise AssertionError(path)


def snap(rgb):
    return min((k for k in PALETTE if k != "window"), key=lambda k: sum((a - b) ** 2 for a, b in zip(PALETTE[k], rgb)))


def by_colour(table, c, default):
    for ref, mat in table:
        if c and all(abs(a - b) < 0.01 for a, b in zip(ref, c)):
            return mat
    return default


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("step")
    ap.add_argument("out_dir")
    ap.add_argument("--proto", help="also write the Webots PROTO (mesh URLs relative to it)")
    ap.add_argument("--deflection", type=float, default=DEFLECTION_MM, help="default linear deflection, mm")
    ap.add_argument("--angle", type=float, default=ANGLE_RAD, help="default angular deflection, rad")
    ap.add_argument("-v", "--verbose", action="store_true", help="list every kept/dropped item")
    ap.add_argument("--from-json", action="store_true",
                    help="skip the CAD: rewrite only the PROTO from <out_dir>/haas_vf1.json (step argument ignored)")
    args = ap.parse_args()
    if args.from_json:
        assert args.proto, "--from-json needs --proto"
        with open(os.path.join(args.out_dir, "haas_vf1.json")) as fh:
            write_proto(args.proto, args.out_dir, json.load(fh))
        return 0
    os.makedirs(args.out_dir, exist_ok=True)

    items = collect_items(read_step(args.step))

    def find(rx):
        return [it for it in items if re.search(rx, it["path"])]

    def union_box(shapes):
        bs = [bbox_mm(s) for s in shapes]
        return [min(b[0][i] for b in bs) for i in range(3)], [max(b[1][i] for b in bs) for i in range(3)]

    # ---- machine frame from the levelling pads
    p_lo, p_hi = union_box([it["shape"] for it in find(PADS_RE)])
    CX, CZ, FLOOR = (p_lo[0] + p_hi[0]) / 2000.0, (p_lo[2] + p_hi[2]) / 2000.0, p_lo[1] / 1000.0

    def to_m(p):                                   # STEP metres -> machine frame metres
        return (p[0] - CX, -(p[2] - CZ), p[1] - FLOOR)

    def to_m_box(lo, hi):                          # STEP mm box -> machine m box
        a, b = to_m([v / 1000 for v in lo]), to_m([v / 1000 for v in hi])
        return [min(a[i], b[i]) for i in range(3)], [max(a[i], b[i]) for i in range(3)]

    # ---- spindle axis: lowest vertical cylinder of the head (spindle nose / drawbar)
    head_items = [it for it in find(HEAD_RE) if bbox_mm(it["shape"])[0][2] > BEHIND_COLUMN_Z_MM]
    cyl = []
    for it in head_items:
        cyl += [c for c in vertical_cylinders(it["shape"]) if 20.0 < c[0] < 100.0]
    cyl.sort(key=lambda c: (round(c[3], 3), -c[0]))
    axis_x, axis_z = cyl[0][1], cyl[0][2]

    def is_spindle(shape):
        lo, hi = bbox_mm(shape)
        if lo[1] > SPINDLE_Y_MAX_MM:
            return False
        cx, cz = (lo[0] + hi[0]) / 2 - axis_x, (lo[2] + hi[2]) / 2 - axis_z
        half = max(hi[0] - lo[0], hi[2] - lo[2]) / 2
        centred = math.hypot(cx, cz) < 6.0 and half <= SPINDLE_R_MM
        inside = all(math.hypot(x - axis_x, z - axis_z) <= SPINDLE_R_MM for x in (lo[0], hi[0]) for z in (lo[2], hi[2]))
        return centred or inside

    # ---- umbrella tool changer: carriage cut out of the fused solid, carousel, pockets, travels
    nose_y = min(bbox_mm(it["shape"])[0][1] for it in head_items if is_spindle(it["shape"]))
    chg = changer_setup(items, head_items, axis_x, axis_z, nose_y)
    carousel_ids = {id(it) for it in chg["carousel"]}

    # ---- rear cabinet door: cut out of its enclosure solid and turned 90 deg about its vertical hinge (closed)
    rear = [it for it in find(ENC_BODY_RE) if bbox_mm(it["shape"])[0][2] < -1500]
    assert len(rear) == 1, "rear enclosure solid with the open cabinet door not found"
    rear = rear[0]
    rd = close_rear_door(rear["shape"], rear["face_colours"])

    # ---- front solid with the fused doors -> boolean split
    enc_body = [it for it in find(ENC_BODY_RE)]
    front = [it for it in enc_body if bbox_mm(it["shape"])[0][2] > 700 and math.dist(*bbox_mm(it["shape"])) > 1500]
    assert len(front) == 1, "front enclosure solid not found"
    front = front[0]
    front_main, handles = split_shells(front["shape"])
    fcol = dict(zip((hash(f) for f in faces_of(front["shape"])), front["face_colours"]))
    main_cols = [fcol.get(hash(f)) for f in faces_of(front_main)]
    n_skin, w_skin = door_skin(front_main)
    vols = {"door_left": door_volume(DOOR_LEFT_X0, DOOR_SPLIT_X, (DOOR_ROOF_X[0], DOOR_SPLIT_X),
                                     DOOR_HANDLES["door_left"], n_skin, w_skin),
            "door_right": door_volume(DOOR_SPLIT_X, DOOR_RIGHT_X1, (DOOR_SPLIT_X, DOOR_ROOF_X[1]),
                                      DOOR_HANDLES["door_right"], n_skin, w_skin)}
    cmap = ColourMap(front_main, main_cols)
    door_shapes, door_cols, door_handles = {}, {}, {"door_left": [], "door_right": []}
    for h in handles:                                           # handles: separate shells, by side of the split
        lo, hi = bbox_mm(h)
        door_handles["door_left" if (lo[0] + hi[0]) / 2 < DOOR_SPLIT_X else "door_right"].append(h)
    for name, vol in vols.items():
        op = BRepAlgoAPI_Common(front_main, vol)
        door_shapes[name] = op.Shape()
        cm = ColourMap(front_main, main_cols)
        cm.add_op(op)
        door_cols[name] = cm.colours(door_shapes[name])
    op = BRepAlgoAPI_Cut(front_main, BRepAlgoAPI_Fuse(vols["door_left"], vols["door_right"]).Shape())
    front_rest = op.Shape()
    cmap.add_op(op)
    front_rest_cols = cmap.colours(front_rest)

    # ---- door travel: sweep the left door towards -x against the static enclosure sheet metal (baseline = CAD pose)
    obstacles = [front_rest] + [it["shape"] for it in enc_body if it is not front]
    dl = door_shapes["door_left"]
    dl_lo, dl_hi = bbox_mm(dl)
    near = [s for s in obstacles if bbox_mm(s)[0][0] < dl_hi[0] and bbox_mm(s)[1][2] > dl_lo[2] - 50]

    def overlap(t):
        moved = translated(dl, -t)
        return sum(volume(BRepAlgoAPI_Common(moved, s).Shape()) for s in near)

    base = overlap(0.0)
    t, hit = 0.0, None
    while t < 900.0:
        t += DOOR_SWEEP_STEP_MM
        if overlap(t) - base > DOOR_COLLISION_MM3:
            hit = t
            break
    assert hit is not None, "left door never collides"
    lo_t, hi_t = hit - DOOR_SWEEP_STEP_MM, hit                # bisection to 1 mm
    while hi_t - lo_t > 1.0:
        mid = (lo_t + hi_t) / 2
        if overlap(mid) - base > DOOR_COLLISION_MM3:
            hi_t = mid
        else:
            lo_t = mid
    door_travel = round((lo_t - DOOR_CLEARANCE_MM) / 1000.0, 3)

    # ---- assemble the work list: (group, material, part, shape, defl, ang, face colours, path, double-sided)
    work, dropped = [], []
    for it in items:
        grp, mat, defl, ang = classify(it["path"])
        lo, hi = bbox_mm(it["shape"])
        diag = math.dist(lo, hi)
        if grp == "drop" or diag < MIN_DIAG_MM:
            dropped.append((it["path"], "rule" if grp == "drop" else "small %.0f mm" % diag))
            continue
        path = it["path"]
        if grp == "head":
            if lo[2] < BEHIND_COLUMN_Z_MM:
                grp, path = "static", path + " (behind the column)"
            elif is_spindle(it["shape"]):
                grp, mat, defl, ang = "spindle", "steel", SPINDLE_DEFL[0], SPINDLE_DEFL[1]
            if mat is None:
                mat = by_colour(HEAD_COLOURS, it["colour"], "grey")
        if re.search(CHANGER_RE, path) and mat is None:
            mat = by_colour(CHANGER_COLOURS, it["colour"], "grey")
        if mat is None:
            c = it["colour"]
            mat = snap(c) if c and c not in DEFAULT_XCAF else "grey"
        fcols = it["face_colours"] if re.search(FACE_COLOUR_RE, path) else None
        defl, ang = defl or args.deflection, ang or args.angle
        if it is rear:
            work.append(("static", mat, "", rd["rest"], defl, ang, rd["rest_cols"], path + " (without the rear door)",
                         False))
            work.append(("static", mat, "", rd["door"], defl, ang, rd["door_cols"], path + " (rear door, closed)",
                         False))
            continue
        if it is chg["body"]:
            work.append(("static", mat, "", chg["rest"], defl, ang, None, path + " (fixed part)", False))
            work.append(("changer", mat, "", chg["carriage"], defl, ang, None, path + " (sliding carriage)", False))
            continue
        if id(it) in carousel_ids:
            grp = "carousel"
        if it is front:
            work.append(("static", mat, "", front_rest, defl, ang, front_rest_cols, path + " (without doors)", False))
            for name in sorted(door_shapes):
                work.append((name, mat, "", door_shapes[name], defl, ang, door_cols[name], path + " (%s)" % name,
                             False))
                for h in door_handles[name]:
                    work.append((name, "steel", "", h, defl, ang, None, path + " (%s handle)" % name, False))
            continue
        work.append((grp, mat, "", it["shape"], defl, ang, fcols, path, it["surface"]))

    # ---- tessellate into buckets keyed by (group, part, material), triangles in the machine frame
    car_m = to_m((chg["cx"] / 1000, 0.0, chg["cz"] / 1000))
    ca, sa = math.cos(math.radians(-chg["a0"])), math.sin(math.radians(-chg["a0"]))

    def rot_c(p, point=True):
        ox, oy = (car_m[0], car_m[1]) if point else (0.0, 0.0)
        x, y = p[0] - ox, p[1] - oy
        return (ox + ca * x - sa * y, oy + sa * x + ca * y, p[2])
    buckets, log = {}, []
    n_double = 0
    for grp, mat, part, shape, defl, ang, fcols, path, double in work:
        ntri = 0
        for T, fc in tessellate(shape, defl, ang, fcols, 0.0, normals=True):
            m = mat
            if fc and fc not in DEFAULT_XCAF:
                m = snap(fc)
            tl = [(tuple(to_m(p) for p in t), tuple((n[0], -n[2], n[1]) for n in nn)) for t, nn in T]
            if grp == "carousel":                  # pre-rotate: pocket 1 exactly at the change station (angle 0)
                tl = [(tuple(rot_c(p) for p in t), tuple(rot_c(n, False) for n in nn)) for t, nn in tl]
            if double:
                tl += [((t[0], t[2], t[1]), tuple((-n[0], -n[1], -n[2]) for n in (nn[0], nn[2], nn[1])))
                       for t, nn in tl]
                n_double += len(tl) // 2
            ntri += len(tl)
            buckets.setdefault((grp, part, m), []).append(tl)      # one list per face: files split between faces
        log.append((grp, mat, ntri, path + (" (2 sides)" if double else "")))

    # window panes (both sides), in each door group
    def flat(t):
        u = [t[1][j] - t[0][j] for j in range(3)]
        v = [t[2][j] - t[0][j] for j in range(3)]
        n = _unit((u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]))
        return (t, (n, n, n))
    panes = {}
    for name, shape in sorted(door_shapes.items()):
        q = window_pane(shape, n_skin, w_skin)
        if q:
            q = [to_m(p) for p in q]
            panes[name] = q
            buckets[(name, "", "window")] = [[flat(t) for t in ((q[0], q[1], q[2]), (q[0], q[2], q[3]),
                                                                (q[0], q[2], q[1]), (q[0], q[3], q[2]))]]

    # ---- group pivots (machine frame, CAD pose)
    def gbox(grp):
        pts = [p for (g, _, _), faces in buckets.items() if g == grp for tl in faces for t, _ in tl for p in t]
        return [min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)]

    boxes = {g: gbox(g) for g in sorted({k[0] for k in buckets})}
    sp_lo, sp_hi = boxes["spindle"]
    ax_m = to_m((axis_x / 1000, 0, axis_z / 1000))
    nose = (ax_m[0], ax_m[1], sp_lo[2])                        # spindle axis at the lowest spindle point
    table_top = None
    for it in find(TABLE_RE):
        for f in faces_of(it["shape"]):
            a = BRepAdaptor_Surface(f)
            if a.GetType() == GeomAbs_Plane and a.Plane().Axis().Direction().Y() > 0.999:
                ar = face_area(f)
                if table_top is None or ar > table_top[0]:
                    table_top = (ar, a.Plane().Location().Y())
    t_lo, t_hi = boxes["table"]
    s_lo, s_hi = boxes["saddle"]

    def ctr(lo, hi):
        return [(lo[i] + hi[i]) / 2 for i in range(3)]
    origins = {
        "static": (0.0, 0.0, 0.0),
        "saddle": (ctr(s_lo, s_hi)[0], ctr(s_lo, s_hi)[1], s_lo[2]),
        "table": (ctr(t_lo, t_hi)[0], ctr(t_lo, t_hi)[1], table_top[1] / 1000 - FLOOR),
        "head": nose, "spindle": nose,
        "changer": (car_m[0], car_m[1], chg["groove_y"] / 1000 - FLOOR),
        "carousel": (car_m[0], car_m[1], chg["groove_y"] / 1000 - FLOOR),
    }
    assert abs(nose[2] - (nose_y / 1000 - FLOOR)) < 0.001, (nose, nose_y)
    for name in ("door_left", "door_right"):
        origins[name] = tuple(ctr(*boxes[name]))
    origins = {k: tuple(round(v, 4) + 0.0 for v in o) for k, o in origins.items()}

    # CAD pose on each axis (joint position; 0 = table centred under the spindle, Z at the top)
    x_off = ax_m[0] - origins["table"][0]                       # table centre -> spindle axis along x
    y_off = ax_m[1] - origins["table"][1]
    cad_x = -TRAVEL_X / 2 if x_off > 0 else TRAVEL_X / 2
    cad_y = -TRAVEL_Y / 2 if y_off > 0 else TRAVEL_Y / 2
    assert abs(abs(x_off) - TRAVEL_X / 2) < 0.01 and abs(abs(y_off) - TRAVEL_Y / 2) < 0.01, (x_off, y_off)
    gap_z = nose[2] - origins["table"][2]                       # spindle nose to table top, CAD pose (Z top)

    # ---- write OBJ files (split above the shadow limit, between faces, in work order); stale files removed first
    for fn in sorted(os.listdir(args.out_dir)):
        if fn.startswith(FILE_PREFIX) and fn.endswith(".obj"):
            os.remove(os.path.join(args.out_dir, fn))
    files = {}
    total = 0
    group_files = {}
    for key in sorted(buckets):
        grp, part, mat = key
        base_name = FILE_NAMES.get(key) or "_".join(x for x in (grp, part, mat) if x)
        chunks, cur = [], []
        for tl in buckets[key]:
            if cur and len(cur) + len(tl) > SHADOW_MAX_TRIANGLES:
                chunks.append(cur)
                cur = []
            cur.extend(tl)
        chunks.append(cur)
        o = origins[grp]
        for ci, tris in enumerate(chunks):
            name = FILE_PREFIX + base_name + ("_%d" % (ci + 1) if len(chunks) > 1 else "")
            vid, V, NV, F = {}, [], [], []
            for t, nn in tris:
                idx = []
                for p, n in zip(t, nn):
                    v = tuple(round(p[i] - o[i], DIGITS) + 0.0 for i in range(3))
                    nr = tuple(round(n[i], 3) + 0.0 for i in range(3))
                    if (v, nr) not in vid:                 # a vertex per (position, normal): crisp face edges
                        vid[(v, nr)] = len(V)
                        V.append(v)
                        NV.append(nr)
                    idx.append(vid[(v, nr)])
                if len({V[k] for k in idx}) == 3:
                    F.append(idx)
            fn = name + ".obj"
            with open(os.path.join(args.out_dir, fn), "w") as fh:
                fh.write("# Haas VF-1 (official STEP) - group %s, %s; metres, %s group frame\n" % (grp, mat, grp))
                for v in V:
                    fh.write("v %s %s %s\n" % tuple(num(x, DIGITS) for x in v))
                for n in NV:
                    fh.write("vn %s %s %s\n" % tuple(num(x, 3) for x in n))
                for f in F:
                    fh.write("f %d//%d %d//%d %d//%d\n" % (f[0] + 1, f[0] + 1, f[1] + 1, f[1] + 1, f[2] + 1, f[2] + 1))
            lo = [min(v[i] for v in V) for i in range(3)]
            hi = [max(v[i] for v in V) for i in range(3)]
            files[fn] = dict(group=grp, part=part, material=mat, colour=list(PALETTE[mat]),
                             transparency=TRANSPARENCY.get(mat, 0.0), triangles=len(F), vertices=len(V),
                             cast_shadows=len(F) <= SHADOW_MAX_TRIANGLES,
                             bbox_group_frame=[rnd(lo), rnd(hi)],
                             bytes=os.path.getsize(os.path.join(args.out_dir, fn)))
            group_files.setdefault(grp, []).append(fn)
            total += len(F)

    # ---- key positions (machine frame)
    def mbox(rx):
        return [rnd(v) for v in to_m_box(*union_box([it["shape"] for it in find(rx)]))]

    env_lo = [min(boxes[g][0][i] for g in boxes) for i in range(3)]
    env_hi = [max(boxes[g][1][i] for g in boxes) for i in range(3)]
    dlo, dhi = boxes["door_left"]
    rlo, rhi = boxes["door_right"]
    opening = dict(x=[round(dhi[0] - door_travel, 4), round(rlo[0] + door_travel, 4)],
                   note="clear width between the inner edges of the two doors when both are fully open")
    meta = dict(
        source=os.path.basename(args.step),
        frame="machine frame: x_m = x_h - %.4f, y_m = -(z_h - %.4f), z_m = y_h - (%.4f) (STEP metres); origin on "
              "the floor at the centre of the levelling-pad footprint; x = machine X (table length), +y = towards "
              "the back (column), the front (doors, pendant) faces -y; z up" % (CX, CZ, FLOOR),
        units="metres; each file's vertices are in its group frame = machine frame translated to the group origin "
              "(group origins given for the CAD pose)",
        tessellation=dict(default_deflection_mm=args.deflection, default_angle_rad=args.angle,
                          min_diag_mm=MIN_DIAG_MM, shadow_max_triangles=SHADOW_MAX_TRIANGLES),
        total_triangles=total,
        double_sided_triangles=n_double,
        files=files,
        groups=dict(
            static=dict(origin=list(origins["static"]), joint="fixed", files=group_files["static"]),
            saddle=dict(origin=list(origins["saddle"]), origin_def="saddle bbox centre (x, y), underside (z)",
                        joint="prismatic", axis=[0, 1, 0], motor="y motor", travel=TRAVEL_Y,
                        range=[-TRAVEL_Y / 2, TRAVEL_Y / 2], cad_position=cad_y, files=group_files["saddle"],
                        note="Y axis: position 0 = table centred under the spindle; + = table towards the back. "
                             "CAD pose = table fully forward (position %.3f)" % cad_y),
            table=dict(origin=list(origins["table"]), origin_def="table top centre (CAD pose)",
                       parent="saddle", joint="prismatic", axis=[1, 0, 0], motor="x motor", travel=TRAVEL_X,
                       range=[-TRAVEL_X / 2, TRAVEL_X / 2], cad_position=cad_x, files=group_files["table"],
                       note="X axis: position 0 = table centred under the spindle; + = table towards +x. "
                            "CAD pose = position %.3f" % cad_x),
            head=dict(origin=list(origins["head"]), origin_def="spindle axis at the lowest point of the spindle",
                      joint="prismatic", axis=[0, 0, 1], motor="z motor", travel=TRAVEL_Z, range=[-TRAVEL_Z, 0.0],
                      cad_position=0.0, files=group_files["head"],
                      note="Z axis: position 0 = head at the top (CAD pose, Haas Z home); negative = down"),
            spindle=dict(origin=list(origins["spindle"]), parent="head", origin_def="same as head",
                         joint="revolute", axis=[0, 0, 1], motor="spindle motor", files=group_files["spindle"],
                         note="rotating nose of the spindle (parts within %.0f mm of the axis, below y_h = %.0f mm); "
                              "the cartridge housing stays in head" % (SPINDLE_R_MM, SPINDLE_Y_MAX_MM)),
            door_left=dict(origin=list(origins["door_left"]), origin_def="bbox centre of the closed door",
                           joint="prismatic", axis=[-1, 0, 0], motor="door left motor", range=[0.0, door_travel],
                           closed_bbox=[rnd(dlo), rnd(dhi)], files=group_files["door_left"],
                           note="opens towards -x behind the left front panel; travel measured by sweeping the door "
                                "against the enclosure (first contact at %.0f mm, minus %.0f mm clearance)"
                                % (lo_t, DOOR_CLEARANCE_MM)),
            door_right=dict(origin=list(origins["door_right"]), origin_def="bbox centre of the closed door",
                            joint="prismatic", axis=[1, 0, 0], motor="door right motor", range=[0.0, door_travel],
                            closed_bbox=[rnd(rlo), rnd(rhi)], files=group_files["door_right"],
                            note="opens towards +x behind the right front panel (pendant side). Same travel as the "
                                 "left door: in the STEP the right front panel is a defeatured SOLID block, so the "
                                 "door pocket is not modelled and the door slides into that block (hidden)"),
            changer=dict(origin=list(origins["changer"]), origin_def="carousel axis at the pocket clip plane "
                         "(V-groove height of the tools), carriage retracted (CAD pose)",
                         joint="prismatic", axis=[1, 0, 0], motor="carousel slide motor",
                         range=[0.0, round(chg["travel"] / 1000, 4)], files=group_files["changer"],
                         note="umbrella carriage (shroud + hub), cut out of the fused solid 37-2104; slides +x until "
                              "the station pocket is on the spindle axis"),
            carousel=dict(origin=list(origins["carousel"]), parent="changer", joint="revolute", axis=[0, 0, 1],
                          motor="carousel motor", files=group_files["carousel"], pockets=POCKETS,
                          pocket_radius=round(chg["radius"] / 1000, 4), pitch_deg=chg["pitch"],
                          cad_pocket_angle_deg=round(chg["a0"], 3),
                          note="carousel plate + 20 pocket clips; meshes pre-rotated by %.3f deg so that at position "
                               "0 pocket 1 is at the change station (+x); pocket n sits at angle -(n-1)*%g deg, i.e. "
                               "carousel position (n-1)*%g deg brings it to the station"
                               % (-chg["a0"], chg["pitch"], chg["pitch"])),
        ),
        key_positions=dict(
            spindle_axis_xy=rnd([ax_m[0], ax_m[1]]),
            spindle_lowest_z_cad=round(nose[2], 4),
            spindle_lowest_z_min=round(nose[2] - TRAVEL_Z, 4),
            table_top_z=round(origins["table"][2], 4),
            spindle_to_table_cad_mm=round(gap_z * 1000, 1),
            spindle_to_table_min_mm=round((gap_z - TRAVEL_Z) * 1000, 1),
            table_bbox_cad=[rnd(t_lo), rnd(t_hi)],
            saddle_bbox_cad=[rnd(s_lo), rnd(s_hi)],
            head_bbox_cad=[rnd(v) for v in boxes["head"]],
            door_skin=dict(normal_step=rnd(n_skin, 5), w_mm=round(w_skin, 2),
                           tilt_deg=round(math.degrees(math.atan2(n_skin[1], n_skin[2])), 2),
                           note="door skin plane n . p = w in STEP mm; leans back by tilt_deg"),
            door_travel=door_travel,
            door_opening_when_open=opening,
            window_panes={k: [rnd([min(p[i] for p in q) for i in range(3)]), rnd([max(p[i] for p in q) for i in range(3)])]
                          for k, q in panes.items()},
            pendant_bbox=mbox(PENDANT_RE),
            castings_bbox=mbox(CASTINGS_RE),
            pads_bbox=mbox(PADS_RE),
            tool_changer_bbox=mbox(CHANGER_RE),
            floor_z=0.0,
            envelope=[rnd(env_lo), rnd(env_hi)],
            group_bboxes_cad={g: [rnd(b[0]), rnd(b[1])] for g, b in boxes.items()},
            tool_change=dict(
                slide_travel=round(chg["travel"] / 1000, 4),
                z_clamp=round(-chg["ztc"] / 1000, 4),
                z_release=0.0,
                pocket_radius=round(chg["radius"] / 1000, 4),
                groove_z=round(chg["groove_y"] / 1000 - FLOOR, 4),
                groove_to_nose=TOOL_GROOVE_TO_NOSE_MM / 1000,
                station_y_misalignment_mm=round(chg["y_misalign"], 2),
                shroud_radius=round(chg["r_shroud"] / 1000, 4), shroud_top_z=round(chg["top_y"] / 1000 - FLOOR, 4),
                keyhole_radius=round(chg["key"][0] / 1000, 4),
                head_parts_through_keyhole=len(chg["through"]),
                shroud_clearance_cuts=[dict(part=c[2], box_step_mm=[rnd(c[0], 1), rnd(c[1], 1)]) for c in chg["cuts"]],
                shroud_volume_removed_cm3=round(chg["removed"] / 1000, 1),
                note="z_clamp: Z (z motor) where the spindle nose is %.0f mm above the pocket clip plane (CAT40 "
                     "V-groove), i.e. a tool clamped in the spindle sits in the station pocket; z_release = Z top "
                     "(CAD pose): the knob of a tool left in the pocket clears the nose. The Z travel of the model "
                     "([-0.508, 0], 0 = CAD top) is kept: the tool change happens at the top of it." % TOOL_GROOVE_TO_NOSE_MM),
            rear_cabinet_door=dict(
                hinge_xy=rnd(to_m((rd["hinge"][0] / 1000, 0.0, rd["hinge"][1] / 1000))[:2]),
                hinge_axis=[0, 0, 1], rotation_deg=90.0, width=round(rd["width"] / 1000, 4),
                gap_to_cabinet_mm=round(rd["gap"], 2), overlap_mm3=round(rd["overlap"], 1),
                volumes_cm3=dict(door=round(rd["door_volume"] / 1000, 1), rest=round(rd["rest_volume"] / 1000, 1),
                                 solid=round(rd["volume"] / 1000, 1)),
                closed_bbox=[rnd(v) for v in to_m_box(*rd["closed_bbox"])],
                note="electrical cabinet door, open at 90 deg in the STEP and fused into the enclosure solid: cut out, "
                     "turned +90 deg about its vertical hinge (counter-clockwise seen from above): hinge edge flush "
                     "with the cabinet side wall, flanges %.1f mm off the cabinet back; static" % REAR_DOOR_GAP_MM),
            tools=[dict(n=i + 1, name=t["name"], length_below_groove=round(-min(p[2] - (p[1] if p[0] == "sphere" else 0)
                                                                             for p in t["parts"]), 4),
                        max_radius_below_flange=max(p[1] for p in t["parts"])) for i, t in enumerate(TOOLS)],
        ),
        dropped_by_rule=sorted({p for p, why in dropped if why == "rule"}),
        dropped_small_count=sum(1 for _, why in dropped if why != "rule"),
    )
    with open(os.path.join(args.out_dir, "haas_vf1.json"), "w") as fh:
        json.dump(meta, fh, indent=1, sort_keys=True)

    if args.proto:
        write_proto(args.proto, args.out_dir, meta)

    if args.verbose:
        for grp, mat, n, path in log:
            print("%-10s %-6s %6d  %s" % (grp, mat, n, path))
        for p, why in dropped:
            print("DROP %-14s %s" % (why, p))
    for fn, f in files.items():
        print("%-32s %-10s %7d tri  %8.1f kB" % (fn, f["group"], f["triangles"], f["bytes"] / 1024))
    print("total %d triangles, %.1f MB; door travel %.3f m" % (total, sum(f["bytes"] for f in files.values()) / 1e6,
                                                              door_travel))
    print("rear cabinet door closed: hinge (%.1f, %.1f) STEP mm, gap %.2f mm, overlap %.1f mm3"
          % (rd["hinge"][0], rd["hinge"][1], rd["gap"], rd["overlap"]))
    print("tool changer: slide %.1f mm, Z clamp %.1f mm, pocket radius %.1f mm, CAD pocket angle %.2f deg, station y "
          "misalignment %.2f mm, %d shroud cuts (%.0f cm3), %d head parts through the keyhole"
          % (chg["travel"], -chg["ztc"], chg["radius"], chg["a0"], chg["y_misalign"], len(chg["cuts"]),
             chg["removed"] / 1000, len(chg["through"])))



# ----------------------------------------------------------------------------------------------- PROTO
def write_proto(path, mesh_dir, meta):
    """Webots R2025a PROTO: static Robot root, Y saddle > X table, Z head > spindle, two sliding doors."""
    rel = os.path.relpath(os.path.abspath(mesh_dir), os.path.dirname(os.path.abspath(path))).replace(os.sep, "/")
    G, F = meta["groups"], meta["files"]
    ROUGH = {"white": (0.45, 0.0), "grey": (0.55, 0.2), "steel": (0.3, 0.8), "dark": (0.5, 0.1), "black": (0.7, 0.0),
             "window": (0.05, 0.0)}
    used = set()

    def shapes(grp, ind):
        out = []
        for fn in G[grp]["files"]:
            f = F[fn]
            mat = f["material"]
            if mat in used:
                app = "USE VF1_%s" % mat.upper()
            else:
                used.add(mat)
                r, m = ROUGH[mat]
                app = "DEF VF1_%s PBRAppearance { baseColor %s roughness %g metalness %g%s }" % (
                    mat.upper(), " ".join("%g" % c for c in f["colour"]), r, m,
                    " transparency %g" % f["transparency"] if f["transparency"] else "")
            out.append("%sShape { appearance %s geometry Mesh { url \"%s/%s\" } castShadows %s }" % (
                ind, app, rel, fn, "TRUE" if f["cast_shadows"] else "FALSE"))
        return "\n".join(out)

    def v3(v):
        return " ".join("%.4f" % (x + 0.0) for x in v)

    def sub(a, b):
        return [a[i] - b[i] for i in range(3)]

    def bbox_node(grp, ind, pad=0.0):
        lo, hi = meta["key_positions"]["group_bboxes_cad"][grp]
        o = G[grp]["origin"]
        c = [(lo[i] + hi[i]) / 2 - o[i] for i in range(3)]
        s = [hi[i] - lo[i] + pad for i in range(3)]
        return "%sPose { translation %s children [ Box { size %s } ] }" % (ind, v3(c), v3(s))

    kp = meta["key_positions"]
    env_lo, env_hi = kp["envelope"]
    # static collision boxes: plinth below the door opening, side blocks, roof band, column zone
    dl, dr = G["door_left"]["closed_bbox"], G["door_right"]["closed_bbox"]
    sill = min(dl[0][2], dr[0][2])
    top = max(dl[1][2], dr[1][2])
    front_y = min(dl[0][1], dr[0][1])
    x0, x1 = dl[0][0], dr[1][0]
    front_y -= 0.01                                            # front sheet metal, just ahead of the doors
    back_y = kp["castings_bbox"][1][1]                         # back of the column (cabinet door / tank ignored)
    col_y = kp["spindle_axis_xy"][1] + 0.45                    # column front, behind the head
    sboxes = [((env_lo[0], front_y, 0.0), (env_hi[0], back_y, sill)),
              ((env_lo[0], front_y, sill), (x0, back_y, top)),
              ((x1, front_y, sill), (env_hi[0], back_y, top)),
              ((x0, front_y, top), (x1, back_y, top + 0.08)),
              ((x0, col_y, sill), (x1, back_y, top))]
    sb = "\n".join("        Pose { translation %s children [ Box { size %s } ] }" % (
        v3([(a[i] + b[i]) / 2 for i in range(3)]), v3([b[i] - a[i] for i in range(3)])) for a, b in sboxes)

    sad, tab, hed, spn = G["saddle"], G["table"], G["head"], G["spindle"]
    t_rel = sub(tab["origin"], sad["origin"])
    t_rel[0] -= tab["cad_position"]                             # table position 0 = centred
    s_zero = list(sad["origin"])
    s_zero[1] -= sad["cad_position"]
    txt = """#VRML_SIM R2025a utf8
# template language: javascript
# license: derived from the official Haas Automation VF-1 STEP model (Haas Automation rights apply)
# Haas VF-1 vertical machining centre, generated by tools/haas_vf1_to_meshes.py from the official Haas STEP.
# Origin on the floor at the centre of the levelling-pad footprint; x = machine X (table length), the front (doors,
# pendant) faces -y, z up. Root Robot without physics (fixed to the world). Moving bodies are kinematic unless
# movingPhysics is TRUE (then they get masses; ODE lets heavy slides sag by a few mm). Moving bodies:
#   "y motor"  saddle, +y = table towards the back      [-%(ty).3f, %(ty).3f] m, 0 = table centred under the spindle
#   "x motor"  table (on the saddle), +x                [-%(tx).3f, %(tx).3f] m, 0 = table centred under the spindle
#   "z motor"  head, +z = up                            [-%(tz).3f, 0] m, 0 = head at the top (Haas Z home)
#   "spindle motor"  spindle about the vertical axis (continuous)
#   "door left motor" / "door right motor"  sliding doors, + = opening (left door towards -x, right towards +x),
#                                           [0, %(td).3f] m
#   "carousel slide motor"  umbrella tool changer carriage, + = towards the spindle  [0, %(tc).4f] m, 0 = retracted
#   "carousel motor"  carousel (20 pockets) about the vertical axis (continuous); position (n-1)*2pi/20 brings
#                     pocket n to the change station
# Each motor has a PositionSensor ("x sensor", "y sensor", "z sensor", "spindle sensor", "door left sensor",
# "door right sensor", "carousel slide sensor", "carousel sensor").
# Tool change (Haas umbrella, fixed pockets: tool n always returns to pocket n): Z at z_clamp = %(zc).4f m with the
# carriage in -> the station pocket holds the V-flange of the tool clamped in the spindle; Z at 0 (top) = released,
# the knob clears the spindle nose. Tools are CAT40 holders built here: each tool of `tools` exists twice, a pocket
# copy (DEF TOOL_P<n>, SliderJoint in the carousel) and a spindle copy (DEF TOOL_S<n>, SliderJoint in the spindle);
# the copy that is not in use is parked %(park)g m below its seat (joint position -%(park)g). A supervisor controller
# transfers a tool by swapping the two joint positions with Node.setJointPosition (see controllers/vf1_demo).
# Keep the pockets next to a loaded pocket empty (default: every other pocket): in the CAD, fittings under the head
# would hit the retention knobs of tools in the pockets beside the station while the carriage slides in
# (tools/vf1_atc_check.py).
# Equipment (all optional, defaults keep the bare machine):
#   tableSlot  children of the table, frame = top of the table, centred on the table top (x = table length, y towards
#              the column, z up); T-slots (3 x 16 mm) along x at y = -%(slot).3f, 0, +%(slot).3f m. Devices of the
#              nodes put there (e.g. SchunkKSP3 jaw motors) belong to this Robot.
#   probeTool  tool number (in `tools`) that is a Renishaw OMP40-2 spindle probe instead of a cutter (procedural:
#              CAT40 shank, D40 x 50 body, IR window, D6 ruby ball, tip %(ptip).3f m below the V-groove);
#              probeLedColor = colour of its LEDs / window glow (not used by the template: change it live).
#   probeReceiver  Renishaw OMM-2 optical receiver under the enclosure roof (its LED also shows probeLedColor).
#   airGun     Haas Auto Air Gun nozzle under the head (M83 / M84); airBlastTransparency = jet visibility (1 = off,
#              not used by the template: change it live, like coolantTransparency of HaasST20).
# Generated file: edit tools/haas_vf1_to_meshes.py instead.

PROTO HaasVF1 [
  field SFVec3f    translation       0 0 0
  field SFRotation rotation          0 0 1 0
  field SFString   name              "Haas VF-1"
  field SFString   controller        "<none>"
  field MFString   controllerArgs    []
  field SFBool     supervisor        FALSE
  field SFFloat    xPosition         0      # initial X (table), m
  field SFFloat    yPosition         0      # initial Y (saddle), m
  field SFFloat    zPosition         0      # initial Z (head), m, <= 0
  field SFFloat    spindleAngle      0      # initial spindle angle, rad
  field SFFloat    doorLeftPosition  0      # initial opening of the left door, m
  field SFFloat    doorRightPosition 0      # initial opening of the right door, m
  field SFBool     movingPhysics     FALSE  # FALSE: kinematic joints (exact positions); TRUE: masses on moving bodies
  field SFFloat    carouselSlide     0      # initial slide of the tool changer carriage towards the spindle, m
  field SFFloat    carouselAngle     0      # initial carousel angle, rad (pocket n at the station: (n-1)*2pi/20)
  field MFInt32    tools             [1 3 5 7 10 12 14 16]  # tools loaded (tool n lives in pocket n, 1..20); see below
  field SFInt32    spindleTool       1      # tool clamped in the spindle at start (0 = none); its pocket is empty
  field MFNode     tableSlot         []     # equipment on the table (frame: table top centre), see above
  field SFInt32    probeTool         0      # tool number that is a Renishaw OMP40-2 probe (0 = none)
  field SFColor    probeLedColor     0 0 0  # probe / receiver LEDs (live)
  field SFBool     probeReceiver     FALSE  # Renishaw OMM-2 receiver in the enclosure
  field SFBool     airGun            FALSE  # Haas Auto Air Gun nozzle on the head
  field SFFloat    airBlastTransparency 1   # air jet: 1 = off, ~0.6 = visible (live)
]
{
  %%<
    const clamp = (v, a, b) => Math.min(Math.max(v, a), b);
    const px = clamp(fields.xPosition.value, -%(tx).4f, %(tx).4f);
    const py = clamp(fields.yPosition.value, -%(ty).4f, %(ty).4f);
    const pz = clamp(fields.zPosition.value, -%(tz).4f, 0);
    const pa = fields.spindleAngle.value;
    const pl = clamp(fields.doorLeftPosition.value, 0, %(td).4f);
    const pr = clamp(fields.doorRightPosition.value, 0, %(td).4f);
    const pc = clamp(fields.carouselSlide.value, 0, %(tc).4f);
    const pq = fields.carouselAngle.value;
    const TOOLS = %(tools_js)s;
    const PROBE = %(probe_js)s;
    const toolSet = [...new Set(fields.tools.value.filter(n => n >= 1 && n <= %(np)d))].sort((a, b) => a - b);
    const st = toolSet.includes(fields.spindleTool.value) ? fields.spindleTool.value : 0;
    const f4 = v => (v + 0).toFixed(4);
    function toolShapes(n) {
      const t = n === fields.probeTool.value ? PROBE : TOOLS[(n - 1) %% TOOLS.length];
      let out = '';
      for (const [kind, r, z0, z1, mat] of t.parts) {
        const app = mat === 'holder' ? 'USE VF1_STEEL' : mat === 'body' ? 'USE VF1_DARK' :
          mat === 'probe' ? 'PBRAppearance { baseColor 0.1 0.12 0.16 roughness 0.35 metalness 0.3 }' :
          mat === 'stylus' ? 'PBRAppearance { baseColor 0.45 0.46 0.48 roughness 0.3 metalness 0.8 }' :
          mat === 'window' ? 'PBRAppearance { baseColor 0.3 0.04 0.05 roughness 0.1 metalness 0 transparency 0.25 ' +
                             'emissiveColor IS probeLedColor emissiveIntensity 4 }' :
          'PBRAppearance { baseColor ' + t.colour.join(' ') + ' roughness 0.3 metalness 0.7 }';
        if (kind === 'cyl')
          out += 'Pose { translation 0 0 ' + f4((z0 + z1) / 2) + ' children [ Shape { appearance ' + app +
                 ' geometry Cylinder { height ' + f4(z1 - z0) + ' radius ' + f4(r) + ' subdivision 24 } } ] } ';
        else if (kind === 'cone')
          out += 'Pose { translation 0 0 ' + f4((z0 + z1) / 2) + ' rotation 1 0 0 3.14159 children [ Shape { ' +
                 'appearance ' + app + ' geometry Cone { bottomRadius ' + f4(r) + ' height ' + f4(z1 - z0) +
                 ' subdivision 24 } } ] } ';
        else
          out += 'Pose { translation 0 0 ' + f4(z0) + ' children [ Shape { appearance ' + app +
                 ' geometry Sphere { radius ' + f4(r) + ' subdivision 2 } } ] } ';
      }
      return out;
    }
    function toolCopy(n, inSpindle) {
      const used = inSpindle ? (n === st) : (n !== st);
      const p = used ? 0 : -%(park)g;
      const a = -(n - 1) * 2 * Math.PI / %(np)d;
      const xy = inSpindle ? '0 0' : f4(%(pr)s * Math.cos(a)) + ' ' + f4(%(pr)s * Math.sin(a));
      const z0 = inSpindle ? -%(g2n)s : 0;
      return 'DEF TOOL_' + (inSpindle ? 'S' : 'P') + n + ' SliderJoint { jointParameters JointParameters { position ' +
             p + ' axis 0 0 1 } endPoint Solid { translation ' + xy + ' ' + f4(z0 + p) + ' name "tool ' + n +
             (inSpindle ? ' spindle' : ' pocket') + '" children [ ' + toolShapes(n) + '] } }\\n';
    }
  >%%
  Robot {
    translation IS translation
    rotation IS rotation
    name IS name
    model "Haas VF-1"
    controller IS controller
    controllerArgs IS controllerArgs
    supervisor IS supervisor
    children [
%(static)s
      %%< if (fields.probeReceiver.value) { >%%
%(omm)s
      %%< } >%%
      SliderJoint {
        jointParameters JointParameters { position %%<= py >%% axis 0 1 0 minStop -%(ty).4f maxStop %(ty).4f }
        device [
          LinearMotor { name "y motor" minPosition -%(ty).4f maxPosition %(ty).4f maxVelocity 0.42 maxForce 30000 }
          PositionSensor { name "y sensor" }
        ]
        endPoint Solid {
          translation %(sx).4f %%<= (%(sy).4f + py).toFixed(4) >%% %(sz).4f
          name "saddle"
          children [
%(saddle)s
            SliderJoint {
              jointParameters JointParameters { position %%<= px >%% axis 1 0 0 minStop -%(tx).4f maxStop %(tx).4f }
              device [
                LinearMotor { name "x motor" minPosition -%(tx).4f maxPosition %(tx).4f maxVelocity 0.42 maxForce 30000 }
                PositionSensor { name "x sensor" }
              ]
              endPoint Solid {
                translation %%<= (%(tbx).4f + px).toFixed(4) >%% %(tby).4f %(tbz).4f
                name "table"
                children [
%(table)s
                  Pose {
                    translation %(ttx).4f %(tty).4f %(ttz).4f
                    children IS tableSlot
                  }
                ]
                boundingObject %(table_bo)s
                %%< if (fields.movingPhysics.value) { >%% physics Physics { density -1 mass 250 } %%< } >%%
              }
            }
          ]
          boundingObject %(saddle_bo)s
          %%< if (fields.movingPhysics.value) { >%% physics Physics { density -1 mass 350 } %%< } >%%
        }
      }
      SliderJoint {
        jointParameters JointParameters { position %%<= pz >%% axis 0 0 1 minStop -%(tz).4f maxStop 0 }
        device [
          LinearMotor { name "z motor" minPosition -%(tz).4f maxPosition 0 maxVelocity 0.42 maxForce 30000 controlPID 200 0 0 }
          PositionSensor { name "z sensor" }
        ]
        endPoint Solid {
          translation %(hx).4f %(hy).4f %%<= (%(hz).4f + pz).toFixed(4) >%%
          name "head"
          children [
%(head)s
            %%< if (fields.airGun.value) { >%%
%(airgun)s
            %%< } >%%
            HingeJoint {
              jointParameters HingeJointParameters { position %%<= pa >%% axis 0 0 1 anchor 0 0 0 }
              device [
                RotationalMotor { name "spindle motor" maxVelocity 850 maxTorque 200 }
                PositionSensor { name "spindle sensor" }
              ]
              endPoint Solid {
                rotation 0 0 1 %%<= pa >%%
                name "spindle"
                children [
%(spindle)s
                  %%< for (const n of toolSet) { >%% %%<= toolCopy(n, true) >%% %%< } >%%
                ]
                boundingObject Pose { translation 0 0 %(spc).4f children [ Cylinder { height %(sph).4f radius %(spr).4f } ] }
                %%< if (fields.movingPhysics.value) { >%% physics Physics { density -1 mass 20 } %%< } >%%
              }
            }
          ]
          boundingObject %(head_bo)s
          %%< if (fields.movingPhysics.value) { >%% physics Physics { density -1 mass 500 } %%< } >%%
        }
      }
%(doors)s
      SliderJoint {
        jointParameters JointParameters { position %%<= pc >%% axis 1 0 0 minStop 0 maxStop %(tc).4f }
        device [
          LinearMotor { name "carousel slide motor" minPosition 0 maxPosition %(tc).4f maxVelocity 0.6 maxForce 5000 }
          PositionSensor { name "carousel slide sensor" }
        ]
        endPoint Solid {
          translation %%<= (%(cgx).4f + pc).toFixed(4) >%% %(cgy).4f %(cgz).4f
          name "tool changer"
          children [
%(changer)s
            HingeJoint {
              jointParameters HingeJointParameters { position %%<= pq >%% axis 0 0 1 anchor 0 0 0 }
              device [
                RotationalMotor { name "carousel motor" maxVelocity 6 maxTorque 500 }
                PositionSensor { name "carousel sensor" }
              ]
              endPoint Solid {
                rotation 0 0 1 %%<= pq >%%
                name "carousel"
                children [
%(carousel)s
                  %%< for (const n of toolSet) { >%% %%<= toolCopy(n, false) >%% %%< } >%%
                ]
                boundingObject %(carousel_bo)s
                %%< if (fields.movingPhysics.value) { >%% physics Physics { density -1 mass 60 } %%< } >%%
              }
            }
          ]
          boundingObject %(changer_bo)s
          %%< if (fields.movingPhysics.value) { >%% physics Physics { density -1 mass 120 } %%< } >%%
        }
      }
    ]
    boundingObject Group {
      children [
%(sboxes)s
      ]
    }
  }
}
"""
    body = {g: shapes(g, ind) for g, ind in (("static", "      "), ("saddle", "            "),
                                              ("table", "                  "), ("head", "            "),
                                              ("spindle", "                  "), ("changer", "            "),
                                              ("carousel", "                  "))}    # file order: DEF before USE
    doors = []
    for grp, sign, mot in (("door_left", "-", "left"), ("door_right", "+", "right")):
        d = G[grp]
        o = d["origin"]
        var = "pl" if mot == "left" else "pr"
        doors.append("""      SliderJoint {
        jointParameters JointParameters { position %%<= %(var)s >%% axis %(ax)s 0 0 minStop 0 maxStop %(td).4f }
        device [
          LinearMotor { name "door %(mot)s motor" minPosition 0 maxPosition %(td).4f maxVelocity 0.5 maxForce 2000 }
          PositionSensor { name "door %(mot)s sensor" }
        ]
        endPoint Solid {
          translation %%<= (%(ox).4f %(sign)s %(var)s).toFixed(4) >%% %(oy).4f %(oz).4f
          name "door %(mot)s"
          children [
%(shapes)s
          ]
          boundingObject %(bo)s
          %%< if (fields.movingPhysics.value) { >%% physics Physics { density -1 mass 30 } %%< } >%%
        }
      }""" % dict(var=var, sign=sign, ax="-1" if sign == "-" else "1", mot=mot, td=d["range"][1], ox=o[0], oy=o[1], oz=o[2],
                  shapes=shapes(grp, "            "), bo=bbox_node(grp, "")))
    # table top centre (top face vertices of the table mesh, table group frame)
    tv = []
    with open(os.path.join(mesh_dir, G["table"]["files"][0])) as fh:
        for line in fh:
            if line.startswith("v "):
                tv.append(tuple(float(c) for c in line.split()[1:4]))
    ztop = max(v[2] for v in tv)
    top = [v for v in tv if v[2] > ztop - 1e-6]
    ttop = [(min(v[i] for v in top) + max(v[i] for v in top)) / 2 for i in range(2)] + [ztop]

    def rot_to(a, b):
        """Axis-angle rotation turning unit vector a onto b (Webots SFRotation text)."""
        b = _unit(b)
        ax = (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
        ang = math.acos(max(-1.0, min(1.0, sum(a[i] * b[i] for i in range(3)))))
        ax = tuple(round(c, 4) + 0.0 for c in _unit(ax)) if ang > 1e-9 else (0.0, 0.0, 1.0)
        return "%.4f %.4f %.4f %.4f" % (ax[0] + 0.0, ax[1] + 0.0, ax[2] + 0.0, ang)

    # air gun (head frame)
    tip, tgt = AIR_NOZZLE_TIP, AIR_JET_TARGET
    d = _unit([tgt[i] - tip[i] for i in range(3)])
    jet_c = [tip[i] + d[i] * AIR_JET_LENGTH / 2 for i in range(3)]
    noz_c = [tip[i] - d[i] * 0.012 for i in range(3)]
    pipe_h = AIR_NOZZLE_PIPE_TOP - (tip[2] - d[2] * 0.024)
    airgun = """            Pose {
              translation %s %s %.4f
              children [ Shape { appearance DEF VF1_AIRGUN PBRAppearance { baseColor 0.72 0.5 0.25 roughness 0.3 metalness 0.9 } geometry Cylinder { height %.4f radius 0.004 subdivision 12 } } ]
            }
            Pose {
              translation %s
              rotation %s
              children [ Shape { appearance USE VF1_AIRGUN geometry Cylinder { height 0.024 radius 0.0035 subdivision 12 } } ]
            }
            Pose {
              translation %s
              rotation %s
              children [ Shape { appearance PBRAppearance { baseColor 0.85 0.92 1 roughness 0.2 metalness 0 transparency IS airBlastTransparency } geometry Cone { bottomRadius 0.022 height %.4f subdivision 16 bottom FALSE } castShadows FALSE } ]
            }""" % ("%.4f" % (tip[0] - d[0] * 0.024), "%.4f" % (tip[1] - d[1] * 0.024),
                    (tip[2] - d[2] * 0.024 + AIR_NOZZLE_PIPE_TOP) / 2, pipe_h, v3(noz_c), rot_to((0, 0, 1), d),
                    v3(jet_c), rot_to((0, 0, 1), [-c for c in d]), AIR_JET_LENGTH)
    # OMM-2 receiver (machine frame), window facing +x of its own frame, turned towards OMM_AIM
    c = [OMM_ROOF[0], OMM_ROOF[1], OMM_ROOF[2] - OMM_DROP]
    aim = _unit([OMM_AIM[i] - c[i] for i in range(3)])
    omm = """      Pose {
        translation %.4f %.4f %.4f
        children [ Shape { appearance USE VF1_DARK geometry Box { size 0.03 0.03 %.4f } } ]
      }
      Pose {
        translation %s
        rotation %s
        children [
          Shape { appearance PBRAppearance { baseColor 0.12 0.14 0.18 roughness 0.4 metalness 0.3 } geometry Box { size 0.045 0.084 0.064 } }
          Pose { translation 0.0235 0 0 children [ Shape { appearance PBRAppearance { baseColor 0.25 0.03 0.04 roughness 0.1 metalness 0 transparency 0.2 emissiveColor IS probeLedColor emissiveIntensity 3 } geometry Box { size 0.003 0.066 0.046 } } ] }
        ]
      }""" % (c[0], c[1], OMM_ROOF[2] - OMM_DROP / 2, OMM_DROP, v3(c), rot_to((1, 0, 0), aim))
    sp_lo, sp_hi = kp["group_bboxes_cad"]["spindle"]
    spo = spn["origin"]
    txt = txt % dict(
        tx=G["table"]["range"][1], ty=G["saddle"]["range"][1], tz=TRAVEL_Z, td=G["door_left"]["range"][1],
        static=body["static"], saddle=body["saddle"], table=body["table"], head=body["head"],
        spindle=body["spindle"],
        sx=s_zero[0], sy=s_zero[1], sz=s_zero[2], tbx=t_rel[0], tby=t_rel[1], tbz=t_rel[2],
        hx=hed["origin"][0], hy=hed["origin"][1], hz=hed["origin"][2],
        spc=(sp_lo[2] + sp_hi[2]) / 2 - spo[2], sph=sp_hi[2] - sp_lo[2],
        spr=max(sp_hi[0] - sp_lo[0], sp_hi[1] - sp_lo[1]) / 2,
        table_bo=bbox_node("table", ""), saddle_bo=bbox_node("saddle", ""), head_bo=bbox_node("head", ""),
        doors="\n".join(doors), sboxes=sb,
        tc=G["changer"]["range"][1], zc=kp["tool_change"]["z_clamp"], park=TOOL_PARK_M, np=POCKETS,
        pr="%.4f" % G["carousel"]["pocket_radius"], g2n="%.4f" % kp["tool_change"]["groove_to_nose"],
        ttx=ttop[0], tty=ttop[1], ttz=ttop[2], airgun=airgun, omm=omm, slot=0.125, ptip=PROBE_TIP_BELOW_GROOVE,
        probe_js=json.dumps(dict(parts=[list(p) for p in TOOL_TOP + PROBE["parts"]], colour=list(PROBE["colour"])),
                            separators=(",", ":")),
        tools_js=json.dumps([dict(parts=[list(p) for p in tool_parts(i + 1)], colour=list(t["colour"]))
                             for i, t in enumerate(TOOLS)], separators=(",", ":")),
        cgx=G["changer"]["origin"][0], cgy=G["changer"]["origin"][1], cgz=G["changer"]["origin"][2],
        changer=body["changer"], carousel=body["carousel"],
        changer_bo=bbox_node("changer", ""), carousel_bo=bbox_node("carousel", ""))
    with open(path, "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    sys.exit(main())
