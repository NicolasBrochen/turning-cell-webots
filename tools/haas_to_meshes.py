#!/usr/bin/env python3
"""Haas ST-20 STEP -> grouped OBJ meshes (one rigid body per group, one file per group x colour), for Webots + three.js.

    uv run --no-project --python 3.12 --with cadquery-ocp python -I tools/haas_to_meshes.py \
        cad/haas_st20/ST20-25_Reboot_2-axis_STEP_2019-05.STEP protos/meshes/haas_st20

Reads the official Haas STEP (XCAF: names, colours, placements), keeps one option of each alternative
(8" chuck, BOT turret, tailstock), drops hidden / tiny parts, tessellates coarsely, maps everything into the
"lathe local frame" of tools/gen_world.py and writes, per group, OBJ files whose vertices are expressed in the
group's own frame (origin = pivot, axes parallel to the lathe frame), plus haas_st20.json describing files,
pivots, joint axes/ranges and key positions.

Frames
  STEP (after OCP read, mm; written in m): x = spindle axis (towards tailstock), y = up, z = towards operator.
  lathe local frame (m, z up):  x_l = z_h - 0.40,  y_l = x_h - 0.5567,  z_l = y_h + 1.10   (cyclic permutation:
  proper rotation, triangle winding preserved).  Facade plane x_l = 0, interior towards -x_l, spindle along +y_l.

Groups: static, door (prismatic, -y_l), spindle (revolute about y_l), jaws (jaw_1..3, prismatic radial, same frame
as spindle), turret (cross slide + BOT turret, X slide along the slant, disc indexes about y_l), carriage (Z saddle,
prismatic along y_l), presetter (automatic tool presetter arm, stowed in the CAD pose, revolute about the axis of
its rotary actuator, parallel to the X slide). Deterministic: same input -> byte-identical output.
"""
import argparse
import json
import math
import os
import re
import sys

from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
try:
    from OCP.TDF import TDF_LabelSequence
except ImportError:                       # recent OCP: NCollection sequences live in OCP.collections
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL, TopAbs_SOLID, TopAbs_WIRE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool
from OCP.gp import gp_Pnt

# ----------------------------------------------------------------------------------------------- configuration
# STEP (m) -> lathe local frame (m)
OFF_X_L, OFF_Y_L, OFF_Z_L = -0.40, -0.5567, 1.10   # chuck body front face at y_l = -0.40 (SEAT_FRONT)


def to_lathe(p):                               # p = (x_h, y_h, z_h) in metres
    return (p[2] + OFF_X_L, p[0] + OFF_Y_L, p[1] + OFF_Z_L)


DEFLECTION_MM = 2.0                            # default linear deflection (mm, STEP units after read)
ANGLE_RAD = 0.5                                # default angular deflection
MIN_DIAG_MM = 15.0                             # drop solids/shells whose bbox diagonal is smaller
DIGITS = 5                                     # vertex precision in the OBJ (m)
FILE_PREFIX = "st20_"                          # unique basenames: the HTML5 export flattens mesh URLs to meshes/<name>

# Palette (colour clusters). XCAF colours of this STEP are almost all the SolidWorks default blue-grey, so colours
# come from the rules below; a rule with material None keeps the (non-default) XCAF colour, snapped to the palette.
PALETTE = {
    "white":  (0.88, 0.89, 0.89),              # Haas sheet metal
    "grey":   (0.50, 0.52, 0.54),              # castings, way covers, slides
    "steel":  (0.70, 0.71, 0.73),              # machined parts: chuck, turret, rails
    "dark":   (0.22, 0.23, 0.24),              # door frame, trays, pendant body
    "black":  (0.05, 0.05, 0.05),              # wipers, cables, pads, screen
    "window": (0.72, 0.82, 0.86),              # door polycarbonate (generated, transparency 0.6)
}
TRANSPARENCY = {"window": 0.6}
DEFAULT_XCAF = [(0.591, 0.638, 0.855), (0.591, 0.638, 0.847), (1.0, 1.0, 1.0)]   # "no colour" values
# faces smaller than this (mm^2) are skipped for matching items: tapped holes, small fillets (dense, invisible)
MIN_FACE_AREA = {r"^BOT Turret": 20.0}
FACE_COLOUR_RE = r"Control \(Thin Style\)"     # only these items honour per-face XCAF colours (pendant screen)

# Rules: first match on the item path ("A / B / C", instance NAUO ids stripped) wins.
# (regex, group or "drop", material or None, part tag, deflection_mm or None, angle or None)
RULES = [
    # alternatives we do not keep
    (r"^BMT65 Turret|^VDI Turret", "drop", None, "", None, None),
    (r"^ST-20 Sub Spindle|^5 Inch Chuck", "drop", None, "", None, None),
    (r"^ST-20Y CHUCK 10inch", "drop", None, "", None, None),
    # hidden inside the headstock enclosure / behind the front bulkhead / inside the base
    (r"^Main Spindle / (Hyd Cyl|Main Spindle Casting|Tube1|Spindle1|Drawtube1|Adapter1|ring1)", "drop", None, "", None, None),
    (r"^Enclosure\(Default\) / Level bolt", "drop", None, "", None, None),
    (r"^X Axis - Turret.* / 10000081561", "drop", None, "", None, None),          # inside the turret housing
    (r"^(X Axis - Turret.* / 50-0044_02|Z Axis Assembly / 50-9011_02)", "drop", None, "", None, None),  # trucks

    # moving groups
    (r"^Door / ", "door", None, "", None, None),
    (r"^Main Spindle / Face ring", "spindle", "steel", "", 2.5, 0.6),
    (r"^ST15 8 Inch - 200mm Chuck", "spindle", "steel", "", 1.5, 0.5),        # split into body + jaws below
    (r"^BOT Turret", "turret", "steel", "disc", 3.0, 0.8),
    (r"^X Axis - Turret", "turret", "grey", "slide", None, None),
    (r"^Z Axis Assembly / (10000093407|10000094280)", "static", "grey", "", None, None),   # telescopic Z covers
    (r"^Z Axis Assembly / 50-0044_01", "carriage", "steel", "", 4.0, 1.0),        # X rails (under the slide)
    (r"^Z Axis Assembly / (10000091434|10000091435)", "carriage", "black", "", None, None),  # wipers
    (r"^Z Axis Assembly", "carriage", "grey", "", None, None),
    # static
    (r"^Enclosure\(Default\) / (wiper|Cable)", "static", "black", "", None, None),
    (r"^Enclosure\(Default\) / Leveling Pad", "static", "black", "", 4.0, 0.8),
    (r"^Enclosure\(Default\) / (Door Track|door tray1)", "static", "dark", "", None, None),
    (r"^Enclosure\(Default\) / (front casting cover|front tray|Control)", "static", None, "", None, None),
    (r"^Enclosure\(Default\) / Z way", "static", "steel", "", None, None),
    (r"^Enclosure\(Default\) / (tray1|Frame|Frame2|spacer|Hyd Pan1|Exhaust port|1 /)", "static", "grey", "", None, None),
    (r"^Enclosure\(Default\)", "static", "white", "", None, None),
    (r"^Front Bulkhead", "static", "grey", "", None, None),
    (r"^Base Casting", "static", "grey", "", 4.0, 0.8),
    (r"^Tailstock and Sub Spindle Rails.* / SS linear rail", "static", "steel", "", None, None),
    (r"^Tailstock and Sub Spindle Rails", "static", "grey", "", None, None),
    # automatic tool presetter: hub + arm + head + stylus swing about the rotary actuator axis (group "presetter");
    # base housing, mounting plate and tip guard stay on the headstock
    (r"^Tool Setting Probe\(Default\) / Tool Probe assembly / (tre|kj)( |$)", "presetter", "steel", "", 3.0, 0.8),
    (r"^Tool Setting Probe", "static", "steel", "", 3.0, 0.8),
    (r"^ST-20 Tailstock Option", "static", "grey", "", None, None),
    (r"^ST-20 Reboot Chip Conveyor", "static", "grey", "", None, None),
    (r"^ST-20 55 Gal Coolant Tank", "static", "dark", "", 3.0, 0.8),
    (r".*", "static", "grey", "", None, None),
]
FILE_NAMES = {("door", "", "dark"): "door_frame", ("door", "", "grey"): "door_handle",
              ("door", "", "window"): "door_window", ("spindle", "", "steel"): "spindle"}

CHUCK_RE = r"^ST15 8 Inch - 200mm Chuck"
TURRET_RE = r"^BOT Turret"
TURRET_STATIONS = 12
DOOR_PANEL_RE = r"^Door / 10000090638"
WINDOW_MARGIN_M = 0.015                        # pane overlaps the sheet-metal cut-out by this much
WINDOW_BEHIND_M = 0.002                        # pane sits this far behind the inner skin of the door panel
DOOR_TRACK_RE = r"^Enclosure\(Default\) / Door Track$"
OPENING_LEFT_RE = r"^Enclosure\(Default\) / spindle cover 1"        # front sheet left of the opening (headstock)
OPENING_RIGHT_RE = r"^Enclosure\(Default\) / Front Right outside panel"
OPENING_BOTTOM_RE = r"^Enclosure\(Default\) / (front tray|door tray1)"
X_RAIL_RE = r"^Z Axis Assembly / 50-0044_01"
PENDANT_RE = r"^Enclosure\(Default\) / Control"
PRESETTER_RE = r"^Tool Setting Probe\(Default\) / Tool Probe assembly / (tre|kj)( |$)"
PRESETTER_STYLUS_RE = r"^Tool Setting Probe\(Default\) / Tool Probe assembly / kj"


# ----------------------------------------------------------------------------------------------- STEP reading
def read_step(path):
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    rd = STEPCAFControl_Reader()
    rd.SetColorMode(True)
    rd.SetNameMode(True)
    assert rd.ReadFile(path) == IFSelect_RetDone, "cannot read " + path
    rd.Transfer(doc)
    return doc


def colour_tuple(c):
    return (round(c.Red(), 3), round(c.Green(), 3), round(c.Blue(), 3))


def collect_items(doc):
    """Walk the XCAF tree; return [(path, located solid/shell, raw solid/shell, colour)] in deterministic order."""
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

    def name_of(lab):
        a = TDataStd_Name()
        return a.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), a) else "part"

    def lab_colour(lab):
        c = Quantity_Color()
        for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if ct.GetColor_s(lab, t, c):
                return colour_tuple(c)
        return None

    def shape_colour(s):
        c = Quantity_Color()
        for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
            try:
                if ct.GetColor(s, t, c):
                    return colour_tuple(c)
            except (TypeError, AttributeError):
                pass
        return None

    items = []

    def walk(lab, loc, col, path):
        col = lab_colour(lab) or col
        if st.IsReference_s(lab):
            ref = TDF_Label()
            st.GetReferredShape_s(lab, ref)
            walk(ref, loc.Multiplied(st.GetLocation_s(lab)), col, path)     # instance names are NAUO ids: skip
            return
        nm = name_of(lab)
        if st.IsAssembly_s(lab):
            seq = TDF_LabelSequence()
            st.GetComponents_s(lab, seq)
            for i in range(1, seq.Length() + 1):
                walk(seq.Value(i), loc, col, path + [nm])
            return
        shape = st.GetShape_s(lab)
        subs = []
        for kind in (TopAbs_SOLID, TopAbs_SHELL):        # solids, or shells of surface models
            e = TopExp_Explorer(shape, kind)
            while e.More():
                subs.append(e.Current())
                e.Next()
            if subs:
                break
        for s in subs:
            fcols = []
            f = TopExp_Explorer(s, TopAbs_FACE)
            while f.More():
                fcols.append(shape_colour(f.Current()))
                f.Next()
            items.append(dict(path=" / ".join(path[1:] + [nm]), shape=s.Moved(loc),
                              colour=shape_colour(s) or col, face_colours=fcols,
                              surface=s.ShapeType() == TopAbs_SHELL))

    roots = TDF_LabelSequence()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), None, [])
    return items


# ----------------------------------------------------------------------------------------------- geometry helpers
def bbox_mm(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b, False)
    lo, hi = b.CornerMin(), b.CornerMax()                 # Bnd_Box.Get() is broken in this OCP
    return [lo.X(), lo.Y(), lo.Z()], [hi.X(), hi.Y(), hi.Z()]


def faces_of(shape):
    e = TopExp_Explorer(shape, TopAbs_FACE)
    while e.More():
        yield TopoDS.Face(e.Current())
        e.Next()


def face_area(f):
    g = GProp_GProps()
    BRepGProp.SurfaceProperties_s(f, g)
    return g.Mass()


def axis_cylinders(shape):
    """Cylindrical faces whose axis is parallel to x_h: [(radius, y, z)] in mm."""
    out = []
    for f in faces_of(shape):
        a = BRepAdaptor_Surface(f)
        if a.GetType() == GeomAbs_Cylinder:
            ax = a.Cylinder().Axis()
            if abs(ax.Direction().X()) > 0.999:
                p = ax.Location()
                out.append((a.Cylinder().Radius(), p.Y(), p.Z()))
    return sorted(out, reverse=True)


def x_planes(shape):
    """Planar faces normal to x_h: [(area, x)] in mm."""
    out = []
    for f in faces_of(shape):
        a = BRepAdaptor_Surface(f)
        if a.GetType() == GeomAbs_Plane and abs(a.Plane().Axis().Direction().X()) > 0.999:
            out.append((face_area(f), a.Plane().Location().X()))
    return sorted(out, reverse=True)


def _unit(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / n, v[1] / n, v[2] / n) if n > 1e-15 else (0.0, 0.0, 1.0)


def tessellate(shape, defl, ang, face_colours=None, min_area=0.0, normals=False):
    """-> list of (triangles [(p0,p1,p2)] in STEP metres, face colour or None) grouped per face.
    normals=True: triangles are ((p0,p1,p2), (n0,n1,n2)) with vertex normals smoothed inside the face only (area
    weighted), so that edges between faces stay crisp whatever the importer does."""
    BRepMesh_IncrementalMesh(shape, defl, False, ang, True)
    out = []
    for k, face in enumerate(faces_of(shape)):
        lc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, lc)
        if tri is None or (min_area > 0 and face_area(face) < min_area):
            continue
        tr = lc.Transformation()
        P = []
        for i in range(1, tri.NbNodes() + 1):
            p = tri.Node(i).Transformed(tr)
            P.append((p.X() / 1000.0, p.Y() / 1000.0, p.Z() / 1000.0))
        rev = face.Orientation() == TopAbs_REVERSED
        T, I = [], []
        for i in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(i).Get()              # 1-based per face
            ix = (a - 1, c - 1, b - 1) if rev else (a - 1, b - 1, c - 1)
            I.append(ix)
            T.append(tuple(P[k] for k in ix))
        if normals:
            N = [[0.0, 0.0, 0.0] for _ in P]
            for ix in I:
                p0, p1, p2 = (P[k] for k in ix)
                u = [p1[j] - p0[j] for j in range(3)]
                v = [p2[j] - p0[j] for j in range(3)]
                n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                for k in ix:
                    for j in range(3):
                        N[k][j] += n[j]
            N = [_unit(n) for n in N]
            T = [(t, tuple(N[k] for k in ix)) for t, ix in zip(T, I)]
        fc = face_colours[k] if face_colours and k < len(face_colours) else None
        out.append((T, fc))
    return out


def split_chuck(shape):
    """Cut the 8" chuck at its body front face: body + separate jaw solids (portion in front of the face)."""
    planes = x_planes(shape)
    face_x = planes[0][1]                                 # largest x-normal plane = body front face
    lo, hi = bbox_mm(shape)
    big = 1000.0
    box = BRepPrimAPI_MakeBox(gp_Pnt(face_x, -big, -big), gp_Pnt(hi[0] + 10.0, big, big)).Shape()
    body = BRepAlgoAPI_Cut(shape, box).Shape()
    front = BRepAlgoAPI_Common(shape, box).Shape()
    jaws = []
    e = TopExp_Explorer(front, TopAbs_SOLID)
    while e.More():
        jaws.append(e.Current())
        e.Next()
    return face_x, hi[0], body, jaws


def door_window(shape):
    """Largest z-normal planar face with an inner wire = door skin with the window cut-out -> pane rectangle."""
    best = None
    for f in faces_of(shape):
        a = BRepAdaptor_Surface(f)
        if a.GetType() != GeomAbs_Plane or abs(a.Plane().Axis().Direction().Z()) < 0.999:
            continue
        outer = BRepTools.OuterWire_s(f)
        w = TopExp_Explorer(f, TopAbs_WIRE)
        while w.More():
            if not w.Current().IsSame(outer):
                lo, hi = bbox_mm(w.Current())
                key = (face_area(f), -a.Plane().Location().Z())
                if best is None or key > best[0]:
                    best = (key, lo, hi, a.Plane().Location().Z())
            w.Next()
    if best is None:
        return None
    return best[1], best[2]


# ----------------------------------------------------------------------------------------------- main
def classify(path):
    for rx, grp, mat, part, defl, ang in RULES:
        if re.search(rx, path):
            return grp, mat, part, defl, ang
    raise AssertionError(path)


def snap(rgb):
    return min((k for k in PALETTE if k != "window"), key=lambda k: sum((a - b) ** 2 for a, b in zip(PALETTE[k], rgb)))


def num(x, digits):
    """Compact fixed-point text: no trailing zeros, no '-0' (smaller OBJ files)."""
    t = ("%.*f" % (digits, x)).rstrip("0").rstrip(".")
    return "0" if t in ("-0", "") else t


def rnd(v, n=4):
    return [round(x, n) for x in v]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("step")
    ap.add_argument("out_dir")
    ap.add_argument("--deflection", type=float, default=DEFLECTION_MM, help="default linear deflection, mm")
    ap.add_argument("--angle", type=float, default=ANGLE_RAD, help="default angular deflection, rad")
    ap.add_argument("-v", "--verbose", action="store_true", help="list every kept/dropped item")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    items = collect_items(read_step(args.step))

    # ---- key geometry read from the CAD (mm, STEP frame)
    def find(rx):
        return [it for it in items if re.search(rx, it["path"])]

    chuck = find(CHUCK_RE)[0]["shape"]
    cyl = axis_cylinders(chuck)
    spindle_yz = (cyl[0][1], cyl[0][2])
    face_x, jaw_tip_x, chuck_body, jaw_solids = split_chuck(chuck)
    turret_shape = find(TURRET_RE)[0]["shape"]
    tc = axis_cylinders(turret_shape)[0]
    turret_yz = (tc[1], tc[2])
    t_lo, t_hi = bbox_mm(turret_shape)
    panel = max((it["shape"] for it in find(DOOR_PANEL_RE)), key=lambda s: math.dist(*bbox_mm(s)))
    win_lo, win_hi = door_window(panel)
    panel_planes = []
    for f in faces_of(panel):
        a = BRepAdaptor_Surface(f)
        if a.GetType() == GeomAbs_Plane and abs(a.Plane().Axis().Direction().Z()) > 0.999:
            panel_planes.append((face_area(f), a.Plane().Location().Z()))
    panel_planes.sort(reverse=True)
    skin_back_z = min(z for _, z in panel_planes[:2])     # the two big skins of the front sheet

    # X slide direction = principal direction of the X rails (mesh vertices, PCA in y-z)
    rail_pts = []
    for it in find(X_RAIL_RE)[:1]:
        for T, _ in tessellate(it["shape"], args.deflection, args.angle):
            for t in T:
                rail_pts.extend(t)
    n = len(rail_pts)
    my = sum(p[1] for p in rail_pts) / n
    mz = sum(p[2] for p in rail_pts) / n
    syy = sum((p[1] - my) ** 2 for p in rail_pts)
    szz = sum((p[2] - mz) ** 2 for p in rail_pts)
    syz = sum((p[1] - my) * (p[2] - mz) for p in rail_pts)
    th = 0.5 * math.atan2(2 * syz, syy - szz)                # principal axis angle in (y, z)
    dy, dz = math.cos(th), math.sin(th)
    if dy < 0:
        dy, dz = -dy, -dz                                    # +X = upwards (away from the spindle)
    slant_deg = math.degrees(math.atan2(dy, abs(dz)))

    # presetter pivot: axis of the largest cylindrical face of the arm parts that is parallel to the X slide
    best = None
    for it in find(PRESETTER_RE):
        for f in faces_of(it["shape"]):
            a = BRepAdaptor_Surface(f)
            if a.GetType() == GeomAbs_Cylinder:
                ax = a.Cylinder().Axis()
                d = ax.Direction()
                if abs(d.Y() * dy + d.Z() * dz) > 0.999 and (best is None or a.Cylinder().Radius() > best[0]):
                    best = (a.Cylinder().Radius(), ax.Location())
    pv = best[1]
    stylus_lo, stylus_hi = bbox_mm(find(PRESETTER_STYLUS_RE)[0]["shape"])

    # ---- frames (lathe local frame, m)
    sp_origin = to_lathe((face_x / 1000, spindle_yz[0] / 1000, spindle_yz[1] / 1000))
    tu_origin = to_lathe((t_hi[0] / 1000, turret_yz[0] / 1000, turret_yz[1] / 1000))

    # ---- assemble the work list: (group, material, part, shape, defl, ang, face colours, path, double-sided)
    work, dropped = [], []
    for it in items:
        grp, mat, part, defl, ang = classify(it["path"])
        lo, hi = bbox_mm(it["shape"])
        diag = math.dist(lo, hi)
        if grp == "drop" or diag < MIN_DIAG_MM:
            dropped.append((it["path"], "rule" if grp == "drop" else "small %.0f mm" % diag))
            continue
        if mat is None:
            c = it["colour"]
            mat = snap(c) if c and c not in DEFAULT_XCAF else "grey"
        fcols = it["face_colours"] if re.search(FACE_COLOUR_RE, it["path"]) else None
        defl, ang = defl or args.deflection, ang or args.angle
        if re.search(CHUCK_RE, it["path"]):
            work.append(("spindle", mat, part, chuck_body, defl, ang, None, it["path"] + " (body)", False))
            for k, j in enumerate(jaw_solids):
                work.append(("jaws", mat, "jaw%d" % k, j, defl, ang, None, it["path"] + " (jaw)", False))
            continue
        # surface models (open shells: tailstock, conveyor, tank, pendant...) are written with both windings,
        # otherwise their inner side disappears with back-face culling
        work.append((grp, mat, part, it["shape"], defl, ang, fcols, it["path"], it["surface"]))
    min_area = {w[7]: next((a for rx, a in MIN_FACE_AREA.items() if re.search(rx, w[7])), 0.0) for w in work}

    # ---- tessellate into buckets keyed by file name, triangles in the lathe frame
    buckets = {}
    jaw_tris = {}
    log = []
    n_double = 0
    for grp, mat, part, shape, defl, ang, fcols, path, double in work:
        ntri = 0
        for T, fc in tessellate(shape, defl, ang, fcols, min_area[path], normals=True):
            m = mat
            if fc and fc not in DEFAULT_XCAF:
                m = snap(fc)
            # ((p0, p1, p2), (n0, n1, n2)) in the lathe frame; normals: same permutation, no offset
            tl = [(tuple(to_lathe(p) for p in t), tuple((n[2], n[0], n[1]) for n in nn)) for t, nn in T]
            if double:
                tl += [((t[0], t[2], t[1]), tuple((-n[0], -n[1], -n[2]) for n in (nn[0], nn[2], nn[1])))
                       for t, nn in tl]
                n_double += len(tl) // 2
            ntri += len(tl)
            if grp == "jaws":
                jaw_tris.setdefault(part, []).extend(tl)
            else:
                buckets.setdefault((grp, part, m), []).extend(tl)
        log.append((grp, mat, ntri, path + (" (2 sides)" if double else "")))

    # window pane: rectangle behind the cut-out, both sides
    z_h = skin_back_z / 1000 - WINDOW_BEHIND_M
    x0, x1 = win_lo[0] / 1000 - WINDOW_MARGIN_M, win_hi[0] / 1000 + WINDOW_MARGIN_M
    y0, y1 = win_lo[1] / 1000 - WINDOW_MARGIN_M, win_hi[1] / 1000 + WINDOW_MARGIN_M
    q = [to_lathe((x0, y0, z_h)), to_lathe((x1, y0, z_h)), to_lathe((x1, y1, z_h)), to_lathe((x0, y1, z_h))]
    def flat(t):
        u = [t[1][j] - t[0][j] for j in range(3)]
        v = [t[2][j] - t[0][j] for j in range(3)]
        n = _unit((u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]))
        return (t, (n, n, n))
    buckets[("door", "", "window")] = [flat(t) for t in ((q[0], q[1], q[2]), (q[0], q[2], q[3]),
                                                         (q[0], q[2], q[1]), (q[0], q[3], q[2]))]

    # jaws: sort by angle around the spindle axis (lathe x-z plane), 0 deg = +x_l (towards the operator), CCW seen
    # from +y_l looking back at the chuck is not assumed: angle = atan2(z_l - cz, x_l - cx)
    jaw_info = []
    for part, tl in jaw_tris.items():
        pts = [p for t, _ in tl for p in t]
        cx = sum(p[0] for p in pts) / len(pts) - sp_origin[0]
        cz = sum(p[2] for p in pts) / len(pts) - sp_origin[2]
        ang = math.degrees(math.atan2(cz, cx)) % 360
        r = [math.hypot(p[0] - sp_origin[0], p[2] - sp_origin[2]) for p in pts]
        jaw_info.append((ang, part, tl, min(r), max(r)))
    jaw_info.sort()
    jaws_meta = []
    for k, (ang, part, tl, rmin, rmax) in enumerate(jaw_info):
        buckets[("jaws", "jaw_%d" % (k + 1), "steel")] = tl
        jaws_meta.append(dict(name="jaw_%d" % (k + 1), angle_deg=round(ang, 2),
                              radial_dir_xz=rnd([math.cos(math.radians(ang)), math.sin(math.radians(ang))], 4),
                              grip_radius=round(rmin, 4), outer_radius=round(rmax, 4)))

    origins = {"static": (0.0, 0.0, 0.0), "spindle": sp_origin, "jaws": sp_origin,
               "turret": tu_origin, "carriage": tu_origin,
               "presetter": to_lathe((pv.X() / 1000, pv.Y() / 1000, pv.Z() / 1000))}
    door_pts = [p for (g, _, _), tl in buckets.items() if g == "door" for t, _ in tl for p in t]
    door_lo = [min(p[i] for p in door_pts) for i in range(3)]
    door_hi = [max(p[i] for p in door_pts) for i in range(3)]
    origins["door"] = tuple(round(v, 3) for v in door_lo)

    # ---- write OBJ files
    files = {}
    total = 0
    for key in sorted(buckets):
        grp, part, mat = key
        name = FILE_NAMES.get(key) or (part if grp == "jaws" else "_".join(x for x in (grp, part, mat) if x))
        name = FILE_PREFIX + name
        o = origins[grp]
        vid, V, NV, F = {}, [], [], []
        for t, nn in buckets[key]:
            idx = []
            for p, n in zip(t, nn):
                v = tuple(round(p[i] - o[i], DIGITS) + 0.0 for i in range(3))
                nr = tuple(round(n[i], 3) + 0.0 for i in range(3))
                if (v, nr) not in vid:                     # a vertex per (position, normal): crisp face edges
                    vid[(v, nr)] = len(V)
                    V.append(v)
                    NV.append(nr)
                idx.append(vid[(v, nr)])
            if len({V[k] for k in idx}) == 3:
                F.append(idx)
        fn = name + ".obj"
        with open(os.path.join(args.out_dir, fn), "w") as fh:
            fh.write("# Haas ST-20 (official STEP) - group %s, %s; metres, %s group frame\n" % (grp, mat, grp))
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
                         bbox_group_frame=[rnd(lo), rnd(hi)],
                         bytes=os.path.getsize(os.path.join(args.out_dir, fn)))
        total += len(F)

    # ---- key positions (lathe frame)
    def lbox(rx):
        los, his = [], []
        for it in find(rx):
            lo, hi = bbox_mm(it["shape"])
            a, b = to_lathe([v / 1000 for v in lo]), to_lathe([v / 1000 for v in hi])
            los.append([min(a[i], b[i]) for i in range(3)])
            his.append([max(a[i], b[i]) for i in range(3)])
        return [rnd([min(l[i] for l in los) for i in range(3)]), rnd([max(h[i] for h in his) for i in range(3)])]

    track = lbox(DOOR_TRACK_RE)
    door_travel = round(door_lo[1] - track[0][1], 3)       # door -y edge down to the track end
    left, right, bottom = lbox(OPENING_LEFT_RE), lbox(OPENING_RIGHT_RE), lbox(OPENING_BOTTOM_RE)
    opening = dict(plane_x=round(to_lathe((0, 0, skin_back_z / 1000))[0], 4),
                   y=[left[1][1], right[0][1]], z=[bottom[1][2], round(door_hi[2], 4)],
                   note="front opening between the headstock-side front sheet and the right front panel, "
                        "bottom = top of the front/door trays, top = top of the closed door; the door also wraps "
                        "over the roof, so the opening continues on the roof back to x_l = %.3f" % door_lo[0])
    clear_y_open = [round(door_hi[1] - door_travel, 4), right[0][1]]
    all_pts_lo = [min(files[f]["bbox_group_frame"][0][i] + origins[files[f]["group"]][i] for f in files) for i in range(3)]
    all_pts_hi = [max(files[f]["bbox_group_frame"][1][i] + origins[files[f]["group"]][i] for f in files) for i in range(3)]
    xs = (dy, dz)                                           # +X in STEP (y, z)
    x_dir_l = [round(xs[1], 4), 0.0, round(xs[0], 4)]      # rotation only: (x_l, y_l, z_l) = (z_h, x_h, y_h)
    turret_x_from_axis = math.hypot(tu_origin[0] - sp_origin[0], tu_origin[2] - sp_origin[2])
    # stylus tip (end of the stylus towards -y_l in the stowed pose) and its position after a +90 deg swing
    tip = to_lathe(((stylus_lo[0]) / 1000, (stylus_lo[1] + stylus_hi[1]) / 2000, (stylus_lo[2] + stylus_hi[2]) / 2000))
    pa = origins["presetter"]
    ax_ = x_dir_l
    v = [tip[i] - pa[i] for i in range(3)]
    al = sum(v[i] * ax_[i] for i in range(3))
    pp = [v[i] - al * ax_[i] for i in range(3)]
    cr = [ax_[1] * pp[2] - ax_[2] * pp[1], ax_[2] * pp[0] - ax_[0] * pp[2], ax_[0] * pp[1] - ax_[1] * pp[0]]
    tip_dep = [pa[i] + al * ax_[i] + cr[i] for i in range(3)]
    rel = [tip_dep[0] - sp_origin[0], tip_dep[2] - sp_origin[2]]

    meta = dict(
        source=os.path.basename(args.step),
        frame="lathe local frame of tools/gen_world.py: x_l = z_h - 0.40, y_l = x_h - 0.5567, z_l = y_h + 1.10 "
              "(STEP metres); facade x_l = 0, interior -x_l, spindle along +y_l, z up",
        units="metres; each file's vertices are in its group frame = lathe frame translated to the group origin",
        tessellation=dict(default_deflection_mm=args.deflection, default_angle_rad=args.angle,
                          min_diag_mm=MIN_DIAG_MM, min_face_area_mm2=MIN_FACE_AREA),
        total_triangles=total,
        double_sided_triangles=n_double,
        files=files,
        groups=dict(
            static=dict(origin=list(origins["static"]), joint="fixed"),
            door=dict(origin=list(origins["door"]), origin_def="bbox min corner of the closed door (lathe frame)",
                      joint="prismatic", axis=[0, -1, 0], travel_open=door_travel,
                      range=[0.0, door_travel], closed_bbox=[rnd(door_lo), rnd(door_hi)],
                      note="opens towards the headstock (-y_l) along the Door Track; at full travel the door's "
                           "+y edge stands at y_l = %.3f" % (door_hi[1] - door_travel)),
            spindle=dict(origin=rnd(sp_origin), origin_def="spindle axis at the 8in chuck body front face",
                         joint="revolute", axis=[0, 1, 0]),
            jaws=dict(origin=rnd(sp_origin), origin_def="same as spindle (child of the spindle)",
                      joint="prismatic, radial, one per jaw", jaws=jaws_meta,
                      note="jaws = part of the single-solid 8in chuck in front of the body face, split by a boolean "
                           "cut; the master-jaw part inside the body slots stays in spindle.obj"),
            turret=dict(origin=rnd(tu_origin), origin_def="turret indexing axis at the turret front face "
                        "(x_h max of the BOT turret, towards the tailstock)",
                        joint="X slide (prismatic) + indexing (revolute)",
                        x_slide_axis=x_dir_l, slant_deg_from_horizontal=round(slant_deg, 2),
                        indexing_axis=[0, 1, 0], stations=TURRET_STATIONS,
                        station_pitch_deg=360.0 / TURRET_STATIONS,
                        turret_axis_distance_from_spindle_axis=round(turret_x_from_axis, 4),
                        note="+X slide = away from the spindle (up and back); the turret axis lies on the X line "
                             "through the spindle axis. turret_disc_* files = indexing disc, turret_slide_* = "
                             "cross slide (same frame; split them if the disc must index on its own)"),
            carriage=dict(origin=rnd(tu_origin), origin_def="same point as the turret origin in the CAD pose "
                          "(so the turret can be a child of the carriage with zero offset)",
                          joint="prismatic", axis=[0, 1, 0],
                          note="Z saddle with its X rails/trucks/top covers; telescopic Z covers are static"),
            presetter=dict(origin=rnd(origins["presetter"]), origin_def="point of the rotary actuator axis (largest "
                           "cylinder of the hub parallel to the X slide)", joint="revolute", axis=x_dir_l,
                           stowed=0.0, deployed=round(math.pi / 2, 4), stylus_tip_stowed=rnd(tip),
                           stylus_tip_deployed=rnd(tip_dep),
                           stylus_tip_deployed_x_from_spindle_axis=round(rel[0] * x_dir_l[0] + rel[1] * x_dir_l[2], 4),
                           stylus_tip_deployed_off_tool_plane=round(-rel[0] * x_dir_l[2] + rel[1] * x_dir_l[0], 4),
                           note="CAD pose = stowed (arm up beside the chuck, tip in its guard); +90 deg about the "
                                "axis brings the stylus tip in front of the chuck, in the plane swept by the tool "
                                "tips (spindle axis + X slide)"),
        ),
        key_positions=dict(
            spindle_axis_xz=rnd([sp_origin[0], sp_origin[2]]),
            chuck_body_front_y=round(sp_origin[1], 4),
            chuck_jaw_front_y=round(to_lathe((jaw_tip_x / 1000, 0, 0))[1], 4),
            chuck_back_y=round(to_lathe((bbox_mm(chuck)[0][0] / 1000, 0, 0))[1], 4),
            chuck_radius=round((bbox_mm(chuck)[1][1] - bbox_mm(chuck)[0][1]) / 2000, 4),
            chuck_bbox=lbox(CHUCK_RE),
            turret_axis_xz=rnd([tu_origin[0], tu_origin[2]]),
            turret_bbox=lbox(TURRET_RE),
            turret_disc_y=[round(to_lathe((t_lo[0] / 1000, 0, 0))[1], 4), round(to_lathe((t_hi[0] / 1000, 0, 0))[1], 4)],
            door_closed_bbox=[rnd(door_lo), rnd(door_hi)],
            door_open_bbox=[rnd([door_lo[0], door_lo[1] - door_travel, door_lo[2]]),
                            rnd([door_hi[0], door_hi[1] - door_travel, door_hi[2]])],
            door_track=track,
            door_opening=opening,
            door_opening_clear_y_when_open=clear_y_open,
            window_pane=[rnd([min(p[i] for p in q) for i in range(3)]), rnd([max(p[i] for p in q) for i in range(3)])],
            pendant_bbox=lbox(PENDANT_RE),
            floor_z=round(to_lathe((0, -1.005, 0))[2], 4),
            lowest_point_z=round(all_pts_lo[2], 4),
            envelope=[rnd(all_pts_lo), rnd(all_pts_hi)],
        ),
        dropped_by_rule=sorted({p for p, why in dropped if why == "rule"}),
        dropped_small_count=sum(1 for _, why in dropped if why != "rule"),
    )
    with open(os.path.join(args.out_dir, "haas_st20.json"), "w") as fh:
        json.dump(meta, fh, indent=1, sort_keys=True)

    if args.verbose:
        for grp, mat, n, path in log:
            print("%-9s %-6s %6d  %s" % (grp, mat, n, path))
        for p, why in dropped:
            print("DROP %-14s %s" % (why, p))
    for fn, f in files.items():
        print("%-28s %-9s %7d tri  %8.1f kB" % (fn, f["group"], f["triangles"], f["bytes"] / 1024))
    print("total %d triangles, %.1f MB" % (total, sum(f["bytes"] for f in files.values()) / 1e6))


if __name__ == "__main__":
    sys.exit(main())
