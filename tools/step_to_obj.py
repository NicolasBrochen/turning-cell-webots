#!/usr/bin/env python3
"""STEP assembly -> one Wavefront OBJ per part (assembly coordinates, metres), for Webots Mesh nodes.

    uv run --no-project --python 3.12 --with cadquery-ocp tools/step_to_obj.py <file.stp> <out_dir> [deflection_mm]

Reads the STEP with OpenCASCADE (XCAF: names, colours, assembly placements), meshes every leaf part and writes
<out_dir>/<k>_<name>.obj plus <out_dir>/parts.json (name, colour, bounding box, triangle count).
"""
import json
import os
import re
import sys

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
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
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_ColorType, XCAFDoc_DocumentTool

src, out = sys.argv[1], sys.argv[2]
defl = float(sys.argv[3]) if len(sys.argv) > 3 else 0.08
os.makedirs(out, exist_ok=True)

doc = TDocStd_Document(TCollection_ExtendedString("doc"))
rd = STEPCAFControl_Reader()
rd.SetColorMode(True)
rd.SetNameMode(True)
assert rd.ReadFile(src) == IFSelect_RetDone, "cannot read " + src
rd.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def name_of(lab):
    a = TDataStd_Name()
    return a.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), a) else "part"


def colour_of(lab, shape):
    c = Quantity_Color()
    for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        found = False
        for call in (lambda: ct.GetColor_s(lab, t, c), lambda: ct.GetColor(shape, t, c)):
            try:
                found = found or call()
            except (TypeError, AttributeError):
                pass
        if found:
            return [round(c.Red(), 3), round(c.Green(), 3), round(c.Blue(), 3)]
    return None


leaves = []


def walk(lab, loc, inherited_colour, path):
    shape = st.GetShape_s(lab)
    col = colour_of(lab, shape) or inherited_colour
    if st.IsReference_s(lab):
        ref = TDF_Label()
        st.GetReferredShape_s(lab, ref)
        walk(ref, loc.Multiplied(st.GetLocation_s(lab)), col, path + [name_of(lab)])
        return
    if st.IsAssembly_s(lab):
        seq = TDF_LabelSequence()
        st.GetComponents_s(lab, seq)
        for i in range(1, seq.Length() + 1):
            walk(seq.Value(i), loc, col, path + [name_of(lab)])
        return
    leaves.append((path + [name_of(lab)], shape.Moved(loc), col))


roots = TDF_LabelSequence()
st.GetFreeShapes(roots)
for i in range(1, roots.Length() + 1):
    walk(roots.Value(i), TopLoc_Location(), None, [])

summary = []
for k, (path, shape, col) in enumerate(leaves):
    BRepMesh_IncrementalMesh(shape, defl, False, 0.35, True)
    V, F = [], []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = (TopoDS.Face_s if hasattr(TopoDS, "Face_s") else TopoDS.Face)(exp.Current())
        lc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, lc)
        if tri is not None:
            tr = lc.Transformation()
            b = len(V)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(tr)
                V.append((p.X() / 1000, p.Y() / 1000, p.Z() / 1000))      # mm -> m
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b_, c = tri.Triangle(i).Get()
                a, b_, c = a - 1 + b, b_ - 1 + b, c - 1 + b              # 1-based per face -> 0-based global
                F.append((a, c, b_) if rev else (a, b_, c))
        exp.Next()
    if not F:
        continue
    nm = re.sub(r"[^A-Za-z0-9]+", "_", path[-1]).strip("_")[:40]
    fn = "%d_%s.obj" % (k, nm)
    with open(os.path.join(out, fn), "w") as fh:
        fh.write("# %s\n" % " / ".join(path))
        for v in V:
            fh.write("v %.6f %.6f %.6f\n" % v)
        for t in F:
            fh.write("f %d %d %d\n" % (t[0] + 1, t[1] + 1, t[2] + 1))
    lo = [min(v[i] for v in V) for i in range(3)]
    hi = [max(v[i] for v in V) for i in range(3)]
    summary.append(dict(file=fn, path=path, colour=col, bbox=[lo, hi], triangles=len(F)))
    print("%-34s %6d tri  bbox %s .. %s  col %s" % (fn, len(F), [round(x * 1000, 1) for x in lo],
                                                    [round(x * 1000, 1) for x in hi], col))
json.dump(summary, open(os.path.join(out, "parts.json"), "w"), indent=1)
