"""Briques de génération de mondes Webots (reprises du projet palettisation) : primitives box/cyl fusionnables,
fusion de la géométrie statique par matériau en IndexedFaceSet (1 appel de rendu par matériau), profilés alu."""
import math

YELLOW = "1 0.78 0.05"
BLACK = "0.08 0.08 0.08"
out = []

def w(s):
    out.append(s)


def f(v):
    return " ".join("%.4g" % x for x in v)


def app(color, rough=0.5, metal=0.0, emissive=None, transp=0.0, def_=None):
    d = "DEF %s " % def_ if def_ else ""
    e = " emissiveColor %s" % emissive if emissive else ""
    t = " transparency %g" % transp if transp else ""
    return "%sPBRAppearance { baseColor %s roughness %g metalness %g%s%s }" % (d, color, rough, metal, e, t)


class Prim(str):
    """Primitive (box / cylindre) : se comporte comme sa chaîne VRML, mais peut être fusionnée par render()."""


def _rotm(rot):
    if not rot:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    x, y, z, a = rot
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s_, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return ((c + x * x * C, x * y * C - z * s_, x * z * C + y * s_),
            (y * x * C + z * s_, c + y * y * C, y * z * C - x * s_),
            (z * x * C - y * s_, z * y * C + x * s_, c + z * z * C))


def _mergeable(appearance, def_):
    return def_ is None and appearance.startswith("PBRAppearance") and "DEF " not in appearance


def box(size, appearance, pos=(0, 0, 0), rot=None, def_=None, cs=True):
    r = " rotation %s" % f(rot) if rot else ""
    d = "DEF %s " % def_ if def_ else ""
    c = "" if cs else " castShadows FALSE"
    o = Prim("Pose { translation %s%s children [ Shape {%s appearance %s geometry %sBox { size %s } } ] }" % (
        f(pos), r, c, appearance, d, f(size)))
    o.geo = ("box", tuple(size), tuple(pos), rot, appearance, cs) if _mergeable(appearance, def_) else None
    return o


def cyl(radius, height, appearance, pos=(0, 0, 0), rot=None):
    r = " rotation %s" % f(rot) if rot else ""
    o = Prim("Pose { translation %s%s children [ Shape { appearance %s geometry Cylinder { radius %g height %g "
             "subdivision 20 } } ] }" % (f(pos), r, appearance, radius, height))
    o.geo = ("cyl", (radius, height), tuple(pos), rot, appearance, True) if _mergeable(appearance, None) else None
    return o


def _flatten(items):
    for it in items:
        if isinstance(it, (list, tuple)):
            yield from _flatten(it)
        elif it:
            yield it


def render(items):
    """Fusionne toutes les primitives de même matériau en un IndexedFaceSet (1 appel de rendu par matériau)."""
    groups, others = {}, []
    for it in _flatten(items):
        g = getattr(it, "geo", None)
        if g is None:
            others.append(str(it))
            continue
        kind, dims, pos, rot, appearance, cs = g
        pts, idx = groups.setdefault((appearance, cs), ([], []))
        Rm = _rotm(rot)

        def tr(v):
            return tuple(pos[i] + Rm[i][0] * v[0] + Rm[i][1] * v[1] + Rm[i][2] * v[2] for i in range(3))
        b = len(pts)
        if kind == "box":
            sx, sy, sz = dims
            for i in range(8):
                pts.append(tr(((sx if i & 4 else -sx) / 2, (sy if i & 2 else -sy) / 2, (sz if i & 1 else -sz) / 2)))
            for q in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
                idx.append([k + b for k in q])
        else:
            r_, h = dims
            N = 14
            for i in range(N):
                a = 2 * math.pi * i / N
                pts.append(tr((r_ * math.cos(a), r_ * math.sin(a), -h / 2)))
                pts.append(tr((r_ * math.cos(a), r_ * math.sin(a), h / 2)))
            for i in range(N):
                j = (i + 1) % N
                idx.append([b + 2 * i, b + 2 * j, b + 2 * j + 1, b + 2 * i + 1])
            idx.append([b + 2 * i + 1 for i in range(N)])
            idx.append([b + 2 * i for i in reversed(range(N))])
    out_ = list(others)
    for (appearance, cs), (pts, idx) in groups.items():
        out_.append("Shape {%s appearance %s geometry IndexedFaceSet { coord Coordinate { point [ %s ] } "
                    "coordIndex [ %s ] creaseAngle 0.6 } }" % (
                        "" if cs else " castShadows FALSE", appearance,
                        ", ".join(f(p_) for p_ in pts), " ".join(" ".join(map(str, q)) + " -1" for q in idx)))
    return " ".join(out_)


def cyl_x(radius, length, appearance, pos):
    return cyl(radius, length, appearance, pos, (0, 1, 0, math.pi / 2))


def cyl_y(radius, length, appearance, pos):
    return cyl(radius, length, appearance, pos, (1, 0, 0, math.pi / 2))


def solid(name, pos, children, rot=None, def_=None, bo=None):
    d = "DEF %s " % def_ if def_ else ""
    r = " rotation %s" % f(rot) if rot else ""
    b = " boundingObject %s" % bo if bo else ""
    return '%sSolid { translation %s%s name "%s" children [ %s ]%s }' % (d, f(pos), r, name, render(children), b)


ALU = app("0.66 0.67 0.66", 0.35, 0.85)
SLOT = app("0.25 0.26 0.27", 0.6, 0.5)
STEEL = app("0.55 0.57 0.6", 0.35, 0.85)
GALVA = "GalvanizedMetal { }"
DARK = app("0.2 0.2 0.22", 0.6, 0.3)
RAL5010 = app("0.06 0.24 0.45", 0.45, 0.3)        # bleu gentiane (châssis convoyeurs)
RAL7035 = app("0.80 0.81 0.79", 0.5, 0.2)         # gris clair (machines)
RAL2004 = app("0.95 0.38 0.05", 0.45, 0.2)        # orange (lisses racks)
MOTOR = app("0.30 0.38 0.48", 0.45, 0.5)
YEL = app(YELLOW, 0.5, 0.1)
BLK = app(BLACK, 0.6, 0.1)
RED = app("0.85 0.05 0.05", 0.4, 0.1)
BLUE = app("0.1 0.3 0.85", 0.4, 0.1)
PC = app("0.72 0.77 0.8", 0.08, 0, transp=0.8)


def profile(size, pos):
    """Profilé alu rainuré (barre de section carrée), orienté selon sa plus grande dimension."""
    parts = [box(size, ALU, pos)]
    i = max(range(3), key=lambda k: size[k])
    for k in range(3):
        if k == i:
            continue
        for sgn in (-1, 1):
            s = list(size)
            s[k] = 0.002
            o = [3 - k - i]  # l'autre axe
            s[o[0]] = 0.01
            p = list(pos)
            p[k] += sgn * size[k] / 2
            parts.append(box(s, SLOT, p, cs=False))
    return parts


