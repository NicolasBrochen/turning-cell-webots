#!/usr/bin/env python3
"""Génère worlds/tournage.wbt : cellule de tournage robotisée (2 tours CNC face à face + UR10e).

Implantation (m, ENU, z vers le haut) :
  - UR10e sur socle en (0, 0), base à 0.80 m. Outil : capteur d'effort 6 axes + double pince SCHUNK (2 pinces
    parallèles opposées, doigts motorisés avec capteurs de force au bout des doigts).
  - Tour A : façade en x = -0.80 tournée vers +x. Tour B : façade en x = +0.80 tournée vers -x (symétrie centrale).
    Tours Haas ST-20 (maillages du STEP officiel Haas, tools/haas_to_meshes.py) : broche horizontale parallèle à la
    façade (axe y), mandrin 8" 3 mors, porte coulissante automatique (ouverture vers la poupée), tourelle BOT 12
    postes sur glissière X inclinée à 55° et chariot Z, bac d'arrosage, convoyeur à copeaux, pupitre CNC.
  - Table à caisses côté opérateur (y ≈ 0.85) : caisse des bruts (x = -0.25) et caisse des pièces finies (x = +0.25),
    12 alvéoles chacune (3 x 4).
  - Côté opérateur ouvert (zone collaborative) surveillé par 2 scrutateurs laser ; carter fixe à l'arrière.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wbtgen import (ALU, BLK, BLUE, DARK, PC, RED, STEEL, YEL, YELLOW, BLACK, app, box, cyl, cyl_x, cyl_y,  # noqa
                    f, out, profile, render, solid, w, _rotm)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers", "ur_tending"))
import dual_gripper as DG  # noqa: E402
import json  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "worlds", "tournage.wbt")
R = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/"
PROTOS = [
    "objects/backgrounds/protos/TexturedBackground.proto",
    "appearances/protos/RoughConcrete.proto",
    "appearances/protos/CorrugatedMetal.proto",
    "robots/universal_robots/protos/UR10e.proto",
    "objects/factory/fire_extinguisher/protos/FireExtinguisher.proto",
    "humans/pedestrian/protos/Pedestrian.proto",
]

# ---------------------------------------------------------------- géométrie partagée avec cell_plc.py
ROBOT_Z = 0.80
LATHE_X = 0.80                       # plan des façades (tours écartés de 1.60 m)
CX, CZ = -0.40, 1.10                 # axe de broche (repère tour)
CHUCK_FACE = -0.40                   # face avant du corps de mandrin (= appui de la pièce, SEAT_FRONT)
JAW_FACE = -0.358                    # face avant des mors
# Haas ST-20 : repères des groupes mobiles (repère tour, voir protos/meshes/haas_st20/haas_st20.json)
DOOR_CLOSED = (-0.56, -0.582, 0.717)  # origine du groupe porte, porte fermée
DOOR_TRAVEL = 0.722                  # course d'ouverture selon -y (vers la poupée)
JAW_DIRS = ((-0.0386, 0.9993), (-0.8459, -0.5333), (0.8888, -0.4583))   # directions radiales des mors (x, z)
JAW_R0 = 0.0255                      # rayon de serrage des mors dans la pose CAO
TUR0 = (-0.6622, 0.3151, 1.474)      # chariot / tourelle en pose CAO : axe de tourelle, face +y du disque
X_DIR = (-0.5737, 0.0, 0.8191)       # glissière X (55°) : +X = s'éloigne de la broche
TUR_AXIS_R = 0.4568                  # distance axe de tourelle - axe de broche (pose CAO)
TOOL_R, TOOL_DY = 0.305, -0.128      # pointe d'outil : vers la broche depuis l'axe de tourelle, décalage axial (y)
TUR_PARK = (0.0, 0.0)                # position de repos (Z chariot selon +y, X glissière) par rapport à la pose CAO
TABLE = (0.0, 0.85, 0.70)            # centre, hauteur du dessus
CRATES = {"BRUTS": (-0.25, 0.85), "FINIES": (0.25, 0.85)}
CELL_X = (-0.12, 0.0, 0.12)
CELL_Y = (-0.195, -0.065, 0.065, 0.195)
PART_L, PART_R = 0.09, 0.025

WHITE = app("0.93 0.93 0.91", 0.45, 0.1)
ANTHRA = app("0.22 0.24 0.27", 0.5, 0.3)
LGREY = app("0.62 0.64 0.66", 0.45, 0.4)
LED = app("1 1 0.95", 0.3, 0, emissive="0.9 0.9 0.85")
COOL = app("0.3 0.6 1", 0.1, 0, emissive="0.05 0.15 0.3", transp=0.45)
RAW = app("0.40 0.41 0.43", 0.75, 0.6)
MACH = app("0.80 0.82 0.85", 0.18, 1.0)
SCHUNK = app("0.70 0.72 0.74", 0.3, 0.8)       # corps anodisé gris clair
SCHUNK_B = app("0.12 0.13 0.15", 0.5, 0.3)


# ---------------------------------------------------------------- géométrie de collision
# Chaque volume de collision sert deux fois : boundingObject Webots (contacts réels, visibles avec View > Optional
# Rendering > Show All Bounding Objects / Show Contact Points) et liste d'obstacles exportée pour le vérificateur hors
# ligne tools/check_collisions.py (worlds/obstacles.json).
OBST = []


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def frame(pos=(0, 0, 0), rot=None, parent=None):
    """Repère (origine, matrice) ; `parent` = repère englobant."""
    R = [list(r) for r in _rotm(rot)]
    if parent is None:
        return (list(pos), R)
    o, P = parent
    return ([o[i] + sum(P[i][k] * pos[k] for k in range(3)) for i in range(3)], _mul(P, R))


def bo(items):
    """items : ('box', (sx,sy,sz), pos, rot) | ('cyl', (r, h), pos, rot) -> boundingObject Webots."""
    s_ = []
    for kind, dims, pos, rot in items:
        geo = "Box { size %s }" % f(dims) if kind == "box" else "Cylinder { radius %g height %g }" % dims
        s_.append("Pose { translation %s%s children [ %s ] }" % (f(pos), " rotation %s" % f(rot) if rot else "", geo))
    return "Group { children [ %s ] }" % " ".join(s_)


def register(owner, items, fr, tag=""):
    for kind, dims, pos, rot in items:
        c, R = frame(pos, rot, fr)
        OBST.append(dict(owner=owner, tag=tag, kind=kind, dims=list(dims), c=c, R=R))


RX90 = (1, 0, 0, math.pi / 2)        # cylindre d'axe y

w("#VRML_SIM R2025a utf8")
for p in PROTOS:
    w('%sEXTERNPROTO "%s%s"' % ("IMPORTABLE " if p.endswith("Pedestrian.proto") else "", R, p))
w('EXTERNPROTO "../protos/GuardBay.proto"')
w("""WorldInfo {
  info [ "Cellule de tournage robotisee - 2 tours CNC face a face, UR10e, double pince SCHUNK, capteurs d'effort"
         "Keys (3D view focused): E = emergency stop, R = reset, P = operator walks in, S = end-of-cycle stop" ]
  title "Robotic turning cell"
  basicTimeStep 8
  contactProperties [ ContactProperties { material1 "doigt" coulombFriction [ 1.2 ] } ]
}
Viewpoint {
  orientation 0.26862 0.20099 -0.94204 1.91362
  position 0.9 3.3 3.1
  near 0.05
}
TexturedBackground { texture "factory" skybox FALSE skyColor [ 0.2 0.21 0.23 ] luminosity 0.9 }
DirectionalLight { direction -0.3 -0.45 -1 intensity 1.7 castShadows TRUE }
DirectionalLight { direction 0.5 0.4 -1 intensity 0.7 castShadows FALSE }
""")

# =============================================================== atelier
BX0, BX1, BY0, BY1, BH = -8.0, 8.0, -8.5, 7.0, 6.0
W_, D_ = BX1 - BX0, BY1 - BY0
cxb, cyb = (BX0 + BX1) / 2, (BY0 + BY1) / 2
w('DEF SOL Solid { translation %s name "sol atelier" children [ %s ] boundingObject Pose { translation 0 0 -0.05 '
  'children [ Box { size %g %g 0.1 } ] } locked TRUE }' % (
      f((cxb, cyb, 0)), box((W_, D_, 0.1), 'RoughConcrete { colorOverride 0.72 0.76 0.74 textureTransform '
                             'TextureTransform { scale 8 7 } }', (0, 0, -0.05)), W_, D_))
bld = []
clad = 'CorrugatedMetal { colorOverride 0.8 0.82 0.85 textureTransform TextureTransform { scale %g 2 } }'
for (p, s) in (((cxb, BY0), (W_, 0.2)), ((cxb, BY1), (W_, 0.2)), ((BX0, cyb), (0.2, D_)), ((BX1, cyb), (0.2, D_))):
    L = max(s)
    bld.append(box((s[0], s[1], 1.2), app("0.6 0.6 0.58", 0.9), (p[0], p[1], 0.6)))
    bld.append(box((s[0], s[1], BH - 1.2), clad % (L / 2), (p[0], p[1], 1.2 + (BH - 1.2) / 2)))
    bld.append(box((s[0] + 0.01, s[1] + 0.01, 0.5), app("0.75 0.85 0.95", 0.1, 0, emissive="0.35 0.4 0.45"),
                   (p[0], p[1], BH - 0.8)))
COL = app("0.28 0.35 0.45", 0.5, 0.4)
for x in (BX0 + 0.3, 0.0 if False else -2.7, 2.7, BX1 - 0.3):
    for y in (BY0 + 0.3, BY1 - 0.3):
        bld.append(box((0.3, 0.3, BH), COL, (x, y, BH / 2)))
        bld.append(box((0.4, 0.4, 1.0), YEL, (x, y, 0.5)))
for x in (-5.3, -2.7, 0.0, 2.7, 5.3):
    bld.append(box((0.2, D_, 0.3), COL, (x, cyb, BH + 0.15)))
lamp = app("1 0.98 0.92", 0.3, 0, emissive="1 0.97 0.9")
for x in (-4.0, 0.0, 4.0):
    for y in (-5.5, -1.5, 2.5, 5.5):
        bld.append(cyl(0.008, 1.0, DARK, (x, y, BH - 0.5)))
        bld.append(box((1.2, 0.25, 0.08), DARK, (x, y, BH - 1.05)))
        bld.append(box((1.15, 0.2, 0.01), lamp, (x, y, BH - 1.095)))
w(solid("batiment atelier", (0, 0, 0), bld))

marks = []


def floor_line(p0, p1, width, color):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    marks.append(box((math.hypot(dx, dy), width, 0.004), app(color, 0.7), ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2,
                                                                         0.002), (0, 0, 1, math.atan2(dy, dx)), cs=False))


# zone collaborative (scrutateur) : champ de protection (rouge) et d'avertissement (orange), entre les plans de
# façade des tours (le bac d'arrosage et le convoyeur du tour A descendent jusqu'à x = -0.85)
ZX = LATHE_X
for (y0, y1, col) in ((1.40, 2.0, "0.85 0.1 0.1"), (2.0, 2.8, "1 0.55 0")):
    floor_line((-ZX, y0), (ZX, y0), 0.05, col)
    floor_line((-ZX, y1), (ZX, y1), 0.05, col)
    floor_line((-ZX, y0), (-ZX, y1), 0.05, col)
    floor_line((ZX, y0), (ZX, y1), 0.05, col)
# limites de la cellule : les Haas ST-20 vont jusqu'à |x| = 2.99 (bac d'arrosage) et y = ±2.37 (convoyeurs)
CXL, CY0 = 3.2, -2.6
for k in range(12):  # hachures jaune/noir à l'arrière de la cellule
    floor_line((-3.15 + 0.55 * k, CY0), (-2.9 + 0.55 * k, CY0), 0.1, YELLOW)
floor_line((-CXL, 3.2), (CXL, 3.2), 0.1, "0.1 0.65 0.2")
floor_line((-CXL, 3.9), (CXL, 3.9), 0.1, "0.1 0.65 0.2")
floor_line((-CXL, CY0), (-CXL, 3.2), 0.1, YELLOW)
floor_line((CXL, CY0), (CXL, 3.2), 0.1, YELLOW)
w('Solid { name "marquage sol" children [ %s ] locked TRUE }' % render(marks))


# =============================================================== maillages (STEP -> OBJ)
MESH = "../protos/meshes/egk40/%s.obj"
EGK_ROT = (0, 0, 1, -math.pi / 2)                  # EGK axes (y = approach, z = jaws) -> gripper local frame
HAAS_MESH = "../protos/meshes/haas_st20/st20_%s.obj"
HAAS = json.load(open(os.path.join(HERE, "..", "protos", "meshes", "haas_st20", "haas_st20.json")))
# matériaux de la palette du convertisseur : (rugosité, métallicité)
HAAS_PBR = {"white": (0.45, 0.1), "grey": (0.5, 0.35), "steel": (0.3, 0.85), "dark": (0.55, 0.3),
            "black": (0.7, 0.0), "window": (0.08, 0.0)}


def mesh(name, appearance):
    return 'Pose { rotation %s children [ Shape { appearance %s geometry Mesh { url "%s" } } ] }' % (
        f(EGK_ROT), appearance, MESH % name)


SHADOW_MAX_TRI = 21845                             # au-delà, Webots ne projette pas l'ombre d'un Mesh (avertissement)


def haas_mesh(fn):
    """Shape du tour Haas (fichier st20_*.obj, sommets dans le repère de son groupe), apparence selon son matériau.
    Pas d'ombre pour la vitre ni pour les maillages trop lourds (fontes, tôles intérieures, disque de tourelle) :
    l'ombre au sol vient du carénage blanc."""
    info = HAAS["files"][fn]
    rough, metal = HAAS_PBR[info["material"]]
    a = app(f(info["colour"]), rough, metal, transp=info["transparency"])
    cs = info["transparency"] or info["triangles"] > SHADOW_MAX_TRI
    return 'Shape {%s appearance %s geometry Mesh { url "%s" } }' % (
        " castShadows FALSE" if cs else "", a, HAAS_MESH % fn[len("st20_"):-len(".obj")])


def haas_group(group, part=None):
    return [haas_mesh(fn) for fn in sorted(HAAS["files"]) if HAAS["files"][fn]["group"] == group
            and (part is None or HAAS["files"][fn]["part"] == part)]


# cohérence des constantes recopiées à la main avec le maillage généré
_G = HAAS["groups"]
assert all(abs(a - b) < 1e-3 for a, b in zip(DOOR_CLOSED, _G["door"]["origin"]))
assert abs(DOOR_TRAVEL - _G["door"]["travel_open"]) < 1e-3
assert all(abs(a - b) < 1e-3 for a, b in zip(TUR0, _G["turret"]["origin"]))
assert all(abs(a - b) < 1e-3 for a, b in zip(X_DIR, _G["turret"]["x_slide_axis"]))
assert abs(TUR_AXIS_R - _G["turret"]["turret_axis_distance_from_spindle_axis"]) < 1e-3
assert all(abs(a - b) < 1e-3 for j, d in zip(_G["jaws"]["jaws"], JAW_DIRS) for a, b in zip(j["radial_dir_xz"], d))
assert abs(CHUCK_FACE - HAAS["key_positions"]["chuck_body_front_y"]) < 1e-3
assert abs(JAW_FACE - HAAS["key_positions"]["chuck_jaw_front_y"]) < 1e-3


def bx(x0, x1, y0, y1, z0, z1):
    """Boîte de collision donnée par ses bornes."""
    return ("box", (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), None)


def ry_to(d):
    """Rotation autour de y qui amène l'axe x local sur la direction d (dans le plan x-z)."""
    return (0, 1, 0, math.atan2(-d[2], d[0]))


def z_to(d):
    """Rotation qui amène l'axe z local sur la direction d."""
    n = math.sqrt(sum(v * v for v in d))
    d = [v / n for v in d]
    ax = (-d[1], d[0], 0.0)
    s_ = math.hypot(ax[0], ax[1])
    return (ax[0] / s_, ax[1] / s_, 0, math.acos(max(-1.0, min(1.0, d[2])))) if s_ > 1e-9 else (1, 0, 0, 0 if d[2] > 0 else math.pi)


# =============================================================== tour CNC Haas ST-20 (repère local : façade en x = 0
# vers +x, intérieur vers -x, broche selon y, mandrin côté y négatif ; sol machine en z = 0.095 sur une plinthe)
U_TOOL = tuple(-v for v in X_DIR)                  # du centre de tourelle vers la broche (poste d'outil actif)
N_X = (-X_DIR[2], 0.0, X_DIR[0])                   # N = X x y : normale à la glissière dans le plan x-z


def on_x_slide(xc, yc, nc):
    """Point du repère tourelle donné en coordonnées (X glissière, y, normale N = X x y)."""
    return tuple(xc * X_DIR[i] + nc * N_X[i] + (yc if i == 1 else 0.0) for i in range(3))


def slide_box(x0, x1, y0, y1, n0, n1):
    """Boîte alignée sur la glissière X (bornes en X, y, N), repère tourelle / chariot."""
    return ("box", (x1 - x0, y1 - y0, n1 - n0), on_x_slide((x0 + x1) / 2, (y0 + y1) / 2, (n0 + n1) / 2),
            ry_to(X_DIR))


def lathe_bati():
    """Volumes de collision du bâti (repère tour), au plus près des maillages ; l'ouverture de porte (y -0.495..0.197,
    z 0.79..1.975, plan x = -0.107) et l'ouverture du toit qui la prolonge (jusqu'à x = -0.56) restent libres."""
    a = math.atan2(0.39, 0.5)                                       # carter Z incliné (talus à copeaux)
    slant = ("box", (0.634, 0.825, 0.05), (-0.9 - 0.025 * math.sin(a), -0.1425, 0.955 - 0.025 * math.cos(a)),
             (0, 1, 0, a))
    b = math.atan2(1.08, 0.65)                                      # convoyeur à copeaux, partie inclinée
    conv = ("box", (0.52, 1.26, 0.2), (-0.31, 1.725, 1.04), (1, 0, 0, b))
    return [bx(-1.62, -0.06, -1.34, 1.25, 0.0, 0.095),                # plinthe de mise à niveau
            bx(-2.06, -0.06, 0.83, 1.57, 0.0, 0.095),                 # plinthe sous le bac d'arrosage
            bx(-1.60, -0.074, -1.33, 1.225, 0.095, 0.55),             # banc, armoire hydraulique, bac à copeaux
            bx(-0.20, -0.057, -1.33, 1.225, 0.55, 0.79),              # plateaux avant (seuil de porte)
            bx(-0.074, -0.02, -0.92, -0.74, 0.095, 0.32),             # pieds avant
            bx(-0.074, -0.02, 0.65, 0.86, 0.095, 0.32),
            bx(-0.64, -0.20, -0.555, 1.225, 0.55, 0.60),              # bord du bac à copeaux
            bx(-2.06, -0.05, 0.84, 1.557, 0.095, 0.34),               # bac d'arrosage
            bx(-2.19, -1.88, 0.90, 1.50, 0.34, 0.76),                 # pompe d'arrosage
            bx(-1.93, -1.565, 0.85, 1.10, 0.34, 0.54),                # filtre du bac
            bx(-1.60, -0.135, -1.33, -0.555, 0.55, 1.97),             # poupée : tôle avant, cloison, armoire électrique
            bx(-0.60, -0.115, -0.556, -0.528, 1.848, 1.94),           # racleur haut (montant gauche de l'ouverture)
            bx(-0.175, -0.115, -0.556, -0.528, 0.72, 1.94),           # racleur vertical
            bx(-0.60, -0.20, -1.635, -1.33, 0.77, 1.70),              # capot moteur broche + porte d'embarreur
            bx(-1.60, -1.235, -1.52, -1.33, 0.49, 1.70),              # armoire pneumatique
            bx(-1.60, -1.27, -0.555, 1.0, 0.55, 2.03),                # garde arrière
            bx(-1.27, -1.12, -0.555, 0.27, 1.10, 2.03),               # carter télescopique Z (partie verticale)
            slant,                                                    # carter télescopique Z (partie inclinée)
            bx(-1.27, -0.65, -0.555, 0.27, 0.55, 0.75),               # sous le carter incliné
            bx(-1.27, -0.65, 0.27, 1.0, 0.55, 0.72),                  # sous le chariot
            bx(-0.70, -0.65, 0.27, 1.0, 0.72, 0.80),
            bx(-1.20, -1.10, -0.22, 1.06, 0.70, 0.76),                # glissière Z
            bx(-0.545, -0.255, -0.16, 1.225, 0.55, 0.715),            # glissières de contre-pointe
            bx(-0.53, -0.175, -0.555, -0.379, 0.55, 0.91),            # palpeur d'outils : embase
            bx(-0.335, -0.14, -0.465, -0.423, 0.89, 1.275),           # palpeur d'outils : bras
            bx(-0.26, -0.215, -0.555, -0.435, 1.23, 1.325),           # palpeur d'outils : tête
            bx(-0.58, -0.22, 0.388, 0.86, 0.69, 1.165),               # contre-pointe
            bx(-0.44, -0.36, 0.30, 0.39, 1.06, 1.14),                 # pointe tournante
            bx(-0.345, -0.12, 0.95, 1.22, 1.70, 1.89),                # éclairage machine
            bx(-0.12, -0.08, 0.197, 1.225, 0.79, 1.965),              # façade droite (côté contre-pointe)
            bx(-0.65, -0.08, 0.197, 1.225, 1.90, 1.965),              # toit droit
            bx(-0.65, -0.08, 1.15, 1.235, 0.70, 1.965),               # flanc droit
            bx(-0.08, 0.064, 0.29, 0.77, 1.233, 1.761),               # pupitre CNC
            bx(-0.08, 0.035, 0.30, 0.41, 1.15, 1.233),                # articulation du pupitre
            bx(-0.10, 0.035, 0.266, 0.406, 1.761, 2.015),             # câble du pupitre (montée)
            bx(-0.65, -0.10, 0.266, 0.406, 1.94, 2.015),              # câble du pupitre (sur le toit)
            bx(-1.60, -0.555, -1.33, 0.36, 1.90, 2.055),              # toit principal
            bx(-0.71, -0.555, -1.32, 0.30, 2.055, 2.095),             # rail haut de porte
            bx(-1.25, -0.555, 0.279, 0.393, 2.055, 2.09),             # goulottes de câbles
            bx(-0.185, -0.105, 0.379, 1.21, 1.965, 2.005),
            bx(-1.31, -1.155, -0.46, 0.305, 2.055, 2.08),
            bx(-1.465, -1.25, -1.44, -1.33, 1.70, 1.85),
            bx(-1.25, -0.645, 0.34, 1.0, 1.91, 2.05),                 # toit au-dessus du chariot
            bx(-1.25, -0.645, 1.0, 1.665, 0.55, 2.05),                # panneau arrière droit
            bx(-0.57, -0.05, 1.225, 1.60, 0.095, 0.70),               # convoyeur à copeaux : bac
            conv,
            bx(-0.575, -0.05, 1.90, 2.37, 1.10, 1.70),                # convoyeur à copeaux : tête
            bx(-0.18, -0.10, 0.71, 0.79, 1.965, 2.27)]                # colonne lumineuse


def door_col():
    """Porte (repère du groupe porte) : panneau avant, partie toit, poignée."""
    o = DOOR_CLOSED
    return [bx(-0.155 - o[0], -0.065 - o[0], 0.0, 0.809, 0.0, 1.258),
            bx(0.0, -0.155 - o[0], 0.0, 0.809, 1.183, 1.258),
            bx(-0.065 - o[0], -0.024 - o[0], 0.692, 0.732, 0.34, 0.68)]


def lathe(prefix, pos, yaw, dynamic=True):
    fr = frame(pos, (0, 0, 1, yaw))
    col = lathe_bati()
    p = haas_group("static")
    p.append(render([box((1.56, 2.59, 0.095), app("0.32 0.33 0.34", 0.85, 0.1), (-0.84, -0.045, 0.0475)),  # plinthe
                     box((2.0, 0.74, 0.095), app("0.32 0.33 0.34", 0.85, 0.1), (-1.06, 1.20, 0.0475))]))
    # colonne lumineuse sur le toit, au-dessus du pupitre
    lamps = []
    for i, (c, e) in enumerate((("0.1 0.5 0.1", "G"), ("0.6 0.4 0.05", "O"), ("0.5 0.05 0.05", "R"))):
        lamps.append('Pose { translation 0 0 %g children [ Shape { appearance %s geometry Cylinder { radius 0.035 '
                     'height 0.07 } } ] }' % (0.10 + 0.075 * i, app(c, 0.3, 0, emissive="0 0 0", transp=0.1,
                                                                    def_="%s_L%s" % (prefix, e) if dynamic else None)))
    p.append('Pose { translation -0.14 0.75 1.965 children [ %s %s ] }' % (
        render([cyl(0.012, 0.065, DARK, (0, 0, 0.0325)), cyl(0.036, 0.01, DARK, (0, 0, 0.005))]), " ".join(lamps)))
    dyn = []
    if dynamic:
        # porte coulissante (translation selon -y, course DOOR_TRAVEL)
        dcol = door_col()
        dyn.append('DEF %s_DOOR Solid { translation %s name "porte" children [ %s ] boundingObject %s }'
                   % (prefix, f(DOOR_CLOSED), " ".join(haas_group("door")), bo(dcol)))
        dopen = (DOOR_CLOSED[0], DOOR_CLOSED[1] - DOOR_TRAVEL, DOOR_CLOSED[2])
        register(prefix + "_DOOR", dcol, frame(dopen, None, fr), "door_open")
        register(prefix + "_DOOR", dcol, frame(DOOR_CLOSED, None, fr), "door_closed")
        # mandrin 8" (corps + bague de nez) et ses 3 mors (pose CAO = serrage au rayon JAW_R0, translation radiale)
        chuck_fr = frame((CX, CHUCK_FACE, CZ), None, fr)
        jaws = []
        for i, (dx, dz) in enumerate(JAW_DIRS):
            jcol = [("box", (0.079, 0.042, 0.034), (0.0625 * dx, 0.021, 0.0625 * dz), ry_to((dx, 0, dz)))]
            jaws.append('DEF %s_JAW%d Solid { translation 0 0 0 name "mors %d" children [ %s ] boundingObject %s }'
                        % (prefix, i + 1, i + 1, " ".join(haas_group("jaws", "jaw_%d" % (i + 1))), bo(jcol)))
        # mors (ouverts) vus par le vérificateur hors ligne comme un disque r 0.105 devant le corps du mandrin
        register(prefix + "_CHUCK", [("cyl", (0.105, 0.042), (0, 0.021, 0), RX90)], chuck_fr, "jaws")
        ccol = [("cyl", (0.105, 0.117), (0, -0.0585, 0), RX90), ("cyl", (0.127, 0.042), (0, -0.138, 0), RX90)]
        dyn.append('DEF %s_CHUCK Solid { translation %g %g %g name "mandrin" children [ %s %s ] boundingObject %s }' % (
            prefix, CX, CHUCK_FACE, CZ, " ".join(haas_group("spindle")), " ".join(jaws), bo(ccol)))
        register(prefix + "_CHUCK", ccol, chuck_fr, "chuck")
        # chariot Z (translation selon y) portant la glissière X inclinée et la tourelle BOT 12 postes
        park_c = (TUR0[0], TUR0[1] + TUR_PARK[0], TUR0[2])
        park_t = tuple(TUR_PARK[1] * v for v in X_DIR)
        kcol = [slide_box(-0.64, 0.665, -0.046, 0.747, -0.06, 0.865)]
        tcol = [("cyl", (0.241, 0.109), (0, -0.0545, 0), RX90),                          # disque de tourelle
                slide_box(-0.21, 0.641, -0.013, 0.59, -0.18, 0.305),                     # glissière X
                slide_box(-0.11, 0.255, 0.59, 0.731, -0.126, 0.208),
                ("box", (0.10, 0.10, 0.06), (0.255 * U_TOOL[0], -0.078, 0.255 * U_TOOL[2]), ry_to(U_TOOL))]  # outil
        tool = [box((0.06, 0.07, 0.06), DARK, (0.245 * U_TOOL[0], -0.07, 0.245 * U_TOOL[2]), ry_to(U_TOOL)),
                box((0.056, 0.024, 0.02), app("0.35 0.36 0.38", 0.4, 0.8),
                    (0.268 * U_TOOL[0], TOOL_DY + 0.018, 0.268 * U_TOOL[2]), ry_to(U_TOOL)),
                box((0.014, 0.006, 0.014), app("0.85 0.7 0.2", 0.3, 0.8),                # plaquette carbure
                    ((TOOL_R - 0.007) * U_TOOL[0], TOOL_DY + 0.003, (TOOL_R - 0.007) * U_TOOL[2]), ry_to(U_TOOL))]
        # buse d'arrosage et jet (DEF _COOLANT : échelle z 0.001 = coupé, 1 = arrosage) dirigés sur la pointe d'outil
        noz = (0.235 * U_TOOL[0] + 0.035 * N_X[0], TOOL_DY + 0.004, 0.235 * U_TOOL[2] + 0.035 * N_X[2])
        tip = ((TOOL_R - 0.004) * U_TOOL[0], TOOL_DY + 0.004, (TOOL_R - 0.004) * U_TOOL[2])
        jet = [tip[i] - noz[i] for i in range(3)]
        jl = math.sqrt(sum(v * v for v in jet))
        back = tuple(noz[i] - 0.015 * jet[i] / jl for i in range(3))
        tool.append(cyl(0.005, 0.03, app("0.2 0.2 0.22", 0.6), back, z_to(jet)))
        tool.append('DEF %s_COOLANT Transform { translation %s rotation %s scale 1 1 0.001 children [ %s ] }' % (
            prefix, f(noz), f(z_to(jet)), render([cyl(0.004, jl, COOL, (0, 0, jl / 2))])))
        turret = ('DEF %s_TURRET Solid { translation %s name "tourelle" children [ %s %s ] boundingObject %s }' % (
            prefix, f(park_t), " ".join(haas_group("turret")), render(tool), bo(tcol)))
        dyn.append('DEF %s_CARRIAGE Solid { translation %s name "chariot" children [ %s %s ] boundingObject %s }' % (
            prefix, f(park_c), " ".join(haas_group("carriage")), turret, bo(kcol)))
        cfr = frame(park_c, None, fr)
        register(prefix + "_TURRET", kcol, cfr, "turret")
        register(prefix + "_TURRET", tcol, frame(park_t, None, cfr), "turret")
    else:
        # tours de décor : porte fermée, sans pièce mobile
        p.append('Pose { translation %s children [ %s ] }' % (f(DOOR_CLOSED), " ".join(haas_group("door"))))
        col += [(k, d, tuple(c[i] + DOOR_CLOSED[i] for i in range(3)), r) for k, d, c, r in door_col()]
    register(prefix, col, fr, "bati")
    w('DEF %s Solid { translation %s rotation 0 0 1 %g name "tour %s" children [ %s %s ] boundingObject %s }' % (
        prefix, f(pos), yaw, prefix, " ".join(p), " ".join(dyn), bo(col)))


lathe("TOUR_A", (-LATHE_X, 0, 0), 0.0)
lathe("TOUR_B", (LATHE_X, 0, 0), math.pi)
lathe("DECO_1", (-LATHE_X - 3.8, 4.4, 0), 0.0, dynamic=False)
lathe("DECO_2", (LATHE_X + 3.8, 4.4, 0), math.pi, dynamic=False)

# =============================================================== UR10e + capteur d'effort + double pince SCHUNK
EGK_ALU = app("0.80 0.82 0.85", 0.5, 0.45)         # housing: natural anodized aluminium
EGK_JAW = app("0.06 0.06 0.065", 0.45, 0.4)        # base jaws (black, from the STEP colours)
PLATE = app("0.85 0.86 0.88", 0.3, 0.6)
FINGER_COL = app("0.55 0.57 0.6", 0.35, 0.85)


def finger(name, side, g):
    """Base jaw of the SCHUNK EGK 40 (STEP mesh) on a motorized slide (6 mm stroke, 140 N) + custom finger with a
    contact sensor (TouchSensor); the clamping force is read on the jaw motor."""
    geo = DG.finger_geometry(g, side)
    shapes = [mesh("jaw_pos" if side > 0 else "jaw_neg", EGK_JAW)]
    shapes += [box(d, app("0.15 0.15 0.16", 0.7, 0) if k == "pad" else FINGER_COL, p_) for _, d, p_, k in geo]
    fcol = [("box", d, p_, None) for _, d, p_, k in geo if k != "block"]
    return """SliderJoint {
          jointParameters JointParameters { axis 0 0 %d }
          device [ LinearMotor { name "%s" maxVelocity 0.03 maxForce 140 minPosition -0.0005 maxPosition %g }
                   PositionSensor { name "%s_pos" } ]
          endPoint TouchSensor {
            name "%s_effort"
            type "bumper"
            contactMaterial "doigt"
            children [ %s ]
            boundingObject %s
            physics Physics { density -1 mass 0.05 }
          }
        }""" % (-side, name, DG.JAW_STROKE, name, name, render(shapes), bo(fcol))


def tri_prism(p0, p1, p2, z0, z1, appearance):
    """Gousset triangulaire (prisme) dans le plan x-y, entre z0 et z1."""
    pts = [(x, y, z) for z in (z0, z1) for (x, y) in (p0, p1, p2)]
    return ("Shape { appearance %s geometry IndexedFaceSet { coord Coordinate { point [ %s ] } coordIndex [ "
            "0 2 1 -1 3 4 5 -1 0 1 4 3 -1 1 2 5 4 -1 2 0 3 5 -1 ] } }" % (appearance, ", ".join(f(q) for q in pts)))


# --- capteur d'effort 6 axes + plaque d'adaptation + support coudé à 90° (pinces à ±45° de l'axe de la bride)
BRK = app("0.25 0.27 0.3", 0.45, 0.5)
S45 = DG.S
G = [cyl_y(0.045, 0.045, app("0.78 0.8 0.82", 0.3, 0.9), (0, 0.0225, 0)),          # capteur d'effort 6 axes
     cyl_y(0.047, 0.008, SCHUNK_B, (0, 0.03, 0)),
     box((0.16, 0.012, 0.11), ALU, (0, 0.051, 0)),                                  # plaque d'adaptation
     cyl(0.011, 0.11, BRK, (0, DG.APEX_Y - 0.009, 0))]                              # pli du support coudé
gcol = [("cyl", (0.047, 0.05), (0, 0.025, 0), RX90), ("box", (0.16, 0.012, 0.11), (0, 0.051, 0), None)]
for g in (1, 2):
    th = DG.THETA[g]
    d = DG.direction(g)
    fc = DG.FACE[g]
    leg = (fc[0] - 0.007 * d[0], fc[1] - 0.007 * d[1], 0)                           # aile du support (ép. 14 mm)
    G.append(box((0.014, DG.BEND_LEN + 0.012, 0.11), BRK, leg, (0, 0, 1, th)))
    gcol.append(("box", (0.014, DG.BEND_LEN + 0.012, 0.11), leg, (0, 0, 1, th)))
    lo, hi = DG.HOUSING
    body = DG.local_to_slot(g, tuple((lo[i] + hi[i]) / 2 for i in range(3)))
    gcol.append(("box", tuple(hi[i] - lo[i] for i in range(3)), body, (0, 0, 1, th)))
gx = DG.BEND_LEN * S45
for zz in (-0.05, 0.044):                                                           # goussets latéraux
    G.append(tri_prism((-gx, DG.PLATE_TOP), (gx, DG.PLATE_TOP), (0, DG.APEX_Y), zz, zz + 0.006, BRK))
G.append(cyl(0.007, 0.16, app("0.15 0.4 0.85", 0.4, 0), (0.0, 0.075, 0.062), (0, 1, 0, math.pi / 2)))  # air / câbles


def gripper_unit(g):
    """SCHUNK EGK 40-MB-M-B (STEP meshes) in its local frame (x = approach) + fingers + TCP."""
    u = [mesh("housing", EGK_ALU), mesh("plate_1", PLATE), mesh("plate_2", PLATE)]
    tcp = "DEF TCP%d Pose { translation %g 0 0 rotation 0.707107 0 0.707107 3.14159 }" % (g, DG.TCP_X)
    return "Pose { translation %s rotation 0 0 1 %.6f children [ %s %s %s %s ] }" % (
        f(DG.FACE[g]), DG.THETA[g], " ".join(u), finger("g%d_doigt_a" % g, 1, g), finger("g%d_doigt_b" % g, -1, g), tcp)


gripper = """Pose {
  children [
    Solid {
      name "double pince"
      children [ %s ]
      boundingObject %s
      physics Physics { density -1 mass 1.2 }
    }
    %s
    %s
    Emitter { name "robot_tx" channel 2 }
    Receiver { name "plc_rx" channel 1 }
  ]
}""" % (render(G), bo(gcol), gripper_unit(1), gripper_unit(2))
TOOL_COL = gcol
w("""DEF UR10E UR10e {
  translation 0 0 %g
  rotation 0 0 1 %g
  name "UR10e"
  controller "ur_tending"
  supervisor TRUE
  selfCollision TRUE
  toolSlot [ %s ]
}""" % (ROBOT_Z, DG.ROBOT_YAW, gripper))
w(solid("socle robot", (0, 0, 0), def_="SOCLE", children=[cyl(0.16, ROBOT_Z - 0.03, app("0.25 0.27 0.3", 0.5, 0.5), (0, 0, (ROBOT_Z - 0.03) / 2)),
                                   box((0.5, 0.5, 0.025), app("0.25 0.27 0.3", 0.5, 0.5), (0, 0, 0.0125)),
                                   box((0.3, 0.3, 0.02), app("0.25 0.27 0.3", 0.5, 0.5), (0, 0, ROBOT_Z - 0.01))]
            + [cyl(0.015, 0.04, STEEL, (sx * 0.2, sy * 0.2, 0.03)) for sx in (-1, 1) for sy in (-1, 1)],
            bo="Pose { translation 0 0 %g children [ Cylinder { radius 0.16 height %g } ] }" % (ROBOT_Z / 2, ROBOT_Z)))
register("socle robot", [("cyl", (0.16, ROBOT_Z - 0.03), (0, 0, (ROBOT_Z - 0.03) / 2), None)], frame(), "pedestal")

# =============================================================== table à caisses + caisses + pièces
tx, ty, tz = TABLE
tb = [box((1.0, 0.62, 0.02), app("0.55 0.57 0.6", 0.4, 0.7), (tx, ty, tz - 0.01))]
for sx in (-0.47, 0.47):
    for sy in (-0.28, 0.28):
        tb.append(profile((0.045, 0.045, tz - 0.02), (tx + sx, ty + sy, (tz - 0.02) / 2)))
for sy in (-0.28, 0.28):
    tb.append(profile((0.95, 0.045, 0.045), (tx, ty + sy, 0.15)))
for sx in (-0.2, 0.2):
    tb.append(box((0.42, 0.02, 0.03), YEL, (tx + sx, ty - 0.31, tz + 0.015)))   # butées de positionnement
tcol = [("box", (1.0, 0.62, 0.02), (tx, ty, tz - 0.01), None)]
w(solid("table a caisses", (0, 0, 0), tb, def_="TABLE", bo=bo(tcol)))
register("TABLE", tcol, frame(), "table")


def crate(cx_, cy_, z0, col, name):
    """Bac plastique 400x600x180 (type KLT) : fond, parois nervurées, rebord, poignées."""
    pa = app(col, 0.55, 0)
    dk = app(" ".join("%.3f" % (0.6 * float(c)) for c in col.split()), 0.6, 0)
    L, W, H, t = 0.4, 0.6, 0.18, 0.008
    c = [box((L, W, 0.006), pa, (cx_, cy_, z0 + 0.003))]
    for sx in (-1, 1):
        c.append(box((t, W, H), pa, (cx_ + sx * (L - t) / 2, cy_, z0 + H / 2)))
        c.append(box((0.02, W + 0.02, 0.012), pa, (cx_ + sx * (L / 2 + 0.002), cy_, z0 + H - 0.006)))  # rebord
        for k in range(3):
            c.append(box((0.006, W - 0.04, 0.008), dk, (cx_ + sx * (L / 2 + 0.003), cy_, z0 + 0.03 + 0.045 * k)))
    for sy in (-1, 1):
        c.append(box((L, t, H), pa, (cx_, cy_ + sy * (W - t) / 2, z0 + H / 2)))
        c.append(box((L + 0.02, 0.02, 0.012), pa, (cx_, cy_ + sy * (W / 2 + 0.002), z0 + H - 0.006)))
        c.append(box((0.12, 0.01, 0.035), app("0.05 0.05 0.06", 0.8), (cx_, cy_ + sy * (W / 2 + 0.001), z0 + H - 0.04)))
        for k in range(3):
            c.append(box((L - 0.04, 0.006, 0.008), dk, (cx_, cy_ + sy * (W / 2 + 0.003), z0 + 0.03 + 0.045 * k)))
    ccol = [("box", (L, W, 0.006), (cx_, cy_, z0 + 0.003), None)]
    for sx in (-1, 1):
        ccol.append(("box", (t + 0.02, W + 0.02, H), (cx_ + sx * (L / 2 - 0.002), cy_, z0 + H / 2), None))
    for sy in (-1, 1):
        ccol.append(("box", (L + 0.02, t + 0.02, H), (cx_, cy_ + sy * (W / 2 - 0.002), z0 + H / 2), None))
    w(solid("caisse %s" % name, (0, 0, 0), c, def_="CAISSE_" + name, bo=bo(ccol)))
    register("CAISSE_" + name, ccol[:1], frame(), "crate_floor")
    register("CAISSE_" + name, ccol[1:], frame(), "crate")


for name, (cx_, cy_) in CRATES.items():
    col = "0.1 0.3 0.75" if name == "BRUTS" else "0.25 0.55 0.3"
    crate(cx_, cy_, tz, col, name)
    dv = []
    for xx in (-0.06, 0.06):
        dv.append(box((0.004, 0.56, 0.05), app(col, 0.6), (cx_ + xx, cy_, tz + 0.03)))
    for yy in (-0.13, 0.0, 0.13):
        dv.append(box((0.37, 0.004, 0.05), app(col, 0.6), (cx_, cy_ + yy, tz + 0.03)))
    dcol = [("box", (0.004, 0.56, 0.05), (cx_ + xx, cy_, tz + 0.03), None) for xx in (-0.06, 0.06)]
    dcol += [("box", (0.37, 0.004, 0.05), (cx_, cy_ + yy, tz + 0.03), None) for yy in (-0.13, 0.0, 0.13)]
    w(solid("alveoles %s" % name.lower(), (0, 0, 0), dv, def_="ALVEOLES_" + name, bo=bo(dcol)))
    register("ALVEOLES_" + name, dcol, frame(), "divider")


def part(k, pos, rot, finished):
    s = 0.001 if finished else 0.9999      # != 1 : sinon Webots convertit le Transform en Pose (plus de champ scale)
    r_grip = 0.015 if finished else PART_R
    return """DEF PART_%d Solid {
  translation %s
  rotation %s
  name "piece %d"
  children [
    %s
    DEF SLV_%d Transform { translation 0 0 %g scale 1 1 %g children [ %s ] }
  ]
  boundingObject Group { children [
    Pose { translation 0 0 -0.0225 children [ DEF PBA_%d Cylinder { radius 0.025 height 0.045 } ] }
    Pose { translation 0 0 0.0225 children [ DEF PBO_%d Cylinder { radius %g height 0.045 } ] } ] }
}""" % (k, f(pos), f(rot), k,
        render([cyl(PART_R, 0.045, RAW, (0, 0, -0.0225)), cyl(0.0185, 0.025, MACH, (0, 0, 0.0125)),
                cyl(0.0125, 0.02, MACH, (0, 0, 0.035)), cyl(0.008, 0.0015, app("0.6 0.62 0.65", 0.3, 1), (0, 0, 0.045))]),
        k, 0.0225 * s, s, render([cyl(0.0251, 0.045, RAW, (0, 0, 0))]), k, k, r_grip)


def cell_pos(crate, i):
    cx_, cy_ = CRATES[crate]
    return (cx_ + CELL_X[i % 3], cy_ + CELL_Y[i // 3], tz + 0.0063 + PART_L / 2)


for k in range(1, 17):
    if k <= 6:
        w(part(k, cell_pos("BRUTS", 5 + k), (0, 0, 1, 0), False))         # alvéoles 6..11 de la caisse des bruts
    elif k <= 12:
        w(part(k, cell_pos("FINIES", k - 7), (0, 0, 1, 0), True))         # alvéoles 0..5 de la caisse des finies
    else:
        w(part(k, (3.0 + 0.1 * k, -7.5, -1.0), (0, 0, 1, 0), False))     # réserve hors champ

# =============================================================== carter arrière, scrutateurs, armoire robot, AU
for i in range(2):
    w('GuardBay { translation %s name "carter arriere %d" width %g height 2.0%s }' % (
        f((-LATHE_X + LATHE_X * i, -1.1, 0)), i, LATHE_X, " rightPost TRUE" if i == 1 else ""))
w(solid("armoire UR + pupitre", (-1.25, 2.75, 0), [            # derrière le convoyeur à copeaux du tour A
    profile((0.04, 0.04, 1.0), (0, 0, 0.5)), box((0.5, 0.3, 0.04), ALU, (0, 0, 0.02)),
    box((0.47, 0.25, 0.45), app("0.85 0.86 0.86", 0.4, 0.2), (0, 0, 0.85)),
    box((0.3, 0.05, 0.2), app("0.2 0.2 0.22", 0.5, 0.2), (0.0, -0.15, 1.2), (1, 0, 0, 0.3)),
    box((0.22, 0.01, 0.13), app("0.1 0.2 0.35", 0.2, 0, emissive="0.2 0.35 0.55"), (0.0, -0.18, 1.21), (1, 0, 0, 0.3)),
    cyl(0.02, 0.02, RED, (0.11, -0.18, 1.28), (1, 0, 0, 1.87))]))
for (x, y, yaw) in ((-0.55, 1.18, 0), (0.55, 1.18, 0)):
    w(solid("AU table %g" % x, (x, y, 0.78), [box((0.08, 0.08, 0.08), YEL), cyl(0.03, 0.04, RED, (0, 0.05, 0), (1, 0, 0, 1.5708))]))
w('FireExtinguisher { translation -2.2 2.0 0 name "extincteur" }')
# opérateur (déplacé et animé par l'automate pour l'échange des caisses)
w('DEF OPERATEUR Pedestrian { translation 3.6 3.55 1.27 rotation 0 0 1 3.14159 name "operateur" controller "<none>" '
  'controllerArgs [ "anime" ] shirtColor 0.1 0.3 0.7 pantsColor 0.15 0.15 0.18 enableBoundingObject TRUE }')
# décor atelier : rack à barres, établi, armoire à outils, bacs à copeaux
dec = []
for k in range(5):
    dec.append(box((0.08, 0.6, 1.6), app("0.08 0.25 0.55", 0.5, 0.3), (-6.5 + 0.0 * k, -6.0 + 0.7 * k, 0.8)))
for k in range(12):
    dec.append(cyl_y(0.025 + 0.004 * (k % 3), 3.0, RAW, (-6.3, -4.6, 0.45 + 0.09 * k)))
dec.append(box((1.8, 0.8, 0.05), app("0.55 0.4 0.25", 0.8), (5.5, -6.0, 0.9)))
dec.append(box((1.8, 0.8, 0.85), app("0.3 0.35 0.45", 0.5, 0.3), (5.5, -6.0, 0.43)))
dec.append(box((1.0, 0.55, 1.9), app("0.85 0.2 0.15", 0.5, 0.2), (3.5, -7.9, 0.95)))
for k in range(4):
    dec.append(box((0.98, 0.01, 0.005), DARK, (3.5, -7.62, 0.3 + 0.4 * k)))
w(solid("decor atelier", (0, 0, 0), dec))

# =============================================================== automate de cellule (scrutateurs = faisceaux)
def dsensor(name, pos, yaw, rng):
    return ('DistanceSensor { translation %s rotation 0 0 1 %g name "%s" lookupTable [ 0 0 0, %g %g 0 ] type "generic" }'
            % (f(pos), yaw, name, rng, rng))


dev = []
for lname, lpos in (("scrutateur_gauche", (-0.45, 1.2, 0.18)), ("scrutateur_droit", (0.45, 1.2, 0.18))):
    for kk in range(31):
        dev.append(dsensor("%s_%02d" % (lname, kk), lpos, math.pi * kk / 30, 4.0))
    dev.append(solid("tete " + lname, lpos, [cyl(0.06, 0.15, YEL), cyl(0.05, 0.04, BLK, (0, 0, 0.09))]))
dev.append('Emitter { name "plc_tx" channel 1 }')
dev.append('Receiver { name "robot_rx" channel 2 }')
w("""DEF API Robot {
  name "automate cellule"
  controller "cell_plc"
  supervisor TRUE
  children [
    %s
  ]
}""" % "\n    ".join(dev))

# =============================================================== marqueurs de collision (déplacés par l'automate
# sur les points de contact du robot ; créés d'avance pour apparaître aussi dans l'animation HTML5)
for k in range(8):
    w('DEF COLL_%d Pose { translation %g -7.9 -2 children [ Shape { castShadows FALSE appearance PBRAppearance { '
      'baseColor 1 0 0 emissiveColor 1 0.1 0.1 transparency 0.35 roughness 1 metalness 0 } geometry Sphere { '
      'radius 0.035 subdivision 2 } } ] }' % (k, 0.2 * k))

# obstacles pour le vérificateur hors ligne (pièces en alvéoles comprises)
for crate_name, cells in (("BRUTS", range(12)), ("FINIES", range(12))):
    for i in cells:
        c = cell_pos(crate_name, i)
        OBST.append(dict(owner="piece %s %d" % (crate_name, i), tag="part", kind="cyl", dims=[PART_R, PART_L], c=list(c),
                         R=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], cell=[crate_name, i]))
with open(os.path.join(HERE, "..", "worlds", "obstacles.json"), "w") as fh:
    json.dump(dict(obstacles=OBST, tool=[dict(kind=k, dims=list(d), pos=list(p_), rot=r) for k, d, p_, r in TOOL_COL],
                   lathe_x=LATHE_X, tur_park=[TUR0[0], TUR0[1] + TUR_PARK[0], TUR0[2]]), fh, indent=0)

with open(OUT, "w") as fh:
    fh.write("\n".join(out) + "\n")
print("écrit", os.path.abspath(OUT), len(out), "noeuds")
