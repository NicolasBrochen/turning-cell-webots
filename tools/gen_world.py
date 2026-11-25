#!/usr/bin/env python3
"""Génère worlds/tournage.wbt : cellule de tournage robotisée (2 tours CNC face à face + UR10e).

Implantation (m, ENU, z vers le haut) :
  - UR10e sur socle en (0, 0), base à 0.80 m. Outil : capteur d'effort 6 axes + double pince SCHUNK (2 pinces
    parallèles opposées, doigts motorisés avec capteurs de force au bout des doigts).
  - Tour A : façade en x = -0.65 tournée vers +x. Tour B : façade en x = +0.65 tournée vers -x (face à face).
    Broche horizontale parallèle à la façade (axe y) ; mandrin 3 mors, porte coulissante automatique.
  - Table à caisses côté opérateur (y ≈ 0.85) : caisse des bruts (x = -0.25) et caisse des pièces finies (x = +0.25),
    12 alvéoles chacune (3 x 4).
  - Côté opérateur ouvert (zone collaborative) surveillé par 2 scrutateurs laser ; carter fixe à l'arrière.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wbtgen import (ALU, BLK, BLUE, DARK, PC, RED, STEEL, YEL, YELLOW, BLACK, app, box, cyl, cyl_x, cyl_y,  # noqa
                    f, out, profile, render, solid, w)

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
LATHE_X = 0.65                       # plan des façades
CX, CZ = -0.40, 1.10                 # axe de broche (repère tour)
CHUCK_Y = -0.46                      # centre du mandrin (corps de y -0.52 à -0.40)
JAW_FACE = -0.355                    # face avant des mors
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


# zone collaborative (scrutateur) : champ de protection (rouge) et d'avertissement (orange)
for (y0, y1, col) in ((1.40, 2.0, "0.85 0.1 0.1"), (2.0, 2.8, "1 0.55 0")):
    floor_line((-0.9, y0), (0.9, y0), 0.05, col)
    floor_line((-0.9, y1), (0.9, y1), 0.05, col)
    floor_line((-0.9, y0), (-0.9, y1), 0.05, col)
    floor_line((0.9, y0), (0.9, y1), 0.05, col)
for k in range(9):  # hachures jaune/noir autour de la cellule
    floor_line((-2.5 + 0.55 * k, -1.6), (-2.25 + 0.55 * k, -1.6), 0.1, YELLOW)
floor_line((-2.6, 3.2), (2.6, 3.2), 0.1, "0.1 0.65 0.2")
floor_line((-2.6, 3.9), (2.6, 3.9), 0.1, "0.1 0.65 0.2")
floor_line((-2.6, -1.6), (-2.6, 3.2), 0.1, YELLOW)
floor_line((2.6, -1.6), (2.6, 3.2), 0.1, YELLOW)
w('Solid { name "marquage sol" children [ %s ] locked TRUE }' % render(marks))


# =============================================================== tour CNC (repère local : façade en x = 0 vers +x,
# broche selon y, mandrin côté y négatif)
def lathe(prefix, pos, yaw, dynamic=True):
    p = []
    y0, y1, xb = -1.05, 1.35, -1.70
    L, cy_ = y1 - y0, (y0 + y1) / 2
    p.append(box((1.66, L - 0.04, 0.78), ANTHRA, (-0.85, cy_, 0.39)))                      # socle / banc
    p.append(box((1.6, L - 0.1, 0.06), BLK, (-0.82, cy_, 0.03)))                          # plinthe
    # carénage
    p.append(box((0.04, L, 1.3), WHITE, (xb + 0.02, cy_, 1.43)))                           # dos
    for yy in (y0, y1):
        p.append(box((-xb, 0.04, 1.3), WHITE, (xb / 2, yy, 1.43)))                         # flancs
    p.append(box((-xb, L, 0.04), WHITE, (xb / 2, cy_, 2.08)))                              # toit
    # façade autour de l'ouverture de porte (y -0.70 .. 0.30, z 0.85 .. 1.67)
    p.append(box((0.04, -0.72 - y0, 1.3), WHITE, (-0.02, (y0 - 0.72) / 2, 1.43)))
    p.append(box((0.04, y1 - 0.32, 1.3), WHITE, (-0.02, (y1 + 0.32) / 2, 1.43)))
    p.append(box((0.04, 1.04, 0.38), WHITE, (-0.02, -0.2, 1.88)))
    p.append(box((0.04, 1.04, 0.1), WHITE, (-0.02, -0.2, 0.83)))
    p.append(box((0.05, L, 0.16), ANTHRA, (-0.02, cy_, 0.86)))                             # bandeau bas
    p.append(box((0.05, L, 0.05), app("0.1 0.35 0.75", 0.4, 0.2), (-0.02, cy_, 2.0)))      # liseré
    p.append(box((0.06, 1.1, 0.03), ALU, (0.02, 0.35, 1.72)))                              # rail de porte
    # intérieur visible : bac à copeaux, poupée, broche, banc incliné
    p.append(box((1.3, 1.95, 0.02), app("0.18 0.19 0.2", 0.7, 0.4), (-0.75, -0.1, 0.86)))
    p.append(box((1.1, 0.5, 0.75), LGREY, (-0.85, -0.78, 1.25)))
    p.append(cyl_y(0.13, 0.05, LGREY, (CX, -0.545, CZ)))
    p.append(box((0.5, 1.6, 0.06), ANTHRA, (-0.95, 0.15, 1.15), (0, 1, 0, -math.pi / 4)))   # banc incliné 45°
    p.append(box((0.04, 1.9, 0.02), LED, (-1.5, -0.15, 2.0)))                              # éclairage machine
    # pupitre CNC (à gauche de la porte)
    p.append(box((0.1, 0.32, 0.75), ANTHRA, (0.07, -0.88, 1.45)))
    p.append(box((0.01, 0.26, 0.22), app("0.1 0.25 0.45", 0.2, 0, emissive="0.15 0.35 0.6"), (0.125, -0.88, 1.62)))
    for i in range(4):
        for j in range(5):
            p.append(box((0.01, 0.035, 0.02), app("0.75 0.75 0.75", 0.5), (0.125, -0.96 + 0.04 * j, 1.42 - 0.03 * i)))
    p.append(cyl_x(0.02, 0.02, RED, (0.13, -0.78, 1.25)))
    p.append(cyl_x(0.012, 0.015, app("0.1 0.8 0.2", 0.4), (0.13, -0.98, 1.25)))
    # convoyeur à copeaux + bac
    p.append(box((0.45, 0.6, 0.35), LGREY, (-0.7, y1 + 0.3, 0.45), (1, 0, 0, -0.35)))
    p.append(box((0.6, 0.5, 0.45), app("0.1 0.3 0.7", 0.5, 0.2), (-0.7, y1 + 0.75, 0.25)))
    # interrupteur de porte (interverrouillage) + capotage moteur broche
    p.append(box((0.03, 0.05, 0.08), YEL, (0.02, -0.74, 1.25)))
    dyn = []
    if dynamic:
        door = [box((0.03, 1.04, 0.16), WHITE, (0, 0, 0.08)), box((0.03, 1.04, 0.14), WHITE, (0, 0, 0.81)),
                box((0.03, 0.16, 0.6), WHITE, (0, -0.44, 0.45)), box((0.03, 0.16, 0.6), WHITE, (0, 0.44, 0.45)),
                box((0.012, 0.74, 0.6), PC, (0, 0, 0.45), cs=False),
                box((0.03, 0.04, 0.4), ALU, (0.035, 0.42, 0.45)), box((0.02, 0.02, 0.06), ALU, (0.02, 0.42, 0.62)),
                box((0.02, 0.02, 0.06), ALU, (0.02, 0.42, 0.28))]
        dyn.append('DEF %s_DOOR Pose { translation 0.04 -0.2 0.84 children [ %s ] }' % (prefix, render(door)))
        jaws = []
        for i in range(3):
            a = 2 * math.pi * i / 3
            jaws.append('DEF %s_JAW%d Pose { translation %g 0.0825 %g rotation 0 1 0 %g children [ %s ] }' % (
                prefix, i + 1, 0.07 * math.cos(a), 0.07 * math.sin(a), -a,
                render([box((0.04, 0.045, 0.03), STEEL), box((0.004, 0.045, 0.026), app("0.3 0.3 0.32", 0.4, 0.8),
                                                          (-0.02, 0, 0))])))
        chuck = [cyl_y(0.105, 0.12, STEEL, (0, 0, 0)), cyl_y(0.03, 0.122, DARK, (0, 0, 0))]
        for i in range(3):
            a = 2 * math.pi * i / 3 + math.pi / 3
            chuck.append(box((0.03, 0.005, 0.012), DARK, (0.08 * math.cos(a), 0.061, 0.08 * math.sin(a))))
        dyn.append('DEF %s_CHUCK Pose { translation %g %g %g children [ %s %s ] }' % (
            prefix, CX, CHUCK_Y, CZ, render(chuck), " ".join(jaws)))
        tur = [cyl_y(0.16, 0.09, LGREY, (0, 0, 0)), cyl_y(0.06, 0.1, DARK, (0, 0, 0)),
               box((0.3, 0.2, 0.3), ANTHRA, (-0.12, 0.06, 0.12))]
        for i in range(8):
            a = -math.pi / 4 + 2 * math.pi * i / 8
            c, s_ = math.cos(a), math.sin(a)
            tur.append(box((0.09, 0.05, 0.05), DARK, (0.2 * c, 0, 0.2 * s_), (0, 1, 0, -a)))
            if i == 0:   # outil actif : porte-plaquette + plaquette carbure
                tur.append(box((0.07, 0.03, 0.025), STEEL, (0.27 * c, -0.01, 0.27 * s_), (0, 1, 0, -a)))
                tur.append(box((0.015, 0.012, 0.012), app("0.85 0.7 0.2", 0.3, 0.8), (0.31 * c, -0.02, 0.31 * s_),
                               (0, 1, 0, -a)))
        tur.append(cyl(0.006, 0.25, app("0.2 0.2 0.22", 0.6), (0.05, 0.05, 0.25)))           # buse arrosage
        tur.append('DEF %s_COOLANT Transform { translation 0.16 -0.02 -0.05 rotation 0 1 0 0.85 scale 1 1 0.001 '
                   'children [ %s ] }' % (prefix, render([cyl(0.004, 0.22, COOL, (0, 0, 0))])))
        dyn.append('DEF %s_TURRET Pose { translation -0.72 0.10 1.42 children [ %s ] }' % (prefix, render(tur)))
        lamps = []
        for i, (c, e) in enumerate((("0.1 0.5 0.1", "G"), ("0.6 0.4 0.05", "O"), ("0.5 0.05 0.05", "R"))):
            lamps.append('Pose { translation 0 0 %g children [ Shape { appearance %s geometry Cylinder { radius 0.04 '
                         'height 0.08 } } ] }' % (0.08 + 0.085 * i, app(c, 0.3, 0, emissive="0 0 0", transp=0.1,
                                                                         def_="%s_L%s" % (prefix, e))))
        dyn.append('Pose { translation -0.1 -0.98 2.1 children [ %s %s ] }' % (
            render([cyl(0.015, 0.04, DARK, (0, 0, 0.02))]), " ".join(lamps)))
    else:
        p.append(box((0.03, 1.04, 0.82), WHITE, (0.04, -0.2, 1.25)))
        p.append(box((0.012, 0.74, 0.6), PC, (0.06, -0.2, 1.29), cs=False))
    w('DEF %s Solid { translation %s rotation 0 0 1 %g name "tour %s" children [ %s %s ] }' % (
        prefix, f(pos), yaw, prefix, render(p), " ".join(dyn)))


lathe("TOUR_A", (-LATHE_X, 0, 0), 0.0)
lathe("TOUR_B", (LATHE_X, 0, 0), math.pi)
lathe("DECO_1", (-LATHE_X - 3.8, 4.4, 0), 0.0, dynamic=False)
lathe("DECO_2", (LATHE_X + 3.8, 4.4, 0), math.pi, dynamic=False)

# =============================================================== UR10e + capteur d'effort + double pince SCHUNK
def finger(name, x0, side):
    """Doigt de pince parallèle : glissière motorisée + TouchSensor 3 axes (effort au bout du doigt)."""
    zs = 0.045 * side
    pad_z = -0.009 * side
    shapes = render([box((0.075, 0.03, 0.012), app("0.55 0.57 0.6", 0.35, 0.85), (0.0375, 0, 0)),
                     box((0.02, 0.026, 0.006), app("0.15 0.15 0.16", 0.7, 0), (0.045, 0, pad_z)),
                     box((0.03, 0.034, 0.02), SCHUNK_B, (-0.005, 0, side * 0.004))])
    return """SliderJoint {
          jointParameters JointParameters { axis 0 0 %d }
          device [ LinearMotor { name "%s" maxVelocity 0.12 maxForce 140 minPosition -0.001 maxPosition 0.04 }
                   PositionSensor { name "%s_pos" } ]
          endPoint TouchSensor {
            translation %g 0.082 %g
            name "%s_effort"
            type "bumper"
            contactMaterial "doigt"
            children [ %s ]
            boundingObject Pose { translation 0.045 0 %g children [ Box { size 0.02 0.026 0.006 } ] }
            physics Physics { density -1 mass 0.06 }
          }
        }""" % (-side, name, name, x0, zs, name, shapes, pad_z)


G = [cyl_y(0.045, 0.045, app("0.78 0.8 0.82", 0.3, 0.9), (0, 0.0225, 0)),          # capteur d'effort 6 axes
     cyl_y(0.047, 0.008, SCHUNK_B, (0, 0.03, 0)),
     box((0.12, 0.012, 0.12), ALU, (0, 0.051, 0)),
     box((0.36, 0.05, 0.07), app("0.25 0.27 0.3", 0.45, 0.5), (0, 0.082, 0)),       # adaptateur double pince
     cyl(0.008, 0.08, app("0.15 0.4 0.85", 0.4, 0), (0.06, 0.11, 0.0), (0, 0, 1, 0)),  # tuyau d'alimentation
     ]
for sx in (1, -1):  # corps des 2 pinces parallèles (type SCHUNK EGK) + plaque d'identification
    G.append(box((0.12, 0.065, 0.09), SCHUNK, (sx * 0.23, 0.082, 0)))
    G.append(box((0.02, 0.07, 0.095), SCHUNK_B, (sx * 0.175, 0.082, 0)))
    G.append(box((0.04, 0.002, 0.03), app("0.1 0.3 0.75", 0.4, 0.2), (sx * 0.23, 0.0485 + 0.067, 0)))
    G.append(box((0.01, 0.05, 0.08), SCHUNK_B, (sx * 0.285, 0.082, 0)))
    G.append(cyl(0.006, 0.01, app("0.1 0.9 0.2", emissive="0 0.6 0"), (sx * 0.2, 0.116, 0.03)))
TCP1 = "DEF TCP1 Pose { translation 0.335 0.082 0 rotation 0.707107 0 0.707107 3.14159 }"
TCP2 = "DEF TCP2 Pose { translation -0.335 0.082 0 rotation 0 1 0 -1.5708 }"
gripper = """Pose {
  children [
    %s
    %s
    %s
    %s
    %s
    %s
    %s
    Emitter { name "robot_tx" channel 2 }
    Receiver { name "plc_rx" channel 1 }
  ]
}""" % (render(G), finger("g1_doigt_a", 0.29, 1), finger("g1_doigt_b", 0.29, -1),
        finger("g2_doigt_a", -0.29, 1).replace("translation -0.29 0.082", "rotation 0 0 1 3.14159 translation -0.29 0.082"),
        finger("g2_doigt_b", -0.29, -1).replace("translation -0.29 0.082", "rotation 0 0 1 3.14159 translation -0.29 0.082"),
        TCP1, TCP2)
w("""DEF UR10E UR10e {
  translation 0 0 %g
  name "UR10e"
  controller "ur_tending"
  supervisor TRUE
  selfCollision FALSE
  toolSlot [ %s ]
}""" % (ROBOT_Z, gripper))
w(solid("socle robot", (0, 0, 0), [cyl(0.16, ROBOT_Z - 0.03, app("0.25 0.27 0.3", 0.5, 0.5), (0, 0, (ROBOT_Z - 0.03) / 2)),
                                   box((0.5, 0.5, 0.025), app("0.25 0.27 0.3", 0.5, 0.5), (0, 0, 0.0125)),
                                   box((0.3, 0.3, 0.02), app("0.25 0.27 0.3", 0.5, 0.5), (0, 0, ROBOT_Z - 0.01))]
            + [cyl(0.015, 0.04, STEEL, (sx * 0.2, sy * 0.2, 0.03)) for sx in (-1, 1) for sy in (-1, 1)],
            bo="Pose { translation 0 0 %g children [ Cylinder { radius 0.16 height %g } ] }" % (ROBOT_Z / 2, ROBOT_Z)))

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
w(solid("table a caisses", (0, 0, 0), tb))
def crate(cx_, cy_, z0, col, name):
    """Bac plastique 400x600x180 (type KLT) : fond, parois nervurées, rebord, poignées (visuel, sans collision)."""
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
    w(solid("caisse %s" % name, (0, 0, 0), c, def_="CAISSE_" + name))


for name, (cx_, cy_) in CRATES.items():
    col = "0.1 0.3 0.75" if name == "BRUTS" else "0.25 0.55 0.3"
    crate(cx_, cy_, tz, col, name)
    dv = []
    for xx in (-0.06, 0.06):
        dv.append(box((0.004, 0.56, 0.05), app(col, 0.6), (cx_ + xx, cy_, tz + 0.03)))
    for yy in (-0.13, 0.0, 0.13):
        dv.append(box((0.37, 0.004, 0.05), app(col, 0.6), (cx_, cy_ + yy, tz + 0.03)))
    w(solid("alveoles %s" % name.lower(), (0, 0, 0), dv))


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
    w('GuardBay { translation %s name "carter arriere %d" width 0.65 height 2.0%s }' % (
        f((-0.65 + 0.65 * i, -1.1, 0)), i, " rightPost TRUE" if i == 1 else ""))
w(solid("armoire UR + pupitre", (-1.25, 1.75, 0), [
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

with open(OUT, "w") as fh:
    fh.write("\n".join(out) + "\n")
print("écrit", os.path.abspath(OUT), len(out), "noeuds")
