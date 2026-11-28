"""Generate protos/HaasST20.proto from the same helpers / constants as tools/gen_world.py (read-only reuse).

    python3 tools/gen_st20_proto.py
"""
import ast, json, math, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from wbtgen import app, box, cyl, f, render, DARK, _rotm  # noqa
src = open(os.path.join(ROOT, "tools", "gen_world.py")).read()
tree = ast.parse(src)
ns = dict(math=math, app=app, box=box, cyl=cyl, f=f, render=render, DARK=DARK, _rotm=_rotm)
CONSTS = {"CX", "CZ", "CHUCK_FACE", "JAW_FACE", "DOOR_CLOSED", "DOOR_TRAVEL", "JAW_DIRS", "JAW_R0", "TUR0", "X_DIR",
          "TUR_AXIS_R", "TOOL_R", "TOOL_DY", "TOOL_EXT", "HAAS_PBR", "SHADOW_MAX_TRI", "U_TOOL", "N_X", "RX90", "TOOL_STATIONS",
          "COOL", "CATCH_PIVOT", "CATCH_L", "CATCH_W", "CATCH_STOW", "CATCH_TRAVEL", "CHUTE", "BIN", "AIR_NOZZLE",
          "AIR_TARGET", "DOOR_ACT", "DOOR_ROD", "DOOR_LUG", "DOOR_SW"}
FUNCS = {"bx", "ry_to", "z_to", "on_x_slide", "slide_box", "lathe_bati", "door_col", "station_rot", "tool_col",
         "catcher_col", "options_col"}
for node in tree.body:
    if isinstance(node, ast.Assign):
        names = [t.id for t in node.targets if isinstance(t, ast.Name)] + \
                [e.id for t in node.targets if isinstance(t, ast.Tuple) for e in t.elts]
        if set(names) & CONSTS:
            exec(compile(ast.Module([node], []), "gen_world", "exec"), ns)
    elif isinstance(node, ast.FunctionDef) and node.name in FUNCS:
        exec(compile(ast.Module([node], []), "gen_world", "exec"), ns)
g = type("G", (), ns)
HAAS = json.load(open(os.path.join(ROOT, "protos/meshes/haas_st20/haas_st20.json")))


def haas_mesh(fn):
    info = HAAS["files"][fn]
    rough, metal = ns["HAAS_PBR"][info["material"]]
    a = app(f(info["colour"]), rough, metal, transp=info["transparency"])
    cs = info["transparency"] or info["triangles"] > ns["SHADOW_MAX_TRI"]
    return 'Shape {%s appearance %s geometry Mesh { url "meshes/haas_st20/%s" } }' % (
        " castShadows FALSE" if cs else "", a, fn)


def group(gr, part=None):
    return "\n      ".join(haas_mesh(fn) for fn in sorted(HAAS["files"]) if HAAS["files"][fn]["group"] == gr
                           and (part is None or HAAS["files"][fn]["part"] == part))


def bo(items, ind="      ", dz=False, wrap=True):
    s_ = []
    for kind, dims, pos, rot in items:
        if dz:
            pos = "%g %g %%<= %g + dz >%%" % tuple(pos)
        geo = "Box { size %s }" % f(dims) if kind == "box" else "Cylinder { radius %g height %g }" % dims
        s_.append("Pose { translation %s%s children [ %s ] }" % (pos if dz else f(pos), " rotation %s" % f(rot) if rot else "", geo))
    if not wrap:
        return ("\n" + ind).join(s_)
    return "Group { children [\n%s  %s\n%s] }" % (ind, ("\n  " + ind).join(s_), ind)


DOOR_CLOSED, TUR0, X_DIR, U_TOOL, N_X = g.DOOR_CLOSED, g.TUR0, g.X_DIR, g.U_TOOL, g.N_X
CX, CZ, CHUCK_FACE, TOOL_R, TOOL_DY, RX90 = g.CX, g.CZ, g.CHUCK_FACE, g.TOOL_R, g.TOOL_DY, g.RX90
bati = g.lathe_bati(options=False)               # options: own boundingObjects, only when the option is fitted
plinth_col, bati_col = bati[:2], bati[2:]
OPT_COL = g.options_col()
GROUND = app("0.32 0.33 0.34", 0.85, 0.1)
plinth_vis = render([box((1.56, 2.59, 0.095), GROUND, (-0.84, -0.045, 0.0475)),
                     box((2.0, 0.74, 0.095), GROUND, (-1.06, 1.20, 0.0475))])
lamps = []
for i, (c, on, nm) in enumerate((("0.1 0.5 0.1", "0 1 0", "green"), ("0.6 0.4 0.05", "1 0.55 0", "orange"),
                                 ("0.5 0.05 0.05", "1 0 0", "red"))):
    # emissiveColor IS <colour>Light: the lamp is driven either by the LED device (robot controller) or by a supervisor
    # through the PROTO field (no template regeneration: the field is not used by the JavaScript template)
    lamps.append('LED { translation 0 0 %g name "stack light %s" color [ %s ] children [ Shape { appearance '
                 'PBRAppearance { baseColor %s roughness 0.3 metalness 0 emissiveColor IS %sLight transparency 0.1 } '
                 'geometry Cylinder { radius 0.035 height 0.07 } } ] }' % (0.10 + 0.075 * i, nm, on, c, nm))
stack = 'Pose { translation -0.14 0.75 1.965 children [ %s\n        %s ] }' % (
    render([cyl(0.012, 0.065, DARK, (0, 0, 0.0325)), cyl(0.036, 0.01, DARK, (0, 0, 0.005))]), "\n        ".join(lamps))

U, N = U_TOOL, N_X
A0 = math.atan2(-U[2], U[0])                       # ry_to(U) angle: local x -> U (working direction)


def tp(u, y, n=0.0):
    """Point of the turret frame given along the working direction U (from the turret axis), y and N."""
    return (u * U[0] + n * N[0], y, u * U[2] + n * N[2])


HOLDER = DARK                                        # VDI tool block (same as the former single tool)
SHANK = app("0.35 0.36 0.38", 0.4, 0.8)
BLADE = app("0.55 0.56 0.58", 0.35, 0.85)
TIN = app("0.85 0.7 0.2", 0.3, 0.8)                  # TiN-coated carbide insert
COATED = app("0.3 0.32 0.36", 0.25, 0.9)             # AlTiN-coated (grey) finishing insert
NOZZLE = app("0.2 0.2 0.22", 0.6, 0)
# coolant nozzle (one per station) and jet aimed at the tool tip of the station in the working position
NOZ = tp(0.235, TOOL_DY + 0.004, 0.035)
TIP = tp(TOOL_R - 0.004, TOOL_DY + 0.004)
JET = [TIP[i] - NOZ[i] for i in range(3)]
JL = math.sqrt(sum(v * v for v in JET))
BACK = tuple(NOZ[i] - 0.015 * JET[i] / JL for i in range(3))


def tool_shapes(kind):
    """Procedural tool of the given kind, mounted on station 0 (tip at TOOL_R from the turret axis along U,
    TOOL_DY along y): VDI block + shank / blade + insert, and the station coolant nozzle."""
    s_ = [box((0.06, 0.07 + g.TOOL_EXT, 0.06), HOLDER, tp(0.245, -0.07 - g.TOOL_EXT / 2), g.ry_to(U)),
          cyl(0.005, 0.03, NOZZLE, BACK, g.z_to(JET))]
    if kind == "ebauche":            # roughing: 80 deg square-ish insert (14 x 14 mm) on a straight shank
        s_ += [box((0.056, 0.024, 0.02), SHANK, tp(0.268, TOOL_DY + 0.018), g.ry_to(U)),
               box((0.014, 0.006, 0.014), TIN, tp(TOOL_R - 0.007, TOOL_DY + 0.003), g.ry_to(U))]
    elif kind == "finition":         # finishing: narrow shank, small rhombic insert, corner on the tip
        s_ += [box((0.052, 0.022, 0.016), SHANK, tp(0.267, TOOL_DY + 0.016), g.ry_to(U)),
               box((0.011, 0.005, 0.011), COATED, tp(TOOL_R - 0.0078, TOOL_DY + 0.0025), (0, 1, 0, A0 + math.pi / 4))]
    else:                            # grooving: clamp block, 3 mm blade along U, insert on the blade end
        s_ += [box((0.04, 0.03, 0.03), SHANK, tp(0.245, TOOL_DY + 0.018), g.ry_to(U)),
               box((0.058, 0.003, 0.02), BLADE, tp(0.270, TOOL_DY + 0.0015), g.ry_to(U)),
               box((0.006, 0.0032, 0.008), TIN, tp(TOOL_R - 0.003, TOOL_DY + 0.0016), g.ry_to(U))]
    return s_


tools = "\n                        ".join(
    "Pose { rotation 0 1 0 %.6f children [ %s ] }  # station %d: %s" % (g.station_rot(k), render(tool_shapes(kind)), k, kind)
    for kind, k in g.TOOL_STATIONS)
cool = g.COOL.replace("transparency 0.45", "transparency IS coolantTransparency")
assert "IS coolantTransparency" in cool
jet = ("Pose { translation %s rotation %s children [ Pose { translation 0 0 %g children [ Shape { castShadows FALSE "
       "appearance %s geometry Cylinder { radius 0.004 height %g subdivision 12 } } ] } ] }"
       % (f(NOZ), f(g.z_to(JET)), JL / 2, cool, JL))
kcol = [g.slide_box(-0.64, 0.665, -0.046, 0.747, -0.06, 0.865)]
slide_col = [g.slide_box(-0.21, 0.641, -0.013, 0.59, -0.18, 0.305), g.slide_box(-0.11, 0.255, 0.59, 0.731, -0.126, 0.208)]
disc_col = [("cyl", (0.241, 0.109), (0, -0.0545, 0), RX90)] + g.tool_col(0.0)
ccol = [("cyl", (0.105, 0.117), (0, -0.0585, 0), RX90), ("cyl", (0.127, 0.042), (0, -0.138, 0), RX90)]
door_col = g.door_col()

# travels
Z_MIN, Z_MAX = -0.533, 0.0      # carriage (machine Z) from the CAD pose, towards the chuck: 533 mm (Haas layout drawing)
X_MIN, X_MAX = -0.15, 0.065     # cross slide (machine X) from the CAD pose: -0.1518 puts the tool tip on the axis
J_MIN, J_MAX = -0.0155, 0.0245  # jaw radial stroke from the CAD pose (grip radius 0.0255 -> 0.010 .. 0.050)


def motor(name, mn, mx, vmax, force, kind="Linear"):
    lim = "minPosition %g maxPosition %g " % (mn, mx) if mn is not None else ""
    ff = "maxForce" if kind == "Linear" else "maxTorque"
    return ('%sMotor { name "%s motor" %smaxVelocity %g %s %g }\n          PositionSensor { name "%s sensor" }'
            % (kind, name, lim, vmax, ff, force, name))


def stops(mn, mx):
    return "minStop %g maxStop %g " % (mn, mx)


jaws = []
for i, (dx, dz) in enumerate(g.JAW_DIRS):
    k = i + 1
    jcol = [("box", (0.079, 0.042, 0.034), (0.0625 * dx, 0.021, 0.0625 * dz), g.ry_to((dx, 0, dz)))]
    jaws.append("""DEF JAW%(k)d_JOINT SliderJoint {
            jointParameters JointParameters { position %%<= fields.jaw%(k)dPosition.value >%% axis %(dx)g 0 %(dz)g %(st)s}
            device [
              %(mot)s
            ]
            endPoint DEF JAW%(k)d Solid {
              translation %%<= %(dx)g * fields.jaw%(k)dPosition.value >%% 0 %%<= %(dz)g * fields.jaw%(k)dPosition.value >%%
              name "jaw %(k)d"
              children [ %(mesh)s ]
              boundingObject %(bo)s
            }
          }""" % dict(k=k, dx=dx, dz=dz, st=stops(J_MIN, J_MAX), mot=motor("jaw %d" % k, J_MIN, J_MAX, 0.1, 5000),
                     mesh=group("jaws", "jaw_%d" % k), bo=bo(jcol, "              ")))

P = dict(
    door_mesh=group("door"), door_bo=bo(door_col, "        "), dc=DOOR_CLOSED, dmot=motor("door", 0, g.DOOR_TRAVEL, 1.0, 2000),
    dst=stops(0, g.DOOR_TRAVEL),
    cx=CX, cf=CHUCK_FACE, cz=CZ, smot=motor("spindle", None, None, 419, 233, "Rotational"),
    spindle_mesh=group("spindle"), jaws="\n          ".join(jaws), chuck_bo=bo(ccol, "        "),
    t0=TUR0, zmot=motor("z", Z_MIN, Z_MAX, 0.4, 10000), zst=stops(Z_MIN, Z_MAX), car_mesh=group("carriage"),
    car_bo=bo(kcol, "        "), xd=X_DIR, xmot=motor("x", X_MIN, X_MAX, 0.4, 10000), xst=stops(X_MIN, X_MAX),
    slide_mesh=group("turret", "slide"), slide_bo=bo(slide_col, "            "),
    tmot=motor("turret", None, None, 10, 500, "Rotational"), disc_mesh=group("turret", "disc"), tools=tools, jet=jet,
    disc_bo=bo(disc_col, "                "),
    static=group("static"), plinth=plinth_vis, stack=stack,
    bati_bo=bo(bati_col, "        ", True, False), plinth_bo=bo(plinth_col, "        ", True, False),
)
P["dcx"], P["dcy"], P["dcz"] = DOOR_CLOSED
P["t0x"], P["t0y"], P["t0z"] = TUR0
P["xdx"], P["xdz"] = X_DIR[0], X_DIR[2]

# ---------------------------------------------------------------- Haas automation options (see tools/gen_world.py)
SHEET = app("0.62 0.64 0.66", 0.4, 0.6)             # stainless sheet metal (catcher, chute, bin)
BLACKP = app("0.12 0.12 0.13", 0.6, 0.2)            # actuator bodies
CHROME = app("0.82 0.83 0.85", 0.15, 0.95)          # piston rod, nozzle
SWITCH = app("0.85 0.7 0.1", 0.5, 0.1)              # limit / proximity switch bodies
STEEL_C = app("0.55 0.56 0.58", 0.35, 0.8)          # hydraulic cylinder


def led(name, field, pos, on="0 1 0"):
    """LED device + lamp driven either by the LED (robot controller) or by the supervisor through the PROTO field."""
    return ('LED { translation %s name "%s" color [ %s ] children [ Shape { appearance PBRAppearance { baseColor '
            '0.15 0.2 0.15 roughness 0.3 metalness 0 emissiveColor IS %s } geometry Cylinder { radius 0.007 height 0.006 '
            '} } ] }' % (f(pos), name, on, field))


def chan(L, W, h, t=0.003, end=True):
    """Sheet-metal channel along -x from the origin (bottom on z = 0): bottom, two sides, optional end wall."""
    s_ = [box((L, W, t), SHEET, (-L / 2, 0, t / 2))]
    s_ += [box((L, t, h), SHEET, (-L / 2, k * (W - t) / 2, h / 2)) for k in (-1, 1)]
    if end:
        s_.append(box((t, W, h), SHEET, (-L + t / 2, 0, h / 2)))
    return s_


# parts catcher (M35/M36/M37): tray on a pivot shaft (moving), descent chute + outer bin + rotary actuator (fixed)
CP = g.CATCH_PIVOT
catcher_tray = render(chan(g.CATCH_L, g.CATCH_W, 0.03) + [cyl(0.012, g.CATCH_W + 0.03, CHROME, (0, 0, 0), RX90)])
(xa, za), (xb, zb) = g.CHUTE
ach = math.atan2(za - zb, xb - xa)
LCH = math.hypot(xb - xa, zb - za)
b0, b1, b2, b3, b4, b5 = g.BIN
bin_ = [box((b1 - b0, b3 - b2, 0.004), SHEET, ((b0 + b1) / 2, (b2 + b3) / 2, b4 + 0.002))]
bin_ += [box((0.003, b3 - b2, b5 - b4), SHEET, (x_, (b2 + b3) / 2, (b4 + b5) / 2)) for x_ in (b0 + 0.0015, b1 - 0.0015)]
bin_ += [box((b1 - b0, 0.003, b5 - b4), SHEET, ((b0 + b1) / 2, y_, (b4 + b5) / 2)) for y_ in (b2 + 0.0015, b3 - 0.0015)]
bin_ += [box((0.025, b3 - b2, 0.11), DARK, (-0.0075, (b2 + b3) / 2, 0.505))]                    # bracket on the facade
act_y = CP[1] - g.CATCH_W / 2 - 0.01
catcher_fixed = ("Pose { translation %s rotation 0 1 0 %.6f children [ %s ] }\n      %s" % (
    f(((xa + xb) / 2 + LCH / 2 * math.cos(ach), CP[1], (za + zb) / 2 - LCH / 2 * math.sin(ach))), ach,
    render(chan(LCH, g.CATCH_W, 0.02, end=False)),
    render(bin_ + [box((0.05, 0.02, 0.05), BLACKP, (-0.22, act_y, 0.79)), cyl(0.018, 0.012, DARK, (CP[0], act_y + 0.016, CP[2]), RX90)])))
catcher_bo = bo([b for b in OPT_COL["catcher"]], "        ", True, False)

# auto air jet (M12/M13): nozzle on the headstock sheet aimed at the jaws; visible jet (transparency IS airJetTransparency)
AN, AT = g.AIR_NOZZLE, g.AIR_TARGET
AD = [AT[i] - AN[i] for i in range(3)]
ADL = math.sqrt(sum(v * v for v in AD))
AU = [v / ADL for v in AD]
atip = [AN[i] + 0.07 * AU[i] for i in range(3)]
jl = ADL - 0.07
air_fixed = render([box((0.04, 0.555 + AN[1] + 0.015, 0.04), BLACKP, (AN[0], (-0.555 + AN[1] + 0.015) / 2, AN[2])),
                    cyl(0.006, 0.07, CHROME, tuple(AN[i] + 0.035 * AU[i] for i in range(3)), g.z_to(AD)),
                    cyl(0.009, 0.012, CHROME, tuple(AN[i] + 0.066 * AU[i] for i in range(3)), g.z_to(AD))])
air_jet = ("Pose { translation %s rotation %s children [ Shape { castShadows FALSE appearance PBRAppearance { baseColor "
           "0.85 0.92 1 roughness 0.2 metalness 0 emissiveColor 0.25 0.3 0.35 transparency IS airJetTransparency } "
           "geometry Cone { bottomRadius 0.035 height %g subdivision 16 } } ] }"
           % (f([atip[i] + jl / 2 * AU[i] for i in range(3)]), f(g.z_to([-v for v in AD])), jl))
air_bo = bo(OPT_COL["airjet"], "        ", True, False)

# auto door: actuator on the roof (fixed body + supports), rod + lug carried by the door, limit switch "door completely
# open" (body + roller lever + LED) actuated by the door lug
dx_, dz_, dy0, dy1, dr = g.DOOR_ACT
s0, s1, s2, s3, s4, s5 = g.DOOR_SW
door_fixed = render([cyl(dr, dy1 - dy0, BLACKP, (dx_, (dy0 + dy1) / 2, dz_), RX90),
                     cyl(dr + 0.006, 0.03, DARK, (dx_, dy0 + 0.015, dz_), RX90),
                     cyl(dr + 0.006, 0.03, DARK, (dx_, dy1 - 0.015, dz_), RX90),
                     box((0.14, 0.03, 0.045), DARK, (-0.53, -1.255, dz_ - 0.0075)),
                     box((0.14, 0.03, 0.045), DARK, (-0.53, -0.685, dz_ - 0.0075)),
                     box((s1 - s0, s3 - s2, s5 - s4), SWITCH, ((s0 + s1) / 2, (s2 + s3) / 2, (s4 + s5) / 2)),
                     box((0.004, 0.012, 0.004), CHROME, ((s0 + s1) / 2, s3 + 0.006, (s4 + s5) / 2))])
# (a DistanceSensor on the switch does not see the door lug: ray sensors ignore the bodies of their own robot, so the
# switch state is derived from the door position by the controllers, see the header)
door_fixed += "\n      " + led("door open switch led", "doorOpenLight", ((s0 + s1) / 2, (s2 + s3) / 2, s5 + 0.003))
door_bo = bo(OPT_COL["door"], "        ", True, False)
ry, rl, rr = g.DOOR_ROD
lx0, lx1, ly0, ly1, lz0, lz1 = g.DOOR_LUG
DC = DOOR_CLOSED
door_moving = render([cyl(rr, rl, CHROME, (dx_ - DC[0], ry - rl / 2 - DC[1], dz_ - DC[2]), RX90),
                      box((lx1 - lx0, ly1 - ly0, lz1 - lz0), DARK,
                          ((lx0 + lx1) / 2 - DC[0], (ly0 + ly1) / 2 - DC[1], (lz0 + lz1) / 2 - DC[2]))])

# hydraulic chuck: rotary cylinder + draw tube at the rear of the spindle (inside the headstock, CAD "Main Spindle /
# Hyd Cyl" envelope y -1.297..-1.06, dropped from the meshes) rotating with the spindle; fixed oil distributor with the
# two proximity switches "chuck clamped / unclamped" (LED devices, lamps IS chuckClampedLight / chuckUnclampedLight)
hyd_rot = render([cyl(0.112, 0.20, STEEL_C, (0, -0.76, 0), RX90), cyl(0.04, 0.55, STEEL_C, (0, -0.39, 0), RX90)])
hyd_fixed = render([cyl(0.05, 0.05, DARK, (CX, -1.285, CZ), RX90),
                    box((0.08, 0.02, 0.05), SWITCH, (CX, -1.32, CZ + 0.075))])
hyd_fixed += "\n      " + led("chuck clamped switch", "chuckClampedLight", (CX - 0.02, -1.331, CZ + 0.075))
hyd_fixed += "\n      " + led("chuck unclamped switch", "chuckUnclampedLight", (CX + 0.02, -1.331, CZ + 0.075), "1 0.55 0")

# automatic tool presetter (AD0072): CAD arm (mesh group "presetter") on its rotary actuator, stowed = CAD pose
PRE = HAAS["groups"]["presetter"]
PV, PAX = PRE["origin"], PRE["axis"]
pre_col = [g.bx(-0.335, -0.14, -0.465, -0.423, 0.89, 1.275), g.bx(-0.266, -0.207, -0.51, -0.422, 1.242, 1.30)]
pre_col = [(k, d, tuple(c[i] - PV[i] for i in range(3)), r) for k, d, c, r in pre_col]

proto = """#VRML_SIM R2025a utf8
# license: Haas Automation (meshes derived from the official Haas ST-20 STEP model, see CLAUDE.md)
# documentation url: https://github.com/NicolasBrochen/turning-cell-webots
# keywords: industrial/machine tool
# template language: javascript
# Haas ST-20 CNC lathe (2-axis, 8in 3-jaw chuck, 12-station BOT turret on a 55 deg slant bed, sliding door).
# Same geometry, materials and motion directions as the turning cell (tools/gen_world.py, cell_plc.py).
# Frame: facade (operator side) at x = 0 facing +x, machine interior towards -x, spindle axis along +y at
# (x, z) = (-0.40, 1.10), chuck on the -y side, z up, ground z = 0 under the 95 mm plinth.
# Joint positions are measured from the CAD pose of the STEP model:
#   door motor     SliderJoint axis (0,-1,0), 0 = closed .. 0.722 m = open (towards the headstock)
#   spindle motor  HingeJoint axis (0,1,0) through the spindle axis at the chuck body face (y = -0.40), unlimited
#   jaw k motor    SliderJoint radial (k = 1..3, carried by the spindle), 0 = grip radius 0.0255 m, + = opening
#   z motor        SliderJoint axis (0,1,0): carriage along the spindle axis, -0.533 .. 0 m (- = towards the chuck)
#   x motor        SliderJoint axis (-0.5737,0,0.8191) (55 deg cross slide, + = away from the spindle), -0.15 .. 0.065 m
#   turret motor   HingeJoint axis (0,1,0) through the turret axis, unlimited, 12 stations every pi/6 rad
#                  (station k in the working position at k * pi / 6); tools: station 0 roughing, 11 finishing,
#                  10 grooving (tip 0.305 m from the turret axis, axial offset -0.140 m for all three)
#   presetter motor   HingeJoint axis = X slide through the rotary actuator of the CAD tool presetter arm (AD0072),
#                     0 = stowed (CAD pose) .. pi/2 = deployed (stylus tip in the tool-tip plane, 0.048 m from the spindle
#                     axis along X, y = -0.145)
#   parts catcher motor  HingeJoint axis (0,1,0) at (-0.22, y, 0.79), 0 = stowed (tray tip down) .. 1.33 = catching
# LEDs "stack light green/orange/red" on the roof light column, "door open switch led", "chuck clamped switch",
# "chuck unclamped switch". Simulated switches (no physics in this PROTO): "door completely open" = door motor position
# >= 0.714 m (lug on the roller over the last 8 mm of the stroke); chuck clamped / unclamped = jaw position at the
# clamping / fully open stop; tool presetter probe trigger = tool tip on the stylus faces (from the axis positions).
# Options (template fields, default TRUE): toolPresetter, partsCatcher, autoAirJet, autoDoor.
# Supervisor control without a robot controller (turning cell PLC): internal DEFs DOOR_JOINT, SPINDLE_JOINT,
# JAW1_JOINT..JAW3_JOINT, Z_JOINT, X_JOINT, TURRET_JOINT, PRESETTER_JOINT, CATCHER_JOINT (getFromProtoDef +
# setJointPosition, same coordinates as the motors; the last two only when the option is fitted) and their end points
# DOOR, CHUCK, JAW1..JAW3, CARRIAGE, CROSS_SLIDE, TURRET, PRESETTER, PARTS_CATCHER; fields greenLight / orangeLight /
# redLight, coolantTransparency, airJetTransparency, doorOpenLight, chuckClampedLight, chuckUnclampedLight.
# Never link a template field (doorPosition...) to a joint position.

PROTO HaasST20 [
  field SFVec3f    translation      0 0 0
  field SFRotation rotation         0 0 1 0
  field SFString   name             "Haas ST-20"
  field SFString   controller       "<none>"
  field MFString   controllerArgs   []
  field SFString   window           "<none>"
  field SFString   customData       ""
  field SFBool     supervisor       FALSE
  field SFBool     synchronization  TRUE
  field SFBool     plinth           TRUE   # 95 mm levelling plinth; FALSE lowers the whole machine by 0.095 m onto z = 0
  field SFFloat    doorPosition     0      # [0, 0.722] m, 0 = closed
  field SFFloat    spindleAngle     0      # rad
  field SFFloat    jaw1Position     0      # [-0.0155, 0.0245] m from the grip radius 0.0255 m (+ = open)
  field SFFloat    jaw2Position     0
  field SFFloat    jaw3Position     0
  field SFFloat    zPosition        0      # [-0.533, 0] m, carriage along +y from the CAD pose
  field SFFloat    xPosition        0      # [-0.15, 0.065] m, cross slide along the 55 deg X axis
  field SFFloat    turretAngle      0      # rad, station k = k * pi / 6
  # supervisor-driven appearance (plain IS links, not used by the template: changing them does not regenerate the PROTO)
  field SFColor    greenLight       0 0 0  # emissive colour of the stack light lamps (also driven by the LED devices)
  field SFColor    orangeLight      0 0 0
  field SFColor    redLight         0 0 0
  field SFFloat    coolantTransparency 1   # coolant jet on the working station: 1 = off (invisible), 0.45 = on
  field SFFloat    airJetTransparency 1    # auto air jet (M12 on / M13 off): 1 = off (invisible), 0.6 = blowing
  field SFColor    doorOpenLight    0 0 0  # LED of the "door completely open" limit switch
  field SFColor    chuckClampedLight   0 0 0  # LEDs of the hydraulic chuck proximity switches
  field SFColor    chuckUnclampedLight 0 0 0
  # Haas automation options (template fields: changing them regenerates the PROTO)
  field SFBool     toolPresetter    TRUE   # automatic tool presetter arm (AD0072); FALSE: CAD arm removed
  field SFFloat    presetterAngle   0      # [0, pi/2] rad, 0 = stowed
  field SFBool     partsCatcher     TRUE   # parts catcher (M35/M36/M37) + chute + outer bin
  field SFFloat    partsCatcherAngle 0     # [0, 1.33] rad, 0 = stowed
  field SFBool     autoAirJet       TRUE   # auto air jet (M12/M13) aimed at the chuck
  field SFBool     autoDoor         TRUE   # auto door actuator on the roof + "door completely open" switch
]
{
  %%< const dz = fields.plinth.value ? 0 : -0.095; >%%
  Robot {
    translation IS translation
    rotation IS rotation
    name IS name
    model "Haas ST-20"
    controller IS controller
    controllerArgs IS controllerArgs
    window IS window
    customData IS customData
    supervisor IS supervisor
    synchronization IS synchronization
    children [
      Pose {
        translation 0 0 %%<= dz >%%
        children [
      %(static)s
      %%< if (fields.plinth.value) { >%%
      %(plinth)s
      %%< } >%%
      %(stack)s
      %(hyd_fixed)s
      %%< if (fields.partsCatcher.value) { >%%
      %(catcher_fixed)s
      %%< } >%%
      %%< if (fields.autoAirJet.value) { >%%
      %(air_fixed)s
      %(air_jet)s
      %%< } >%%
      %%< if (fields.autoDoor.value) { >%%
      %(door_fixed)s
      %%< } >%%
        ]
      }
      # ---------------------------------------------------------------- automatic tool presetter (CAD arm)
      %%< if (fields.toolPresetter.value) { >%%
      DEF PRESETTER_JOINT HingeJoint {
        jointParameters HingeJointParameters { position %%<= fields.presetterAngle.value >%% axis %(pax)s anchor %(pvx)g %(pvy)g %%<= %(pvz)g + dz >%% }
        device [
          %(pmot)s
        ]
        endPoint DEF PRESETTER Solid {
          translation %(pvx)g %(pvy)g %%<= %(pvz)g + dz >%%
          rotation %(pax)s %%<= fields.presetterAngle.value >%%
          name "tool presetter"
          children [
      %(pre_mesh)s
          ]
          boundingObject %(pre_bo)s
        }
      }
      %%< } >%%
      # ---------------------------------------------------------------- parts catcher (tray)
      %%< if (fields.partsCatcher.value) { >%%
      DEF CATCHER_JOINT HingeJoint {
        jointParameters HingeJointParameters { position %%<= fields.partsCatcherAngle.value >%% axis 0 1 0 anchor %(cpx)g %(cpy)g %%<= %(cpz)g + dz >%% }
        device [
          %(cmot)s
        ]
        endPoint DEF PARTS_CATCHER Solid {
          translation %(cpx)g %(cpy)g %%<= %(cpz)g + dz >%%
          rotation 0 1 0 %%<= %(cstow)g + fields.partsCatcherAngle.value >%%
          name "parts catcher"
          children [ %(catcher_tray)s ]
          boundingObject %(catcher_tray_bo)s
        }
      }
      %%< } >%%
      # ---------------------------------------------------------------- sliding door
      DEF DOOR_JOINT SliderJoint {
        jointParameters JointParameters { position %%<= fields.doorPosition.value >%% axis 0 -1 0 %(dst)s}
        device [
          %(dmot)s
        ]
        endPoint DEF DOOR Solid {
          translation %(dcx)g %%<= %(dcy)g - fields.doorPosition.value >%% %%<= %(dcz)g + dz >%%
          name "door"
          children [
      %(door_mesh)s
      %%< if (fields.autoDoor.value) { >%%
      %(door_moving)s
      %%< } >%%
          ]
          boundingObject %(door_bo)s
        }
      }
      # ---------------------------------------------------------------- spindle + 8in chuck + 3 jaws
      DEF SPINDLE_JOINT HingeJoint {
        jointParameters HingeJointParameters { position %%<= fields.spindleAngle.value >%% axis 0 1 0 anchor %(cx)g %(cf)g %%<= %(cz)g + dz >%% }
        device [
          %(smot)s
        ]
        endPoint DEF CHUCK Solid {
          translation %(cx)g %(cf)g %%<= %(cz)g + dz >%%
          rotation 0 1 0 %%<= fields.spindleAngle.value >%%
          name "chuck"
          children [
      %(spindle_mesh)s
          %(hyd_rot)s
          %(jaws)s
          ]
          boundingObject %(chuck_bo)s
        }
      }
      # ---------------------------------------------------------------- Z carriage -> X cross slide -> turret
      DEF Z_JOINT SliderJoint {
        jointParameters JointParameters { position %%<= fields.zPosition.value >%% axis 0 1 0 %(zst)s}
        device [
          %(zmot)s
        ]
        endPoint DEF CARRIAGE Solid {
          translation %(t0x)g %%<= %(t0y)g + fields.zPosition.value >%% %%<= %(t0z)g + dz >%%
          name "carriage"
          children [
      %(car_mesh)s
            DEF X_JOINT SliderJoint {
              jointParameters JointParameters { position %%<= fields.xPosition.value >%% axis %(xdx)g 0 %(xdz)g %(xst)s}
              device [
                %(xmot)s
              ]
              endPoint DEF CROSS_SLIDE Solid {
                translation %%<= %(xdx)g * fields.xPosition.value >%% 0 %%<= %(xdz)g * fields.xPosition.value >%%
                name "cross slide"
                children [
      %(slide_mesh)s
                  # coolant jet of the station in the working position (fixed on the cross slide)
                  %(jet)s
                  DEF TURRET_JOINT HingeJoint {
                    jointParameters HingeJointParameters { position %%<= fields.turretAngle.value >%% axis 0 1 0 anchor 0 0 0 }
                    device [
                      %(tmot)s
                    ]
                    endPoint DEF TURRET Solid {
                      rotation 0 1 0 %%<= fields.turretAngle.value >%%
                      name "turret"
                      children [
      %(disc_mesh)s
                        %(tools)s
                      ]
                      boundingObject %(disc_bo)s
                    }
                  }
                ]
                boundingObject %(slide_bo)s
              }
            }
          ]
          boundingObject %(car_bo)s
        }
      }
    ]
    boundingObject Group {
      children [
        %(bati_bo)s
        %%< if (fields.plinth.value) { >%%
        %(plinth_bo)s
        %%< } >%%
        %%< if (fields.partsCatcher.value) { >%%
        %(catcher_bo)s
        %%< } >%%
        %%< if (fields.autoAirJet.value) { >%%
        %(air_bo)s
        %%< } >%%
        %%< if (fields.autoDoor.value) { >%%
        %(door_opt_bo)s
        %%< } >%%
      ]
    }
  }
}
""" % dict(P, hyd_fixed=hyd_fixed, catcher_fixed=catcher_fixed, air_fixed=air_fixed, air_jet=air_jet,
           door_fixed=door_fixed, door_moving=door_moving, hyd_rot=hyd_rot, pax=f(PAX), pvx=PV[0], pvy=PV[1], pvz=PV[2],
           pmot=motor("presetter", 0, round(math.pi / 2, 6), 2.0, 50, "Rotational"), pre_mesh=group("presetter"),
           pre_bo=bo(pre_col, "            "), cpx=CP[0], cpy=CP[1], cpz=CP[2], cstow=g.CATCH_STOW,
           cmot=motor("parts catcher", 0, g.CATCH_TRAVEL, 2.5, 20, "Rotational"), catcher_tray=catcher_tray,
           catcher_tray_bo=bo(g.catcher_col(), "            "), catcher_bo=catcher_bo, air_bo=air_bo, door_opt_bo=door_bo)
open(os.path.join(ROOT, "protos", "HaasST20.proto"), "w").write(proto)
print(len(proto))
