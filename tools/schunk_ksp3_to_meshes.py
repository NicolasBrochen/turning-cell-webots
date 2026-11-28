#!/usr/bin/env python3
"""SCHUNK KSP3-F 100 / 160 pneumatic vices: STEP -> OBJ meshes (fixed body + two moving base jaws) and Webots PROTO.

    uv run --no-project --python 3.12 --with cadquery-ocp python -I tools/schunk_ksp3_to_meshes.py \
        "<SCHUNK-1433684 KSP3-F 100-AS-PM_ 0.stp>" "<SCHUNK-1409350 KSP3-F 160_ 0.stp>" protos/meshes/schunk_ksp3 \
        --proto protos/SchunkKSP3.proto

CAD data: CADENAS GmbH, component: SCHUNK SE & Co. KG. Licence of the CAD data and of the derived meshes: CC BY-ND 4.0
(https://creativecommons.org/licenses/by-nd/4.0/). The meshes are a format conversion only (re-tessellation, change of
frame, no change of shape); the attribution is repeated in the JSON and in the PROTO header.

Assembly (both sizes, read with OCP, mm): three solids, no colours.
  GK  = Grundkoerper (body, fixed; includes the four M8 mounting screws protruding 15 mm below the mounting face)
  GBA = Grundbacken (base jaws, moving). The two base jaws are drawn fully OPEN: their outer faces are flush with the
        body ends, and their hole patterns (M5 rows + central dowel hole) are symmetric about the body centre.
STEP frame: x across the jaws, y up (y = 0 = mounting face), z = jaw stroke axis, body z in [-L, 0].
Vice frame (written, metres): origin at the centre of the mounting face, x = jaw stroke axis (jaw "a" on -x closes
towards +x, jaw "b" on +x closes towards -x), z up. Mapping: x_v = (z_s + L/2), y_v = x_s, z_v = y_s (a cyclic
permutation: proper rotation, winding kept).
Catalogue data (SCHUNK, not in the CAD): stroke per jaw 4 mm (100) / 6 mm (160), clamping force 14.5 kN / 45 kN,
3-7 bar, closing time 0.2 s (100; the 160 is assumed 0.3 s), height 69.2 mm (100).
Deterministic: same input -> byte-identical output.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # python -I drops the script directory

from haas_to_meshes import bbox_mm, collect_items, num, read_step, rnd, tessellate   # noqa: E402

DIGITS = 6
SHADOW_MAX_TRIANGLES = 21845
ATTRIBUTION = "CAD data: CADENAS GmbH, component: SCHUNK SE & Co. KG"
LICENCE = "CC BY-ND 4.0 (https://creativecommons.org/licenses/by-nd/4.0/)"
SERRATION_MM = 2.5             # top jaws sit this far down on the serrated base jaws (100: 71.7 - 2.5 = 69.2 = height)
SIZES = {
    "100": dict(order_no="1433684", designation="KSP3-F 100-AS-PM", stroke=0.004, force=14500.0, close_time=0.2,
                deflection=0.12, angle=0.35),
    "160": dict(order_no="1409350", designation="KSP3-F 160", stroke=0.006, force=45000.0, close_time=0.3,
                deflection=0.18, angle=0.35),
}
COLOURS = {"body": (0.24, 0.26, 0.29), "jaw": (0.72, 0.73, 0.75)}


def convert(step, size, out_dir):
    cfg = SIZES[size]
    items = collect_items(read_step(step))
    assert len(items) == 3, "expected body + 2 base jaws, got %d solids" % len(items)
    body = [it for it in items if " GK " in " %s " % it["path"]]
    jaws = [it for it in items if "GBA" in it["path"]]
    assert len(body) == 1 and len(jaws) == 2, [it["path"] for it in items]
    blo, bhi = bbox_mm(body[0]["shape"])
    L = bhi[2] - blo[2]
    zc = (blo[2] + bhi[2]) / 2.0

    def to_v(p):                                   # STEP metres -> vice metres
        return (p[2] - zc / 1000.0, p[0], p[1])

    def to_v_box(lo, hi):
        a, b = to_v([x / 1000.0 for x in lo]), to_v([x / 1000.0 for x in hi])
        return [min(a[i], b[i]) for i in range(3)], [max(a[i], b[i]) for i in range(3)]

    groups = {"body": body[0]}
    for j in jaws:
        lo, hi = bbox_mm(j["shape"])
        groups["jaw_a" if (lo[2] + hi[2]) / 2 < zc else "jaw_b"] = j
    files, meta_groups = {}, {}
    for grp in ("body", "jaw_a", "jaw_b"):
        it = groups[grp]
        tris = [t for T, _ in tessellate(it["shape"], cfg["deflection"], cfg["angle"], normals=True) for t in T]
        chunks = [tris[i:i + SHADOW_MAX_TRIANGLES] for i in range(0, len(tris), SHADOW_MAX_TRIANGLES)]
        names = []
        for ci, ch in enumerate(chunks):
            name = "ksp3_%s_%s%s.obj" % (size, grp, "_%d" % (ci + 1) if len(chunks) > 1 else "")
            vid, V, NV, F = {}, [], [], []
            for t, nn in ch:
                idx = []
                for p, n in zip(t, nn):
                    v = tuple(round(c, DIGITS) + 0.0 for c in to_v(p))
                    nr = tuple(round(c, 3) + 0.0 for c in to_v(n))
                    if (v, nr) not in vid:
                        vid[(v, nr)] = len(V)
                        V.append(v)
                        NV.append(nr)
                    idx.append(vid[(v, nr)])
                if len({V[k] for k in idx}) == 3:
                    F.append(idx)
            path = os.path.join(out_dir, name)
            with open(path, "w") as fh:
                fh.write("# SCHUNK %s (%s) - %s; metres, vice frame (open jaws)\n# %s; licence %s\n"
                         % (cfg["designation"], cfg["order_no"], grp, ATTRIBUTION, LICENCE))
                for v in V:
                    fh.write("v %s %s %s\n" % tuple(num(x, DIGITS) for x in v))
                for n in NV:
                    fh.write("vn %s %s %s\n" % tuple(num(x, 3) for x in n))
                for f in F:
                    fh.write("f %d//%d %d//%d %d//%d\n" % (f[0] + 1, f[0] + 1, f[1] + 1, f[1] + 1, f[2] + 1, f[2] + 1))
            lo = [min(v[i] for v in V) for i in range(3)]
            hi = [max(v[i] for v in V) for i in range(3)]
            files[name] = dict(group=grp, triangles=len(F), vertices=len(V), bbox=[rnd(lo), rnd(hi)],
                               cast_shadows=len(F) <= SHADOW_MAX_TRIANGLES, bytes=os.path.getsize(path),
                               material="jaw" if grp != "body" else "body",
                               colour=list(COLOURS["jaw" if grp != "body" else "body"]))
            names.append(name)
        lo, hi = to_v_box(*bbox_mm(it["shape"]))
        meta_groups[grp] = dict(files=names, cad_part=it["path"], bbox_open=[rnd(lo), rnd(hi)])
    s = cfg["stroke"]
    a_lo, a_hi = meta_groups["jaw_a"]["bbox_open"]
    b_lo, b_hi = meta_groups["jaw_b"]["bbox_open"]
    bo_lo, bo_hi = meta_groups["body"]["bbox_open"]
    meta_groups["body"].update(joint="fixed", origin=[0, 0, 0])
    meta_groups["jaw_a"].update(joint="prismatic", axis=[1, 0, 0], range=[0.0, s], origin=[0, 0, 0],
                                note="position 0 = open (CAD pose), + = closing towards the centre")
    meta_groups["jaw_b"].update(joint="prismatic", axis=[-1, 0, 0], range=[0.0, s], origin=[0, 0, 0],
                                note="mirror of jaw_a; coupled to it (centric vice)")
    jaw_top = max(a_hi[2], b_hi[2])
    gx = (a_hi[0] + b_lo[0]) / 2
    body_top = None
    # body top below the jaws: highest body triangle above the jaw gap centre (x = gx, y = 0)
    for t, _ in [t for T, _ in tessellate(groups["body"]["shape"], cfg["deflection"], cfg["angle"], normals=True)
                 for t in T]:
        q = [to_v(p) for p in t]
        d = (q[1][1] - q[2][1]) * (q[0][0] - q[2][0]) + (q[2][0] - q[1][0]) * (q[0][1] - q[2][1])
        if abs(d) < 1e-14:
            continue
        l1 = ((q[1][1] - q[2][1]) * (gx - q[2][0]) + (q[2][0] - q[1][0]) * (0.0 - q[2][1])) / d
        l2 = ((q[2][1] - q[0][1]) * (gx - q[2][0]) + (q[0][0] - q[2][0]) * (0.0 - q[2][1])) / d
        l3 = 1 - l1 - l2
        if min(l1, l2, l3) >= 0:
            z = l1 * q[0][2] + l2 * q[1][2] + l3 * q[2][2]
            body_top = z if body_top is None else max(body_top, z)
    kp = dict(
        mounting_face_z=0.0, body_size_xy=[round(bo_hi[0] - bo_lo[0], 4), round(bo_hi[1] - bo_lo[1], 4)],
        body_top_in_gap_z=round(body_top, 4), body_bbox=meta_groups["body"]["bbox_open"],
        screws_below_mounting_face=round(-bo_lo[2], 4),
        base_jaw_top_z=round(jaw_top, 4), top_jaw_seat_z=round(jaw_top - SERRATION_MM / 1000.0, 4),
        base_jaw_width_y=round(a_hi[1] - a_lo[1], 4),
        jaw_inner_faces_open_x=[round(a_hi[0], 4), round(b_lo[0], 4)],
        jaw_gap_open=round(b_lo[0] - a_hi[0], 4), jaw_gap_closed=round(b_lo[0] - a_hi[0] - 2 * s, 4),
        jaw_gap_centre_x=round((a_hi[0] + b_lo[0]) / 2, 4),
        stroke_per_jaw=s, clamping_force_N=cfg["force"], closing_time_s=cfg["close_time"],
        jaw_speed=round(s / cfg["close_time"], 4),
        note="catalogue: stroke, force, pressure 3-7 bar, closing time (160: assumed). Open pose = CAD pose (outer "
             "jaw faces flush with the body ends, hole patterns symmetric about x = 0). Top jaws (procedural, in the "
             "PROTO) sit on the base jaw serration at top_jaw_seat_z (serration engagement %.1f mm, assumed)"
             % SERRATION_MM)
    return dict(designation=cfg["designation"], order_no=cfg["order_no"], source=os.path.basename(step),
                frame="vice frame: origin at the centre of the mounting face, x = jaw stroke axis, z up; "
                      "x_v = z_s + L/2, y_v = x_s, z_v = y_s (STEP metres)",
                body_length_mm=round(L, 2), tessellation=dict(deflection_mm=cfg["deflection"], angle_rad=cfg["angle"]),
                groups=meta_groups, files=files, key_positions=kp,
                total_triangles=sum(f["triangles"] for f in files.values()))


PROTO = r"""#VRML_SIM R2025a utf8
# template language: javascript
# license: CC BY-ND 4.0 (meshes). %(attribution)s.
# license url: https://creativecommons.org/licenses/by-nd/4.0/
# SCHUNK KSP3-F pneumatic centric vice (sizes 100 and 160), generated by tools/schunk_ksp3_to_meshes.py.
# Body and base jaws: meshes converted from the SCHUNK STEP (format conversion only). Top jaws, base plate, purge
# nozzle and sensors: procedural.
# Frame: origin on the surface the vice stands on (bottom of the base plate), centred on the vice; x = jaw stroke
# axis (jaw a on -x, jaw b on +x), z up. Without a base plate (plateThickness 0) the four M8 mounting screws protrude
# 15 mm below the origin.
# Devices (they belong to the enclosing Robot, e.g. inside HaasVF1.tableSlot; prefix = deviceName):
#   "<prefix>jaw motor::a", "<prefix>jaw motor::b"  LinearMotors of the two base jaws, Webots coupled motors (same
#       name before "::"): commanding one moves both, like the single piston + wedge hooks of the real centric vice.
#       Position 0 = open, + = closing, [0, stroke] m (100: 4 mm, 160: 6 mm per jaw).
#   "<prefix>jaw sensor"   PositionSensor of jaw a (same value for jaw b).
#   "<prefix>part sensor"  1 x 1 pixel RangeFinder in the body, looking up between the jaws (stands for the
#       part-seating sensor): depth ~ supportHeight + 1 mm (100) / 8.5 mm (160) with a part on the ledges, 0.1 (max) or
#       inf without. A RangeFinder (rendering) is used because DistanceSensor rays ignore the Solids of their own Robot,
#       and the vice and its parts belong to the enclosing Robot (HaasVF1).
# Clamped / unclamped (SCHUNK monitors the piston with inductive switches): read "jaw sensor" in the controller:
#   unclamped (open) = position < 0.2 mm; clamped = position stopped within +-0.5 mm of clampAt (the top jaws touch
#   a part of width clampWidth); position > stroke - 0.2 mm = closed without a part (fault). The joints are
#   kinematic (no physics): the controller must command the jaws to clampAt when a part is present.
# Top jaws (procedural soft jaws, to be machined to the part): jawType "flat" (prismatic parts) or "vee" (90 deg
# V for round parts; clampWidth = distance between the jaw faces at contact = D * sqrt(2) - 2 * vee depth).
# The inner faces of the top jaws are clampWidth apart when the jaw position equals clampAt, a ledge of
# supportHeight x supportWidth on each jaw carries the part. Parts go in `workpiece` (frame: centre of the vice,
# on the ledges, i.e. at the underside of the clamped part).

PROTO SchunkKSP3 [
  field SFVec3f    translation      0 0 0
  field SFRotation rotation         0 0 1 0
  field SFString   name             "KSP3"
  field SFString   size             "100"    # "100" (KSP3-F 100-AS-PM, 1433684) or "160" (KSP3-F 160, 1409350)
  field SFString   deviceName       ""       # device name prefix (unique per vice inside one Robot)
  field SFFloat    jawPosition      0        # initial jaw position, m (0 = open)
  field SFFloat    plateThickness   0.02     # base plate between the table and the vice (T-slot bolts), m; 0 = none
  field SFString   jawType          "flat"   # top jaws: "flat" or "vee"
  field SFFloat    clampWidth       0.05     # distance between the top jaw faces at contact, m
  field SFFloat    clampAt          -1       # jaw position at contact, m (-1: 60 %% of the stroke)
  field SFFloat    jawHeight        0.03     # top jaw height above the base jaw seat, m
  field SFFloat    jawThickness     0.02     # top jaw thickness along the stroke, m
  field SFFloat    supportHeight    0.008    # ledge under the part, m (0 = none)
  field SFFloat    supportWidth     0.004    # ledge width, m
  field SFFloat    veeDepth         0.012    # depth of the V notch (jawType "vee"), m
  field SFColor    jawColor         0.80 0.81 0.83   # soft jaws (aluminium)
  field SFBool     purgeNozzle      TRUE     # cleaning air nozzle blowing across the jaws
  field SFFloat    purgeTransparency 1       # 1 = no jet; ~0.6 = jet visible (not used by the template: change it live)
  field MFNode     workpiece        []       # parts (frame: centre, on the ledges)
]
{
  %%<
    const SIZES = %(sizes)s;
    const S = SIZES[fields.size.value] || SIZES['100'];
    if (!SIZES[fields.size.value]) console.error('SchunkKSP3: unknown size ' + fields.size.value + ', using 100');
    const clamp = (v, a, b) => Math.min(Math.max(v, a), b);
    const f5 = v => (v + 0).toFixed(5);
    const pre = fields.deviceName.value;
    const st = S.stroke;
    const p0 = clamp(fields.jawPosition.value, 0, st);
    const at = fields.clampAt.value < 0 ? 0.6 * st : clamp(fields.clampAt.value, 0, st);
    const h0 = Math.max(0, fields.plateThickness.value);
    const cw = Math.max(0.001, fields.clampWidth.value);
    const jh = Math.max(0.005, fields.jawHeight.value);
    const jt = Math.max(0.005, fields.jawThickness.value);
    const sh = clamp(fields.supportHeight.value, 0, jh - 0.002);
    const sw = sh > 0 ? Math.max(0, fields.supportWidth.value) : 0;
    const w = S.jawWidth;
    const vd = clamp(fields.veeDepth.value, 0, 0.45 * w);
    const vee = fields.jawType.value === 'vee';
    const zs = S.seat;                                   // top jaw seat above the mounting face
    // convex prism (IndexedFaceSet) from a counter-clockwise polygon [[x, y], ...] between z0 and z1
    function prism(P, z0, z1) {
      const n = P.length;
      let pts = '', idx = '';
      for (const [x, y] of P) pts += f5(x) + ' ' + f5(y) + ' ' + f5(z0) + ', ';
      for (const [x, y] of P) pts += f5(x) + ' ' + f5(y) + ' ' + f5(z1) + ', ';
      for (let i = n - 1; i >= 0; i--) idx += i + ' ';
      idx += '-1 ';
      for (let i = 0; i < n; i++) idx += (n + i) + ' ';
      idx += '-1 ';
      for (let i = 0; i < n; i++) {
        const j = (i + 1) %% n;
        idx += i + ' ' + j + ' ' + (n + j) + ' ' + (n + i) + ' -1 ';
      }
      return 'IndexedFaceSet { coord Coordinate { point [ ' + pts + '] } coordIndex [ ' + idx + '] creaseAngle 0.2 }';
    }
    // top jaw of side s (-1 = jaw a on -x, +1 = jaw b on +x), in the base jaw frame at position 0 (open)
    function topJaw(s) {
      const xf = s * (cw / 2 + at);                      // clamping face when open
      const xo = xf + s * jt;                            // outer face
      const polys = [];
      const R = (a, b, y0, y1) => [[Math.min(a, b), y0], [Math.max(a, b), y0], [Math.max(a, b), y1], [Math.min(a, b), y1]];
      if (vee && vd > 0) {
        // two convex halves around the V notch (apex at xf + s * vd, mouth +-vd on the face)
        const half = (sy) => {
          const P = [[xo, sy * w / 2], [xf, sy * w / 2], [xf, sy * vd], [xf + s * vd, 0], [xo, 0]];
          const area = P.reduce((A, p, i) => A + p[0] * P[(i + 1) %% P.length][1] - P[(i + 1) %% P.length][0] * p[1], 0);
          return area > 0 ? P : P.reverse();
        };
        polys.push(half(1), half(-1));
      } else
        polys.push(R(xo, xf, -w / 2, w / 2));
      let out = '';
      for (const P of polys)
        out += 'Shape { appearance DEF_JAW_APP geometry ' + prism(P, zs, zs + jh) + ' } ';
      if (sw > 0)
        out += 'Shape { appearance DEF_JAW_APP geometry ' + prism(R(xf, xf - s * sw, -w / 2, w / 2), zs, zs + sh) + ' } ';
      return out;
    }
    let appDone = false;
    function jawApp(txt) {                               // first use DEFs the appearance, later ones USE it
      return txt.replace(/DEF_JAW_APP/g, () => {
        if (appDone) return 'USE KSP3_TOPJAW';
        appDone = true;
        return 'DEF KSP3_TOPJAW PBRAppearance { baseColor IS jawColor roughness 0.35 metalness 0.6 }';
      });
    }
    function jawBox(s) {                                  // bounding box of base + top jaw (jaw frame, open)
      const g = s < 0 ? S.jawA : S.jawB;
      const xf = s * (cw / 2 + at), xo = xf + s * jt;
      const x0 = Math.min(g[0][0], xf, xo), x1 = Math.max(g[1][0], xf, xo);
      return 'Group { children [ Pose { translation ' + f5((g[0][0] + g[1][0]) / 2) + ' 0 ' + f5((g[0][2] + zs) / 2) +
             ' children [ Box { size ' + f5(g[1][0] - g[0][0]) + ' ' + f5(w) + ' ' + f5(zs - g[0][2]) + ' } ] } ' +
             'Pose { translation ' + f5((Math.min(xf, xo) + Math.max(xf, xo)) / 2) + ' 0 ' + f5(zs + jh / 2) +
             ' children [ Box { size ' + f5(jt) + ' ' + f5(w) + ' ' + f5(jh) + ' } ] } ] }';
    }
    const L = S.body[1][0] - S.body[0][0], W = S.body[1][1] - S.body[0][1];
    const bolt = W / 2 + 0.02;                           // T-slot bolts of the base plate, along y
  >%%
  Solid {
    translation IS translation
    rotation IS rotation
    name IS name
    model %%<= '"SCHUNK ' + S.designation + '"' >%%
    children [
      %%< if (h0 > 0) { >%%
      Pose {
        translation 0 0 %%<= f5(h0 / 2) >%%
        children [
          Shape {
            appearance PBRAppearance { baseColor 0.42 0.44 0.47 roughness 0.45 metalness 0.7 }
            geometry Box { size %%<= f5(L + 0.01) >%% %%<= f5(W + 0.08) >%% %%<= f5(h0) >%% }
          }
        ]
      }
      %%< for (const sy of [-1, 1]) { >%%
      Pose {
        translation 0 %%<= f5(sy * bolt) >%% %%<= f5(h0 + 0.005) >%%
        children [
          Shape {
            appearance PBRAppearance { baseColor 0.1 0.1 0.11 roughness 0.4 metalness 0.6 }
            geometry Cylinder { height 0.01 radius 0.012 subdivision 6 }
          }
        ]
      }
      %%< } >%%
      %%< } >%%
      Pose {
        translation 0 0 %%<= f5(h0) >%%
        children [
          %%< for (const fn of S.bodyFiles) { >%%
          Shape {
            appearance DEF KSP3_BODY PBRAppearance { baseColor %(body_colour)s roughness 0.45 metalness 0.5 }
            geometry Mesh { url %%<= '"meshes/schunk_ksp3/' + fn + '"' >%% }
          }
          %%< } >%%
          Pose {
            translation %%<= f5(S.gapCentre) >%% 0 %%<= f5(S.bodyTopGap + 0.0005) >%%
            rotation 0 1 0 -1.5708
            children [
              RangeFinder {
                name %%<= '"' + pre + 'part sensor"' >%%
                width 1
                height 1
                fieldOfView 0.05
                near 0.0005
                minRange 0.0005
                maxRange 0.1
              }
            ]
          }
          %%< if (fields.purgeNozzle.value) { >%%
          Pose {
            translation %%<= f5(S.gapCentre) >%% %%<= f5(-W / 2 - 0.006) >%% %%<= f5(zs + 0.006) >%%
            children [
              Pose {
                rotation 1 0 0 1.5708
                children [
                  Shape {
                    appearance PBRAppearance { baseColor 0.75 0.6 0.2 roughness 0.3 metalness 0.9 }
                    geometry Cylinder { height 0.012 radius 0.004 subdivision 12 }
                  }
                ]
              }
              Pose {
                translation 0 %%<= f5(0.006 + 0.5 * (W + 0.02)) >%% 0.004
                rotation 1 0 0 1.5708
                children [
                  Shape {
                    appearance PBRAppearance { baseColor 0.85 0.92 1 roughness 0.2 metalness 0 transparency IS purgeTransparency }
                    geometry Cone { bottomRadius %%<= f5(0.25 * W) >%% height %%<= f5(W + 0.02) >%% subdivision 16 side TRUE bottom FALSE }
                    castShadows FALSE
                  }
                ]
              }
            ]
          }
          %%< } >%%
          Pose {
            translation 0 0 %%<= f5(zs + sh) >%%
            children IS workpiece
          }
          SliderJoint {
            jointParameters JointParameters { position %%<= f5(p0) >%% axis 1 0 0 minStop 0 maxStop %%<= f5(st) >%% }
            device [
              LinearMotor {
                name %%<= '"' + pre + 'jaw motor::a"' >%%
                minPosition 0 maxPosition %%<= f5(st) >%% maxVelocity %%<= f5(S.speed) >%% maxForce %%<= S.force / 2 >%%
              }
              PositionSensor { name %%<= '"' + pre + 'jaw sensor"' >%% }
            ]
            endPoint Solid {
              translation %%<= f5(p0) >%% 0 0
              name "jaw a"
              children [
                %%< for (const fn of S.jawAFiles) { >%%
                Shape { appearance DEF KSP3_JAW PBRAppearance { baseColor %(jaw_colour)s roughness 0.3 metalness 0.85 } geometry Mesh { url %%<= '"meshes/schunk_ksp3/' + fn + '"' >%% } }
                %%< } >%%
                %%<= jawApp(topJaw(-1)) >%%
              ]
              boundingObject %%<= jawBox(-1) >%%
            }
          }
          SliderJoint {
            jointParameters JointParameters { position %%<= f5(p0) >%% axis -1 0 0 minStop 0 maxStop %%<= f5(st) >%% }
            device [
              LinearMotor {
                name %%<= '"' + pre + 'jaw motor::b"' >%%
                minPosition 0 maxPosition %%<= f5(st) >%% maxVelocity %%<= f5(S.speed) >%% maxForce %%<= S.force / 2 >%%
              }
            ]
            endPoint Solid {
              translation %%<= f5(-p0) >%% 0 0
              name "jaw b"
              children [
                %%< for (const fn of S.jawBFiles) { >%%
                Shape { appearance USE KSP3_JAW geometry Mesh { url %%<= '"meshes/schunk_ksp3/' + fn + '"' >%% } }
                %%< } >%%
                %%<= jawApp(topJaw(1)) >%%
              ]
              boundingObject %%<= jawBox(1) >%%
            }
          }
        ]
      }
    ]
    boundingObject Group {
      children [
        %%< if (h0 > 0) { >%%
        Pose { translation 0 0 %%<= f5(h0 / 2) >%% children [ Box { size %%<= f5(L + 0.01) >%% %%<= f5(W + 0.08) >%% %%<= f5(h0) >%% } ] }
        %%< } >%%
        Pose { translation 0 0 %%<= f5(h0 + S.bodyTopGap / 2) >%% children [ Box { size %%<= f5(L) >%% %%<= f5(W) >%% %%<= f5(S.bodyTopGap) >%% } ] }
      ]
    }
  }
}
"""


def write_proto(path, metas):
    sizes = {}
    for size, m in metas.items():
        kp, g = m["key_positions"], m["groups"]
        sizes[size] = dict(designation=m["designation"], stroke=kp["stroke_per_jaw"], force=kp["clamping_force_N"],
                           speed=kp["jaw_speed"], seat=kp["top_jaw_seat_z"], jawWidth=kp["base_jaw_width_y"],
                           body=kp["body_bbox"], bodyTopGap=kp["body_top_in_gap_z"], gapCentre=kp["jaw_gap_centre_x"],
                           jawA=g["jaw_a"]["bbox_open"], jawB=g["jaw_b"]["bbox_open"],
                           bodyFiles=g["body"]["files"], jawAFiles=g["jaw_a"]["files"], jawBFiles=g["jaw_b"]["files"])
    txt = PROTO % dict(attribution=ATTRIBUTION, sizes=json.dumps(sizes, sort_keys=True, separators=(",", ":")),
                       body_colour=" ".join("%g" % c for c in COLOURS["body"]),
                       jaw_colour=" ".join("%g" % c for c in COLOURS["jaw"]))
    with open(path, "w") as fh:
        fh.write(txt)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("step100")
    ap.add_argument("step160")
    ap.add_argument("out_dir")
    ap.add_argument("--proto", help="also write the Webots PROTO (expects out_dir = <proto dir>/meshes/schunk_ksp3)")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    metas = {"100": convert(args.step100, "100", args.out_dir), "160": convert(args.step160, "160", args.out_dir)}
    meta = dict(attribution=ATTRIBUTION, licence=LICENCE,
                licence_note="meshes = format conversion of the SCHUNK/CADENAS STEP (no modification of the shape); "
                             "keep the attribution with any redistribution",
                units="metres", sizes=metas)
    with open(os.path.join(args.out_dir, "schunk_ksp3.json"), "w") as fh:
        json.dump(meta, fh, indent=1, sort_keys=True)
    if args.proto:
        write_proto(args.proto, metas)
    for size, m in metas.items():
        kp = m["key_positions"]
        print("KSP3-F %s: body L %.1f mm, %d triangles; jaw gap open %.1f / closed %.1f mm, centre x %.1f mm, "
              "base jaw top %.1f mm, seat %.1f mm, body top in gap %.1f mm, screws %.1f mm below"
              % (size, m["body_length_mm"], m["total_triangles"], kp["jaw_gap_open"] * 1000, kp["jaw_gap_closed"] * 1000,
                 kp["jaw_gap_centre_x"] * 1000, kp["base_jaw_top_z"] * 1000, kp["top_jaw_seat_z"] * 1000,
                 kp["body_top_in_gap_z"] * 1000, kp["screws_below_mounting_face"] * 1000))
        for fn, f in m["files"].items():
            print("  %-26s %6d tri %8.1f kB" % (fn, f["triangles"], f["bytes"] / 1024))


if __name__ == "__main__":
    sys.exit(main())
