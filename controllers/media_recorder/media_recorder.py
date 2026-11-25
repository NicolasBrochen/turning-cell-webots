"""Enregistre les vidéos du README, puis quitte. Ajouté à une copie du monde par tools/build_media.sh.

    controllerArgs: <plans.json>
plans.json : {"width": W, "height": H, "clips": [
    {"file": "...mp4", "start": s, "end": s, "accel": k, "position": [x,y,z], "look_at": [x,y,z]}]}
Les plans sont enchaînés dans l'ordre (temps simulé absolu, sans recouvrement).
"""
import json
import math
import sys

from controller import Supervisor


def look_rotation(pos, tgt):
    """Rotation axe-angle d'un Viewpoint ENU (visée selon +x, haut selon +z) regardant `tgt`."""
    d = [tgt[i] - pos[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in d))
    x = [c / n for c in d]
    y = [-x[1], x[0], 0.0]                       # up(0,0,1) ^ x
    ny = math.sqrt(y[0] ** 2 + y[1] ** 2)
    y = [y[0] / ny, y[1] / ny, 0.0]
    z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]]
    m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    c = max(-1.0, min(1.0, (m[0][0] + m[1][1] + m[2][2] - 1) / 2))
    a = math.acos(c)
    s = 2 * math.sin(a)
    return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, a]


sup = Supervisor()
dt = int(sup.getBasicTimeStep())
cfg = json.load(open(sys.argv[1]))
vp = sup.getFromDef("VIEWPOINT")
clips = list(cfg["clips"])
clip, recording = None, False
while sup.step(dt) != -1 and (clips or clip):
    t = sup.getTime()
    if clip is None:
        clip = clips.pop(0)
    if not recording and t >= clip["start"]:
        vp.getField("position").setSFVec3f(clip["position"])
        vp.getField("orientation").setSFRotation(look_rotation(clip["position"], clip["look_at"]))
        sup.movieStartRecording(clip["file"], cfg["width"], cfg["height"], 0, 95, clip.get("accel", 1), False)
        recording = True
    if recording and t >= clip["end"]:
        sup.movieStopRecording()
        while not sup.movieIsReady():
            sup.step(dt)
        recording, clip = False, None
sup.simulationQuit(0)
