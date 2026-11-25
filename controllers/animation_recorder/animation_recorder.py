"""Enregistre la cellule en animation Webots HTML5 (scène 3D interactive lisible dans un navigateur), puis quitte.
Ajouté à une copie du monde par tools/build_site.sh.

    controllerArgs: <sortie.html> <début_s> <fin_s>
"""
import sys

from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
out, t_start, t_end = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
recording = False
while sup.step(dt) != -1:
    t = sup.getTime()
    if not recording and t >= t_start:
        sup.animationStartRecording(out)
        recording = True
    if recording and t >= t_end:
        sup.animationStopRecording()
        sup.step(dt)
        break
sup.simulationQuit(0)
