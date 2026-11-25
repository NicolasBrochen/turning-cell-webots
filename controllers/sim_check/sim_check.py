"""Stops the simulation after argv[1] simulated seconds (used by tools/sim_check.sh)."""
import sys

from controller import Supervisor

sv = Supervisor()
end = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
dt = int(sv.getBasicTimeStep())
while sv.step(dt) != -1:
    if sv.getTime() >= end:
        print("SIM_CHECK end at %.1f s" % sv.getTime())
        sys.stdout.flush()
        sv.simulationQuit(0)
        break
