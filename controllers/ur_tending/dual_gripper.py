"""Geometry of the dual gripper on the UR10e flange (shared by the robot controller, the PLC, the world generator and
the offline collision checker).

toolSlot frame: +y = flange normal (out of the flange), x / z in the flange plane.
A 90° bent bracket ("Λ" shape) carries the two parallel grippers at ±45° from the flange normal:
    gripper 1 points along d1 = (+s, c, 0), gripper 2 along d2 = (-s, c, 0)       (s = c = sin 45°)
so a half turn of wrist 3 brings either gripper onto the other's approach direction.
The fingers close along toolSlot z (perpendicular to the bracket plane).
"""
import math

ROBOT_YAW = -1.5708                  # orientation of the UR10e base in the cell: its zero pose (arm stretched out) points
                                   # to the free back side, so the start-up never sweeps through a machine


def base_q(q_cell):
    """Joint values given with the cell axes (shoulder pan measured from the cell x axis) -> robot joint values."""
    a = q_cell[0] - ROBOT_YAW
    return [(a + math.pi) % (2 * math.pi) - math.pi] + list(q_cell[1:])

S = math.sqrt(0.5)
PLATE_TOP = 0.057                  # top of the aluminium adapter plate on the F/T sensor
BEND_LEN = 0.10                    # length of each bracket leg (in the bracket plane)
APEX_Y = PLATE_TOP + BEND_LEN * S  # bend line of the bracket, on the flange axis
THETA = {1: math.pi / 4, 2: 3 * math.pi / 4}                      # gripper local x = Rz(theta) x
FACE = {g: ((1 if g == 1 else -1) * BEND_LEN / 2 * S, APEX_Y - BEND_LEN / 2 * S, 0.0) for g in (1, 2)}
BODY_L = 0.12                      # gripper body length along the approach (from the bracket face)
FINGER_X0 = BODY_L                 # finger base (gripper local x)
TCP_X = BODY_L + 0.045             # TCP = middle of the finger pads (gripper local x)
FINGER_L = 0.057                   # finger length: tip 12 mm beyond the TCP


def direction(g):
    t = THETA[g]
    return (math.cos(t), math.sin(t), 0.0)


def local_to_slot(g, p):
    """Point in gripper g's local frame (x = approach, z = finger axis) -> toolSlot frame."""
    t = THETA[g]
    c, s = math.cos(t), math.sin(t)
    f = FACE[g]
    return (f[0] + c * p[0] - s * p[1], f[1] + s * p[0] + c * p[1], f[2] + p[2])


def tcp_matrix(g):
    """4x4 pose of TCP g in the toolSlot frame: z = approach, x = finger axis (toolSlot z), y = z × x."""
    z = direction(g)
    x = (0.0, 0.0, 1.0)
    y = (z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0])
    p = local_to_slot(g, (TCP_X, 0, 0))
    return [[x[0], y[0], z[0], p[0]], [x[1], y[1], z[1], p[1]], [x[2], y[2], z[2], p[2]], [0, 0, 0, 1]]
