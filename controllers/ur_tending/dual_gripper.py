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
# SCHUNK EGK 40-MB-M-B (STEP from SCHUNK, meshes in protos/meshes/egk40): housing from the mounting face (x = 0) to
# 82.7 mm, base jaws up to 83.5 mm, jaw stroke 6 mm each, gripping force 140 N. Gripper local frame: x = approach,
# z = jaw motion (EGK z), y = -EGK x.
BODY_L = 0.0835                    # mounting face -> top of the base jaws
HOUSING = ((0.0, -0.034, -0.0764), (0.0827, 0.034, 0.0585))       # housing bounding box (gripper local frame)
JAW_STROKE = 0.006
JAW_Z0 = 0.0066                    # inner face of the base jaws, open (|z|)
GRIP_R = {1: 0.025, 2: 0.015}      # gripper 1 always takes blanks (Ø50), gripper 2 finished parts (Ø30 end)
PAD_GAP = 0.003                    # radial clearance of the open pads
FINGER_X0 = BODY_L + 0.012         # custom fingers: 12 mm adapter block on the base jaw, then the finger plate
TCP_X = 0.165                      # TCP = middle of the finger pads (gripper local x)
FINGER_END = TCP_X + 0.012         # finger tip, 12 mm beyond the TCP
FINGER_T = 0.010                   # finger plate thickness (along z)


def finger_plate_z(g):
    """|z| of the finger plate centre (open): pad inner face at GRIP_R + PAD_GAP, pad 4 mm, plate behind it."""
    return GRIP_R[g] + PAD_GAP + 0.004 + FINGER_T / 2


def finger_geometry(g, side):
    """Custom finger on a base jaw, open position (gripper local frame): adapter block, plate, pad."""
    zc = finger_plate_z(g)
    z_in = GRIP_R[g] + PAD_GAP                       # pad inner face (open)
    blk_z0, blk_z1 = JAW_Z0, max(0.0349, zc + FINGER_T / 2)
    L = FINGER_END - FINGER_X0
    return [("box", (0.012, 0.04, blk_z1 - blk_z0), (BODY_L + 0.006, 0, side * (blk_z0 + blk_z1) / 2), "block"),
            ("box", (L, 0.03, FINGER_T), (FINGER_X0 + L / 2, 0, side * zc), "plate"),
            ("box", (0.02, 0.026, 0.004), (TCP_X, 0, side * (z_in + 0.002)), "pad")]


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
