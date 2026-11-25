"""Cinématique du Universal Robots UR10e (modèle Webots R2025a) : FK exacte + IK numérique 6 ddl.

Chaîne relevée dans UR10e.proto : chaque liaison = translation fixe, puis rotation articulaire, puis rotation fixe
du Solid (l'ancre de chaque HingeJoint coïncide avec la translation du Solid).
Le repère `toolSlot` est à +0.12 m selon y de la liaison 6 : son axe +y est la normale sortante de la bride.

L'IK est un Levenberg-Marquardt (moindres carrés amortis) sur l'erreur position + orientation, jacobien par
différences finies, sans dépendance (ni numpy, ni ikpy). Démarrée depuis la configuration courante, elle converge
en 1 à 3 itérations pendant un mouvement linéaire.
"""
import math

LIMITS = [(-6.2, 6.2), (-6.2, 6.2), (-3.1416, 3.1416),
          (-6.2, 6.2), (-6.2, 6.2), (-6.2, 6.2)]          # kept inside the ±2π motor limits


# ---------------------------------------------------------------- matrices 4x4 (listes)
def mat_mul(a, b):
    return [[a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j] for j in range(4)]
            for i in range(4)]


def tr(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def rot(axis, a):
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s, 0],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s, 0],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C, 0],
            [0, 0, 0, 1]]


def inv(T):
    R = [[T[j][i] for j in range(3)] for i in range(3)]
    p = [-(R[i][0] * T[0][3] + R[i][1] * T[1][3] + R[i][2] * T[2][3]) for i in range(3)]
    return [R[0] + [p[0]], R[1] + [p[1]], R[2] + [p[2]], [0, 0, 0, 1]]


Z, Y = (0, 0, 1), (0, 1, 0)
HALF = math.pi / 2
CHAIN = [  # (translation, axe articulaire, rotation fixe du Solid (axe, angle))
    ((0, 0, 0.181), Z, None),
    ((0, 0.176, 0), Y, (Y, HALF)),
    ((0, -0.137, 0.613), Y, None),
    ((0, 0, 0.571), Y, (Y, HALF)),
    ((0, 0.135, 0), Z, None),
    ((0, 0, 0.12), Y, None),
]
TOOL_SLOT = tr(0, 0.12, 0)


def fk(q, tcp=None):
    """Pose 4x4 du toolSlot (ou du TCP si `tcp` = pose 4x4 du TCP dans le toolSlot), repère base robot."""
    T = tr(0, 0, 0)
    for (t, ax, fixed), qi in zip(CHAIN, q):
        T = mat_mul(T, tr(*t))
        T = mat_mul(T, rot(ax, qi))
        if fixed:
            T = mat_mul(T, rot(*fixed))
    T = mat_mul(T, TOOL_SLOT)
    return mat_mul(T, tcp) if tcp is not None else T


def _err(T, G):
    """Erreur 6D : position + vecteur rotation (angle * axe) de R_err = R_G R_T^T (log exact, y compris près de pi)."""
    e = [G[i][3] - T[i][3] for i in range(3)]
    R = [[sum(G[i][k] * T[j][k] for k in range(3)) for j in range(3)] for i in range(3)]   # G_R * T_R^T
    c = max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
    ang = math.acos(c)
    if ang < 1e-9:
        return e + [0.0, 0.0, 0.0]
    if ang > math.pi - 1e-3:     # ~180° : axe depuis la diagonale de R
        ax = [math.sqrt(max(0.0, (R[i][i] + 1) / 2)) for i in range(3)]
        k = max(range(3), key=lambda i: ax[i])
        for i in range(3):
            if i != k:
                ax[i] = math.copysign(ax[i], R[k][i] + R[i][k])
        return e + [a_ * ang for a_ in ax]
    s_ = 2 * math.sin(ang)
    ax = [(R[2][1] - R[1][2]) / s_, (R[0][2] - R[2][0]) / s_, (R[1][0] - R[0][1]) / s_]
    return e + [a_ * ang for a_ in ax]


def _solve(A, b):
    """Résout A x = b (Gauss avec pivot partiel), A 6x6."""
    n = len(b)
    M = [A[i][:] + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-12:
            return None
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            for k in range(c, n + 1):
                M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x


def ik(G, seed, tcp=None, iters=60, tol=1e-5, lam=0.02):
    """Configuration articulaire amenant le TCP sur la pose 4x4 G (repère base). None si non convergée."""
    q = list(seed)
    for _ in range(iters):
        T = fk(q, tcp)
        e = _err(T, G)
        if math.sqrt(sum(v * v for v in e)) < tol:
            return q
        J = []
        h = 1e-6
        for j in range(6):
            dq = list(q)
            dq[j] += h
            ej = _err(fk(dq, tcp), G)
            J.append([(e[i] - ej[i]) / h for i in range(6)])     # colonne j = d(pose)/dq_j
        # (J^T J + lam^2 I) dq = J^T e   (J stocké par colonnes)
        A = [[sum(J[a][i] * J[b][i] for i in range(6)) + (lam * lam if a == b else 0) for b in range(6)]
             for a in range(6)]
        rhs = [sum(J[a][i] * e[i] for i in range(6)) for a in range(6)]
        d = _solve(A, rhs)
        if d is None:
            return None
        step = max(abs(v) for v in d)
        if step > 0.4:
            d = [v * 0.4 / step for v in d]
        q = [qi + di for qi, di in zip(q, d)]
        q[2] = max(-3.1416, min(3.1416, q[2]))      # the elbow is the only joint without full-turn freedom
    T = fk(q, tcp)
    e = _err(T, G)
    return q if math.sqrt(sum(v * v for v in e)) < 1e-3 else None


def wrap_near(q, seed):
    """For each ±2π joint, pick the 2π-equivalent angle closest to the seed that stays within LIMITS."""
    out = []
    for j, (qi, si) in enumerate(zip(q, seed)):
        lo, hi = LIMITS[j]
        if j == 2:
            out.append(qi)
            continue
        cands = [qi + 2 * math.pi * k for k in range(-3, 4)]
        cands = [c for c in cands if lo <= c <= hi]
        out.append(min(cands, key=lambda c: abs(c - si)) if cands else max(lo, min(hi, qi)))
    return out


def pose(p, z_axis, x_axis):
    """Pose 4x4 à partir d'une position, de l'axe z (approche) et d'un axe x approché (orthonormalisé)."""
    zn = math.sqrt(sum(c * c for c in z_axis))
    z = [c / zn for c in z_axis]
    d = sum(a * b for a, b in zip(x_axis, z))
    x = [a - d * b for a, b in zip(x_axis, z)]
    xn = math.sqrt(sum(c * c for c in x))
    x = [c / xn for c in x]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    return [[x[0], y[0], z[0], p[0]], [x[1], y[1], z[1], p[1]], [x[2], y[2], z[2], p[2]], [0, 0, 0, 1]]


if __name__ == "__main__":
    import random
    print("FK zéro :", [round(v, 4) for v in [fk([0] * 6)[i][3] for i in range(3)]])
    random.seed(1)
    ok = 0
    for k in range(200):
        qt = [random.uniform(-2.5, 2.5) for _ in range(6)]
        qt[2] = random.uniform(-2.5, 2.5)
        G = fk(qt)
        seed = [a + random.uniform(-0.3, 0.3) for a in qt]
        q = ik(G, seed)
        if q is not None:
            T = fk(q)
            if max(abs(T[i][j] - G[i][j]) for i in range(3) for j in range(4)) < 1e-4:
                ok += 1
    print("IK : %d/200 convergées (graine à ±0.3 rad)" % ok)
