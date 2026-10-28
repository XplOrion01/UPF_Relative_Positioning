"""
fim_toolkit.py -- Fisher Information Matrix (FIM) tools for 2D UWB-range + odometry relative pose
Based on: T. H. Nguyen and L. Xie, "Relative Transformation Estimation Based on Fusion of
Odometry and UWB Ranging Data", IEEE T-RO, vol. 39, no. 4, 2023  (2-D case: Appendix B-B, eq. 60-69)

Paper's unknown (2-D, no common heading reference):   Theta = [tx, ty, theta]
   = the INITIAL frame transformation T_L1L2 (pose of B's start frame in A's start frame).
Measurements: range d_i = | t + R(theta) o_i - phi_i |
   phi_i = position of A in its own start frame L1 (dead-reckoned odometry)
   o_i   = position of B in its own start frame L2 (dead-reckoned odometry)
Jacobian row (eq. 61):   G_i = [gx/d, gy/d, gtheta/d]
   gx = tx + ox cos(th) - oy sin(th) - phix ;  gy = ty + ox sin(th) + oy cos(th) - phiy
   gtheta = gx*(-ox sin(th) - oy cos(th)) + gy*(ox cos(th) - oy sin(th))
FIM (eq. 14/16):          F = (1/sigma_r^2) * sum_i G_i^T G_i ,   CRLB = F^-1
NOTE: the paper's FIM assumes noise-free odometry and only UWB noise sigma_r, so it is a
best-case bound; odometry drift makes the real error larger.
"""
import numpy as np


# ---------------- SE(2) helpers: pose = (x, y, yaw) ----------------
def compose(a, b):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]])


def inverse(a):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), -a[2]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ---------------- core FIM math ----------------
def jacobian_rows(theta_vec, phi, o):
    """theta_vec=[tx,ty,th]; phi, o: (n,2) arrays -> (G (n,3), d (n,))"""
    tx, ty, th = theta_vec
    c, s = np.cos(th), np.sin(th)
    gx = tx + o[:, 0] * c - o[:, 1] * s - phi[:, 0]
    gy = ty + o[:, 0] * s + o[:, 1] * c - phi[:, 1]
    d = np.maximum(np.hypot(gx, gy), 1e-9)
    gth = gx * (-o[:, 0] * s - o[:, 1] * c) + gy * (o[:, 0] * c - o[:, 1] * s)
    return np.stack([gx / d, gy / d, gth / d], 1), d


def fim_matrix(G, sigma_r):
    return G.T @ G / sigma_r ** 2


def analyse_fim(F, cond_thr=1e6):
    """condition number, eigenvalues, CRLB-based standard errors, singular flag.
    Parameters that share a (near-)null direction of F get sigma = inf (not resolvable)."""
    w, V = np.linalg.eigh(F)
    lam_max = max(w[-1], 1e-300)
    lam_min = max(w[0], 0.0)
    floor = 1e-10 * lam_max
    cond = np.inf if lam_min <= floor else lam_max / lam_min
    var = (V ** 2 / np.maximum(w, floor)).sum(axis=1)          # diag of pinv with floored eigenvalues
    sig = np.sqrt(var)
    null = w <= 1e-8 * lam_max
    if null.any():
        sig = np.where((V[:, null] ** 2).sum(axis=1) > 1e-6, np.inf, sig)   # params touched by a null direction
    sig = np.where(sig > 1e3, np.inf, sig)
    out = {
        "cond": cond, "eig_min": lam_min, "eig_max": w[-1],
        "sigma_tx": float(sig[0]), "sigma_ty": float(sig[1]),
        "sigma_t": float(np.sqrt(sig[0] ** 2 + sig[1] ** 2)),            # sqrt(CRLB_t), eq. 17
        "sigma_theta_deg": float(np.degrees(sig[2])),                    # sqrt(CRLB_theta)
        "singular": bool(cond > cond_thr),
        "det": float(np.linalg.det(F)),
    }
    out["ci95_t"] = 1.96 * out["sigma_t"]                                # eq. 51 style 95% interval
    return out


# ---------------- closed-form det(F) for k = 3 (eq. 29), used to verify the maths ----------------
def det_F_eq29(theta_vec, phi, o, sigma_r):
    """det(G)^2 for exactly 3 measurements from the paper's closed form (eq. 29).
    (The paper prints a 1/sigma_r^2 prefactor; for a 3x3 FIM the exact factor is sigma_r^-6, so
    we return the sigma-free part and compare it with det(G^T G).)"""
    tx, ty, th = theta_vec
    c, s = np.cos(th), np.sin(th)
    Co = np.stack([o[:, 0] * c - o[:, 1] * s, o[:, 0] * s + o[:, 1] * c], 1)
    g = np.array([tx, ty]) + Co - phi
    d = np.linalg.norm(g, axis=1)
    u = g / d[:, None]
    tot = 0.0
    for i in range(3):
        p, q = [j for j in range(3) if j != i]
        cross_z = u[p, 0] * u[q, 1] - u[p, 1] * u[q, 0]
        alpha = np.arctan2(cross_z, u[p] @ u[q])
        # gamma_l = pi/2 - angle(uz x C o_l, u_l);  uz x v = (-vy, vx)
        w = np.array([-Co[i, 1], Co[i, 0]])
        cosang = (w @ u[i]) / max(np.linalg.norm(w) * np.linalg.norm(u[i]), 1e-12)
        gamma = np.pi / 2 - np.arccos(np.clip(cosang, -1, 1))
        rho = np.linalg.norm(o[i])
        tot += (-1) ** i * rho * np.sin(alpha) * np.sin(gamma)
    return tot ** 2


# ---------------- online monitor ----------------
class FIMMonitor:
    """Feed it the SAME shared data as the filter: both odometry increments + UWB range.
    It dead-reckons both robots from the start of the run, keeps a sliding window of
    (phi_i, o_i), and evaluates the FIM at the filter's own estimate."""

    def __init__(self, sigma_r, window=500, cond_thr=1e6, move_thr=0.05, move_len=100):
        self.sigma_r, self.window, self.cond_thr = sigma_r, window, cond_thr
        self.move_thr, self.move_len = move_thr, move_len
        self.pa = np.zeros(3)      # A's dead-reckoned pose in its start frame L1
        self.pb = np.zeros(3)      # B's dead-reckoned pose in its start frame L2
        self.phi, self.o = [], []

    def push(self, odo_a, odo_b):
        self.pa = compose(self.pa, np.asarray(odo_a, float))
        self.pb = compose(self.pb, np.asarray(odo_b, float))
        self.phi.append(self.pa[:2].copy()); self.o.append(self.pb[:2].copy())

    def theta_from_rel(self, rel):
        """initial transformation from the filter's CURRENT relative pose (B in A's body frame)."""
        T = compose(compose(self.pa, np.asarray(rel, float)), inverse(self.pb))
        return np.array([T[0], T[1], wrap(T[2])])

    def moving_enough(self):
        """paper's cheap pre-check (Sec. V-D1): both robots must have moved over the last samples."""
        if len(self.phi) < self.move_len:
            return False
        a = np.array(self.phi[-self.move_len:]); b = np.array(self.o[-self.move_len:])
        return bool(a.std(axis=0).max() > self.move_thr and b.std(axis=0).max() > self.move_thr)

    def report(self, rel_est=None, theta=None):
        if theta is None:
            theta = self.theta_from_rel(rel_est)
        phi = np.array(self.phi[-self.window:]); o = np.array(self.o[-self.window:])
        G, d = jacobian_rows(theta, phi, o)
        r = analyse_fim(fim_matrix(G, self.sigma_r), self.cond_thr)
        r["moving_enough"] = self.moving_enough()
        return r


# ---------------- FIM-based measurement selection (D-optimal, greedy) ----------------
def greedy_d_optimal(G, m, reg=None, exchange_passes=2, min_gap=0):
    """pick m rows of G (n,3) that (approximately) maximise log det(sum G_i^T G_i):
    greedy rank-one build-up, then Fedorov-style exchange passes. Use it to feed only the most
    informative measurements to an algebraic solver."""
    n = G.shape[0]
    m = min(m, n)
    if reg is None:
        reg = 1e-2 * max(np.trace(G.T @ G) / n, 1e-12)      # keeps early picks well-conditioned
    ld = lambda ids: np.linalg.slogdet(reg * np.eye(3) + G[ids].T @ G[ids])[1]
    M = reg * np.eye(3); chosen = []
    free = np.ones(n, bool)
    for _ in range(m):
        gains = np.einsum('ij,jk,ik->i', G, np.linalg.inv(M), G)
        gains[~free] = -np.inf
        i = int(np.argmax(gains)); chosen.append(i); free[i] = False
        if min_gap:
            free[max(0, i - min_gap):i + min_gap + 1] = False      # forbid near-duplicate samples
        M += np.outer(G[i], G[i])
    best = ld(chosen)
    for _ in range(exchange_passes):
        improved = False
        for pos in range(m):
            for cand in np.nonzero(free)[0]:
                if min_gap and any(abs(cand - c) <= min_gap for j, c in enumerate(chosen) if j != pos):
                    continue
                trial = chosen.copy(); trial[pos] = cand
                v = ld(trial)
                if v > best + 1e-9:
                    free[chosen[pos]] = True; free[cand] = False
                    chosen, best, improved = trial, v, True
        if not improved:
            break
    return sorted(chosen)
