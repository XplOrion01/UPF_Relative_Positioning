import numpy as np, fim_toolkit as F
rng = np.random.default_rng(1)

# 1) verify the paper's closed form (eq. 29) against the numeric Jacobian-based FIM for 3 measurements
errs = []
for _ in range(200):
    th = np.array([*rng.uniform(-5, 5, 2), rng.uniform(-3, 3)])
    phi = rng.uniform(-5, 5, (3, 2)); o = rng.uniform(-5, 5, (3, 2))
    G, d = F.jacobian_rows(th, phi, o)
    num = np.linalg.det(G.T @ G)
    cf = F.det_F_eq29(th, phi, o, 0.1)
    errs.append(abs(num - cf) / max(abs(num), 1e-12))
print(f"[check] eq.(29) closed form vs numeric det(G^T G): max rel diff = {max(errs):.2e}")

# 2) simulate trajectories (noise-free odometry, UWB sigma 0.1) and look at the FIM
def sim(mode, steps=1500, hz=50, seed=0, d0=3.0):
    r = np.random.default_rng(seed)
    A = np.zeros(3); B = np.array([d0, 0.0, 0.0 if mode == 'parallel straight' else 0.7])
    mon = F.FIMMonitor(0.1, window=1500)
    va, wa, vb, wb = 0.05, 0.01, 0.05, -0.01
    out = []
    for k in range(1, steps + 1):
        if k % 100 == 0:
            va, wa, vb, wb = r.uniform(.01, .04), r.uniform(-.02, .02), r.uniform(.01, .04), r.uniform(-.02, .02)
        if mode == "A static": oa = np.zeros(3); ob = np.array([vb, 0, wb])
        elif mode == "B static": oa = np.array([va, 0, wa]); ob = np.zeros(3)
        elif mode == "parallel straight": oa = np.array([0.03, 0, 0]); ob = np.array([0.03, 0, 0])
        elif mode == "both turning": oa = np.array([va, 0, wa]); ob = np.array([vb, 0, wb])
        A = F.compose(A, oa); B = F.compose(B, ob)
        mon.push(oa, ob)
        if k in (100, 300, 750, 1500):
            rel = F.compose(F.inverse(A), B)          # TRUE relative pose
            out.append((k / hz, mon.report(rel_est=rel)))
    return out

print(f"\n{'scenario':20s} {'t[s]':>5s} {'cond(F)':>10s} {'sigma_t[m]':>11s} {'sigma_th[deg]':>14s} {'singular':>9s} {'moving':>7s}")
for mode in ("both turning", "A static", "B static", "parallel straight"):
    for t, r in sim(mode):
        print(f"{mode:20s} {t:5.1f} {r['cond']:10.3g} {r['sigma_t']:11.3f} {r['sigma_theta_deg']:14.2f} {str(r['singular']):>9s} {str(r['moving_enough']):>7s}")

# 3) D-optimal selection demo: from 600 measurements of the 'both turning' run, which 6 are most informative?
r = np.random.default_rng(3); A = np.zeros(3); B = np.array([3, 0, .7]); mon = F.FIMMonitor(0.1, window=1500)
va, wa, vb, wb = .05, .01, .05, -.01
for k in range(1, 601):
    if k % 100 == 0: va, wa, vb, wb = r.uniform(.01, .04), r.uniform(-.02, .02), r.uniform(.01, .04), r.uniform(-.02, .02)
    oa = np.array([va, 0, wa]); ob = np.array([vb, 0, wb]); A = F.compose(A, oa); B = F.compose(B, ob); mon.push(oa, ob)
th = mon.theta_from_rel(F.compose(F.inverse(A), B))
G, d = F.jacobian_rows(th, np.array(mon.phi), np.array(mon.o))
idx = F.greedy_d_optimal(G, 6)
full = np.linalg.slogdet(F.fim_matrix(G, 0.1))[1]
sub = np.linalg.slogdet(F.fim_matrix(G[idx], 0.1))[1]
rnd = np.mean([np.linalg.slogdet(F.fim_matrix(G[r.choice(600, 6, replace=False)], 0.1))[1] for _ in range(200)])
uni = np.linalg.slogdet(F.fim_matrix(G[np.linspace(0, 599, 6).astype(int)], 0.1))[1]
print(f"\n[selection] logdet(F): all 600 = {full:.1f} | greedy D-opt 6 = {sub:.1f} | evenly spaced 6 = {uni:.1f} | random 6 (mean) = {rnd:.1f}")
print("[selection] chosen sample indices:", idx)
