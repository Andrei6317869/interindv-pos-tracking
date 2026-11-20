"""
L-17 graph simulation (simulated data only, nothing measured on hardware).
Model = the teacher's beacon_app.html: unit-disc arena, beacon at the centre (node 0),
N participants uniform in the disc, link when distance <= R.  Run:  python3 graph_sim.py
Reproducible: seed 20261120.  Writes the table printed below to ../logs/L-17_graph_sim.txt
"""
import numpy as np, sys, os
SEED, TRIALS, N, G = 20261120, 2000, 10, 3
RS = [0.20, 0.30, 0.46, 0.60, 0.80]
rng = np.random.default_rng(SEED)

def components(A):
    n = len(A); lab = -np.ones(n, int); c = 0
    for s in range(n):
        if lab[s] >= 0: continue
        stack = [s]; lab[s] = c
        while stack:
            u = stack.pop()
            for v in np.nonzero(A[u])[0]:
                if lab[v] < 0: lab[v] = c; stack.append(v)
        c += 1
    return lab, c

def trial(R):
    r = np.sqrt(rng.random(N)); t = rng.random(N) * 2 * np.pi
    P = np.vstack([[0, 0], np.c_[r * np.cos(t), r * np.sin(t)]])      # row 0 = beacon
    D = np.hypot(P[:, None, 0] - P[None, :, 0], P[:, None, 1] - P[None, :, 1])
    A = ((D <= R) & ~np.eye(N + 1, dtype=bool)).astype(int)
    deg = A.sum(1)
    lab, ncomp = components(A)
    out = {}
    out["E"] = A.sum() // 2
    out["deg_mean"] = deg.mean()
    out["deg_min"] = deg.min(); out["deg_max"] = deg.max()
    out["beacon_deg"] = deg[0]
    out["comps"] = ncomp
    out["connected"] = float(ncomp == 1)
    out["reach"] = (lab == lab[0]).sum() - 1                      # participants with a path to the beacon
    out["isolated"] = int((deg[1:] == 0).sum())
    # subgraph detection: components that do not contain the beacon are separate groups
    sizes = np.bincount(lab, minlength=ncomp)
    out["detached_groups"] = int(sum(1 for c in range(ncomp) if c != lab[0] and sizes[c] >= 2))
    # one untrained message-passing layer: H = D~^-1 (A+I) X, X = 8 random skill bits per participant
    X = np.zeros((N + 1, 8)); X[1:] = (rng.random((N, 8)) < 0.35)
    At = A + np.eye(N + 1, dtype=int)
    H = (At @ X) / At.sum(1, keepdims=True)
    own = (X[1:] > 0).sum(1); seen = (H[1:] > 0).sum(1)
    out["skills_own"] = own.mean(); out["skills_1hop"] = seen.mean()
    # layer 2: old groups and group matrix W = S^T A S (participants only)
    g = rng.permutation(np.arange(N) % G)
    S = np.zeros((N, G)); S[np.arange(N), g] = 1
    Ap = A[1:, 1:]
    W = S.T @ Ap @ S
    cross = (W.sum() - np.trace(W)) / 2
    out["cross_links"] = cross
    Gadj = ((W > 0) & ~np.eye(G, dtype=bool)).astype(int)
    out["layer2_connected"] = float(components(Gadj)[1] == 1)
    # candidate new group = participants that hear the beacon directly; leader by connectivity
    cand = np.nonzero(A[0, 1:])[0]
    if len(cand) >= 2:
        sub = Ap[np.ix_(cand, cand)]
        dsub = sub.sum(1)
        lead = int(np.argmax(dsub))                                # ties -> lowest index (first max)
        rand = int(rng.integers(len(cand)))
        out["lead_cov"] = dsub[lead] / (len(cand) - 1)
        out["rand_cov"] = dsub[rand] / (len(cand) - 1)
        out["low_cov"] = dsub[0] / (len(cand) - 1)
        kk = Ap.sum(1)[cand]
        head = np.bincount(g[cand], minlength=G) / len(cand)
        share = np.bincount(g[cand], weights=kk, minlength=G)
        share = share / share.sum() if share.sum() > 0 else head
        out["share_skew"] = np.abs(share - head).sum() / 2
    return out

lines = []
def P(*a):
    s = " ".join(str(x) for x in a); lines.append(s); print(s)

def stat(vals):
    v = np.array(vals, float)
    return v.min(), v.max(), v.mean(), v.std(ddof=1)

P(f"L-17 graph simulation | seed {SEED} | {TRIALS} trials per R | N={N} participants + beacon | links when distance <= R | unit-disc arena")
P("Each cell: min / max / mean / std.dev over trials (sample std).")
keys = [("E","links E"),("deg_mean","mean node degree"),("deg_max","max node degree"),("beacon_deg","beacon degree"),
        ("comps","components (isolated nodes count)"),("detached_groups","detached groups (>=2 members, no path to beacon)"),("reach","participants with a path to the beacon"),("isolated","isolated participants"),
        ("cross_links","links between old groups")]
for R in RS:
    res = [trial(R) for _ in range(TRIALS)]
    P(f"\n--- R = {R:.2f} ---")
    for k, name in keys:
        mn, mx, me, sd = stat([r[k] for r in res])
        P(f"{name:42s} {mn:6.2f} / {mx:6.2f} / {me:6.2f} / {sd:5.2f}")
    P(f"{'P(graph fully connected)':42s} {np.mean([r['connected'] for r in res]):.3f}")
    P(f"{'P(>=1 detached group of 2+ members)':42s} {np.mean([r['detached_groups']>0 for r in res]):.3f}")
    P(f"{'P(old-group layer connected)':42s} {np.mean([r['layer2_connected'] for r in res]):.3f}")
    lc = [r for r in res if 'lead_cov' in r]
    P(f"{'trials with >=2 direct hearers':42s} {len(lc)} of {TRIALS}")
    for k, name in [("lead_cov","leader=max degree: share of members it links"),("rand_cov","random leader: share of members it links"),
                    ("low_cov","lowest-id leader: share of members it links"),("share_skew","degree-weighted vs head-count group share")]:
        mn, mx, me, sd = stat([r[k] for r in lc])
        P(f"{name:42s} {mn:6.2f} / {mx:6.2f} / {me:6.2f} / {sd:5.2f}")
    P(f"{'skills seen: own vs after 1 message-pass':42s} {np.mean([r['skills_own'] for r in res]):.2f} -> {np.mean([r['skills_1hop'] for r in res]):.2f} of 8")
here = os.path.dirname(os.path.abspath(__file__))
os.makedirs(os.path.join(here, "..", "logs"), exist_ok=True)
open(os.path.join(here, "..", "logs", "L-17_graph_sim.txt"), "w").write("\n".join(lines) + "\n")
