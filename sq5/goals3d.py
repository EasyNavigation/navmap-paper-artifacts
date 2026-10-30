#!/usr/bin/env python3
"""Goals for SQ5 in a 3D world, from its SQ1 ground truth (sq1/mesh_truth.py, the collision mesh):
walkable samples in the component of the start, at least `clearance` from the walkable border,
on gentle ground, spread by farthest-point sampling in 3D (so every floor of a building gets
some). Each goal is at least `min_step` from the previous one. Deterministic.

With --navmap (tools/dist_dump of the map the robot navigates), goals must also be on that map,
`navmap_clearance` from its border: real goals a mission would give, not goals the map cannot hold
(its fidelity is SQ1's question). How many candidates this rules out is reported.

Usage: goals3d.py truth.npz out.json [--n 15] [--clearance 1.2] [--max-slope 10]
                  [--min-step 5] [--start 0 0] [--navmap dist.bin] [--navmap-clearance 1.0]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sq1'))
from mesh_truth import MeshTruth  # noqa: E402

Z_WEIGHT = 5.0  # a floor apart counts as far as 5 times its height


def on_navmap(t, cand, path, clearance, xy_tol=0.3, z_tol=0.15):
    """Mask of the candidate samples whose NavCels within xy_tol and z_tol (NavCel centroids,
    rows x y z nz surface dist) exist and are all at least `clearance` from the map's border."""
    D = np.fromfile(path, np.float32).reshape(-1, 6)

    def near(rows):
        mark = np.zeros(t.N, bool)
        r = int(math.ceil(xy_tol / t.res))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if math.hypot(di, dj) * t.res > xy_tol:
                    continue
                P = rows[:, :2] + np.array([di, dj]) * t.res
                start, end = t._cell_runs(t._key(P))
                for lv in range(int((end - start).max(initial=0))):
                    m = start + lv < end
                    k = start[m] + lv
                    close = np.abs(t.z[k] - rows[m, 2]) <= z_tol
                    mark[k[close]] = True
        return mark

    far = D[:, 5] >= clearance
    return (near(D[far]) & ~near(D[~far]))[cand]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('truth')
    ap.add_argument('out')
    ap.add_argument('--n', type=int, default=15)
    ap.add_argument('--clearance', type=float, default=1.2)
    ap.add_argument('--max-slope', type=float, default=10.0)
    ap.add_argument('--min-step', type=float, default=5.0)
    ap.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0])
    ap.add_argument('--navmap', help='tools/dist_dump output of the navigated NavMap')
    ap.add_argument('--navmap-clearance', type=float, default=1.0)
    a = ap.parse_args()
    t = MeshTruth.load(a.truth)
    t.edge_a, t.edge_b = t._edges()
    inner = t._interior(int(math.ceil(a.clearance / t.res)))
    gentle = t.local_slope() <= math.tan(math.radians(a.max_slope))
    # Start: the lowest sample under the spawn point (the robot spawns on the ground)
    d = np.hypot(t.xy[:, 0] - a.start[0], t.xy[:, 1] - a.start[1])
    near = np.flatnonzero(d <= d.min() + 1e-9)
    s = near[np.argmin(t.z[near])]
    cand = np.flatnonzero(inner & gentle & (t.comp == t.comp[s]))
    truth_cand = len(cand)
    if a.navmap:
        cand = cand[on_navmap(t, cand, a.navmap, a.navmap_clearance)]
        print(f'{truth_cand} candidates in the ground truth, {len(cand)} also on the NavMap')
    if len(cand) < a.n:
        raise SystemExit(f'only {len(cand)} candidate samples')
    P = np.c_[t.xy[cand], t.z[cand] * Z_WEIGHT]
    # Farthest-point sampling from the start, then ordered so that consecutive goals are apart
    picked = []
    dist = np.linalg.norm(P - np.r_[t.xy[s], t.z[s] * Z_WEIGHT], axis=1)
    for _ in range(a.n):
        k = int(np.argmax(dist))
        picked.append(k)
        dist = np.minimum(dist, np.linalg.norm(P - P[k], axis=1))
    order, last = [], np.r_[t.xy[s], t.z[s] * Z_WEIGHT]
    left = list(picked)
    while left:
        # The nearest one still at least min_step away (else the farthest): varied legs
        dd = [np.linalg.norm(P[k] - last) for k in left]
        ok = [i for i, x in enumerate(dd) if x >= a.min_step]
        i = min(ok, key=lambda i: dd[i]) if ok else int(np.argmax(dd))
        order.append(left.pop(i))
        last = P[order[-1]]
    rng = np.random.default_rng(0)
    goals = []
    for k in order:
        x, y = t.xy[cand[k]]
        goals.append({'x': round(float(x), 3), 'y': round(float(y), 3),
                      'z': round(float(t.z[cand[k]]), 3),
                      'yaw': round(float(rng.uniform(-math.pi, math.pi)), 3)})
    spec = {'start': [float(t.xy[s][0]), float(t.xy[s][1])], 'start_z': float(t.z[s]),
            'truth': os.path.basename(a.truth), 'clearance': a.clearance,
            'max_slope_deg': a.max_slope, 'truth_candidates': truth_cand,
            'navmap': os.path.basename(a.navmap) if a.navmap else None,
            'navmap_clearance': a.navmap_clearance if a.navmap else None,
            'candidates': len(cand), 'goals': goals}
    json.dump(spec, open(a.out, 'w'), indent=1)
    zs = sorted(g['z'] for g in goals)
    print(f'{len(goals)} goals, z from {zs[0]:.2f} to {zs[-1]:.2f}, start z {t.z[s]:.2f} '
          f'({len(cand)} candidates)')


if __name__ == '__main__':
    main()
