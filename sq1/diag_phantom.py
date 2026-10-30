#!/usr/bin/env python3
"""Where a representation's phantom area comes from: for each NavCel not on the true walkable
surface, what the collision mesh has at its centre.

Usage: diag_phantom.py mesh navmap truth_cache.npz [--ground lowest|highest]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from mesh_truth import CLEARANCE, load_world_mesh, MAX_DZ, MAX_SLOPE, MeshTruth  # noqa: E402
from render import load  # noqa: E402

TOL = 0.3


def surfaces_at(tris, x, y):
    """Heights and slopes (deg) of the mesh faces over (x, y)."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    ok = np.abs(d) > 1e-12
    dd = np.where(ok, d, 1.0)
    l1 = ((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1])) / dd
    l2 = ((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1])) / dd
    m = ok & (l1 >= 0) & (l2 >= 0) & (1 - l1 - l2 >= 0)
    z = (l1 * a[:, 2] + l2 * b[:, 2] + (1 - l1 - l2) * c[:, 2])[m]
    n = np.cross(b[m] - a[m], c[m] - a[m])
    slope = np.degrees(np.arccos(np.clip(np.abs(n[:, 2]) / np.linalg.norm(n, axis=1), 0, 1)))
    return z, slope


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mesh')
    ap.add_argument('navmap')
    ap.add_argument('truth')
    ap.add_argument('--ground', default='lowest')
    ap.add_argument('--samples', type=int, default=3000)
    a = ap.parse_args()
    tris = load_world_mesh(a.mesh, a.ground)
    t = MeshTruth.load(a.truth)
    V, T, _ = load(a.navmap)
    tri = V[T]
    cen = tri.mean(1)
    u, w = tri[:, 1, :2] - tri[:, 0, :2], tri[:, 2, :2] - tri[:, 0, :2]
    area = 0.5 * np.abs(u[:, 0] * w[:, 1] - u[:, 1] * w[:, 0])
    phantom = ~t.on_walkable(cen, TOL)
    print(f'phantom: {area[phantom].sum():.0f} of {area.sum():.0f} m2 '
          f'({area[phantom].sum() / area.sum():.1%})')
    rng = np.random.default_rng(0)
    idx = np.flatnonzero(phantom)
    idx = rng.choice(idx, size=min(a.samples, len(idx)), replace=False, p=area[idx] / area[idx].sum())
    cats = {}
    for k in idx:
        x, y, z = cen[k]
        zs, sl = surfaces_at(tris, x, y)
        near = np.abs(zs - z) <= TOL
        if not near.any():
            # A surface nearby in height but off by more than TOL, or nothing at all
            cat = 'no mesh surface at that height' if len(zs) else 'nothing in the mesh under it'
            if len(zs) and np.min(np.abs(zs - z)) < 1.0:
                cat = f'mesh surface {TOL}-1 m away in z'
        else:
            s = sl[near].min()
            zz = zs[near][np.argmin(sl[near])]
            above = zs[(zs > zz + MAX_DZ) & (zs <= zz + CLEARANCE)]
            if s > MAX_SLOPE:
                cat = 'too steep in the mesh'
            elif len(above):
                cat = f'no clearance (something {round(above.min() - zz, 1)} m above, z {round(zz, 1)})'
            else:
                cat = 'walkable in the mesh (truth sampling/border)'
        cats[cat] = cats.get(cat, 0) + 1
    for c, n in sorted(cats.items(), key=lambda kv: -kv[1]):
        print(f'  {n / len(idx):6.1%}  {c}')
    # Heights of the phantom area
    h = np.histogram(cen[phantom, 2], bins=[-1, 0.5, 2, 3.5, 5, 6.5, 8, 20],
                     weights=area[phantom])[0]
    print('  phantom m2 by z [-1,0.5,2,3.5,5,6.5,8,20]:', np.round(h).astype(int).tolist())


if __name__ == '__main__':
    main()
