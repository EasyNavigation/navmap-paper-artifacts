#!/usr/bin/env python3
"""Steps of a terrain mesh a robot may not climb: on the top surface's height map, jumps between
neighbouring cells (res apart) over `step`, where the local relief (1 m window) is under
`max_relief` (more is a wall or a slope, not a step). Prints how many and where.

Usage: terrain_steps.py mesh.stl [--res 0.1] [--step 0.08] [--max-relief 0.6] [--near x,y,r]
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stitch_rims import load_stl  # noqa: E402


def shift(a, dx, dy, fill):
    o = np.full_like(a, fill)
    nx, ny = a.shape
    o[max(dx, 0):nx + min(dx, 0), max(dy, 0):ny + min(dy, 0)] = \
        a[max(-dx, 0):nx + min(-dx, 0), max(-dy, 0):ny + min(-dy, 0)]
    return o


def raster_top(tris, lo, res, shape):
    """Highest surface over each cell centre."""
    H = np.full(shape, -np.inf)
    for a, b, c in tris:
        x0, y0 = np.floor((np.minimum(np.minimum(a, b), c)[:2] - lo) / res).astype(int)
        x1, y1 = np.ceil((np.maximum(np.maximum(a, b), c)[:2] - lo) / res).astype(int)
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(x1, shape[0] - 1), min(y1, shape[1] - 1)
        xs = lo[0] + (np.arange(x0, x1 + 1) + 0.5) * res
        ys = lo[1] + (np.arange(y0, y1 + 1) + 0.5) * res
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(d) < 1e-12:
            continue
        l1 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
        l2 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
        l3 = 1 - l1 - l2
        m = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
        if m.any():
            ii, jj = np.nonzero(m)
            np.maximum.at(H, (ii + x0, jj + y0), (l1 * a[2] + l2 * b[2] + l3 * c[2])[m])
    return H


def steps(path, res=0.1, step=0.08, max_relief=0.6):
    tris = load_stl(path)
    V = tris.reshape(-1, 3)
    lo = V.min(0)[:2] - 1.0
    shape = tuple(np.ceil((V.max(0)[:2] + 1.0 - lo) / res).astype(int))
    H = raster_top(tris, lo, res, shape)
    Hf = np.where(np.isfinite(H), H, np.nan)
    jump = np.zeros(shape)
    for dx, dy in ((1, 0), (0, 1)):
        jump = np.fmax(jump, np.abs(shift(Hf, dx, dy, np.nan) - Hf))
    w = int(round(0.5 / res))
    mx = Hf.copy()
    mn = Hf.copy()
    for dx in range(-w, w + 1, 2):
        for dy in range(-w, w + 1, 2):
            s = shift(Hf, dx, dy, np.nan)
            mx = np.fmax(mx, s)
            mn = np.fmin(mn, s)
    bad = (jump > step) & ((mx - mn) < max_relief)
    ii, jj = np.nonzero(bad)
    xy = lo + (np.c_[ii, jj] + 0.5) * res
    return xy, jump[ii, jj]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mesh')
    ap.add_argument('--res', type=float, default=0.1)
    ap.add_argument('--step', type=float, default=0.08)
    ap.add_argument('--max-relief', type=float, default=0.6)
    ap.add_argument('--near', default='')
    a = ap.parse_args()
    xy, h = steps(a.mesh, a.res, a.step, a.max_relief)
    print(f'{len(h)} step cells over {a.step} m; height p50 {np.median(h):.3f} p95 '
          f'{np.percentile(h, 95):.3f} max {h.max():.3f}' if len(h) else 'no steps')
    if a.near:
        x, y, r = map(float, a.near.split(','))
        m = np.hypot(xy[:, 0] - x, xy[:, 1] - y) < r
        print(f'  within {r} m of ({x}, {y}): {m.sum()} cells, max {h[m].max() if m.any() else 0:.3f} m')


if __name__ == '__main__':
    main()
