#!/usr/bin/env python3
"""Cloud of a mapping run in the map frame, each point with its sensor position (vp_x, vp_y,
vp_z), from the scans (bag_scans.py) and their poses (track.py).

Usage: assemble.py scans.npz poses.npz out.pcd [--min-fit 0.3]
"""
import argparse

import numpy as np


def write_pcd_vp(path, P, V):
    A = np.hstack([P, V]).astype('<f4')
    with open(path, 'wb') as f:
        f.write(('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
                 'FIELDS x y z vp_x vp_y vp_z\nSIZE 4 4 4 4 4 4\nTYPE F F F F F F\n'
                 'COUNT 1 1 1 1 1 1\n'
                 f'WIDTH {len(A)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                 f'POINTS {len(A)}\nDATA binary\n').encode())
        f.write(A.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scans')
    ap.add_argument('poses')
    ap.add_argument('out')
    ap.add_argument('--min-fit', type=float, default=0.3,
                    help='scans registered worse than this are left out')
    a = ap.parse_args()
    s, p = np.load(a.scans), np.load(a.poses)
    sizes, pts, laser = s['sizes'], s['points'], s['laser']
    T, fit = p['poses'], p['fit']
    ends = np.cumsum(sizes)
    P, V = [], []
    for k in np.flatnonzero(fit >= a.min_fit):
        S = pts[ends[k] - sizes[k]:ends[k]].astype(np.float64)
        P.append(S @ T[k, :3, :3].T + T[k, :3, 3])
        V.append(np.tile(T[k, :3, :3] @ laser[:3, 3] + T[k, :3, 3], (len(S), 1)))
    P, V = np.concatenate(P), np.concatenate(V)
    write_pcd_vp(a.out, P, V)
    print(f'{(fit >= a.min_fit).sum()} of {len(fit)} scans, {len(P)} points')


if __name__ == '__main__':
    main()
