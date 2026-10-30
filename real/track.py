#!/usr/bin/env python3
"""Sensor poses of a mapping run in the map frame: each scan registered to the map cloud, starting
from the previous pose plus the odometry increment.

Usage: track.py map.pcd scans.npz out.npz
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from pcd_voxelize import read_pcd_xyz  # noqa: E402
from register import icp, PlaneMap, pose2  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('scans')
    ap.add_argument('out')
    ap.add_argument('--sample', type=int, default=3000, help='scan points used by ICP')
    a = ap.parse_args()
    M = read_pcd_xyz(a.map).astype(np.float64)
    maps = [PlaneMap(M, v) for v in (1.0, 0.5, 0.25)]
    d = np.load(a.scans)
    odom, sizes, pts = d['odom'], d['sizes'], d['points']
    ends = np.cumsum(sizes)
    rng = np.random.default_rng(0)
    T = np.eye(4)
    prev = pose2(*odom[0])
    poses, fit = np.zeros((len(sizes), 4, 4)), np.zeros(len(sizes))
    for k in range(len(sizes)):
        S = pts[ends[k] - sizes[k]:ends[k]].astype(np.float64)
        if len(S) > a.sample:
            S = S[rng.choice(len(S), a.sample, replace=False)]
        cur = pose2(*odom[k])
        T = T @ np.linalg.inv(prev) @ cur
        prev = cur
        T, fit[k] = icp(maps, S, T, iters=(3, 5, 10))
        poses[k] = T
        if k % 500 == 0:
            print(f'{k}/{len(sizes)} fit {fit[k]:.2f} at {T[:3, 3].round(1)}', flush=True)
    path = np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1).sum()
    print(f'fit mean {fit.mean():.3f}, p10 {np.percentile(fit, 10):.3f}, '
          f'scans under 0.5: {(fit < 0.5).mean():.1%}, path {path:.0f} m')
    np.savez_compressed(a.out, stamps=d['stamps'], poses=poses, fit=fit)


if __name__ == '__main__':
    main()
