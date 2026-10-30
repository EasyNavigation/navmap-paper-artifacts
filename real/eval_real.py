#!/usr/bin/env python3
"""Topological fidelity on real data, with the route the robot drove as the reference: it is floor,
and all of it is connected. For each representation built from the mapping cloud: the fraction of
the route on the map, the fraction in its largest component, and the number of components along it.

Usage: eval_real.py cloud_vp.pcd poses.npz res [--reps navmap,navmap+vp,elev,mls,vox] [--work dir]
       [--json out.json]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sq1'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import eval_sq1 as E  # noqa: E402


def route(poses, step=0.5):
    """The driven route (base_footprint positions) resampled every step metres."""
    X = poses[:, :3, 3]
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(X, axis=0), axis=1))]
    s = np.arange(0, d[-1], step)
    return np.c_[[np.interp(s, d, X[:, k]) for k in range(3)]].T, d[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cloud')
    ap.add_argument('poses')
    ap.add_argument('res', type=float)
    ap.add_argument('--reps', default='navmap,navmap+vp,elev,mls,vox')
    ap.add_argument('--work', default='.')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    R, length = route(np.load(a.poses)['poses'])
    P = E.read_pcd(a.cloud)
    out = {'cloud': os.path.basename(a.cloud), 'res': a.res, 'route_m': float(length),
           'route_samples': len(R), 'results': {}}
    print(f'route {length:.0f} m, {len(R)} samples, z {R[:, 2].min():.1f}..{R[:, 2].max():.1f}')
    for name in a.reps.split(','):
        if name in ('navmap', 'navmap+vp'):
            if name == 'navmap':
                os.environ['NOVP'] = '1'
            rep = E.NavMapRep(a.cloud, a.res, a.work)
            os.environ.pop('NOVP', None)
        elif name == 'elev':
            rep = E.ElevationRep(P, a.res)
        elif name == 'mls':
            rep = E.MLSRep(P, a.res)
        else:
            rep = E.VoxelRep(P, a.res)
        loc = np.array([rep.locate(x, y, z) for x, y, z in R])
        on = loc >= 0
        comp = np.asarray(rep.comp)[loc[on]]
        largest = np.bincount(comp).max() if on.any() else 0
        r = {'on_map': float(on.mean()), 'in_largest_component': float(largest / len(R)),
             'components_along': int(len(np.unique(comp)))}
        out['results'][name] = r
        print(f'{name:10s} on the map {r["on_map"]:6.1%}  in one component '
              f'{r["in_largest_component"]:6.1%}  components along the route {r["components_along"]}')
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
