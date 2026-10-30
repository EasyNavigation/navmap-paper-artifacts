#!/usr/bin/env python3
"""Feasibility of the SQ5 goals in the NavMap the stack navigates (review 2, M4): the clearance of
each goal's NavCel and whether it is connected to the start through NavCels with at least r of
clearance, for the robot's inscribed radius and the inflation radius. The goals were drawn from
the ground truth (all connected to the start there); this tells which of them the stack's own map
can reach at all. Needs the per-NavCel dump of tools/dist_dump (centroid, nz, surface, distance
to the border along the surface).
Usage: check_goals.py scene map.navmap dump.bin [--radii 0.7,1.5] [--json out.json]"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
sys.path.insert(0, os.path.join(HERE, '..', 'sq3'))
from render import load  # noqa: E402
from sq3 import open_edges, components  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scene')
    ap.add_argument('navmap')
    ap.add_argument('dump')
    ap.add_argument('--radii', default='0.5,0.7', help='clearances to test; 0.7 is the Summit inscribed radius (lethal band)')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    V, T, _ = load(a.navmap)
    D = np.fromfile(a.dump, dtype='<f4').reshape(-1, 6).astype(np.float64)
    C, dist = D[:, :3], D[:, 5]
    _, _, adj = open_edges(T)
    g = json.load(open(os.path.join(HERE, f'goals_{a.scene}.json')))
    pts = [('start', g['start'][0], g['start'][1], g['start_z'])]
    pts += [(i, x['x'], x['y'], x['z']) for i, x in enumerate(g['goals'])]

    def cell(x, y, z):  # nearest centroid in the plane, on the surface at that height
        return int(np.argmin(np.hypot(C[:, 0] - x, C[:, 1] - y) + 3 * np.abs(C[:, 2] - z)))
    cells = {n: cell(x, y, z) for n, x, y, z in pts}
    edge = float(np.median(np.linalg.norm(V[T[:, 0]] - V[T[:, 1]], axis=1)))
    out = {'scene': a.scene, 'navmap': os.path.basename(a.navmap), 'navcels': len(C),
           'median_edge_m': round(edge, 2), 'goals': []}
    radii = [float(r) for r in a.radii.split(',')]
    comp = {r: components(len(C), adj, dist >= r) for r in radii}
    s = cells['start']
    for n, x, y, z in pts[1:]:
        c = cells[n]
        rec = {'goal': n, 'clearance_m': round(float(dist[c]), 2)}
        for r in radii:
            rec[f'connected_r{r:g}'] = bool(dist[c] >= r and dist[s] >= r and comp[r][c] == comp[r][s])
        out['goals'].append(rec)
    print(f"{a.scene}: {len(C)} NavCels, median edge {edge:.2f} m")
    for rec in out['goals']:
        print(' goal %2d  clearance %.2f m  ' % (rec['goal'], rec['clearance_m']) +
              '  '.join(f"r={r:g}: {'reachable' if rec[f'connected_r{r:g}'] else 'NO PATH'}" for r in radii))
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
