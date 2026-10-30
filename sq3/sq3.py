#!/usr/bin/env python3
"""SQ3: distance to the static obstacles (the surface borders) on the surface vs projected vs 3D.

  geodesic  NavMap::distance_field from the open edges (dist_dump), what InflationFilter uses
  xy        planar distance to the nearest border of any level (a 2D costmap from a 3D sensor)
  xy_same   planar distance to the nearest border of the same surface (isolates the slope effect)
  3d        Euclidean distance to the nearest border point (a volumetric ESDF)

Usage: sq3.py map.navmap dump.bin out_prefix [--radius 0.7] [--start x,y,z] [--goals x,y,z;...]
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))


def open_edges(T):
    e = np.sort(np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]]), axis=1)
    owner = np.tile(np.arange(len(T)), 3)
    key = e[:, 0].astype(np.int64) * (T.max() + 2) + e[:, 1]
    o = np.argsort(key)
    key, e, owner = key[o], e[o], owner[o]
    first = np.r_[True, key[1:] != key[:-1]]
    cnt = np.diff(np.r_[np.flatnonzero(first), len(key)])
    single = np.flatnonzero(first)[cnt == 1]
    pairs = np.flatnonzero(first)[cnt == 2]
    return e[single], owner[single], np.c_[owner[pairs], owner[pairs + 1]]


def seg_dist(P, A, B, dims, cutoff=3.0, cell=1.0):
    """Min distance from each point to a set of segments, in the given dims (xy or xyz), capped
    at cutoff: only segments in the xy buckets around each point's bucket are checked."""
    out = np.full(len(P), cutoff)
    if len(A) == 0:
        return out
    lo = np.minimum(A[:, :2], B[:, :2]) - cutoff
    hi = np.maximum(A[:, :2], B[:, :2]) + cutoff
    buckets = {}
    for k in range(len(A)):
        for bi in range(int(np.floor(lo[k, 0] / cell)), int(np.floor(hi[k, 0] / cell)) + 1):
            for bj in range(int(np.floor(lo[k, 1] / cell)), int(np.floor(hi[k, 1] / cell)) + 1):
                buckets.setdefault((bi, bj), []).append(k)
    pk = np.floor(P[:, :2] / cell).astype(np.int64)
    key = pk[:, 0] * 1000003 + pk[:, 1]
    order = np.argsort(key)
    starts = np.flatnonzero(np.r_[True, key[order][1:] != key[order][:-1]])
    ends = np.r_[starts[1:], len(order)]
    Ad, Bd = A[:, dims], B[:, dims]
    for s0, e0 in zip(starts, ends):
        idx = order[s0:e0]
        segs = buckets.get((int(pk[idx[0], 0]), int(pk[idx[0], 1])))
        if not segs:
            continue
        a, ab = Ad[segs], Bd[segs] - Ad[segs]
        l2 = np.maximum((ab ** 2).sum(1), 1e-12)
        p = P[idx][:, dims][:, None, :]
        t = np.clip(((p - a[None]) * ab[None]).sum(2) / l2[None], 0, 1)
        q = a[None] + t[..., None] * ab[None]
        out[idx] = np.minimum(np.sqrt(((p - q) ** 2).sum(2)).min(1), cutoff)
    return out


def components(n, edges, ok):
    parent = np.arange(n)

    def find(i):
        r = i
        while parent[r] != r:
            r = parent[r]
        while parent[i] != r:
            parent[i], i = r, parent[i]
        return r
    for a, b in edges:
        if ok[a] and ok[b]:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb
    return np.array([find(i) for i in range(n)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('navmap')
    ap.add_argument('dump')
    ap.add_argument('out')
    ap.add_argument('--radius', type=float, default=0.7)
    ap.add_argument('--start', default='')
    ap.add_argument('--goals', default='')
    a = ap.parse_args()
    from render import load
    V, T, surf = load(a.navmap)
    D = np.fromfile(a.dump, dtype='<f4').reshape(-1, 6).astype(np.float64)
    C, nz, s_id, geo = D[:, :3], D[:, 3], D[:, 4], D[:, 5]
    tri = V[T]
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    E, E_owner, adj = open_edges(T)
    A, B = V[E[:, 0]], V[E[:, 1]]
    d = {'geodesic': geo, 'xy': seg_dist(C, A, B, [0, 1]), '3d': seg_dist(C, A, B, [0, 1, 2])}
    xy_same = np.full(len(C), 3.0)
    for s in np.unique(s_id):
        m, me = s_id == s, s_id[E_owner] == s
        if me.any():
            xy_same[m] = seg_dist(C[m], A[me], B[me], [0, 1])
    d['xy_same'] = xy_same
    slope = np.degrees(np.arccos(np.clip(np.abs(nz), 0, 1)))
    r = a.radius
    res = {'map': a.navmap, 'navcels': int(len(T)), 'area': float(area.sum()),
           'border_edges': int(len(E)), 'radius': r, 'blocked': {}, 'over_blocked': {}}
    for k, v in d.items():
        blocked = v < r
        res['blocked'][k] = float(area[blocked].sum())
        res['over_blocked'][k] = float(area[blocked & (geo >= r)].sum())
    # Over-blocking by the xy metric that comes from other levels (same-surface xy says free)
    res['xy_other_level'] = float(area[(d['xy'] < r) & (xy_same >= r) & (geo >= r)].sum())
    res['xy_slope_only'] = float(area[(xy_same < r) & (geo >= r)].sum())
    # Reachability from a start NavCel under each metric (cells at >= r from the border)
    if a.start:
        def nearest(p):
            return int(np.argmin(((C - np.array(p)) ** 2).sum(1)))
        st = nearest([float(v) for v in a.start.split(',')])
        goals = [[float(v) for v in g.split(',')] for g in a.goals.split(';') if g]
        res['reach'] = {}
        for k, v in d.items():
            ok = v >= r
            comp = components(len(T), adj, ok)
            reach = ok & (comp == comp[st])
            res['reach'][k] = {'area': float(area[reach].sum()),
                               'goals': [bool(reach[nearest(g)]) for g in goals]}
    # Slope study: ratio of planar to surface distance (same surface), against the local slope
    sel = (geo > 0.3) & (geo < 2.5)
    bins = np.arange(0, 46, 5)
    rows = []
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = sel & (slope >= lo) & (slope < hi)
        if m.sum() >= 20:
            q = xy_same[m] / geo[m]
            rows.append([int(lo), int(hi), int(m.sum()), float(np.percentile(q, 10)), float(np.median(q)),
                         float(np.cos(np.radians(0.5 * (lo + hi))))])
    res['slope_ratio'] = rows
    json.dump(res, open(a.out + '.json', 'w'), indent=1)
    np.savez_compressed(a.out + '.npz', C=C, slope=slope, area=area, **d)
    print(json.dumps({k: v for k, v in res.items() if k != 'slope_ratio'}, indent=1))
    for row in rows:
        print('slope %2d-%2d  n %6d  xy/geo p10 %.3f  p50 %.3f  cos %.3f' % tuple(row))


if __name__ == '__main__':
    main()
