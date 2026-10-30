#!/usr/bin/env python3
"""SQ1: topological fidelity of each representation, built from the same cloud, against the
ground truth of the world's collision mesh (mesh_truth.py).

Each representation is reduced to a graph (walkable elements + adjacency) and a locate(x, y, z).
Queries are random start/goal pairs on the true walkable surface, away from its borders.

Usage: eval_sq1.py mesh cloud.pcd res [--ground highest|lowest] [--truth-res 0.1]
                   [--truth-cache gt.npz] [--pairs N] [--reps navmap,elev,mls,vox] [--json out]
Both the mesh (after --ground) and the cloud are in the playground's map frame.
Needs the ROS environment (navmap .navmap loading) and MESHSTATS pointing to the meshstats tool.
"""
import argparse
import json
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mesh_truth import CLEARANCE, load_world_mesh, MAX_DZ, MAX_SLOPE, MeshTruth  # noqa: E402

LOCATE_TOL = 0.3     # vertical tolerance to say "this element is that point"
MARGIN = 0.5         # query points stay this far from any border of the walkable region


# ---------------------------------------------------------------------------- utilities
def read_pcd(path, viewpoints=False):
    """Points of a binary PCD; with viewpoints, also vp_x, vp_y, vp_z (None if absent)."""
    raw = open(path, 'rb').read()
    i = raw.index(b'DATA binary\n') + 12
    fields = [l for l in raw[:i].decode().splitlines() if l.startswith('FIELDS')][0].split()[1:]
    A = np.frombuffer(raw[i:], dtype='<f4').reshape(-1, len(fields)).astype(np.float64)
    if not viewpoints:
        return A[:, :3]
    v = [fields.index(f) for f in ('vp_x', 'vp_y', 'vp_z')] if 'vp_x' in fields else None
    return A[:, :3], (A[:, v] if v else None)


def components(n, edges):
    parent = np.arange(n)

    def find(i):
        root = i
        while parent[root] != root:
            root = parent[root]
        while parent[i] != root:
            parent[i], i = root, parent[i]
        return root
    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    return np.array([find(i) for i in range(n)])


# ---------------------------------------------------------------------------- representations
class Graph:
    """Walkable elements (xy footprint area, centre) + components + locate."""
    name = ''

    def stats(self):
        return {'elements': int(len(self.centres)), 'area': float(self.areas.sum())}


class NavMapRep(Graph):
    name = 'navmap'

    def __init__(self, cloud_path, res, work):
        from render import load
        stem = os.path.splitext(os.path.basename(cloud_path))[0]
        out = os.path.join(work, f'navmap_{stem}_{res}.navmap')
        tool = os.environ.get('MESHSTATS', 'meshstats')
        subprocess.run([tool, cloud_path, str(res), str(CLEARANCE), str(MAX_DZ), '0', out],
                       check=True, capture_output=True,
                       env=dict(os.environ, SLOPE=str(MAX_SLOPE), HOLE='1.0'))
        V, T, _ = load(out)
        self.V, self.T = V, T
        tri = V[T]
        self.centres = tri.mean(1)
        u, w = tri[:, 1, :2] - tri[:, 0, :2], tri[:, 2, :2] - tri[:, 0, :2]
        self.areas = 0.5 * np.abs(u[:, 0] * w[:, 1] - u[:, 1] * w[:, 0])
        # adjacency through shared edges
        e = np.sort(np.concatenate([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]]), axis=1)
        owner = np.tile(np.arange(len(T)), 3)
        key = e[:, 0].astype(np.int64) * (len(V) + 1) + e[:, 1]
        o = np.argsort(key)
        key, owner = key[o], owner[o]
        same = np.flatnonzero(key[1:] == key[:-1])
        self.comp = components(len(T), zip(owner[same], owner[same + 1]))
        # xy buckets for locate
        self.cell = res
        lo = tri[:, :, :2].min(1)
        hi = tri[:, :, :2].max(1)
        self.buckets = {}
        for t in range(len(T)):
            for bi in range(int(lo[t, 0] // res), int(hi[t, 0] // res) + 1):
                for bj in range(int(lo[t, 1] // res), int(hi[t, 1] // res) + 1):
                    self.buckets.setdefault((bi, bj), []).append(t)

    def locate(self, x, y, z):
        best, bd = -1, LOCATE_TOL
        for t in self.buckets.get((int(x // self.cell), int(y // self.cell)), []):
            a, b, c = self.V[self.T[t]]
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-12:
                continue
            l1 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / d
            l2 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / d
            l3 = 1 - l1 - l2
            if min(l1, l2, l3) < -1e-6:
                continue
            dz = abs(l1 * a[2] + l2 * b[2] + l3 * c[2] - z)
            if dz <= bd:
                best, bd = t, dz
        return best


class GridGraph(Graph):
    """Cells (i, j) with one or more standing heights: shared by the 2.5D and MLS baselines."""

    def finish(self, nodes, res):
        # nodes: list of (i, j, h); edges to the 8 neighbours within the climbable step
        self.res = res
        self.nodes = nodes
        self.centres = np.array([((i + 0.5) * res, (j + 0.5) * res, h) for i, j, h in nodes])
        self.areas = np.full(len(nodes), res * res)
        by_cell = {}
        for n, (i, j, h) in enumerate(nodes):
            by_cell.setdefault((i, j), []).append(n)
        self.by_cell = by_cell
        tan = np.tan(np.radians(MAX_SLOPE))
        edges = []
        for n, (i, j, h) in enumerate(nodes):
            for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
                lim = max(MAX_DZ, tan * res * np.hypot(di, dj))
                for m in by_cell.get((i + di, j + dj), []):
                    if abs(nodes[m][2] - h) <= lim:
                        edges.append((n, m))
        self.comp = components(len(nodes), edges)

    def locate(self, x, y, z):
        best, bd = -1, LOCATE_TOL
        for n in self.by_cell.get((int(x // self.res), int(y // self.res)), []):
            dz = abs(self.nodes[n][2] - z)
            if dz <= bd:
                best, bd = n, dz
        return best


class ElevationRep(GridGraph):
    """grid_map / elevation_mapping: one height per cell, the highest return."""
    name = 'elev'

    def __init__(self, P, res):
        ij = np.floor(P[:, :2] / res).astype(np.int64)
        key = ij[:, 0] * 1000003 + ij[:, 1]
        o = np.lexsort((P[:, 2], key))
        key, z, ij = key[o], P[o, 2], ij[o]
        last = np.r_[key[1:] != key[:-1], True]
        self.finish([(int(a), int(b), float(h)) for (a, b), h in zip(ij[last], z[last])], res)


class MLSRep(GridGraph):
    """Multi-level surface map (Triebel 2006): vertical patches per cell; horizontal ones with
    clearance above are standing surfaces."""
    name = 'mls'

    def __init__(self, P, res, gap=None, depth=None):
        gap = gap if gap is not None else MAX_DZ
        # A patch thicker than a climbable slope across the cell's diagonal (plus noise) is a
        # wall or clutter; a fixed threshold cuts ramps into pieces at coarse cells
        if depth is None:
            depth = max(0.3, res * np.sqrt(2.0) * np.tan(np.radians(MAX_SLOPE)) + 0.2)
        ij = np.floor(P[:, :2] / res).astype(np.int64)
        key = ij[:, 0] * 1000003 + ij[:, 1]
        o = np.lexsort((P[:, 2], key))
        key, z, ij = key[o], P[o, 2], ij[o]
        nodes = []
        starts = np.flatnonzero(np.r_[True, key[1:] != key[:-1]])
        ends = np.r_[starts[1:], len(key)]
        for s, e in zip(starts, ends):
            zz = z[s:e]
            cut = np.flatnonzero(np.diff(zz) > gap) + 1
            patches = np.split(zz, cut)
            for p_i, p in enumerate(patches):
                if p[-1] - p[0] > depth:
                    continue  # vertical patch: wall or clutter
                above = patches[p_i + 1][0] if p_i + 1 < len(patches) else np.inf
                if above - p[-1] < CLEARANCE:
                    continue
                nodes.append((int(ij[s, 0]), int(ij[s, 1]), float(p[-1])))
        self.finish(nodes, res)


class VoxelRep(Graph):
    """Floor extraction from an occupancy voxel map (OctoMap/Bonxai): an occupied voxel with free
    space above it for the robot height; 26-neighbours within one voxel of step are connected."""
    name = 'vox'

    def __init__(self, P, res):
        v = np.unique(np.floor(P / res).astype(np.int64), axis=0)
        occ = set(map(tuple, v))
        hv = int(np.ceil(CLEARANCE / res))
        stand = [t for t in occ if not any((t[0], t[1], t[2] + k) in occ for k in range(1, hv + 1))]
        self.res = res
        self.nodes = stand
        idx = {t: n for n, t in enumerate(stand)}
        step = max(1, int(round(MAX_DZ / res)))
        edges = []
        for n, (i, j, k) in enumerate(stand):
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if (di, dj) <= (0, 0):
                        continue
                    for dk in range(-step, step + 1):
                        m = idx.get((i + di, j + dj, k + dk))
                        if m is not None:
                            edges.append((n, m))
        self.comp = components(len(stand), edges)
        a = np.array(stand, float)
        self.centres = np.c_[(a[:, :2] + 0.5) * res, (a[:, 2] + 1.0) * res]  # top of the voxel
        self.areas = np.full(len(stand), res * res)
        self.by_cell = {}
        for n, (i, j, k) in enumerate(stand):
            self.by_cell.setdefault((i, j), []).append(n)

    def locate(self, x, y, z):
        best, bd = -1, LOCATE_TOL + self.res / 2
        for n in self.by_cell.get((int(x // self.res), int(y // self.res)), []):
            dz = abs(self.centres[n, 2] - z)
            if dz <= bd:
                best, bd = n, dz
        return best


# ---------------------------------------------------------------------------- metrics
def evaluate(rep, truth, Q, pairs, cover, apart):
    loc = np.array([rep.locate(x, y, z) for x, y, z, _ in Q])
    found = loc >= 0
    rc = np.where(found, rep.comp[np.maximum(loc, 0)], -1)
    tc = Q[:, 3].astype(np.int64)
    a, b = pairs[:, 0], pairs[:, 1]
    gt_conn = tc[a] == tc[b]
    rep_conn = found[a] & found[b] & (rc[a] == rc[b])
    # Pairs on different true components: any connection is a false bridge
    if len(apart):
        la = np.array([rep.locate(x, y, z) for x, y, z, _ in apart])
        fa = la >= 0
        ra = np.where(fa, rep.comp[np.maximum(la, 0)], -1)
        bridged = fa[0::2] & fa[1::2] & (ra[0::2] == ra[1::2])
        apart_located = fa[0::2] & fa[1::2]
    else:
        bridged = apart_located = np.zeros(0, bool)
    phantom = ~truth.on_walkable(rep.centres, LOCATE_TOL)
    # Walkable surface the representation misses: true samples it cannot locate
    missed = np.array([rep.locate(x, y, z) < 0 for x, y, z in cover])
    res = {
        'located': float(found.mean()),
        'pairs_connected_gt': int(gt_conn.sum()),
        'pairs_disconnected_gt': int((~gt_conn).sum()),
        'false_connection': float((rep_conn & ~gt_conn).sum() / max(1, (~gt_conn).sum())),
        'false_disconnection': float((~rep_conn & gt_conn).sum() / max(1, gt_conn.sum())),
        'false_disconnection_located': float(
            (~rep_conn & gt_conn & found[a] & found[b]).sum() /
            max(1, (gt_conn & found[a] & found[b]).sum())),
        'misclassified': float((rep_conn != gt_conn).mean()),
        'apart_pairs': int(len(bridged)),
        'false_bridge': float(bridged.mean()) if len(bridged) else None,
        'false_bridge_located': float(bridged.sum() / max(1, apart_located.sum()))
        if len(bridged) else None,
        'phantom_area_fraction': float(rep.areas[phantom].sum() / max(1e-9, rep.areas.sum())),
        'missed_area_fraction': float(missed.mean()),
        'components': int(len(np.unique(rep.comp))),
    }
    res.update(rep.stats())
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mesh', help='collision mesh of the world (.dae or binary .stl)')
    ap.add_argument('cloud')
    ap.add_argument('res', type=float)
    ap.add_argument('--ground', default='highest', choices=('highest', 'lowest'),
                    help='z = 0 at the highest (terrain) or lowest (building) surface under (0, 0)')
    ap.add_argument('--truth-res', type=float, default=0.1)
    ap.add_argument('--truth-cache', default='', help='.npz to reuse the ground truth')
    ap.add_argument('--pairs', type=int, default=1000)
    ap.add_argument('--cover', type=int, default=20000, help='true samples for the missed area')
    ap.add_argument('--reps', default='navmap,elev,mls,vox')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--work', default='.', help='folder for the NavMap built from the cloud')
    ap.add_argument('--all-truth', action='store_true',
                    help='queries on the whole walkable surface, observed by the cloud or not')
    ap.add_argument('--observe-cloud', default='',
                    help='cloud (with viewpoints) that defines the observed truth, when the '
                         'evaluated cloud is a filtered copy without them (ablation)')
    ap.add_argument('--mls-gap', type=float, default=None,
                    help='MLS: vertical gap that splits patches (default: the climbable step)')
    ap.add_argument('--mls-margin', type=float, default=0.2,
                    help='MLS: noise margin of the patch thickness over the slope across a cell')
    ap.add_argument('--json', default='')
    a = ap.parse_args()
    if a.truth_cache and os.path.exists(a.truth_cache):
        truth = MeshTruth.load(a.truth_cache)
    else:
        truth = MeshTruth(load_world_mesh(a.mesh, a.ground), res=a.truth_res, margin=MARGIN,
                          seed=a.seed)
        if a.truth_cache:
            truth.save(a.truth_cache)
    P, V = read_pcd(a.cloud, viewpoints=True)
    # Only what the cloud observes (seen from above, with viewpoints): no representation can
    # know the rest. A filtered copy of a cloud is observed through the original, so that every
    # condition of an ablation is scored on the same truth samples.
    Po, Vo = read_pcd(a.observe_cloud, viewpoints=True) if a.observe_cloud else (P, V)
    observed = truth.observe(Po, V=Vo) if not a.all_truth else 1.0
    rng = np.random.default_rng(a.seed)
    Q = truth.sample(2 * a.pairs, rng)
    pairs = np.arange(2 * a.pairs).reshape(-1, 2)
    apart = truth.sample_apart(a.pairs, rng)
    pool = np.flatnonzero(getattr(truth, 'observed', np.ones(truth.N, bool)))
    pick = rng.choice(pool, size=min(a.cover, len(pool)), replace=False)
    cover = np.c_[truth.xy[pick], truth.z[pick]]
    out = {'mesh': os.path.basename(a.mesh), 'cloud': os.path.basename(a.cloud), 'res': a.res,
           'seed': a.seed, 'mls': {'gap': a.mls_gap, 'margin': a.mls_margin},
           'robot': {'max_dz': MAX_DZ, 'max_slope': MAX_SLOPE, 'clearance': CLEARANCE},
           'truth': truth.summary(), 'observed_fraction': observed, 'results': {}}
    print('truth:', out['truth'], 'observed by the cloud: %.3f' % observed)
    for name in a.reps.split(','):
        if name == 'navmap':
            rep = NavMapRep(a.cloud, a.res, a.work)
        elif name == 'elev':
            rep = ElevationRep(P, a.res)
        elif name == 'mls':
            depth = max(0.3, a.res * np.sqrt(2.0) * np.tan(np.radians(MAX_SLOPE)) + a.mls_margin)
            rep = MLSRep(P, a.res, gap=a.mls_gap, depth=depth)
        else:
            rep = VoxelRep(P, a.res)
        r = evaluate(rep, truth, Q, pairs, cover, apart)
        out['results'][name] = r
        fb = r['false_bridge']
        print(f'{name:7s} located {r["located"]:.3f}  false_bridge '
              f'{(f"{fb:.3f}" if fb is not None else "  -  ")} (of {r["apart_pairs"]})  '
              f'false_disc {r["false_disconnection"]:.3f} (located {r["false_disconnection_located"]:.3f})'
              f'  phantom {r["phantom_area_fraction"]:.3f}  missed {r["missed_area_fraction"]:.3f}'
              f'  comps {r["components"]}')
    if a.json:
        json.dump(out, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
