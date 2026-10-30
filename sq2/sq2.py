#!/usr/bin/env python3
"""SQ2: where the cost lives, faces (NavMap) or vertices (mesh_navigation), on the same meshes.

Meshes: a square grid of edge l split into two triangles per cell (as NavMap's lattice meshes),
optionally with its vertices jittered (an irregular mesh of the same density).

1) Lethal area: a small obstacle (disc of radius 0.05 m sampled by points) marks, with faces, the
   faces containing its points; with vertices, the vertex nearest to each point, which makes the
   faces around it lethal (its one-ring). Blocked area against the obstacle's.
2) Narrow passages: a corridor of width w at a random angle and offset, walls sampled by points,
   a robot of radius r. Faces: the faces containing wall points are lethal, faces whose centroid
   is closer than r to a lethal face's centroid are inflated, and a path must cross the corridor
   through free adjacent faces. Vertices: the vertex nearest to each wall point is lethal, vertices
   closer than r to a lethal one are inflated, and a path must cross along mesh edges between free
   vertices (as mesh_navigation's Dijkstra planner). The passage really exists when w >= 2 r.

Usage: sq2.py [--out sq2.json] [--trials 200]
"""
import argparse
import json
from collections import deque

import numpy as np

R_ROBOT = 0.3
SIZE = 6.0


def cross2(u, v):
    """z of the cross product of 2D vectors (rows)."""
    return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]


def grid_mesh(l, jitter, rng):
    n = int(round(SIZE / l))
    xs = np.arange(n + 1) * l - SIZE / 2
    X, Y = np.meshgrid(xs, xs, indexing='ij')
    V = np.c_[X.ravel(), Y.ravel()]
    if jitter > 0:
        inner = (np.abs(V[:, 0]) < SIZE / 2 - 1e-9) & (np.abs(V[:, 1]) < SIZE / 2 - 1e-9)
        V[inner] += rng.uniform(-jitter * l, jitter * l, (inner.sum(), 2))
    idx = np.arange((n + 1) ** 2).reshape(n + 1, n + 1)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel(), idx[:-1, 1:].ravel()
    F = np.r_[np.c_[a, b, c], np.c_[a, c, d]]
    return V, F


def face_adjacency(F, nv):
    e = np.sort(np.r_[F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]], axis=1)
    owner = np.tile(np.arange(len(F)), 3)
    key = e[:, 0] * nv + e[:, 1]
    o = np.argsort(key)
    key, owner = key[o], owner[o]
    same = np.flatnonzero(key[1:] == key[:-1])
    adj = [[] for _ in range(len(F))]
    for i in same:
        adj[owner[i]].append(owner[i + 1])
        adj[owner[i + 1]].append(owner[i])
    return adj


def vertex_adjacency(F, nv):
    adj = [set() for _ in range(nv)]
    for a, b, c in F:
        adj[a] |= {b, c}
        adj[b] |= {a, c}
        adj[c] |= {a, b}
    return [list(s) for s in adj]


class Mesh:
    def __init__(self, l, jitter, rng):
        self.l = l
        self.V, self.F = grid_mesh(l, jitter, rng)
        self.C = self.V[self.F].mean(1)
        T = self.V[self.F]
        self.area = 0.5 * np.abs(cross2(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]))
        self.fadj = face_adjacency(self.F, len(self.V))
        self.vadj = vertex_adjacency(self.F, len(self.V))
        self.cell = l
        # Face lookup: candidate faces per grid cell (jitter < l/2 keeps them in the 3x3 cells)
        self.n = int(round(SIZE / l))
        ij = np.floor((self.C + SIZE / 2) / l).astype(int)
        self.bucket = {}
        for f, (i, j) in enumerate(ij):
            self.bucket.setdefault((i, j), []).append(f)

    def face_of(self, P):
        """Face containing each point (-1 if none): the 18 faces of the 3 x 3 cells around it."""
        n = self.n
        ij = np.floor((P + SIZE / 2) / self.l).astype(int)
        out = np.full(len(P), -1)
        T = self.V[self.F]
        for di in (0, -1, 1):
            for dj in (0, -1, 1):
                i, j = ij[:, 0] + di, ij[:, 1] + dj
                ok = (i >= 0) & (i < n) & (j >= 0) & (j < n) & (out < 0)
                cellid = np.where(ok, i * n + j, 0)
                for half in (0, n * n):
                    f = cellid + half
                    a, b, c = T[f, 0], T[f, 1], T[f, 2]
                    d1 = cross2(b - a, P - a)
                    d2 = cross2(c - b, P - b)
                    d3 = cross2(a - c, P - c)
                    inside = ((d1 >= -1e-12) & (d2 >= -1e-12) & (d3 >= -1e-12)) | \
                        ((d1 <= 1e-12) & (d2 <= 1e-12) & (d3 <= 1e-12))
                    take = ok & inside & (out < 0)
                    out[take] = f[take]
        return out

    def vertex_of(self, P):
        """Nearest vertex of each point, among the 3 x 3 lattice vertices around it."""
        n1 = self.n + 1
        ij0 = np.round((P + SIZE / 2) / self.l).astype(int)
        best = np.full(len(P), -1)
        bd = np.full(len(P), np.inf)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                i, j = ij0[:, 0] + di, ij0[:, 1] + dj
                ok = (i >= 0) & (i < n1) & (j >= 0) & (j < n1)
                v = np.where(ok, i * n1 + j, 0)
                d = np.where(ok, ((self.V[v] - P) ** 2).sum(1), np.inf)
                take = d < bd
                best[take], bd[take] = v[take], d[take]
        return best


def within(points, seeds, r):
    """Mask of points closer than r to any seed point."""
    m = np.zeros(len(points), bool)
    if len(seeds) == 0:
        return m
    for s in range(0, len(seeds), 256):
        d = ((points[:, None, :] - seeds[None, s:s + 256, :]) ** 2).sum(-1)
        m |= (d < r * r).any(1)
    return m


def reachable(adj, free, start, goal):
    seen = np.zeros(len(free), bool)
    q = deque([s for s in start if free[s]])
    for s in q:
        seen[s] = True
    while q:
        u = q.popleft()
        if goal[u]:
            return True
        for v in adj[u]:
            if free[v] and not seen[v]:
                seen[v] = True
                q.append(v)
    return False


def corridor_trial(mesh, w, theta, offset, r=R_ROBOT):
    """Can each semantics cross a corridor of width w? Returns (faces, vertices)."""
    u = np.array([np.cos(theta), np.sin(theta)])   # along the corridor
    nrm = np.array([-u[1], u[0]])                  # across
    centre = offset * nrm
    # Wall points every 1 cm along both walls, over the mesh
    s = np.arange(-SIZE, SIZE, 0.01)
    walls = np.r_[centre + (w / 2) * nrm + s[:, None] * u, centre - (w / 2) * nrm + s[:, None] * u]
    walls = walls[(np.abs(walls) < SIZE / 2 - 1e-6).all(1)]
    along_c = (mesh.C - centre) @ u
    across_c = (mesh.C - centre) @ nrm
    along_v = (mesh.V - centre) @ u
    across_v = (mesh.V - centre) @ nrm
    inside_c, inside_v = np.abs(across_c) < w / 2, np.abs(across_v) < w / 2
    lim = 1.5  # start and goal regions: the corridor's two ends, 1.5 m from its middle
    # Faces
    lethal_f = np.unique(mesh.face_of(walls))
    lethal_f = lethal_f[lethal_f >= 0]
    free_f = ~within(mesh.C, mesh.C[lethal_f], r) & inside_c
    free_f[lethal_f] = False
    ok_f = reachable(mesh.fadj, free_f, np.flatnonzero((along_c < -lim) & inside_c),
                     (along_c > lim) & inside_c)
    # Vertices
    lethal_v = np.unique(mesh.vertex_of(walls))
    free_v = ~within(mesh.V, mesh.V[lethal_v], r) & inside_v
    free_v[lethal_v] = False
    ok_v = reachable(mesh.vadj, free_v, np.flatnonzero((along_v < -lim) & inside_v),
                     (along_v > lim) & inside_v)
    return ok_f, ok_v


def lethal_area(mesh, rng, trials=200, radius=0.05):
    """Blocked area of a disc obstacle of the given radius, faces and vertices."""
    out_f, out_v = [], []
    ang = np.linspace(0, 2 * np.pi, 64, endpoint=False)
    rings = np.r_[[[0.0, 0.0]], *[q * radius * np.c_[np.cos(ang), np.sin(ang)]
                                  for q in (0.33, 0.66, 1.0)]]
    for _ in range(trials):
        c = rng.uniform(-1.0, 1.0, 2)
        P = c + rings
        faces = np.unique(mesh.face_of(P))
        faces = faces[faces >= 0]
        out_f.append(mesh.area[faces].sum())
        verts = set(mesh.vertex_of(P).tolist())
        # Faces with at least one lethal vertex
        near = np.flatnonzero(np.abs(mesh.C - c).max(1) < radius + 2 * mesh.l)
        hit = [f for f in near if any(v in verts for v in mesh.F[f])]
        out_v.append(mesh.area[hit].sum())
    true = np.pi * radius ** 2
    return float(np.mean(out_f) / true), float(np.mean(out_v) / true)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='sq2.json')
    ap.add_argument('--trials', type=int, default=200)
    a = ap.parse_args()
    rng = np.random.default_rng(1)
    res = {'robot_radius': R_ROBOT, 'meshes': []}
    for jitter in (0.0, 0.3):
        for l in (0.05, 0.1, 0.2, 0.4):
            mesh = Mesh(l, jitter, rng)
            fa, va = lethal_area(mesh, rng, trials=a.trials)
            row = {'edge': l, 'jitter': jitter, 'lethal_area_ratio': {'faces': fa, 'vertices': va},
                   'corridor': []}
            widths = np.round(np.arange(2 * R_ROBOT, 2 * R_ROBOT + 1.6 + 1e-9, max(l / 4, 0.025)), 4)
            for w in widths:
                ok = np.array([corridor_trial(mesh, w, rng.uniform(0, np.pi), rng.uniform(-0.5, 0.5))
                               for _ in range(a.trials // 4)])
                row['corridor'].append({'width': float(w), 'faces': float(ok[:, 0].mean()),
                                        'vertices': float(ok[:, 1].mean())})
                if ok.all():
                    break
            def w95(key):
                for c in row['corridor']:
                    if c[key] >= 0.95:
                        return c['width']
                return None
            row['width95'] = {'faces': w95('faces'), 'vertices': w95('vertices')}
            res['meshes'].append(row)
            print(f"edge {l:.2f} jitter {jitter:.1f}: lethal area x{fa:.1f} (faces) x{va:.1f} "
                  f"(vertices); 95% passage at w = {row['width95']['faces']} (faces), "
                  f"{row['width95']['vertices']} (vertices); ideal {2 * R_ROBOT}", flush=True)
    json.dump(res, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
