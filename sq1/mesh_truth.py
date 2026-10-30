#!/usr/bin/env python3
"""SQ1 ground truth from a world's collision mesh: where a robot can stand, and what connects.

A point of the mesh is walkable for the robot model of eval_sq1.py when its face is no steeper than
MAX_SLOPE and no surface of the mesh (ceiling, wall, clutter) lies between MAX_DZ and CLEARANCE above
it. The walkable surface is sampled on an XY grid of `res` (several levels per cell where floors
overlap), and two neighbour samples are connected when the step between them is climbable (the
same rule as the grid baselines). Thin walls are not crossed: the wall's own points make the
samples on both sides fail the clearance test.

Everything is vectorized with numpy; nothing here needs ROS.
"""
import numpy as np

# The robot model (shared with eval_sq1.py)
MAX_DZ = 0.25
MAX_SLOPE = 30.0
CLEARANCE = 1.0


def sample_triangles(tris, density, rng):
    """Uniform random points on the triangles, `density` per m² (Poisson per triangle)."""
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    counts = rng.poisson(area * density)
    idx = np.repeat(np.arange(len(tris)), counts)
    u, v = rng.random(len(idx)), rng.random(len(idx))
    flip = u + v > 1.0
    u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
    return tris[idx, 0] + u[:, None] * e1[idx] + v[:, None] * e2[idx]


def label_components(n, a, b):
    """Connected components of a graph of n nodes and edges (a[i], b[i]): min-label propagation
    with pointer jumping, vectorized."""
    lab = np.arange(n)
    if len(a) == 0:
        return lab
    while True:
        m = np.minimum(lab[a], lab[b])
        new = lab.copy()
        np.minimum.at(new, a, m)
        np.minimum.at(new, b, m)
        while True:  # pointer jumping: every node to its label's label
            jumped = new[new]
            if np.array_equal(jumped, new):
                break
            new = jumped
        if np.array_equal(new, lab):
            return lab
        lab = new


class MeshTruth:
    """Walkable samples of a collision mesh, their connectivity and queries on them."""

    def __init__(self, tris, res=0.1, margin=0.5, seed=0, max_dz=MAX_DZ, max_slope=MAX_SLOPE,
                 clearance=CLEARANCE, level_gap=0.1):
        rng = np.random.default_rng(seed)
        self.res, self.margin = res, margin
        self.max_dz, self.max_slope, self.clearance = max_dz, max_slope, clearance
        density = 6.0 / (res * res)  # several points per cell: no cell of a face is left empty

        # 1) Obstructions: every face of the mesh, as points sorted by (cell, z)
        obs = sample_triangles(tris, density, rng)
        okey = self._key(obs)
        o = np.lexsort((obs[:, 2], okey))
        self._obs_key, self._obs_z = okey[o], obs[o, 2]

        # 2) Walkable faces: no steeper than max_slope (either orientation: collision meshes are
        #    not consistently wound; the underside of a slab fails the clearance test anyway)
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        nn = np.linalg.norm(n, axis=1)
        ok = nn > 1e-12
        cos = np.zeros(len(tris))
        cos[ok] = np.abs(n[ok, 2]) / nn[ok]
        walk = sample_triangles(tris[cos >= np.cos(np.radians(max_slope))], density, rng)

        # 3) Levels per cell: samples of a cell within level_gap in z are one standing height
        wkey = self._key(walk)
        o = np.lexsort((walk[:, 2], wkey))
        wkey, walk = wkey[o], walk[o]
        new = np.r_[True, (wkey[1:] != wkey[:-1]) | (np.diff(walk[:, 2]) > level_gap)]
        grp = np.cumsum(new) - 1
        cnt = np.bincount(grp)
        z = np.bincount(grp, walk[:, 2]) / cnt
        key = wkey[new]

        # 4) Clearance: nothing of the mesh between max_dz and clearance above the standing height
        blocked = self._obstructed(key, z + max_dz, z + clearance)
        key, z = key[~blocked], z[~blocked]
        self.key, self.z = key, z
        self.i, self.j = self._unkey(key)
        self.xy = np.c_[(self.i + 0.5) * res, (self.j + 0.5) * res]
        self.N = len(key)

        # 5) Edges to the 8 neighbours within the climbable step, and components
        a, b = self._edges()
        self.edge_a, self.edge_b = a, b
        self.comp = label_components(self.N, a, b)
        self.interior = self._interior(int(round(margin / res)))

    # ------------------------------------------------------------------ grid keys
    _OFF = 1 << 20

    def _key(self, P):
        i = np.floor(P[:, 0] / self.res).astype(np.int64) + self._OFF
        j = np.floor(P[:, 1] / self.res).astype(np.int64) + self._OFF
        return i * (2 * self._OFF) + j

    def _unkey(self, key):
        return key // (2 * self._OFF) - self._OFF, key % (2 * self._OFF) - self._OFF

    def _obstructed(self, key, zlo, zhi):
        """Whether some obstruction point of the same cell has lo < z <= hi."""
        # Position of the cell's run in the obstruction arrays, then of zlo/zhi within the run
        start = np.searchsorted(self._obs_key, key, 'left')
        end = np.searchsorted(self._obs_key, key, 'right')
        out = np.zeros(len(key), bool)
        has = end > start
        for k in np.flatnonzero(has):  # runs are short (a few levels of a few points)
            zz = self._obs_z[start[k]:end[k]]
            out[k] = np.any((zz > zlo[k]) & (zz <= zhi[k]))
        return out

    def _cell_runs(self, key):
        start = np.searchsorted(self.key, key, 'left')
        end = np.searchsorted(self.key, key, 'right')
        return start, end

    def _edges(self):
        tan = np.tan(np.radians(self.max_slope))
        A, B = [], []
        maxlev = int(np.bincount(np.unique(self.key, return_inverse=True)[1]).max())
        for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
            lim = max(self.max_dz, tan * self.res * np.hypot(di, dj))
            nk = (self.i + di + self._OFF) * (2 * self._OFF) + (self.j + dj + self._OFF)
            start, end = self._cell_runs(nk)
            for lv in range(maxlev):
                m = start + lv < end
                src = np.flatnonzero(m)
                dst = start[m] + lv
                close = np.abs(self.z[dst] - self.z[src]) <= lim
                A.append(src[close])
                B.append(dst[close])
        return np.concatenate(A), np.concatenate(B)

    def _interior(self, k):
        """Samples with all 8 neighbours present, k times over (eroded by k cells)."""
        keep = np.ones(self.N, bool)
        for _ in range(max(k, 1)):
            deg = np.zeros(self.N, np.int64)
            both = keep[self.edge_a] & keep[self.edge_b]
            np.add.at(deg, self.edge_a[both], 1)
            np.add.at(deg, self.edge_b[both], 1)
            keep = keep & (deg >= 8)
        return keep

    # ------------------------------------------------------------------ observation
    def local_slope(self):
        """Tangent of the surface's local slope: steepest rise to a 4-neighbour sample of the
        same surface (within a walkable step and slope)."""
        reach = self.max_dz + np.tan(np.radians(self.max_slope)) * self.res
        s = np.zeros(self.N)
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            start, end = self._cell_runs(self.key + di * (2 * self._OFF) + dj)
            for k in np.flatnonzero(end > start):
                dz = np.abs(self.z[start[k]:end[k]] - self.z[k])
                dz = dz[dz <= reach]
                if len(dz):
                    s[k] = max(s[k], dz.min() / self.res)
        return s

    def observe(self, P, tol=0.15, V=None):
        """Marks as observed the samples with points of cloud P in their cell within tol in z:
        what a representation built from P can know about. Queries and the missed area are
        then taken there only. With the points' viewpoints V, only points that may have been
        seen from above count: rays climbing no more than the local slope + 1 degree (a roof
        seen only from below was not observed as a floor)."""
        pkey = self._key(P)
        if V is not None:
            d = P - V
            el = np.degrees(np.arctan2(d[:, 2], np.linalg.norm(d[:, :2], axis=1)))
            thr = np.degrees(np.arctan(self.local_slope())) + 1.0
        o = np.lexsort((P[:, 2], pkey))
        pkey, pz = pkey[o], P[o, 2]
        if V is not None:
            el = el[o]
        start = np.searchsorted(pkey, self.key, 'left')
        end = np.searchsorted(pkey, self.key, 'right')
        # Within the cell's run, any point with |z - z_sample| <= tol (binary search on z)
        lo = np.empty(self.N, np.int64)
        hi = np.empty(self.N, np.int64)
        for k in np.flatnonzero(end > start):
            zz = pz[start[k]:end[k]]
            lo[k] = np.searchsorted(zz, self.z[k] - tol, 'left')
            hi[k] = np.searchsorted(zz, self.z[k] + tol, 'right')
            if V is not None and hi[k] > lo[k]:
                ok = np.flatnonzero(el[start[k] + lo[k]:start[k] + hi[k]] <= thr[k])
                lo[k], hi[k] = (0, 1) if len(ok) else (0, 0)
        self.observed = (end > start) & (hi > lo)
        return float(self.observed.mean())

    # ------------------------------------------------------------------ queries
    def sample(self, n, rng):
        """n random query points (x, y, z, component) on the walkable surface, `margin` away
        from its borders (and observed, if observe() was called)."""
        mask = self.interior & getattr(self, 'observed', np.ones(self.N, bool))
        idx = np.flatnonzero(mask)
        pick = rng.choice(idx, size=n)
        jit = (rng.random((n, 2)) - 0.5) * self.res
        return np.c_[self.xy[pick] + jit, self.z[pick], self.comp[pick]]

    def sample_apart(self, n, rng, min_area=5.0):
        """n pairs of query points on different components (both of at least min_area m² of
        observed interior): pairs a representation must not connect. Returns (2n, 4), pairs
        as consecutive rows; empty if fewer than two components qualify."""
        mask = self.interior & getattr(self, 'observed', np.ones(self.N, bool))
        comps, counts = np.unique(self.comp[mask], return_counts=True)
        big = comps[counts * self.res ** 2 >= min_area]
        if len(big) < 2:
            return np.zeros((0, 4))
        out = []
        for _ in range(n):
            ca, cb = rng.choice(big, size=2, replace=False)
            for c in (ca, cb):
                k = rng.choice(np.flatnonzero(mask & (self.comp == c)))
                jit = (rng.random(2) - 0.5) * self.res
                out.append([*(self.xy[k] + jit), self.z[k], self.comp[k]])
        return np.array(out)

    def on_walkable(self, P, tol):
        """For each point, whether a walkable sample of its cell is within tol in z."""
        start, end = self._cell_runs(self._key(P))
        out = np.zeros(len(P), bool)
        for k in np.flatnonzero(end > start):
            out[k] = np.any(np.abs(self.z[start[k]:end[k]] - P[k, 2]) <= tol)
        return out

    # ------------------------------------------------------------------ cache
    def save(self, path):
        np.savez_compressed(
            path, key=self.key, z=self.z, comp=self.comp, interior=self.interior,
            meta=np.array([self.res, self.margin, self.max_dz, self.max_slope, self.clearance]))

    @classmethod
    def load(cls, path):
        d = np.load(path)
        t = cls.__new__(cls)
        t.res, t.margin, t.max_dz, t.max_slope, t.clearance = map(float, d['meta'])
        t.key, t.z, t.comp, t.interior = d['key'], d['z'], d['comp'], d['interior']
        t.i, t.j = t._unkey(t.key)
        t.xy = np.c_[(t.i + 0.5) * t.res, (t.j + 0.5) * t.res]
        t.N = len(t.key)
        return t

    def area(self):
        return float(self.N * self.res * self.res)

    def summary(self):
        sizes = np.bincount(np.unique(self.comp, return_inverse=True)[1])
        big = np.sort(sizes)[::-1]
        return {'truth_res': self.res, 'walkable_area_m2': self.area(),
                'components': int(len(sizes)),
                'components_over_10m2': int((big * self.res ** 2 > 10.0).sum()),
                'largest_components_m2': [float(s * self.res ** 2) for s in big[:5]]}


def load_world_mesh(path, ground='highest', spawn=(0.0, 0.0)):
    """Triangles of a Gazebo collision mesh (.dae or binary .stl) in the playground's map frame:
    z = 0 at the surface under the spawn point ('highest' for terrain, 'lowest' for a building
    whose upper floors are over the spawn point)."""
    import os
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
    from mesh_to_pcd import load_dae, load_stl
    tris = load_dae(path) if path.lower().endswith('.dae') else load_stl(path)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    x, y = spawn
    d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    ok = np.abs(d) > 1e-12
    dd = np.where(ok, d, 1.0)
    l1 = ((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1])) / dd
    l2 = ((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1])) / dd
    inside = ok & (l1 >= 0) & (l2 >= 0) & (1 - l1 - l2 >= 0)
    zs = (l1 * a[:, 2] + l2 * b[:, 2] + (1 - l1 - l2) * c[:, 2])[inside]
    z0 = zs.min() if ground == 'lowest' else zs.max()
    tris = tris.copy()
    tris[:, :, 2] -= z0
    return tris
