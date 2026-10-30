#!/usr/bin/env python3
"""Checks of mesh_truth.MeshTruth on toy geometry (a few triangles): python3 test_mesh_truth.py"""
import numpy as np

from mesh_truth import MeshTruth


def quad(x0, y0, x1, y1, z0, z1=None):
    """Two triangles of a rectangle; z goes from z0 at x0 to z1 at x1 (a ramp if they differ)."""
    z1 = z0 if z1 is None else z1
    a, b, c, d = (x0, y0, z0), (x1, y0, z1), (x1, y1, z1), (x0, y1, z0)
    return np.array([[a, b, c], [a, c, d]], float)


def wall(x, y0, y1, z0, z1):
    a, b, c, d = (x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)
    return np.array([[a, b, c], [a, c, d]], float)


def comps(t, pts):
    """Component of the sample nearest to each (x, y, z)."""
    out = []
    for p in pts:
        d = np.hypot(t.xy[:, 0] - p[0], t.xy[:, 1] - p[1]) + 10 * np.abs(t.z - p[2])
        out.append(int(t.comp[np.argmin(d)]))
    return out


def test_two_floors_joined_by_a_ramp_and_a_roof_out_of_reach():
    # Floor 0 (10 x 4), a ramp up 3 m over 8 m (20 deg) to floor 1 (10 x 4, 3 m up, over floor 0
    # in part), and a slab 1 m above part of floor 1 (no clearance there)
    tris = np.concatenate([
        quad(0, 0, 10, 4, 0.0),
        quad(10, 0, 18, 4, 0.0, 3.0),
        quad(18, 0, 28, 4, 3.0),
        quad(22, 0, 26, 4, 3.6), quad(22, 0, 26, 4, 3.8),  # low slab over floor 1
        quad(-6, 6, 4, 10, 6.0),                          # an unreachable roof
    ])
    t = MeshTruth(tris, res=0.2, margin=0.4)
    c0, c_ramp, c1, roof = comps(t, [(2, 2, 0), (14, 2, 1.5), (20, 2, 3), (0, 8, 6)])
    assert c0 == c_ramp == c1, 'floor 0, the ramp and floor 1 connect'
    assert roof != c0, 'the roof is apart'
    under = (t.xy[:, 0] > 22.5) & (t.xy[:, 0] < 25.5) & (np.abs(t.z - 3.0) < 0.2)
    assert not under.any(), 'no standing under the low slab'
    print('ok: ramp joins floors, roof apart, low slab blocks', t.summary())


def test_a_steep_ramp_and_a_thin_wall_disconnect():
    tris = np.concatenate([
        quad(0, 0, 6, 4, 0.0),
        quad(6, 0, 8, 4, 0.0, 2.0),      # 45 deg: not climbable, not walkable
        quad(8, 0, 14, 4, 2.0),
        quad(0, 6, 14, 10, 0.0),         # a second floor area at z 0 ...
        wall(7.0, 6, 10, 0.0, 2.0),      # ... cut in two by a thin wall
    ])
    t = MeshTruth(tris, res=0.1, margin=0.3)
    a, b, c, d = comps(t, [(3, 2, 0), (11, 2, 2), (3, 8, 0), (11, 8, 0)])
    assert a != b, 'the steep ramp does not connect'
    assert c != d, 'the thin wall is not crossed'
    print('ok: steep ramp and thin wall disconnect')


def test_observed_with_viewpoints():
    # A floor, a 20 deg ramp up to a deck, and a roof (zero thickness, as in collision meshes)
    tris = np.concatenate([
        quad(0, 0, 10, 4, 0.0), quad(10, 0, 18, 4, 0.0, 2.9), quad(18, 0, 24, 4, 2.9),
        quad(0, 0, 10, 4, 3.0),
    ])
    t = MeshTruth(tris, res=0.2, margin=0.4)
    rng = np.random.default_rng(0)
    # The sensor stays on the floor (x = 2, 0.8 m up): it sees the floor and the ramp from
    # above or from the ramp's foot, and the roof only from below
    o = np.array([2.0, 2.0, 0.8])
    floor = np.c_[rng.uniform(0, 10, 4000), rng.uniform(0, 4, 4000), np.zeros(4000)]
    xr = rng.uniform(10, 14, 2000)
    ramp = np.c_[xr, rng.uniform(0, 4, 2000), (xr - 10) * 2.9 / 8]
    roof = np.c_[rng.uniform(0, 10, 4000), rng.uniform(0, 4, 4000), np.full(4000, 3.0)]
    P = np.concatenate([floor, ramp, roof])
    V = np.tile(o, (len(P), 1))

    def obs(x, y, z):
        k = np.argmin(np.hypot(t.xy[:, 0] - x, t.xy[:, 1] - y) + 10 * np.abs(t.z - z))
        return bool(t.observed[k])
    t.observe(P)
    assert obs(5, 2, 0) and obs(12, 2, 0.7) and obs(5, 2, 3.0), 'without viewpoints'
    t.observe(P, V=V)
    assert obs(5, 2, 0) and obs(12, 2, 0.7), 'floor and ramp seen from above'
    assert not obs(5, 2, 3.0), 'the roof was only seen from below'
    assert not obs(20, 2, 2.9), 'no points on the deck'
    print('ok: observed with viewpoints')


def test_queries_and_cache(tmp='/tmp/mesh_truth_test.npz'):
    t = MeshTruth(quad(0, 0, 10, 10, 0.0), res=0.2, margin=0.5)
    Q = t.sample(200, np.random.default_rng(1))
    assert (Q[:, 0] > 0.4).all() and (Q[:, 0] < 9.6).all(), 'queries away from the border'
    assert t.on_walkable(np.c_[Q[:, :2], Q[:, 2] + 0.1], 0.3).all()
    assert not t.on_walkable(np.array([[5, 5, 1.0], [20, 20, 0.0]]), 0.3).any()
    t.save(tmp)
    u = MeshTruth.load(tmp)
    assert u.N == t.N and (u.comp == t.comp).all() and (u.interior == t.interior).all()
    print('ok: queries, on_walkable and cache')


if __name__ == '__main__':
    test_two_floors_joined_by_a_ramp_and_a_roof_out_of_reach()
    test_a_steep_ramp_and_a_thin_wall_disconnect()
    test_observed_with_viewpoints()
    test_queries_and_cache()
