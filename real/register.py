#!/usr/bin/env python3
"""Scan-to-map point-to-plane ICP against a voxel map of local planes (numpy only)."""
import numpy as np

KEY = np.array([1, 1 << 21, 1 << 42], np.int64)
OFF = 1 << 20


def keys(P, v):
    return ((np.floor(P / v).astype(np.int64) + OFF) * KEY).sum(1)


class PlaneMap:
    """Per voxel: mean and normal of the map points, kept if planar enough."""

    def __init__(self, M, v, min_pts=5, max_flat=0.1):
        self.v = v
        k = keys(M, v)
        o = np.argsort(k)
        k, M = k[o], M[o]
        u, start, cnt = np.unique(k, return_index=True, return_counts=True)
        keep = cnt >= min_pts
        idx = np.repeat(np.arange(len(u)), cnt)
        s = np.zeros((len(u), 3))
        np.add.at(s, idx, M)
        mean = s / cnt[:, None]
        d = M - mean[idx]
        C = np.zeros((len(u), 3, 3))
        np.add.at(C, idx, d[:, :, None] * d[:, None, :])
        w, V = np.linalg.eigh(C / cnt[:, None, None])
        n = V[:, :, 0]
        flat = w[:, 0] / np.maximum(w.sum(1), 1e-12)
        keep &= flat < max_flat
        self.k, self.mean, self.n = u[keep], mean[keep], n[keep]

    def lookup(self, P):
        k = keys(P, self.v)
        i = np.searchsorted(self.k, k)
        i = np.minimum(i, len(self.k) - 1)
        ok = self.k[i] == k
        return i, ok


def exp_se3(x):
    w, t = x[:3], x[3:]
    th = np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if th < 1e-12:
        R = np.eye(3) + K
    else:
        R = np.eye(3) + np.sin(th) / th * K + (1 - np.cos(th)) / th ** 2 * K @ K
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def icp(maps, S, T, iters=(10, 10, 15), huber=0.1):
    """Coarse to fine over the plane maps; returns the pose and the inlier fraction at the finest."""
    for pm, it in zip(maps, iters):
        h = max(huber, pm.v / 2)
        for _ in range(it):
            P = S @ T[:3, :3].T + T[:3, 3]
            i, ok = pm.lookup(P)
            if ok.sum() < 50:
                break
            p, n = P[ok], pm.n[i[ok]]
            r = ((p - pm.mean[i[ok]]) * n).sum(1)
            w = np.where(np.abs(r) <= h, 1.0, h / np.abs(r))
            J = np.hstack([np.cross(p, n), n])
            H = (J * w[:, None]).T @ J
            g = (J * w[:, None]).T @ r
            dx = -np.linalg.solve(H + 1e-6 * np.eye(6), g)
            T = exp_se3(dx) @ T
            if np.linalg.norm(dx) < 1e-5:
                break
    return T, fitness(maps[-1], S, T)


def fitness(pm, S, T, tol=0.1):
    P = S @ T[:3, :3].T + T[:3, 3]
    i, ok = pm.lookup(P)
    r = np.abs(((P[ok] - pm.mean[i[ok]]) * pm.n[i[ok]]).sum(1))
    return float((r < tol).sum() / len(S))


def pose2(x, y, z, yaw):
    T = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = (x, y, z)
    return T
