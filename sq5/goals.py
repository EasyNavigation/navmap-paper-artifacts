#!/usr/bin/env python3
"""Goals for SQ5 in a map_server map: free cells at least `clearance` from any non-free cell,
in the component of the start, each at least `min_step` from the previous one. Deterministic.

Usage: goals.py map.yaml out.json [--n 20] [--clearance 0.5] [--min-step 2.5] [--start 0 0.1]
"""
import argparse
import json
import os
from collections import deque

import numpy as np


def load(yaml):
    meta = {}
    for line in open(yaml):
        if ':' in line:
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    img = os.path.join(os.path.dirname(yaml), meta['image'])
    raw = open(img, 'rb').read()
    parts, i = [], 0
    while len(parts) < 4:
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        parts.append(raw[i:j])
        i = j
    w, h = int(parts[1]), int(parts[2])
    px = np.frombuffer(raw[i + 1:i + 1 + w * h], np.uint8).reshape(h, w)[::-1]
    p = 1.0 - px / 255.0
    free = p < float(meta.get('free_thresh', 0.196))
    res = float(meta['resolution'])
    origin = [float(x) for x in meta['origin'].strip('[]').split(',')[:2]]
    return free, res, origin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map')
    ap.add_argument('out')
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--clearance', type=float, default=0.5)
    ap.add_argument('--min-step', type=float, default=2.5)
    ap.add_argument('--start', type=float, nargs=2, default=[0.0, 0.1])
    a = ap.parse_args()
    free, res, (ox, oy) = load(a.map)
    h, w = free.shape
    # Distance (in cells) to the nearest non-free cell, by BFS
    dist = np.full(free.shape, np.inf)
    q = deque()
    for j, i in zip(*np.nonzero(~free)):
        dist[j, i] = 0
        q.append((j, i))
    while q:
        j, i = q.popleft()
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            y, x = j + dj, i + di
            if 0 <= y < h and 0 <= x < w and dist[y, x] > dist[j, i] + 1:
                dist[y, x] = dist[j, i] + 1
                q.append((y, x))
    ok = dist * res >= a.clearance
    # Component of the start through cells with the robot's clearance
    sj, si = int((a.start[1] - oy) / res), int((a.start[0] - ox) / res)
    comp = np.zeros(free.shape, bool)
    q = deque([(sj, si)])
    comp[sj, si] = True
    while q:
        j, i = q.popleft()
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            y, x = j + dj, i + di
            if 0 <= y < h and 0 <= x < w and not comp[y, x] and dist[y, x] * res >= 0.25:
                comp[y, x] = True
                q.append((y, x))
    cand = np.argwhere(ok & comp)
    rng = np.random.default_rng(1)
    goals, prev = [], np.array(a.start)
    while len(goals) < a.n:
        j, i = cand[rng.integers(len(cand))]
        p = np.array([ox + (i + 0.5) * res, oy + (j + 0.5) * res])
        if np.linalg.norm(p - prev) >= a.min_step:
            goals.append({'x': round(float(p[0]), 3), 'y': round(float(p[1]), 3),
                          'yaw': round(float(rng.uniform(-np.pi, np.pi)), 3)})
            prev = p
    json.dump({'map': os.path.basename(a.map), 'start': a.start, 'goals': goals}, open(a.out, 'w'),
              indent=1)
    print(f'{len(goals)} goals, {len(cand)} candidate cells')


if __name__ == '__main__':
    main()
