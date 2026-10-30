import sys
import numpy as np
sys.path.insert(0, sys.argv[1])
from render import load  # noqa: E402

def heights(P, T, x, y):
    a, b, c = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
    d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    ok = np.abs(d) > 1e-12
    dd = np.where(ok, d, 1.0)
    l1 = ((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1])) / dd
    l2 = ((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1])) / dd
    l3 = 1 - l1 - l2
    m = ok & (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
    return (l1 * a[:, 2] + l2 * b[:, 2] + l3 * c[:, 2])[m]

Po, To, _ = load(sys.argv[2])
Pn, Tn, _ = load(sys.argv[3])
rng = np.random.default_rng(1)
diffs, layers_old, both = [], [], 0
for _ in range(1500):
    x, y = rng.uniform(-60, 80), rng.uniform(-60, 48)
    ho, hn = heights(Po, To, x, y), heights(Pn, Tn, x, y)
    layers_old.append(len(np.unique(ho.round(2))))
    if len(ho) and len(hn):
        both += 1
        diffs.append(np.min(np.abs(hn[:, None] - ho[None, :])))
diffs = np.array(diffs)
print(f'samples with both maps: {both}; |z_new - z_old| p50 {np.percentile(diffs,50):.2f} p90 {np.percentile(diffs,90):.2f} p99 {np.percentile(diffs,99):.2f} m')
print(f'old map distinct heights over a sample point: p50 {np.percentile(layers_old,50):.0f} p90 {np.percentile(layers_old,90):.0f} max {max(layers_old)}')
for x, y in [(0, 0), (60, 0), (0, -55), (-40, 30)]:
    print((x, y), 'old', np.unique(heights(Po, To, x, y).round(2))[:5], 'new', np.unique(heights(Pn, Tn, x, y).round(2)))
