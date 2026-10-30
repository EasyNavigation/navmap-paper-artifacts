#!/usr/bin/env python3
"""Planning benchmark table from bench_plan's JSON lines: success, time (median, 95th percentile),
length and mean inflation cost per map and planner; for the flat maps, the paired difference of
NavMap against the costmap. Usage: summarize_plan.py plan.jsonl [...]"""
import json
import sys
from collections import defaultdict

import numpy as np

rows = [json.loads(l) for f in sys.argv[1:] for l in open(f) if l.strip()]
by = defaultdict(list)
for r in rows:
    by[(r['map'], r['planner'])].append(r)
print(f"{'map':12s} {'planner':8s} {'ok':>8s} {'t med ms':>9s} {'t p95':>7s} {'len m':>7s} "
      f"{'cost':>6s} {'ascent':>7s}")
for (m, p), rs in sorted(by.items()):
    ok = [r for r in rs if r['ok']]
    t = np.array([r['time_ms'] for r in rs])
    print(f"{m:12s} {p:8s} {len(ok):4d}/{len(rs):<4d} {np.median(t):9.2f} "
          f"{np.percentile(t, 95):7.2f} {np.mean([r['length_m'] for r in ok]):7.2f} "
          f"{np.mean([r['cost'] for r in ok]):6.1f} {np.mean([r['ascent_m'] for r in ok]):7.2f}")
# Paired comparison where both planners ran
for m in sorted({k[0] for k in by}):
    if (m, 'costmap') not in by or (m, 'navmap') not in by:
        continue
    c = {r['pair']: r for r in by[(m, 'costmap')]}
    n = {r['pair']: r for r in by[(m, 'navmap')]}
    both = [k for k in c if k in n and c[k]['ok'] and n[k]['ok']]
    dl = np.array([n[k]['length_m'] - c[k]['length_m'] for k in both])
    rl = np.array([n[k]['length_m'] / c[k]['length_m'] for k in both])
    dt = np.array([n[k]['time_ms'] / c[k]['time_ms'] for k in both])
    only_c = sum(1 for k in c if c[k]['ok'] and not n.get(k, {}).get('ok'))
    only_n = sum(1 for k in n if n[k]['ok'] and not c.get(k, {}).get('ok'))
    print(f"\n{m}: {len(both)} pairs solved by both; only costmap {only_c}, only navmap {only_n}")
    print(f"  length navmap - costmap: mean {dl.mean():+.3f} m, median ratio {np.median(rl):.3f}, "
          f"longer in {100 * np.mean(dl > 0.01):.0f}% of the pairs")
    print(f"  time ratio navmap / costmap: median {np.median(dt):.1f}x")
