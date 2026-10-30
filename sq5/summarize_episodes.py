#!/usr/bin/env python3
"""E11 (review 2): the independent episodes of sq5/run_sq5_episodes.sh. House: the costmap and
NavMap stacks paired by episode (leg, repetition), n = legs x repetitions, with the 90% confidence
interval of the difference and the two one-sided tests against a margin of 10% of the costmap's
mean; also the per-leg means. Summit scenes: outcome of every episode by leg.
Usage: summarize_episodes.py [episodes_dir=$NAVMAP_DATA/sq5/episodes]"""
import glob
import json
import math
import os
import re
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from summarize_sq5 import t_cdf, t_ppf  # noqa: E402

D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ.get('NAVMAP_DATA', 'data'), 'sq5', 'episodes')


def load(scene):
    eps = defaultdict(dict)  # (leg, rep) -> stack -> result
    for f in sorted(glob.glob(os.path.join(D, scene, '*_g*_r*.json'))):
        # tags: <stack>_g<leg>_r<rep> or <stack>_g<from>-<leg>_r<rep> (explicit origin)
        m = re.match(r'(\w+?)_g(?:(\d+)-)?(\d+)_r(\d+)\.json', os.path.basename(f))
        r = json.load(open(f))['results'][0]
        eps[(int(m.group(3)), int(m.group(4)))][m.group(1)] = r
    return eps


def tost(d, margin, label, unit, what='episodes'):
    n = len(d)
    sem = d.std(ddof=1) / math.sqrt(n)
    half = t_ppf(0.95, n - 1) * sem
    p = max(1 - t_cdf((d.mean() + margin) / sem, n - 1), t_cdf((d.mean() - margin) / sem, n - 1))
    print(f"  {label}: NavMap - costmap {d.mean():+.2f} {unit} (90% CI {d.mean() - half:+.2f}.."
          f"{d.mean() + half:+.2f}), margin +-{margin:.2f}, TOST p = {p:.2g}, n = {n} {what}")


def house(eps):
    both = {k: v for k, v in eps.items() if 'costmap' in v and 'navmap' in v}
    if not both:
        return
    for st in ('costmap', 'navmap'):
        rs = [v[st] for v in both.values()]
        ok = [r for r in rs if r['outcome'] == 'finished']
        print(f"{st:8s} reached {len(ok)}/{len(rs)}  time {np.mean([r['time_s'] for r in ok]):6.1f} s"
              f"  path {np.mean([r['travelled_m'] for r in ok]):6.1f} m"
              f"  final err med {np.median([r['final_error_m'] for r in ok]):.3f} m"
              f"  loc {np.mean([r['loc_error_mean_m'] for r in ok if r['loc_error_mean_m']]):.3f} m"
              f"  cpu {np.mean([r['cpu_percent'] for r in ok if r['cpu_percent']]):.0f}%"
              f"  failures {[f'g{k[0]}r{k[1]}:{v[st]['outcome']}' for k, v in both.items() if v[st]['outcome'] != 'finished']}")
    for key, label, unit in (('time_s', 'time', 's'), ('travelled_m', 'path', 'm')):
        pairs = [(v['costmap'][key], v['navmap'][key]) for v in both.values()
                 if v['costmap']['outcome'] == v['navmap']['outcome'] == 'finished']
        a = np.array([p[0] for p in pairs])
        b = np.array([p[1] for p in pairs])
        # The episodes repeat the same legs (review 3, M1): the test's unit is the leg, the mean
        # of its repetitions; the per-episode test only measures the repeatability of a leg
        legs = sorted({k[0] for k in both})
        per_leg = np.array([np.mean([v['navmap'][key] - v['costmap'][key] for k, v in both.items()
                                     if k[0] == g and v['costmap']['outcome'] == v['navmap']['outcome'] == 'finished'])
                            for g in legs])
        tost(per_leg, 0.1 * a.mean(), label, unit, what='legs (mean of their repetitions)')
        tost(b - a, 0.1 * a.mean(), label + ' (per episode, secondary)', unit)
        for st in ('costmap', 'navmap'):
            sds = [np.std([v[st][key] for k, v in both.items() if k[0] == g], ddof=1) for g in legs]
            print(f"     {st} repeatability of a leg: sd {np.mean(sds):.2f} {unit} (mean over legs)")
    legs = sorted({k[0] for k in both})
    print('  per leg (costmap / navmap time s):', ' '.join(
        f"g{g}:{np.mean([v['costmap']['time_s'] for k, v in both.items() if k[0] == g]):.1f}/"
        f"{np.mean([v['navmap']['time_s'] for k, v in both.items() if k[0] == g]):.1f}" for g in legs))


def summit(scene, eps):
    rs = [(k, v['navmap']) for k, v in sorted(eps.items()) if 'navmap' in v]
    if not rs:
        return
    ok = [r for _, r in rs if r['outcome'] == 'finished']
    print(f"{scene:10s} episodes {len(rs)}  reached {len(ok)}  time {np.mean([r['time_s'] for r in ok]) if ok else float('nan'):.1f} s"
          f"  path {np.mean([r['travelled_m'] for r in ok]) if ok else float('nan'):.1f} m"
          f"  loc {np.mean([r['loc_error_mean_m'] for r in ok if r['loc_error_mean_m']]) if ok else float('nan'):.3f} m")
    for (leg, rep), r in rs:
        print(f"   leg {r.get('episode_from', leg - 1)}->{leg} rep {rep}: {r['outcome']:12s} "
              f"{r['time_s']:6.1f} s {r['travelled_m']:6.1f} m  loc mean "
              f"{r['loc_error_mean_m'] if r['loc_error_mean_m'] is not None else float('nan'):.2f} max "
              f"{r['loc_error_max_m'] if r['loc_error_max_m'] is not None else float('nan'):.2f}")


for scene in ('house', 'garage', 'excavation'):
    eps = load(scene)
    if eps:
        print(f'{scene} ({len(eps)} episodes)')
        house(eps) if scene == 'house' else summit(scene, eps)
