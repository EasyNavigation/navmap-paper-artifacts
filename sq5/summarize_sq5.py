#!/usr/bin/env python3
"""SQ5 table from the runs in $NAVMAP_DATA/sq5: house (costmap and NavMap stacks, paired by run,
the independent repetitions, and descriptively by goal averaged over its runs, with the 90%
confidence intervals of the differences and the two one-sided tests against a margin of 10% of
the costmap's mean) and the Summit scenes (means over the runs, and the failures by cause: no path
in the stack's map, the goals that sq5/check_goals.py finds unreachable there; localisation drift,
a mean error over 0.5 m; cascade, no progress after an earlier failure; or a timeout en route).
Usage: summarize_sq5.py [sq5_dir]"""
import glob
import json
import os
import sys
from collections import defaultdict

import math

import numpy as np


# Student's t without scipy: the CDF through the regularized incomplete beta (Lentz's continued
# fraction, as in Numerical Recipes), and the quantile by bisection
def _betacf(a, b, x):
    tiny = 1e-300
    c, d = 1.0, 1.0 - (a + b) * x / (a + 1.0)
    d = 1.0 / (d if abs(d) > tiny else tiny)
    h = d
    for m in range(1, 300):
        m2 = 2 * m
        aa = m * (b - m) * x / ((a + m2 - 1.0) * (a + m2))
        d = 1.0 + aa * d
        d = 1.0 / (d if abs(d) > tiny else tiny)
        c = 1.0 + aa / (c if abs(c) > tiny else tiny)
        h *= d * c
        aa = -(a + m) * (a + b + m) * x / ((a + m2) * (a + m2 + 1.0))
        d = 1.0 + aa * d
        d = 1.0 / (d if abs(d) > tiny else tiny)
        c = 1.0 + aa / (c if abs(c) > tiny else tiny)
        de = d * c
        h *= de
        if abs(de - 1.0) < 1e-12:
            break
    return h


def _betainc(a, b, x):
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    lb = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b) + a * math.log(x) + b * math.log(1 - x)
    if x < (a + 1) / (a + b + 2):
        return math.exp(lb) * _betacf(a, b, x) / a
    return 1.0 - math.exp(lb) * _betacf(b, a, 1 - x) / b


def t_cdf(t, df):
    x = df / (df + t * t)
    p = 0.5 * _betainc(df / 2.0, 0.5, x)
    return 1 - p if t > 0 else p


def t_ppf(q, df):
    lo, hi = -50.0, 50.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if t_cdf(mid, df) < q:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ.get('NAVMAP_DATA', 'data'), 'sq5')


def load(pattern):
    runs = {}
    for f in sorted(glob.glob(os.path.join(D, pattern))):
        runs[os.path.basename(f)] = json.load(open(f))['results']
    return runs


def summary(name, runs):
    rs = [r for run in runs.values() for r in run]
    ok = [r for r in rs if r['outcome'] == 'finished']
    cpu = [r['cpu_percent'] for r in rs if r.get('cpu_percent') is not None]
    rng = [r['min_range_m'] for r in ok if r.get('min_range_m') is not None]
    print(f"{name:10s} runs {len(runs)}  reached {len(ok)}/{len(rs)}  time {np.mean([r['time_s'] for r in ok]):6.1f} s"
          f"  path {np.mean([r['travelled_m'] for r in ok]):6.1f} m  final err med {np.median([r['final_error_m'] for r in ok]):.3f} m"
          f"  |dz| max {max(abs(r['final_dz_m']) for r in ok):.2f}  closest {np.mean(rng) if rng else float('nan'):.2f} m"
          f"  loc {np.mean([r['loc_error_mean_m'] for r in rs]):.3f} m  cpu {np.mean(cpu):.0f}%")
    per_run = [f"{sum(r['outcome'] == 'finished' for r in run)}/{len(run)}" for run in runs.values()]
    fails = [r['outcome'] for r in rs if r['outcome'] != 'finished']
    print(f"{'':10s} per run {per_run}  failures {fails}")


def paired(a_runs, b_runs, key, label, unit):
    """b - a paired by (run index, goal); 90% CI and TOST against +-10% of a's mean."""
    a = [r for run in a_runs.values() for r in run if r['outcome'] == 'finished']
    b = [r for run in b_runs.values() for r in run if r['outcome'] == 'finished']
    n = min(len(a), len(b))
    pa = defaultdict(list)
    pb = defaultdict(list)
    for run in a_runs.values():
        for r in run:
            pa[r['goal']].append(r[key])
    for run in b_runs.values():
        for r in run:
            pb[r['goal']].append(r[key])
    # One difference per goal: the mean over the runs of each stack (the goals of a run are
    # driven in sequence, so the runs, not the goals within a run, are the repetitions)
    d = np.array([np.mean(pb[g]) - np.mean(pa[g]) for g in pa if g in pb])
    m = np.mean([r[key] for r in a])
    margin = 0.1 * m
    df = len(d) - 1
    sem = d.std(ddof=1) / math.sqrt(len(d))
    half = t_ppf(0.95, df) * sem
    ci = (d.mean() - half, d.mean() + half)
    p_low = 1 - t_cdf((d.mean() + margin) / sem, df)   # H0: diff <= -margin
    p_high = t_cdf((d.mean() - margin) / sem, df)      # H0: diff >= +margin
    print(f"  {label}: NavMap - costmap {d.mean():+.2f} {unit} (90% CI {ci[0]:+.2f}..{ci[1]:+.2f}), "
          f"margin +-{margin:.2f}, TOST p = {max(p_low, p_high):.2g}, n = {len(d)}")


def paired_runs(a_runs, b_runs, key, label, unit):
    """b - a with the run as the unit: the mean over the goals of each run, runs paired in order
    (same goals, same scene); 90% CI and TOST against +-10% of a's mean. Few degrees of freedom,
    but the runs are the independent repetitions."""
    ra = [np.mean([r[key] for r in run if r['outcome'] == 'finished']) for run in a_runs.values()]
    rb = [np.mean([r[key] for r in run if r['outcome'] == 'finished']) for run in b_runs.values()]
    n = min(len(ra), len(rb))
    d = np.array(rb[:n]) - np.array(ra[:n])
    margin = 0.1 * np.mean(ra)
    df = n - 1
    sem = d.std(ddof=1) / math.sqrt(n)
    half = t_ppf(0.95, df) * sem
    p_low = 1 - t_cdf((d.mean() + margin) / sem, df)
    p_high = t_cdf((d.mean() - margin) / sem, df)
    print(f"  {label}, run as unit: NavMap - costmap {d.mean():+.2f} {unit} "
          f"(90% CI {d.mean() - half:+.2f}..{d.mean() + half:+.2f}), margin +-{margin:.2f}, "
          f"TOST p = {max(p_low, p_high):.2g}, n = {n} runs; per run A {np.round(ra, 2)} B {np.round(rb, 2)}")


def failures(scene, runs):
    """Every goal not reached, by cause (see the module docstring); the unreachable goals come
    from sq5/check_goals.py (data/sq5/goals_check_<scene>.json), at the inscribed radius."""
    f = os.path.join(D, f'goals_check_{scene}.json')
    unreachable = set()
    if os.path.exists(f):
        chk = json.load(open(f))
        key = [k for k in chk['goals'][0] if k.startswith('connected_r')][-1]
        unreachable = {g['goal'] for g in chk['goals'] if not g[key]}
    cause = defaultdict(list)
    for name, run in runs.items():
        prev_failed = False
        for r in run:
            if r['outcome'] == 'finished':
                prev_failed = False
                continue
            if r['goal'] in unreachable:
                c = 'no path in the map (unreachable goal)'
            elif prev_failed and r['travelled_m'] < 1.0:
                c = 'cascade (no progress after an earlier failure)'
            elif r.get('loc_error_mean_m', 0) > 0.5:
                c = 'localisation drift'
            elif r['outcome'] == 'error':
                c = 'no path'
            else:
                c = 'timeout en route'
            cause[c].append(f"{name.split('_run')[-1].split('.')[0]}:g{r['goal']}")
            prev_failed = True
    total = sum(len(v) for v in cause.values())
    n_goals = sum(len(run) for run in runs.values())
    feasible = n_goals - len(unreachable) * len(runs)
    reached = sum(r['outcome'] == 'finished' for run in runs.values() for r in run)
    print(f"{'':10s} unreachable in the stack's map: {sorted(unreachable)}; reached over feasible "
          f"{reached}/{feasible}; {total} failures by cause:")
    for c, v in cause.items():
        print(f"{'':12s}{c}: {len(v)}  {v}")


house_c = load('costmap_run*.json')
house_n = load('navmap_run*.json')
if house_c and house_n:
    print('House (Kobuki), per stack over the runs')
    summary('costmap', house_c)
    summary('navmap', house_n)
    paired(house_c, house_n, 'time_s', 'time', 's')
    paired(house_c, house_n, 'travelled_m', 'path', 'm')
    paired_runs(house_c, house_n, 'time_s', 'time', 's')
    paired_runs(house_c, house_n, 'travelled_m', 'path', 'm')
for scene in ('garage', 'excavation'):
    runs = load(f'summit_{scene}_run*.json')
    if runs:
        print(f'Summit, {scene}')
        summary(scene, runs)
        failures(scene, runs)
