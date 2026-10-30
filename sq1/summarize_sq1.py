#!/usr/bin/env python3
"""SQ1 tables from the JSON results: mean and standard deviation over seeds, and the MLS sweep.

Usage: summarize_sq1.py [results_dir]   (default $NAVMAP_DATA/sq1)
"""
import glob
import json
import os
import re
import sys

import numpy as np

METRICS = ['false_bridge', 'false_disconnection', 'false_disconnection_located',
           'phantom_area_fraction', 'missed_area_fraction']
SHORT = ['bridge', 'disc', 'disc_loc', 'phantom', 'missed']


def pct(v):
    return '  -  ' if v is None else f'{100 * v:5.1f}'


def seeds(d):
    """main/{scene}_{res}_{plain|vp}_s{seed}.json: the baselines and NavMap from the bare points,
    and NavMap with viewpoints (navmap+vp)."""
    groups = {}
    for f in glob.glob(os.path.join(d, 'main', '*_s*.json')):
        m = re.match(r'(.+)_(plain|vp)_s(\d+)\.json$', os.path.basename(f))
        res = json.load(open(f))['results']
        if m.group(2) == 'vp':
            res = {'navmap+vp': res['navmap']}
        groups.setdefault(m.group(1), {}).setdefault(int(m.group(3)), {}).update(res)
    if not groups:
        return
    print('SQ1, % (mean ± std over seeds)')
    print(f'{"scenario":24s} {"rep":10s} ' + ' '.join(f'{s:>13s}' for s in SHORT))
    for key in sorted(groups):
        runs = list(groups[key].values())
        for rep in ['elev', 'mls', 'vox', 'navmap', 'navmap+vp']:
            if any(rep not in r for r in runs):
                continue
            cells = []
            for m in METRICS:
                v = [r[rep][m] for r in runs if r[rep][m] is not None]
                cells.append(f'{100 * np.mean(v):5.1f} ± {100 * np.std(v):4.1f}' if v else '      -      ')
            print(f'{key:24s} {rep:10s} ' + ' '.join(f'{c:>13s}' for c in cells))
        print(f'{"":24s} n={len(runs)} seeds')


def mls(d):
    rows = []
    for f in glob.glob(os.path.join(d, 'mls', '*.json')):
        m = re.match(r'(.+)_g([\d.]+)_m([\d.]+)\.json$', os.path.basename(f))
        r = json.load(open(f))['results']['mls']
        rows.append((m.group(1), float(m.group(2)), float(m.group(3)), r))
    if not rows:
        return
    print('\nMLS sweep, % (gap and margin in m)')
    print(f'{"scenario":32s} {"gap":>5s} {"margin":>6s} ' + ' '.join(f'{s:>8s}' for s in SHORT))
    for sc, gap, margin, r in sorted(rows, key=lambda x: x[:3]):
        print(f'{sc:32s} {gap:5.2f} {margin:6.2f} ' + ' '.join(f'{pct(r[m]):>8s}' for m in METRICS))


if __name__ == '__main__':
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ.get('NAVMAP_DATA', 'data'), 'sq1')
    seeds(d)
    mls(d)
