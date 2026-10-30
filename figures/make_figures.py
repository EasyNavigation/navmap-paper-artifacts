#!/usr/bin/env python3
"""Figures of the paper from the results in $NAVMAP_DATA.
Usage: make_figures.py out_dir [figure ...]   (sq1_bars real_building garage_sq5 carving simplification)"""
import glob
import json
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
import shutil  # noqa: E402
import subprocess  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
sys.path.insert(0, os.path.join(HERE, '..', 'real'))
from render import load  # noqa: E402
import eval_real  # noqa: E402

D = os.environ.get('NAVMAP_DATA', os.path.join(HERE, '..', 'data'))
WS = os.environ.get('EASYNAV_WS', os.path.join(HERE, '..', '..'))
MAPS = f'{WS}/src/easynav_playgrounds/playground_summit/easynav_playground_summit_worlds/maps'
MESHSTATS = os.environ.get('MESHSTATS', 'meshstats')


def first(*patterns):
    """First existing file among the glob patterns (results, then previous runs)."""
    for pat in patterns:
        hits = sorted(glob.glob(pat))
        if hits:
            return hits[0]
    return None


def garage_sq5(out):
    """The three-floor garage NavMap of the Summit stack, one panel per floor (NavCels by height,
    ramps appear on both floors they join), the 15 goals of SQ5 and the route the robot drove
    (true pose from Gazebo) in the first run; a side view shows the route climbing."""
    trace = first(f'{D}/sq5/summit_garage_run1.csv', f'{D}/sq5/summit/summit_garage_run1.csv',
                  f'{D}/prev_*/sq5/summit/summit_garage_run1.csv')
    if trace is None:
        print('garage_sq5: no trace, skipped')
        return
    V, T, surf = load(f'{MAPS}/parking_garage.navmap')
    tri = V[T]
    z = tri[:, :, 2].mean(axis=1)
    goals = json.load(open(f'{HERE}/../sq5/goals_garage.json'))['goals']
    tr = np.genfromtxt(trace, delimiter=',', names=True)
    floors = [0.0, 2.35, 4.7]
    fig = plt.figure(figsize=(10, 6.4))  # printed at 17 cm: labels sized for that
    gs = fig.add_gridspec(2, 3, height_ratios=[3.0, 1.1])
    lo, hi = V.min(axis=0), V.max(axis=0)
    norm = matplotlib.colors.Normalize(-0.3, 5.2)
    for k, zf in enumerate(floors):
        ax = fig.add_subplot(gs[0, k])
        sel = np.abs(z - zf) < 1.3  # the floor and the halves of the ramps that reach it
        pc = PolyCollection(tri[sel][:, :, :2], array=z[sel], cmap='viridis', norm=norm,
                            edgecolors='none', rasterized=True)
        ax.add_collection(pc)
        on = np.abs(tr['gt_z'] - zf) < 1.3
        rx, ry = tr['gt_x'].copy(), tr['gt_y'].copy()
        rx[~on] = np.nan  # breaks the line where the route is on another floor
        ax.plot(rx, ry, 'k-', lw=1.3)
        for n, g in enumerate(goals):
            if abs(g['z'] - zf) < 1.0:
                ax.plot(g['x'], g['y'], 'o', color='red', ms=7, mec='white', mew=0.8)
                ax.annotate(str(n + 1), (g['x'], g['y']), textcoords='offset points', xytext=(5, 4),
                            fontsize=11, fontweight='bold', color='darkred',
                            bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.85))
        if abs(tr['gt_z'][0] - zf) < 1.0:
            ax.plot(tr['gt_x'][0], tr['gt_y'][0], 'k*', ms=14, mec='white', mew=0.8)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_aspect('equal')
        ax.set_title(f'Floor {k} (z = {zf:.2f} m)', fontsize=12)
        ax.tick_params(labelsize=10)
        ax.set_xlabel('x (m)', fontsize=11)
        if k == 0:
            ax.set_ylabel('y (m)', fontsize=11)
    ax = fig.add_subplot(gs[1, :])
    side = z < 6.0  # the roof is not navigable and hides nothing here
    pc = PolyCollection(tri[side][:, :, [0, 2]], array=z[side], cmap='viridis', norm=norm,
                        edgecolors='none', alpha=0.35, rasterized=True)
    ax.add_collection(pc)
    ax.plot(tr['gt_x'], tr['gt_z'], 'k-', lw=1.3, label='route driven')
    ax.plot([g['x'] for g in goals], [g['z'] for g in goals], 'o', color='red', ms=7, mec='white',
            mew=0.8, label='goals')
    # The same numbers as the floor panels; goals that coincide in x and floor share one label
    groups = []
    for n, g in enumerate(goals):
        for grp in groups:
            if abs(grp['x'] - g['x']) < 1.0 and abs(grp['z'] - g['z']) < 1.0:
                grp['n'].append(n + 1)
                break
        else:
            groups.append({'x': g['x'], 'z': g['z'], 'n': [n + 1]})
    for grp in groups:
        ax.annotate(', '.join(map(str, grp['n'])), (grp['x'], grp['z']), textcoords='offset points',
                    xytext=(0, 6), ha='center', fontsize=10, fontweight='bold', color='darkred',
                    bbox=dict(boxstyle='round,pad=0.12', fc='white', ec='none', alpha=0.85))
    ax.plot(tr['gt_x'][0], tr['gt_z'][0], 'k*', ms=14, mec='white', mew=0.8, label='start')
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(-0.5, 5.5)
    ax.tick_params(labelsize=10)
    ax.set_xlabel('x (m)', fontsize=11)
    ax.set_ylabel('z (m)', fontsize=11)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.42), fontsize=10, ncol=3, frameon=False)
    fig.tight_layout()
    fig.savefig(f'{out}/garage_sq5.pdf', dpi=200)
    plt.close(fig)


def carving(out):
    """The garage at 0.2 m from the bare points and with the sensor positions, seen from the
    side: the ceilings taken for floor disappear with free-space carving."""
    cloud = f'{D}/clouds/parking_garage_lidar_full_vp.pcd'
    if not os.path.exists(cloud):
        print('carving: no cloud, skipped')
        return
    os.makedirs(f'{D}/sq1', exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(12, 8.5), sharex=True)
    for ax, (tag, title, novp) in zip(axes, [('bare', 'From the bare points', True),
                                             ('vp', 'With the sensor positions (carving, faces seen from below dropped)', False)]):
        path = f'{D}/sq1/fig_garage_{tag}.navmap'
        env = dict(os.environ, NOVP='1') if novp else os.environ
        subprocess.run([MESHSTATS, cloud, '0.2', '1.0', '0.25', '4', path], env=env,
                       stdout=subprocess.DEVNULL, check=True)
        V, T, surf = load(path)
        tri = V[T]
        # Side view (x, z): one segment per NavCel edge would be heavy; the faces as thin polygons
        pc = PolyCollection(tri[:, :, [0, 2]], array=tri[:, :, 1].mean(axis=1), cmap='cividis',
                            edgecolors='none', alpha=0.6, rasterized=True)
        ax.add_collection(pc)
        ax.autoscale()
        ax.set_aspect(2.5)  # z stretched: the sheets are 2.35 m apart over 37 m of garage
        for zf in (0.0, 2.35, 4.7):
            ax.axhline(zf, color='0.6', lw=0.5, ls='--', zorder=0)
        ax.set_ylabel('z (m)')
        ax.set_title(f'{title}: {len(T):,} NavCels, {len(set(surf[surf >= 0]))} surfaces', fontsize=10)
    axes[1].set_xlabel('x (m)')
    fig.colorbar(pc, ax=axes, shrink=0.6, pad=0.02, label='y (m)')
    fig.savefig(f'{out}/garage_carving.pdf', dpi=200)
    plt.close(fig)


def simplification(out):
    """SQ4: a corner of the house NavMap at 5 cm, uniform and simplified keeping the occupancy
    layer: the open floor collapses, the boundaries of the layer keep their resolution."""
    src = f'{D}/sq5/home2_5cm.navmap'
    tool = shutil.which('navmap_simplify') or f'{WS}/install/navmap_tools/lib/navmap_tools/navmap_simplify'
    if not os.path.exists(src) or not os.path.exists(tool):
        print('simplification: no house map or navmap_simplify, skipped')
        return
    os.makedirs(f'{D}/sq4', exist_ok=True)
    simp = f'{D}/sq4/home2_simplified_5cm.navmap'
    subprocess.run([tool, src, simp, '--max-error', '0.05'], stdout=subprocess.DEVNULL, check=True)
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.2))
    win = (-2.0, 2.0, -2.0, 2.0)  # x0, x1, y0, y1 of the window shown
    for ax, (path, title) in zip(axes, [(src, 'Uniform, 5 cm'), (simp, 'Simplified, 5 cm tolerance')]):
        V, T, surf = load(path)
        tri = V[T]
        c = tri[:, :, :2].mean(axis=1)
        inside = (c[:, 0] > win[0]) & (c[:, 0] < win[1]) & (c[:, 1] > win[2]) & (c[:, 1] < win[3])
        pc = PolyCollection(tri[inside][:, :, :2], facecolors='#eef2f7', edgecolors='#355070',
                            linewidths=0.3)
        ax.add_collection(pc)
        ax.set_xlim(win[0], win[1])
        ax.set_ylim(win[2], win[3])
        ax.set_aspect('equal')
        ax.set_title(f'{title}: {len(T):,} NavCels in the map, {inside.sum():,} shown', fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(f'{out}/house_simplified.pdf')
    plt.close(fig)


def real_building(out):
    """The two-floor building: NavMap from the bare scans and with viewpoints, and the route."""
    route, _ = eval_real.route(np.load(f'{D}/real/poses_03.npz')['poses'])
    maps = [('bare', 'NavMap from the bare scans', 'NOVP'),
            ('vp', 'NavMap with the sensor positions', None)]
    # The evaluation overwrites its work file: rebuild both here
    tool = os.environ.get('MESHSTATS', 'meshstats')
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
    for ax, (tag, title, novp) in zip(axes, maps):
        path = f'{D}/real/eval/fig_{tag}.navmap'
        env = 'NOVP=1 ' if novp else ''
        # SQ1's robot; surfaces under 4 m2 (debris) left out of the picture
        os.system(f'{env}SLOPE=30 HOLE=1.0 {tool} {D}/real/two_floors_03_vp.pcd 0.3 1.0 0.25 4 {path} '
                  '> /dev/null')
        V, T, surf = load(path)
        tri = V[T]
        big = np.bincount(surf[surf >= 0]).argsort()[::-1][:6]
        colour = np.full(len(T), 6)
        for k, s in enumerate(big):
            colour[surf == s] = k
        cmap = matplotlib.colors.ListedColormap(
            ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#c8c8c8'])
        pc = PolyCollection(tri[:, :, :2], array=colour, cmap=cmap, edgecolors='none')
        pc.set_clim(-0.5, 6.5)
        ax.add_collection(pc)
        ax.plot(route[:, 0], route[:, 1], 'k-', lw=1.2)
        ax.plot(route[0, 0], route[0, 1], 'k*', ms=10)
        ax.autoscale()
        ax.set_aspect('equal')
        ax.set_title(f'{title} ({len(set(surf[surf >= 0]))} surfaces over 4 m$^2$)', fontsize=10)
        ax.set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(f'{out}/real_building.pdf')
    plt.close(fig)


def sq1_bars(out):
    """Phantom area and false disconnections per representation and scene (mean over seeds)."""
    groups = {}
    for f in glob.glob(f'{D}/sq1/main/*_s*.json'):
        m = re.match(r'(.+)_(plain|vp)_s(\d+)\.json$', os.path.basename(f))
        res = json.load(open(f))['results']
        if m.group(2) == 'vp':
            res = {'navmap+vp': res['navmap']}
        groups.setdefault(m.group(1), {}).setdefault(m.group(3), {}).update(res)
    scenes = [('garage_0.2', 'G 0.2'), ('garage_0.4', 'G 0.4'),
              ('excavation_dense_0.2', 'E 0.2'), ('excavation_dense_0.4', 'E 0.4')]
    reps = [('elev', 'Elevation'), ('mls', 'MLS'), ('vox', 'Voxels'), ('navmap', 'NavMap'),
            ('navmap+vp', 'NavMap+vp')]
    metrics = [('phantom_area_fraction', 'Phantom area (%)'),
               ('false_disconnection', 'False disconnections (%)'), ('false_bridge', 'False bridges (%)')]
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))
    w = 0.16
    for ax, (key, label) in zip(axes, metrics):
        for r, (rep, name) in enumerate(reps):
            vals, errs = [], []
            for sc, _ in scenes:
                v = [100 * run[rep][key] for run in groups[sc].values() if run[rep][key] is not None]
                vals.append(np.mean(v))
                errs.append(np.std(v))
            ax.bar(np.arange(len(scenes)) + (r - 2) * w, vals, w, yerr=errs, label=name)
        ax.set_xticks(np.arange(len(scenes)))
        ax.set_xticklabels([s for _, s in scenes], fontsize=9)
        ax.set_ylabel(label)
        ax.set_yscale('symlog', linthresh=1)
    axes[2].legend(fontsize=8, loc='upper left')
    fig.tight_layout()
    fig.savefig(f'{out}/sq1_bars.pdf')
    plt.close(fig)


if __name__ == '__main__':
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    only = sys.argv[2:]
    for name, fn in [('sq1_bars', sq1_bars), ('real_building', real_building),
                     ('garage_sq5', garage_sq5), ('carving', carving),
                     ('simplification', simplification)]:
        if not only or name in only:
            fn(out)
