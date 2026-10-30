#!/usr/bin/env python3
"""Top-down render of .navmap files: elevation and surfaces, plus overlap stats."""
import struct
import zlib
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
import numpy as np  # noqa: E402
from navmap_ros_interfaces.msg import NavMap  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402


def load(path):
    b = open(path, 'rb').read()
    if b[:8] == b'NAVMAP\0\0':
        (version,) = struct.unpack('<I', b[8:12])
        if version >= 2:  # codec, raw and stored sizes; zlib-compressed payload
            codec, _, size = struct.unpack('<IQQ', b[12:32])
            b = b[32:32 + size]
            if codec == 1:
                b = zlib.decompress(b)
        else:
            (size,) = struct.unpack('<Q', b[12:20])
            b = b[20:20 + size]
    m = deserialize_message(b, NavMap)
    P = np.stack([m.positions_x, m.positions_y, m.positions_z], axis=1)
    T = np.stack([m.navcels_v0, m.navcels_v1, m.navcels_v2], axis=1).astype(int)
    surf = np.full(len(T), -1)
    for k, s in enumerate(m.surfaces):
        surf[np.asarray(s.navcels, dtype=int)] = k
    return P, T, surf


def main():
    files, out = sys.argv[1:-1], sys.argv[-1]
    fig, axes = plt.subplots(len(files), 2, figsize=(16, 6.5 * len(files)))
    axes = np.atleast_2d(axes)
    for row, f in enumerate(files):
        P, T, surf = load(f)
        tri = P[T]
        zc = tri[:, :, 2].mean(axis=1)
        area_xy = 0.5 * np.abs(np.cross(tri[:, 1, :2] - tri[:, 0, :2], tri[:, 2, :2] - tri[:, 0, :2]))
        title = (f'{f.split("/")[-1]}: {len(T)} tris, {len(set(surf[surf >= 0]))} surfaces, '
                 f'{(surf < 0).sum()} tris without surface, XY area {area_xy.sum():.0f} m2')
        order = np.argsort(zc)  # higher cells drawn last
        pc = PolyCollection(tri[order, :, :2], array=zc[order], cmap='terrain', edgecolors='none')
        ax = axes[row, 0]
        ax.add_collection(pc)
        ax.autoscale()
        ax.set_aspect('equal')
        ax.plot(0, 0, 'r*', ms=14)
        fig.colorbar(pc, ax=ax, label='z (m, map frame)')
        ax.set_title(title, fontsize=9)

        main_s = np.bincount(surf[surf >= 0]).argmax() if (surf >= 0).any() else -1
        colors = np.where(surf == main_s, 0, np.where(surf < 0, 2, 1))
        pc2 = PolyCollection(tri[:, :, :2], array=colors, cmap=matplotlib.colors.ListedColormap(
            ['#4c9a2a', '#e08a00', '#d62728']), edgecolors='none', alpha=0.6)
        pc2.set_clim(0, 2)
        ax = axes[row, 1]
        ax.add_collection(pc2)
        ax.autoscale()
        ax.set_aspect('equal')
        ax.plot(0, 0, 'k*', ms=14)
        ax.set_title('green: main surface, orange: other surfaces, red: no surface '
                     '(alpha: darker = stacked triangles)', fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    print('wrote', out)


if __name__ == '__main__':
    main()
