"""Plot one representation's standing elements per level band, coloured by component."""
import sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import eval_sq1 as E
env, cloud, res, name, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4], sys.argv[5]
P = E.read_pcd(cloud)
rep = {'mls': lambda: E.MLSRep(P, res), 'vox': lambda: E.VoxelRep(P, res), 'elev': lambda: E.ElevationRep(P, res),
       'navmap': lambda: E.NavMapRep(cloud, res, env)}[name]()
C = rep.centres
big = np.unique(rep.comp, return_counts=True)
order = big[0][np.argsort(-big[1])]
rank = {c: i for i, c in enumerate(order)}
col = np.array([rank[c] if rank[c] < 9 else 9 for c in rep.comp])
bands = [(-0.5, 1.5, 'z 0-1.5'), (1.5, 4.5, 'z 1.5-4.5'), (4.5, 7.5, 'z 4.5-7.5')]
fig, axs = plt.subplots(1, 3, figsize=(24, 6))
for ax, (lo, hi, t) in zip(axs, bands):
    m = (C[:, 2] >= lo) & (C[:, 2] < hi)
    ax.scatter(C[m, 0], C[m, 1], c=col[m], cmap='tab10', vmin=0, vmax=9, s=4, marker='s')
    ax.set_aspect('equal'); ax.set_title(f'{name} {t} (color = componente, 9 mayores)')
fig.tight_layout(); fig.savefig(out, dpi=70)
