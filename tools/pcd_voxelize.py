#!/usr/bin/env python3
"""One point per occupied voxel of a float PCD (x y z first fields) (voxel centre, or the points' centroid with
'centroid'); writes a binary PCD. Usage: pcd_voxelize.py in.pcd out.pcd res [centroid]"""
import sys

import numpy as np


def read_pcd_xyz(path):
    data = open(path, 'rb').read()
    header_end = data.index(b'DATA')
    header = data[:header_end].decode()
    line_end = data.index(b'\n', header_end) + 1
    fmt = data[header_end:line_end].decode().split()[1]
    n = int(next(l for l in header.splitlines() if l.startswith('POINTS')).split()[1])
    nf = len(next(l for l in header.splitlines() if l.startswith('FIELDS')).split()) - 1  # x y z first
    if fmt == 'binary':
        return np.frombuffer(data[line_end:line_end + 4 * nf * n], dtype='<f4').reshape(-1, nf)[:, :3]
    return np.loadtxt(data[line_end:].decode().splitlines(), dtype=np.float32).reshape(-1, nf)[:, :3]


def main():
    src, dst, res = sys.argv[1], sys.argv[2], float(sys.argv[3])
    centroid = len(sys.argv) > 4 and sys.argv[4] == 'centroid'
    pts = read_pcd_xyz(src)
    keys = np.floor(pts / res).astype(np.int64)
    vox, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    if centroid:
        sums = np.zeros((len(vox), 3))
        np.add.at(sums, inverse.ravel(), pts.astype(np.float64))
        centres = (sums / counts[:, None]).astype('<f4')
    else:
        centres = ((vox + 0.5) * res).astype('<f4')
    with open(dst, 'wb') as f:
        f.write(('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n'
                 'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                 f'WIDTH {len(centres)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                 f'POINTS {len(centres)}\nDATA binary\n').encode())
        f.write(centres.tobytes())
    print(f'{len(pts)} points -> {len(centres)} voxels of {res} m')


if __name__ == '__main__':
    main()
