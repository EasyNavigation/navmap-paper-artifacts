#!/usr/bin/env python3
"""Sample a binary STL (Gazebo collision mesh) into an ASCII PCD, in the playground map frame."""
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np


def load_stl(path):
    b = open(path, 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    dt = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    return np.frombuffer(b, dtype=dt, count=n, offset=84)['v'].reshape(-1, 3, 3).astype(np.float64)


def load_dae(path):
    """Triangles of a COLLADA file in model coordinates (node matrices and unit applied)."""
    root = ET.parse(path).getroot()
    ns = {'c': root.tag.split('}')[0].strip('{')}
    unit = root.find('c:asset/c:unit', ns)
    scale = float(unit.get('meter', '1')) if unit is not None else 1.0
    up = root.find('c:asset/c:up_axis', ns)
    up = up.text.strip() if up is not None else 'Y_UP'

    geoms = {}
    for g in root.findall('.//c:library_geometries/c:geometry', ns):
        mesh = g.find('c:mesh', ns)
        if mesh is None:
            continue
        sources = {s.get('id'): s for s in mesh.findall('c:source', ns)}
        vin = mesh.find('c:vertices/c:input[@semantic="POSITION"]', ns)
        src = sources[vin.get('source').lstrip('#')]
        pos = np.array(src.find('c:float_array', ns).text.split(), dtype=np.float64).reshape(-1, 3)
        tris = []
        for prim in list(mesh):
            tag = prim.tag.split('}')[1]
            if tag not in ('triangles', 'polylist', 'polygons'):
                continue
            inputs = prim.findall('c:input', ns)
            stride = max(int(i.get('offset')) for i in inputs) + 1
            voff = next(int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX')
            if tag == 'polygons':
                polys = [np.array(p.text.split(), dtype=np.int64) for p in prim.findall('c:p', ns)]
            else:
                idx = np.array(prim.find('c:p', ns).text.split(), dtype=np.int64)
                if tag == 'triangles':
                    counts = np.full(len(idx) // (3 * stride), 3)
                else:
                    counts = np.array(prim.find('c:vcount', ns).text.split(), dtype=np.int64)
                polys, k = [], 0
                for c in counts:
                    polys.append(idx[k:k + c * stride])
                    k += c * stride
            for poly in polys:
                v = poly.reshape(-1, stride)[:, voff]
                for t in range(1, len(v) - 1):  # fan triangulation
                    tris.append(pos[[v[0], v[t], v[t + 1]]])
        geoms['#' + g.get('id')] = np.array(tris).reshape(-1, 3, 3)

    out = []

    def walk(node, M):
        m = node.find('c:matrix', ns)
        L = np.eye(4)
        if m is not None:
            L = np.array(m.text.split(), dtype=np.float64).reshape(4, 4)
        for t in node.findall('c:translate', ns):
            T = np.eye(4)
            T[:3, 3] = np.array(t.text.split(), dtype=np.float64)
            L = L @ T
        for sc in node.findall('c:scale', ns):
            L = L @ np.diag(list(np.array(sc.text.split(), dtype=np.float64)) + [1.0])
        M = M @ L
        for ig in node.findall('c:instance_geometry', ns):
            tri = geoms[ig.get('url')]
            h = np.concatenate([tri.reshape(-1, 3), np.ones((tri.size // 3, 1))], axis=1)
            out.append((h @ M.T)[:, :3].reshape(-1, 3, 3))
        for child in node.findall('c:node', ns):
            walk(child, M)

    for vs in root.findall('.//c:library_visual_scenes/c:visual_scene', ns):
        for node in vs.findall('c:node', ns):
            walk(node, np.eye(4))
    tris = np.concatenate(out) * scale
    if up == 'Y_UP':  # COLLADA Y-up to Z-up, as gz does
        tris = tris[:, :, [0, 2, 1]] * np.array([1.0, -1.0, 1.0])
    return tris


def terrain_z(tris, x, y, below=np.inf):
    """Highest mesh height under (x, y), not above `below` (the surface a robot lands on)."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    ok = np.abs(d) > 1e-12
    dd = np.where(ok, d, 1.0)
    l1 = ((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1])) / dd
    l2 = ((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1])) / dd
    l3 = 1.0 - l1 - l2
    m = ok & (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
    z = (l1 * a[:, 2] + l2 * b[:, 2] + l3 * c[:, 2])[m]
    return z[z <= below].max()


def main():
    stl, out, density = sys.argv[1], sys.argv[2], float(sys.argv[3])
    spawn_x, spawn_y = 0.0, 0.0
    spawn_z = float(sys.argv[4]) if len(sys.argv) > 4 else np.inf
    tris = load_dae(stl) if stl.lower().endswith('.dae') else load_stl(stl)
    # Map frame: z = 0 at the terrain under the spawn point.
    z_offset = -terrain_z(tris, spawn_x, spawn_y, spawn_z)

    rng = np.random.default_rng(0)
    e1, e2 = tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1)
    counts = rng.poisson(area * density)
    idx = np.repeat(np.arange(len(tris)), counts)
    u, v = rng.random(len(idx)), rng.random(len(idx))
    flip = u + v > 1.0
    u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
    pts = tris[idx, 0] + u[:, None] * e1[idx] + v[:, None] * e2[idx]
    pts[:, 2] += z_offset

    with open(out, 'wb') as f:
        f.write(('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n'
                 'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                 f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\n'
                 'DATA binary\n').encode())
        f.write(pts.astype('<f4').tobytes())
    print(f'{len(tris)} triangles, {area.sum():.0f} m2 -> {len(pts)} points, z_offset {z_offset:.3f}')


if __name__ == '__main__':
    main()
