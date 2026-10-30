#!/usr/bin/env python3
"""Export a mesh (.dae or .stl) to a binary STL, applying the same loaders as mesh_to_pcd."""
import struct
import sys

import os

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mesh_to_pcd import load_dae, load_stl  # noqa: E402


def main():
    src, dst = sys.argv[1], sys.argv[2]
    tris = load_dae(src) if src.lower().endswith('.dae') else load_stl(src)
    with open(dst, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tris)))
        for t in tris.astype('<f4'):
            f.write(np.zeros(3, '<f4').tobytes() + t.tobytes() + b'\0\0')
    print(len(tris), 'triangles ->', dst)


if __name__ == '__main__':
    main()
