#!/usr/bin/env python3
"""Lidar scans and wheel odometry of a mapping rosbag, for scan-to-map registration.

Each scan goes to base_footprint (static TF), voxel-downsampled; the odometry pose is interpolated
at the scan stamp.

Usage: bag_scans.py bag out.npz [--voxel 0.1] [--max N] [--every K] [--min-range 1.0] [--max-range 60]
"""
import argparse

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

LIDAR = '/front_laser/points'
ODOM = '/robotnik_base_control/odom'


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_to_mat(t):
    T = np.eye(4)
    r, p = t.transform.rotation, t.transform.translation
    T[:3, :3] = quat_to_mat((r.x, r.y, r.z, r.w))
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def chain(static, parent, child):
    """parent→child from the static tree (child's parents walked up to parent)."""
    T = np.eye(4)
    f = child
    while f != parent:
        p, M = static[f]
        T = M @ T
        f = p
    return T


def cloud_xyz(m):
    off = {f.name: f.offset for f in m.fields}
    raw = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
    P = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz'], 1)
    return P[np.isfinite(P).all(1)].astype(np.float64)


def voxel_down(P, v):
    k = np.floor(P / v).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return P[np.sort(i)]


def yaw_of(q):
    x, y, z, w = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--voxel', type=float, default=0.1)
    ap.add_argument('--max', type=int, default=0)
    ap.add_argument('--every', type=int, default=1)
    ap.add_argument('--min-range', type=float, default=1.0)
    ap.add_argument('--max-range', type=float, default=60.0)
    a = ap.parse_args()
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[LIDAR, ODOM, '/tf_static']))
    static, odom, scans, stamps, k = {}, [], [], [], 0
    laser = None
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/tf_static':
            for t in m.transforms:
                static[t.child_frame_id] = (t.header.frame_id, tf_to_mat(t))
        elif topic == ODOM:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            odom.append((m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, p.x, p.y, p.z,
                         q.x, q.y, q.z, q.w))
        else:
            k += 1
            if (k - 1) % a.every:
                continue
            if laser is None:
                laser = chain(static, 'base_footprint', m.header.frame_id)
            P = cloud_xyz(m)
            d = np.linalg.norm(P, axis=1)
            P = P[(d > a.min_range) & (d < a.max_range)]
            P = P @ laser[:3, :3].T + laser[:3, 3]
            scans.append(voxel_down(P, a.voxel).astype(np.float32))
            stamps.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            if a.max and len(scans) >= a.max:
                break
    O = np.array(odom)
    t = np.array(stamps)
    # Odometry at the scan stamps: position interpolated, yaw unwrapped and interpolated (2D odom)
    yaw = np.unwrap([yaw_of(q) for q in O[:, 4:8]])
    pose = np.stack([np.interp(t, O[:, 0], O[:, 1]), np.interp(t, O[:, 0], O[:, 2]),
                     np.interp(t, O[:, 0], O[:, 3]), np.interp(t, O[:, 0], yaw)], 1)
    sizes = np.array([len(s) for s in scans])
    np.savez_compressed(a.out, stamps=t, odom=pose, sizes=sizes, points=np.concatenate(scans),
                        laser=laser)
    print(f'{len(scans)} scans, {sizes.mean():.0f} points each, {t[-1] - t[0]:.0f} s, '
          f'odom path {np.linalg.norm(np.diff(pose[:, :2], axis=0), axis=1).sum():.0f} m')


if __name__ == '__main__':
    main()
