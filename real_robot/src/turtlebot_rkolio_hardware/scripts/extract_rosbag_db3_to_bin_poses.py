#!/usr/bin/env python3

import argparse
import bisect
import os
import sqlite3

import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import sensor_msgs_py.point_cloud2 as pc2


def find_db3_file(bag_path):
    for filename in sorted(os.listdir(bag_path)):
        if filename.endswith('.db3'):
            return os.path.join(bag_path, filename)
    raise RuntimeError(f'No .db3 file found in bag directory: {bag_path}')


def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3)

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def odom_to_pose_matrix(odom_msg):
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation

    rotation = quaternion_to_rotation_matrix(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    translation = np.array([[position.x], [position.y], [position.z]], dtype=np.float64)
    return np.hstack((rotation, translation))


def is_valid_pose(transform):
    if not np.all(np.isfinite(transform)):
        return False

    translation = transform[:, 3]
    if np.linalg.norm(translation) > 100000:
        return False

    rotation = transform[:, :3]
    det = np.linalg.det(rotation)
    if not np.isfinite(det):
        return False

    return abs(det - 1.0) <= 0.2


def pointcloud2_to_xyzi(cloud_msg):
    field_names = [field.name for field in cloud_msg.fields]
    has_intensity = 'intensity' in field_names
    read_fields = ['x', 'y', 'z', 'intensity'] if has_intensity else ['x', 'y', 'z']

    points = []
    for point in pc2.read_points(cloud_msg, field_names=read_fields, skip_nans=True):
        x = float(point[0])
        y = float(point[1])
        z = float(point[2])
        intensity = float(point[3]) if has_intensity else 0.0

        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            points.append([x, y, z, intensity])

    if not points:
        return np.empty((0, 4), dtype=np.float32)

    return np.asarray(points, dtype=np.float32)


def load_topics(cursor):
    cursor.execute('SELECT id, name, type FROM topics')
    rows = cursor.fetchall()
    return {
        name: {
            'id': topic_id,
            'type': msg_type,
        }
        for topic_id, name, msg_type in rows
    }


def read_odometry(cursor, odom_topic, odom_topic_id, odom_msg_type):
    cursor.execute(
        'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
        (odom_topic_id,),
    )

    odom_class = get_message(odom_msg_type)
    odom_times = []
    odom_poses = []
    total = 0
    valid = 0

    for timestamp, data in cursor.fetchall():
        total += 1
        try:
            msg = deserialize_message(data, odom_class)
            transform = odom_to_pose_matrix(msg)
            if is_valid_pose(transform):
                odom_times.append(timestamp)
                odom_poses.append(transform)
                valid += 1
        except Exception as exc:
            print(f'[WARN] Failed to deserialize odom message: {exc}')

    print(f'[INFO] Odometry topic: {odom_topic}')
    print(f'[INFO] Odometry messages: {total}')
    print(f'[INFO] Valid poses: {valid}')

    if not odom_times:
        raise RuntimeError('No valid odometry poses found.')

    return odom_times, odom_poses


def find_nearest_pose(timestamp, odom_times, odom_poses, max_diff_ns):
    index = bisect.bisect_left(odom_times, timestamp)
    candidates = []

    if index < len(odom_times):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None, None

    best_index = min(candidates, key=lambda value: abs(odom_times[value] - timestamp))
    delta_t = abs(odom_times[best_index] - timestamp)
    if delta_t > max_diff_ns:
        return None, delta_t

    return odom_poses[best_index], delta_t


def extract_lidar(cursor, lidar_topic, lidar_topic_id, lidar_msg_type,
                  odom_times, odom_poses, out_dir, max_diff_sec):
    os.makedirs(out_dir, exist_ok=True)
    bin_dir = os.path.join(out_dir, 'bin')
    os.makedirs(bin_dir, exist_ok=True)

    poses_path = os.path.join(out_dir, 'poses.txt')
    timestamps_path = os.path.join(out_dir, 'timestamps.txt')

    lidar_class = get_message(lidar_msg_type)
    max_diff_ns = int(max_diff_sec * 1e9)

    cursor.execute(
        'SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp',
        (lidar_topic_id,),
    )
    rows = cursor.fetchall()

    saved = 0
    skipped_empty = 0
    skipped_no_pose = 0

    with open(poses_path, 'w', encoding='utf-8') as poses_file, \
            open(timestamps_path, 'w', encoding='utf-8') as timestamps_file:
        for timestamp, data in rows:
            try:
                cloud_msg = deserialize_message(data, lidar_class)
            except Exception as exc:
                print(f'[WARN] Failed to deserialize cloud: {exc}')
                continue

            points = pointcloud2_to_xyzi(cloud_msg)
            if points.shape[0] == 0:
                skipped_empty += 1
                continue

            transform, delta_t = find_nearest_pose(
                timestamp,
                odom_times,
                odom_poses,
                max_diff_ns,
            )
            if transform is None:
                skipped_no_pose += 1
                continue

            bin_path = os.path.join(bin_dir, f'{saved:06d}.bin')
            points.tofile(bin_path)

            poses_file.write(' '.join(f'{value:.9e}' for value in transform.reshape(-1)) + '\n')
            timestamps_file.write(f'{timestamp}\n')

            if saved % 20 == 0:
                print(
                    f'[INFO] Saved {saved:06d}.bin | '
                    f'points={points.shape[0]} | '
                    f'pose_dt={delta_t / 1e6:.2f} ms'
                )

            saved += 1

    print('\n[DONE]')
    print(f'[INFO] LiDAR topic: {lidar_topic}')
    print(f'[INFO] Total LiDAR messages: {len(rows)}')
    print(f'[INFO] Saved frames: {saved}')
    print(f'[INFO] Skipped empty clouds: {skipped_empty}')
    print(f'[INFO] Skipped clouds without close pose: {skipped_no_pose}')
    print(f'[INFO] Output folder: {out_dir}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path')
    parser.add_argument('out_dir')
    parser.add_argument('--lidar_topic', default='/velodyne_points')
    parser.add_argument('--odom_topic', default='/rko_lio/odometry')
    parser.add_argument('--max_time_diff', type=float, default=0.10)
    args = parser.parse_args()

    db3_path = find_db3_file(args.bag_path)
    print(f'[INFO] Using db3 file: {db3_path}')

    conn = sqlite3.connect(db3_path)
    cursor = conn.cursor()
    topic_info = load_topics(cursor)

    print('\n[INFO] Topics in bag:')
    for name, info in topic_info.items():
        print(f"  id={info['id']} | {name} | {info['type']}")

    if args.lidar_topic not in topic_info:
        raise RuntimeError(f'LiDAR topic not found: {args.lidar_topic}')
    if args.odom_topic not in topic_info:
        raise RuntimeError(f'Odometry topic not found: {args.odom_topic}')

    odom_info = topic_info[args.odom_topic]
    lidar_info = topic_info[args.lidar_topic]

    odom_times, odom_poses = read_odometry(
        cursor,
        args.odom_topic,
        odom_info['id'],
        odom_info['type'],
    )

    extract_lidar(
        cursor,
        args.lidar_topic,
        lidar_info['id'],
        lidar_info['type'],
        odom_times,
        odom_poses,
        args.out_dir,
        args.max_time_diff,
    )

    conn.close()


if __name__ == '__main__':
    main()