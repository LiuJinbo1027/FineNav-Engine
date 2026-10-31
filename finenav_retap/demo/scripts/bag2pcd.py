#!/usr/bin/python3
"""rosbag -> accumulated map-frame PCD + base_link pose track.

Reads /cloud_registered_body (base_link) and /localization_pose (map<-base_link)
straight from the sqlite bag (no playback), rotates each cloud into `map` with
the pose nearest to the cloud stamp, and writes:

  out.pcd    binary x y z, every returned point of every cloud
  traj.txt   one `x y z` per localization pose (map frame)

Usage (after `source /opt/ros/humble/setup.bash`; needs rosbag2_py and
sensor_msgs_py):

  python3 demo/scripts/bag2pcd.py <bag_dir> <out_dir>

The pair it writes feeds the offline path of the resolution/density report:
`retap_map_tool analyze --pcd <out_dir>/bag_accumulated.pcd --trajectory
<out_dir>/traj.txt ...` then `retap_map_tool info --bt ... --trajectory ...`.
"""
import os
import struct
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from sensor_msgs_py import point_cloud2


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    bag_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/localization_pose", "/cloud_registered_body"]))
    types = {"/localization_pose": get_message(
                 "geometry_msgs/msg/PoseWithCovarianceStamped"),
             "/cloud_registered_body": get_message(
                 "sensor_msgs/msg/PointCloud2")}

    poses = []  # (stamp_ns, xyz, quat)
    clouds = []  # (stamp_ns, Nx3 float32)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == "/localization_pose":
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            poses.append((msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                          (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        else:
            pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"),
                                           skip_nans=True)
            xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1)
            clouds.append((msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                           np.asarray(xyz, dtype=np.float32)))
    del reader

    poses.sort(key=lambda item: item[0])
    stamps = np.array([p[0] for p in poses], dtype=np.int64)

    chunks = []
    for stamp, xyz in clouds:
        i = int(np.searchsorted(stamps, stamp))
        i = min(max(i, 0), len(poses) - 1)
        if i > 0 and abs(stamps[i - 1] - stamp) < abs(stamps[i] - stamp):
            i -= 1
        _, t, q = poses[i]
        rot = quat_to_matrix(q)
        world = (xyz.astype(np.float64) @ rot.T) + np.asarray(t)
        chunks.append(world.astype(np.float32))

    points = np.concatenate(chunks, axis=0)
    print("clouds=%d points=%d poses=%d" % (len(clouds), len(points), len(poses)))

    pcd_path = os.path.join(out_dir, "bag_accumulated.pcd")
    with open(pcd_path, "wb") as f:
        f.write(b"# .PCD v0.7 - Point Cloud Data file format\n")
        f.write(b"VERSION 0.7\n")
        f.write(b"FIELDS x y z\n")
        f.write(b"SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(("WIDTH %d\nHEIGHT 1\n" % len(points)).encode())
        f.write(b"VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(("POINTS %d\nDATA binary\n" % len(points)).encode())
        f.write(points.astype("<f4").tobytes())

    traj_path = os.path.join(out_dir, "traj.txt")
    with open(traj_path, "w") as f:
        f.write("# map-frame base_link poses from /localization_pose\n")
        for _, t, _ in poses:
            f.write("%.4f %.4f %.4f\n" % t)
    print(pcd_path, traj_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
