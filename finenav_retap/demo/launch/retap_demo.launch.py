# One-command S4 demo: terrain map + planner + world view + (optional) pose and RViz.
#
#   ros2 launch finenav_retap retap_demo.launch.py
#   ros2 launch finenav_retap retap_demo.launch.py rviz:=false fake_pose:=false
#
# The map file is whatever retap_demo.yaml points at (map_path).  It is an
# absolute path, so the same file works from the source tree and from install/.
#
# Fake pose: phase 1 has no localizer, so the localizer reads an absolute pose
# from /fake_pose.  z=0.3 is the desert surface; the planner still snaps within
# snap_radius, so a small offset is harmless.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FAKE_POSE_MSG = (
    "{header: {frame_id: map}, "
    "pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.3}, "
    "orientation: {w: 1.0}}}}"
)


def generate_launch_description():
    share = get_package_share_directory("finenav_retap")
    default_params = os.path.join(share, "demo", "params", "retap_demo.yaml")
    default_rviz = os.path.join(share, "demo", "rviz", "retap_s4.rviz")

    params_file = LaunchConfiguration("params_file")
    fake_pose = LaunchConfiguration("fake_pose")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Parameter file for retap_demo (map_path lives here)."),
        DeclareLaunchArgument(
            "fake_pose", default_value="true",
            description="Publish a constant pose on /fake_pose (phase 1 localizer)."),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start RViz2 with demo/rviz/retap_s4.rviz."),

        ExecuteProcess(
            cmd=["ros2", "topic", "pub", "-r", "10", "/fake_pose",
                 "geometry_msgs/msg/PoseWithCovarianceStamped", FAKE_POSE_MSG],
            output="log",
            condition=IfCondition(fake_pose),
        ),

        # No `name=` on purpose: launch_ros would pass a process-wide
        # `__node:=<name>` remap, which renames EVERY node the process creates —
        # including the MapServer/ControlLayer private nodes
        # (<engine>/retap_terrain_map, <engine>/retap_layer).  The parameter file
        # keys are their fully-qualified names, so renaming them silently drops
        # the whole YAML.  Let the executable name its own nodes.
        Node(
            package="finenav_retap",
            executable="retap_demo",
            output="screen",
            parameters=[params_file],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", default_rviz],
            output="log",
            condition=IfCondition(rviz),
        ),
    ])
