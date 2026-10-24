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
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FAKE_POSE_MSG = (
    "{header: {frame_id: map}, "
    "pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.3}, "
    "orientation: {w: 1.0}}}}"
)

# Where the parameter file keeps the terrain map path.
MAP_PARAM_PATH = ("/retap_demo/retap_terrain_map", "ros__parameters",
                  "retap_terrain_map", "map_path")


def _ensure_prior_map(context, params_file, prior_pcd, terrain_config):
    """Rebuild the terrain .bt when the parameter file points at a missing one.

    The demo's map_path is the S1 output under /tmp, which does not survive a
    reboot.  A missing file only surfaces as an ERROR line followed by "no
    drivable voxel", so build it again here instead.
    """
    params_path = params_file.perform(context)
    try:
        with open(params_path, "r", encoding="utf-8") as handle:
            entry = yaml.safe_load(handle) or {}
        for key in MAP_PARAM_PATH:
            entry = entry[key]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f"[retap_demo] cannot read map_path from '{params_path}': {exc}")
        return []

    if not entry or os.path.exists(entry):
        return []

    out_dir = os.path.dirname(entry)
    os.makedirs(out_dir, exist_ok=True)
    cmd = [
        "ros2", "run", "finenav_retap", "retap_map_tool", "analyze",
        "--pcd", prior_pcd.perform(context),
        "--resolution", "0.2",
        "--config", terrain_config.perform(context),
        "--output-dir", out_dir,
    ]
    print(f"[retap_demo] map '{entry}' missing; generating: {' '.join(cmd)}")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        print(f"[retap_demo] map generation failed (exit {result.returncode}).")
    return []


def generate_launch_description():
    share = get_package_share_directory("finenav_retap")
    sim_share = get_package_share_directory("finenav_simulator")
    default_params = os.path.join(share, "demo", "params", "retap_demo.yaml")
    default_rviz = os.path.join(share, "demo", "rviz", "retap_s4.rviz")
    default_prior_pcd = os.path.join(sim_share, "resource", "maps", "desert.pcd")
    default_terrain_config = os.path.join(share, "config", "terrain_sim.yaml")

    params_file = LaunchConfiguration("params_file")
    prior_pcd = LaunchConfiguration("prior_pcd")
    terrain_config = LaunchConfiguration("terrain_config")
    fake_pose = LaunchConfiguration("fake_pose")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Parameter file for retap_demo (map_path lives here)."),
        DeclareLaunchArgument(
            "prior_pcd", default_value=default_prior_pcd,
            description="Point cloud the map is rebuilt from when map_path is missing."),
        DeclareLaunchArgument(
            "terrain_config", default_value=default_terrain_config,
            description="Terrain thresholds used for that rebuild."),
        DeclareLaunchArgument(
            "fake_pose", default_value="true",
            description="Publish a constant pose on /fake_pose (phase 1 localizer)."),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start RViz2 with demo/rviz/retap_s4.rviz."),

        OpaqueFunction(
            function=_ensure_prior_map,
            args=[params_file, prior_pcd, terrain_config]),

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
