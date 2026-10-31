# Replay a FAST-LIO rosbag through the online terrain pipeline.
#
#   ros2 launch finenav_retap retap_rosbag.launch.py
#   ros2 launch finenav_retap retap_rosbag.launch.py config:=terrain_res020_ring09.yaml
#   ros2 launch finenav_retap retap_rosbag.launch.py bag_path:=/path/to/bag rviz:=false
#   ros2 launch finenav_retap retap_rosbag.launch.py bag_rate:=0.3   # slow motion
#
# The bag is resolved as: bag_path:= , else $RETAP_BAG, else the single
# rosbag2_* directory in the current working directory.  Nothing here is tied
# to one machine: launch from the workspace root and it finds the bag; launch
# from anywhere else and pass bag_path:= (or export RETAP_BAG=...).
#
# Watching the incremental update: RViz opens demo/rviz/retap_rosbag.rviz,
# which carries the three accumulated clouds (traversability / collision /
# drivable) plus the raw scan.  They are rebuilt at `diagnostics_period_sec`
# (1 s), so at bag_rate 1.0 the map grows in ~10-scan steps; lower the playback
# rate (bag_rate:=0.3) if you want to follow it scan by scan.
#
# Terrain parameter sets live in <share>/config/ (see config/README.md).
# `config:=<file>` supplies the calibration (resolution, thresholds, weights,
# collision_neighbor); demo/params/retap_rosbag.yaml supplies only what the
# robot and this recording are.  merge_params() writes one file holding both,
# and the same config file also feeds the offline tool verbatim:
#
#   ros2 run finenav_retap retap_map_tool analyze --pcd m.pcd \
#       --config <share>/config/<file>.yaml --output-dir /tmp/out
#
# Wiring (see demo/params/retap_rosbag.yaml for the why):
#
#   ros2 bag play  /localization_pose, /cloud_registered_body, /tf, /tf_static
#        │                    │
#        │                    └──────────► retap_demo: RetapTerrainMap
#        │                                    (online_mapping, no prior; each
#        │                                     body-frame cloud is rotated into
#        │                                     `map` with the pose at its stamp)
#        └──────────────────────────────► finenav_localizer (pose_topic)
#
# No TF lookup is needed anywhere on the data path: the localizer reads the
# absolute pose from /localization_pose and the map plugin transforms scans
# with the pose MapRuntime interpolates at each cloud stamp.  This matters for
# replay — a stamped tf2 lookup would fail with "extrapolation into the future"
# because the transform carrying the cloud's stamp is played after the cloud
# (measured: ~75% of clouds dropped).  /tf is still replayed so RViz and
# `tf2_echo` show the recorded map -> odom -> base_link chain.
#
# The bag carries no /clock, so `ros2 bag play --clock` generates it from the
# recorded stamps; every node here runs with use_sim_time=true so the map's
# batch timer and the localizer's 100 Hz tick advance with the bag.

import glob
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# retap_params.py sits next to this file in both the source tree and the
# install space (demo/CMakeLists.txt installs the whole launch directory).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from retap_params import merge_params, resolve_config, summary  # noqa: E402

# Only these are replayed.  /Odometry, /cloud_registered, /aligned_cloud,
# /map and /global_map_viz are recorded in the bag but unused: the pose comes
# from /localization_pose, and the scan is re-expressed in `map` by the
# converter instead of relying on the odom-frame copy.
BAG_TOPICS = ["/tf", "/tf_static", "/localization_pose", "/cloud_registered_body"]


def _default_bag():
    """$RETAP_BAG, else the only rosbag2_* directory in the working directory.

    Empty means "not found"; _start_demo() then fails with an actionable
    message instead of letting `ros2 bag play` die in a subprocess.
    """
    env = os.environ.get("RETAP_BAG")
    if env:
        return env
    hits = sorted(glob.glob(os.path.join(os.getcwd(), "rosbag2_*")))
    return hits[0] if len(hits) == 1 else ""


DEFAULT_BAG = _default_bag()


def _start_demo(context, params_file, config_name, bag_path, play_bag, *args,
                **kwargs):
    # OpaqueFunction runs after argument resolution, so the LaunchConfigurations
    # must be substituted here rather than at description-build time.
    params_file = context.perform_substitution(params_file)
    config_name = context.perform_substitution(config_name)
    share = get_package_share_directory("finenav_retap")

    # Fail here, with the actual fix, rather than inside `ros2 bag play`.
    if context.perform_substitution(play_bag) == "true":
        bag = context.perform_substitution(bag_path)
        if not bag:
            raise RuntimeError(
                "[retap] no rosbag found in {} and $RETAP_BAG is unset.\n"
                "        pass bag_path:=/path/to/rosbag2_... (or export "
                "RETAP_BAG), or set play_bag:=false to publish the topics "
                "yourself.".format(os.getcwd()))
        if not os.path.isdir(bag):
            raise RuntimeError(
                "[retap] bag_path:= '{}' is not a directory.".format(bag))

    config_path = resolve_config(share, config_name)
    merged_path, terrain = merge_params(params_file, config_path)

    demo = Node(
        package="finenav_retap",
        executable="retap_demo",
        output="screen",
        parameters=[merged_path],
    )
    return [LogInfo(msg=summary(config_path, terrain, merged_path)), demo]


def generate_launch_description():
    share = get_package_share_directory("finenav_retap")

    default_params = os.path.join(share, "demo", "params", "retap_rosbag.yaml")
    default_rviz = os.path.join(share, "demo", "rviz", "retap_rosbag.rviz")

    params_file = LaunchConfiguration("params_file")
    config = LaunchConfiguration("config")
    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    bag_delay = LaunchConfiguration("bag_delay")
    play_bag = LaunchConfiguration("play_bag")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # No `name=` on purpose: launch_ros would pass a process-wide
    # `__node:=<name>` remap and rename every node the process creates,
    # including the MapServer/ControlLayer private nodes whose fully-qualified
    # names are the keys of the parameter file.  The node is constructed inside
    # an OpaqueFunction so the config merge happens after argument resolution.
    demo = OpaqueFunction(function=_start_demo,
                          args=[params_file, config, bag_path, play_bag])

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path, "--clock", "100",
             "--rate", bag_rate, "--topics"] + BAG_TOPICS,
        output="screen",
        condition=IfCondition(play_bag),
    )

    delayed_bag = TimerAction(period=bag_delay, actions=[bag_play])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="log",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value="terrain_res020_ring17.yaml",
            description="Terrain calibration from <share>/config/ (bare name or "
                        "absolute path). Supplies resolution, thresholds, "
                        "weights and collision_neighbor; the scenario parameter "
                        "file does not carry them."),
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Scenario parameter file for retap_demo (data source, "
                        "online policy, planner, localizer). No calibration."),
        DeclareLaunchArgument(
            "bag_path", default_value=DEFAULT_BAG,
            description="rosbag2 directory to replay. Default: $RETAP_BAG, else "
                        "the only rosbag2_* directory in the working directory, "
                        "else empty (then bag_path:= is required)."),
        DeclareLaunchArgument(
            "bag_rate", default_value="1.0",
            description="Playback speed multiplier."),
        DeclareLaunchArgument(
            "bag_delay", default_value="3.0",
            description="Seconds to let the nodes come up before playback starts."),
        DeclareLaunchArgument(
            "play_bag", default_value="true",
            description="Start `ros2 bag play` (false: drive the topics yourself)."),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start RViz2 (retap_rosbag.rviz: the scan + the three "
                        "terrain clouds)."),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz,
            description="RViz2 config file (retap_s5.rviz also works)."),
        demo,
        delayed_bag,
        rviz_node,
    ])
