# S5 demo: Gazebo + online incremental terrain mapping + periodic replanning.
#
#   ros2 launch finenav_retap retap_s5_sim.launch.py
#   ros2 launch finenav_retap retap_s5_sim.launch.py rviz:=false world_name:=map_hill.world
#
# What this wires together (see demo/params/retap_demo_s5.yaml for the why):
#
#   Gazebo (map_desert, lio_type=gazebo_odom → /odom + identity map->odom)
#     └─ /livox/lidar ──► point_cloud_converter_node ──► /cloud_registered (map)
#                             │
#   retap_demo ───────────────┘  RetapTerrainMap (online_mapping) builds the
#                                terrain tree as the robot drives; the planner
#                                runs under PeriodicPolicy and replans when the
#                                map generation, the robot's position or the
#                                2 s period say so.
#
# Why the extra converter: the simulator's own converter feeds FAST-LIO
# (/cloud_for_lio) and, in gazebo_odom mode, outputs in base_link.  ReTAP's
# online path wants a map-frame cloud (scan_in_map_frame), so a second instance
# transforms /livox/lidar into `map` with tf2 at the message stamp.
#
# Gazebo starts paused: launch/core/gazebo.launch.py passes no -r, so /clock
# does not advance until the world is unpaused.  The TimerAction below runs
# `ign service ... pause: false` once the world and the robot exist.
#
# Prior map: the demo plans on a static terrain .bt that is exported from the
# SAME Gazebo model the lidar scans (retap_ros2/scripts/export_gazebo_pcd.py),
# then merged with the live scans online.  _ensure_prior_map() rebuilds the
# .pcd and the .bt when the parameter file points at a missing one.

import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where the parameter file keeps the terrain map path.
MAP_PARAM_PATH = ("/retap_demo/retap_terrain_map", "ros__parameters",
                  "retap_terrain_map", "map_path")


def _ensure_prior_map(context, params_file, model_dir, prior_pcd,
                      terrain_config, offset_z, spacing):
    """Export the Gazebo terrain and analyse it when the .bt is missing.

    Two steps, both offline: the mesh -> world-frame .pcd (with the SDF scale
    and the 0.75 m base_origin/base_link shift), then .pcd -> terrain .bt.
    """
    params_path = params_file.perform(context)
    try:
        with open(params_path, "r", encoding="utf-8") as handle:
            entry = yaml.safe_load(handle) or {}
        for key in MAP_PARAM_PATH:
            entry = entry[key]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f"[retap_s5] cannot read map_path from '{params_path}': {exc}")
        return []

    if not entry or os.path.exists(entry):
        return []

    out_dir = os.path.dirname(entry)
    os.makedirs(out_dir, exist_ok=True)
    pcd = prior_pcd.perform(context)
    export = [
        "ros2", "run", "finenav_retap", "export_gazebo_pcd",
        "--model", model_dir.perform(context),
        "--output", pcd,
        "--spacing", spacing.perform(context),
        "--offset-z", offset_z.perform(context),
    ]
    print(f"[retap_s5] prior map missing; {' '.join(export)}")
    if subprocess.run(export, check=False).returncode != 0:
        print("[retap_s5] terrain export failed.")
        return []

    analyze = [
        "ros2", "run", "finenav_retap", "retap_map_tool", "analyze",
        "--pcd", pcd,
        "--resolution", "0.2",
        "--config", terrain_config.perform(context),
        "--output-dir", out_dir,
    ]
    print(f"[retap_s5] {' '.join(analyze)}")
    if subprocess.run(analyze, check=False).returncode != 0:
        print("[retap_s5] terrain analysis failed.")
    return []


def generate_launch_description():
    retap_share = get_package_share_directory("finenav_retap")
    sim_share = get_package_share_directory("finenav_simulator")

    default_params = os.path.join(retap_share, "demo", "params", "retap_demo_s5.yaml")
    default_rviz = os.path.join(retap_share, "demo", "rviz", "retap_s5.rviz")
    default_model_dir = os.path.join(sim_share, "resource", "model", "map_desert")
    default_prior_pcd = "/tmp/retap_gazebo/map_desert_world.pcd"
    default_terrain_config = os.path.join(retap_share, "config", "terrain_sim.yaml")

    params_file = LaunchConfiguration("params_file")
    world_name = LaunchConfiguration("world_name")
    rviz = LaunchConfiguration("rviz")
    unpause = LaunchConfiguration("unpause")
    startup_delay = LaunchConfiguration("startup_delay")
    model_dir = LaunchConfiguration("model_dir")
    prior_pcd = LaunchConfiguration("prior_pcd")
    terrain_config = LaunchConfiguration("terrain_config")
    prior_offset_z = LaunchConfiguration("prior_offset_z")
    prior_spacing = LaunchConfiguration("prior_spacing")

    # ── simulator (ground-truth localization chain) ─────────────────────────
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "complete", "finenav_simulator.launch.py")
        ),
        launch_arguments={
            "lio_type": "gazebo_odom",
            "use_sim_time": "true",
            "world_name": world_name,
            "publish_map_tf": "false",   # the helper node publishes identity map->odom
        }.items(),
    )

    # ── unpause + the two consumers that need a live TF tree ────────────────
    unpause_gazebo = ExecuteProcess(
        cmd=[
            "ign", "service",
            "-s", "/world/default/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000",
            "--req", "pause: false",
        ],
        output="screen",
        condition=IfCondition(unpause),
    )

    # /livox/lidar → /cloud_registered, expressed in `map`.
    # The scan geometry must match the simulated sensor (see
    # resource/robot/model.sdf.xmacro): 64 vertical x 360 horizontal samples,
    # elevation -0.23..1.57 rad in the sensor frame.
    cloud_to_map = Node(
        package="finenav_simulator",
        executable="point_cloud_converter_node",
        name="cloud_to_map",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "pcd_topic": "/livox/lidar",
            "output_topic": "/cloud_registered",
            "output_frame_id": "map",
            "transform_output_to_frame": True,
            "vertical_samples": 64,
            "horizontal_samples": 360,
            "vertical_min_angle": -0.23,
            "vertical_max_angle": 1.57,
            "horizontal_min_angle": 0.0,
            "horizontal_max_angle": 6.2831852,
            "scan_period": 0.1,
            "synthesize_time": True,
            # Reject returns on the robot's own body (same box as the
            # simulator's own converter).
            "self_filter_box.enable": True,
        }],
    )

    # No `name=` on purpose: launch_ros would pass a process-wide
    # `__node:=<name>` remap and rename every node the process creates,
    # including the MapServer/ControlLayer private nodes whose fully-qualified
    # names are the keys of the parameter file.
    demo = Node(
        package="finenav_retap",
        executable="retap_demo",
        output="screen",
        parameters=[params_file],
    )

    delayed_start = TimerAction(
        period=startup_delay,
        actions=[unpause_gazebo, cloud_to_map, demo],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", default_rviz],
        parameters=[{"use_sim_time": True}],
        output="log",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Parameter file for retap_demo (S5 online mapping)."),
        DeclareLaunchArgument(
            "world_name", default_value="map_desert.world",
            description="World under finenav_simulator/resource/worlds."),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Start RViz2 (retap_s5.rviz: terrain clouds + path)."),
        DeclareLaunchArgument(
            "unpause", default_value="true",
            description="Unpause Gazebo after startup (the world launches paused)."),
        DeclareLaunchArgument(
            "startup_delay", default_value="5.0",
            description="Seconds to wait for Gazebo/robot spawn before unpausing and starting the consumers."),
        DeclareLaunchArgument(
            "model_dir", default_value=default_model_dir,
            description="Gazebo model the prior map is exported from."),
        DeclareLaunchArgument(
            "prior_pcd", default_value=default_prior_pcd,
            description="Where the exported world-frame cloud is written."),
        DeclareLaunchArgument(
            "terrain_config", default_value=default_terrain_config,
            description="Terrain thresholds used to analyse that cloud."),
        DeclareLaunchArgument(
            "prior_offset_z", default_value="0.75",
            description="z shift from the Gazebo world to `map` (base_origin vs base_link)."),
        DeclareLaunchArgument(
            "prior_spacing", default_value="0.04",
            description="Mesh sampling distance for the prior cloud (m)."),

        OpaqueFunction(
            function=_ensure_prior_map,
            args=[params_file, model_dir, prior_pcd, terrain_config,
                  prior_offset_z, prior_spacing]),
        simulator,
        delayed_start,
        rviz_node,
    ])
