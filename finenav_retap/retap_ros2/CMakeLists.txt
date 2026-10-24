# Offline tooling and standalone nodes around retap_core.
#
# This subdirectory mirrors finenav_example's layout: one ament package
# (finenav_retap) with several algorithm/tool subdirectories, each added by the
# package-level CMakeLists.txt.

find_package(yaml-cpp REQUIRED)

add_executable(retap_map_tool
    src/retap_map_tool_main.cpp
    src/terrain_config.cpp
)

target_link_libraries(retap_map_tool
    retap_core
    yaml-cpp
)

install(TARGETS retap_map_tool
    DESTINATION lib/${PROJECT_NAME}
)

# S2: standalone global planner (ROS 2 port of ReTAP's GlobalPlanning node).
# The file-based map source needs only tf2; the octomap_msgs topic source is
# enabled automatically once `ros-humble-octomap-msgs` is installed.
find_package(octomap_msgs QUIET)

add_executable(retap_global_planner_node
    src/retap_global_planner_node.cpp
)

target_link_libraries(retap_global_planner_node
    retap_core
)

ament_target_dependencies(retap_global_planner_node
    rclcpp
    tf2
    tf2_ros
    nav_msgs
    geometry_msgs
)

if(octomap_msgs_FOUND)
    target_compile_definitions(retap_global_planner_node
        PRIVATE RETAP_HAS_OCTOMAP_MSGS
    )
    ament_target_dependencies(retap_global_planner_node octomap_msgs)
endif()

install(TARGETS retap_global_planner_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# S5: export a Gazebo terrain model to a world-frame .pcd, so the prior map and
# the live lidar scans are the same surface.  Invoked by
# demo/launch/retap_s5_sim.launch.py when the prior .bt is missing.
install(PROGRAMS scripts/export_gazebo_pcd.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME export_gazebo_pcd
)
