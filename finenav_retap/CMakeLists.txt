cmake_minimum_required(VERSION 3.22)
project(finenav_retap)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(finenav_engine_cpp REQUIRED)

ament_auto_find_build_dependencies()

# retap_core is deliberately ROS-free: it only needs octomap, Eigen and PCL.
add_subdirectory(retap_core)
add_subdirectory(retap_ros2)
# FineNav plugins (mirroring finenav_example/create_custom_*).
add_subdirectory(retap_map)
add_subdirectory(retap_planner)
# The WorldView names both the map and the planner, so it comes after them.
add_subdirectory(retap_world_view)
add_subdirectory(demo)
add_subdirectory(test)

# Terrain parameter sets for `retap_rosbag.launch.py config:=<name>.yaml` and
# for `retap_map_tool analyze --config <name>.yaml` (same file, both readers).
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
