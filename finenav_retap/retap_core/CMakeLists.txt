# retap_core: ROS-free port of ReTAP-Planner's terrain analysis and A*.
# Deliberately does not link rclcpp: this is the "algorithm" layer.
#
# The ported files keep the upstream names and directory layout:
#   include/retap_core/world_representation/ig_tree.h   <- mapping_module/...
#   include/retap_core/utils/*.h                        <- mapping_module/utils/...
#   include/retap_core/astar_3d/*.h                     <- path_planning/...
#   include/retap_core/terrain_evaluation.h
# Only the include root changed (mapping_module -> retap_core), so
# `diff -u <upstream> <port>` shows just the de-ROS edits.

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree search)
find_package(Threads REQUIRED)

add_library(retap_core SHARED
    src/world_representation/ig_tree.cpp
    src/utils/bounding_box.cpp
    src/utils/octomap_utils.cpp
    src/terrain_evaluation.cpp
    src/terrain_grid.cpp
    src/terrain_model.cpp
    src/map_io.cpp
)

target_include_directories(retap_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

target_compile_definitions(retap_core PUBLIC ${PCL_DEFINITIONS})

target_link_libraries(retap_core PUBLIC
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
    Threads::Threads
)

install(TARGETS retap_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
