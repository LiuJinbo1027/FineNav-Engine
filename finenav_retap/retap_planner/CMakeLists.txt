# RetapAstarPlanner — FineNav Planner plugin.
#
# Mirrors finenav_example/create_custom_planner: one generate_parameter_library
# target from retap_astar_planner.yaml, one SHARED library, headers installed.
#
# retap_core is a plain CMake target in this same package (not an ament
# package), so it is linked by target name.  It is ROS-free.

find_package(generate_parameter_library REQUIRED)

generate_parameter_library(retap_astar_planner_params retap_astar_planner.yaml)

add_library(retap_planner SHARED src/retap_planner.cpp)
target_include_directories(retap_planner PUBLIC include)
target_link_libraries(retap_planner
    retap_core
    retap_astar_planner_params)
ament_target_dependencies(retap_planner
    finenav_core
    finenav_visualizer)

install(TARGETS retap_planner
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
