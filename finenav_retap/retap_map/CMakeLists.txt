# RetapTerrainMap — FineNav Map plugin.
#
# Mirrors finenav_example/create_custom_map: one generate_parameter_library
# target from retap_terrain_map.yaml, one SHARED library, headers installed.
#
# retap_core is a plain CMake target in this same package (not an ament
# package), so it is linked by target name.  It is ROS-free and links octomap,
# Eigen and PCL.

find_package(generate_parameter_library REQUIRED)

generate_parameter_library(retap_terrain_map_params retap_terrain_map.yaml)

add_library(retap_terrain_map SHARED src/retap_terrain_map.cpp)
target_include_directories(retap_terrain_map PUBLIC include)
target_link_libraries(retap_terrain_map
    retap_core
    retap_terrain_map_params)
ament_target_dependencies(retap_terrain_map
    finenav_core
    finenav_visualizer
    sensor_msgs)

install(TARGETS retap_terrain_map
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
