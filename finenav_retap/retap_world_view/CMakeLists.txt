# RetapWorldView — FineNav WorldView plugin.
#
# Mirrors finenav_example/create_custom_world_view: one
# generate_parameter_library target from retap_world_view.yaml, one SHARED
# library, headers installed.
#
# Unlike the example, this view depends on the concrete Map and Planner plugin
# libraries (it is the one artifact that names both), so it links them.

find_package(generate_parameter_library REQUIRED)

generate_parameter_library(retap_world_view_params retap_world_view.yaml)

add_library(retap_world_view SHARED src/retap_world_view.cpp)
target_include_directories(retap_world_view PUBLIC include)
target_link_libraries(retap_world_view
    retap_world_view_params
    retap_terrain_map
    retap_terrain_map_params
    retap_planner
    retap_astar_planner_params
    retap_core)
ament_target_dependencies(retap_world_view
    finenav_core
    finenav_visualizer)

install(TARGETS retap_world_view
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
