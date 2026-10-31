# retap_demo — assembly executable, mirroring finenav_example/getting_started.
#
# The plugin .so's (map, planner, world view) are deliberately NOT linked: a
# self-registering library the executable references no symbol from is dropped
# by --as-needed, silently skipping its static-init registrar.  They are
# dlopened via FineNavEngine::loadAlgoPlugins() instead.

add_executable(retap_demo main.cpp)

target_link_libraries(retap_demo
    Eigen3::Eigen)
ament_target_dependencies(retap_demo
    rclcpp
    finenav_engine_cpp
    geometry_msgs)

install(TARGETS retap_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME}/demo)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME}/demo)
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/demo)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/demo)
# RetapTerrainMap's shutdown line is the experiment table (config/README.md);
# one script drives every row through the rosbag launch, one turns the bag into
# the offline inputs (accumulated PCD + pose track), one draws the result, two
# split the result by height (the map holds a floor and a slab).
install(PROGRAMS scripts/retap_table.sh scripts/bag2pcd.py scripts/retap_plot.py
        scripts/retap_bands.py scripts/retap_density.py
        DESTINATION share/${PROJECT_NAME}/demo/scripts)
