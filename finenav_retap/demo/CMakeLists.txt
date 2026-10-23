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
