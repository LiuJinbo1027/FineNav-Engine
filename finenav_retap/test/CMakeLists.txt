find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_ndt test_ndt.cpp)
target_link_libraries(test_ndt retap_core)

ament_add_gtest(test_terrain test_terrain.cpp)
target_link_libraries(test_terrain retap_core)

ament_add_gtest(test_incremental test_incremental.cpp)
target_link_libraries(test_incremental retap_core)

ament_add_gtest(test_astar test_astar.cpp)
target_link_libraries(test_astar retap_core)

ament_add_gtest(test_nearest test_nearest.cpp)
target_link_libraries(test_nearest retap_core)
