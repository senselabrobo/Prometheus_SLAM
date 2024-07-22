cmake_minimum_required(VERSION 2.8.3)
project(prometheus_slam)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Werror)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

cs_add_library(${PROJECT_NAME}
    src/client/prometheus_slam_client.cpp
    src/client/map_server.cpp
    src/server/prometheus_slam_server.cpp
    src/server/client_handler.cpp
    src/server/pose_graph_interface.cpp
    src/server/global_tf_controller.cpp
    src/server/submap_collection.cpp
    src/server/client_tf_optimizer.cpp
    src/server/visualizer/server_visualizer.cpp)

cs_add_executable(prometheus_slam_server_node
    src/prometheus_slam_server_node.cpp)
target_link_libraries(prometheus_slam_server_node ${PROJECT_NAME} )

cs_add_executable(prometheus_slam_client_node
    src/prometheus_slam_client_node.cpp)
target_link_libraries(prometheus_slam_client_node ${PROJECT_NAME})
cs_add_executable(tsdf_recover_node
    src/tsdf_recover_node.cpp)
target_link_libraries(tsdf_recover_node ${PROJECT_NAME})
cs_export()
