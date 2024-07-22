cmake_minimum_required(VERSION 2.8.3)
project(prometheus_slam_mod)

add_definitions(-std=c++14 -Werror)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/opencv3_catkin/opencv3_build")
find_package(OpenCV 3.4.15 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_library(vio_interface
  src/vio_interface.cpp)
target_link_libraries(vio_interface
    ${CATKIN_LIBRARIES}
    ${OpenCV_LIBS}
)

##########
# EXPORT #
##########

cs_install()
cs_export()
