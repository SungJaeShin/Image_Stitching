cmake_minimum_required(VERSION 3.0.2)
project(image_stitching)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES image_stitching
 CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/CMake/opengl_config.cmake
)

## Declare a C++ library
add_executable(stitching_node src/stitching.cpp)
target_link_libraries(stitching_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${realsense_INCLUDE_DIR})


message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
message(WARNING "Librealsense_VERSION: ${realsense2_VERSION}")