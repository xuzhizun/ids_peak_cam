cmake_minimum_required(VERSION 3.0.2)
project(ids_peak_cam)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  camera_info_manager
)

find_package(OpenCV REQUIRED)

find_package(ids_peak REQUIRED)

find_package(ids_peak_ipl REQUIRED)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS thread)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ids_peak_cam
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/ids_peak_cam
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp 
 src/${PROJECT_NAME}.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
	 ${catkin_LIBRARIES} ${OpenCV_LIBS} ids_peak ids_peak_ipl -pthread 
)

