cmake_minimum_required(VERSION 2.8.3)
project(tutorial_vision)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  json_prolog
)

catkin_package()
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(nao_vision_ex1 src/main.cpp src/PrologClient.cpp src/PrologClient.h)

## Specify libraries to link a library or executable target against
target_link_libraries(nao_vision_ex1 ${catkin_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(nao_vision_ex1 ${OpenCV_LIBS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake")
find_package(aruco REQUIRED )
target_link_libraries(nao_vision_ex1  ${aruco_LIBS})
