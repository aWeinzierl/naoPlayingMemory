cmake_minimum_required(VERSION 2.8.12)
project(nao_playing_memory)
catkin_package()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake")

add_subdirectory(src/perception)
add_subdirectory(src/reasoning)
add_subdirectory(src/action)
add_definitions(-std=c++11)


