cmake_minimum_required(VERSION 2.8.12)
project(action)

add_executable(${PROJECT_NAME} main.cpp
        SpeechClient.cpp
        )
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        cv_bridge
        image_transport
        json_prolog
        )
include_directories(
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})