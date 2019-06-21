cmake_minimum_required(VERSION 2.8.3)
project(reasoning)

add_definitions(-std=c++11)

add_executable(${PROJECT_NAME} main.cpp PrologClient.cpp)
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

find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

find_package(aruco REQUIRED)
target_link_libraries(${PROJECT_NAME} ${aruco_LIBS})
