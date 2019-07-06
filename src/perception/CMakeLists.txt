cmake_minimum_required(VERSION 2.8.12)
project(vision)
add_compile_options(-std=c++11)

add_executable(${PROJECT_NAME} main.cpp
        vision/Vision.cpp
        vision/Card.cpp
        vision/GridElement.cpp
        vision/GridBoard.cpp
        )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        naoqi_bridge_msgs
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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake")
find_package(aruco REQUIRED )
target_link_libraries(${PROJECT_NAME}  ${aruco_LIBS})

target_include_directories(${PROJECT_NAME}
        PRIVATE ext/optional-lite/include)