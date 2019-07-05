cmake_minimum_required(VERSION 2.8.3)
project(reasoning)

add_definitions(-std=c++11)

add_executable(${PROJECT_NAME} main.cpp
        prolog/PrologClient.cpp
        prolog/classes/Card.cpp
        prolog/classes/Instance.cpp
        prolog/ObjectProperty.cpp
        prolog/classes/CardPosition.cpp
        prolog/PrologEntities.h
        prolog/DataProperty.cpp
        prolog/ConcealedCard.cpp prolog/ConcealedCard.h prolog/ExposedCard.cpp prolog/ExposedCard.h)

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

target_include_directories(${PROJECT_NAME}
        PRIVATE ext/optional-lite/include)