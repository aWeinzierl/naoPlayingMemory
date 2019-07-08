cmake_minimum_required(VERSION 2.8.3)
project(reasoning)

add_definitions(-std=c++11)

add_executable(${PROJECT_NAME} main.cpp
        prolog/PrologClient.cpp
        prolog/Instance.cpp
        prolog/ObjectProperty.cpp
        prolog/DataProperty.cpp
        model/ConcealedCard.cpp
        model/ConcealedCard.h
        model/ExposedCard.cpp
        model/ExposedCard.h
        model/CardPosition.cpp
        FilterRecognizeTurn.cpp
        StateProcessor.cpp
        NodeManager.cpp
        CardStateRetriever.cpp
        )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        cv_bridge
        image_transport
        json_prolog
        actionlib
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

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})