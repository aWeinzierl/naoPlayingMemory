cmake_minimum_required(VERSION 2.8.12)
project(nao_playing_memory)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
)
add_message_files(
        FILES
        Position.msg
        ConcealedCard.msg
        ExposedCard.msg
        Cards.msg
)
generate_messages(
)
catkin_package(
        CATKIN_DEPENDS
        message_runtime
)



set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake")

add_subdirectory(src/perception)
add_subdirectory(src/reasoning)
add_subdirectory(src/action)

