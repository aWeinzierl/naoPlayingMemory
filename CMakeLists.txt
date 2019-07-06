cmake_minimum_required(VERSION 2.8.12)
project(nao_playing_memory)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
        genmsg
        actionlib_msgs
)
add_action_files(
        FILES
        AskQuestion.action
        SaySomething.action
)
add_message_files(
        FILES
        Position.msg
        ConcealedCard.msg
        ExposedCard.msg
        Cards.msg
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        std_msgs
)
catkin_package(
        CATKIN_DEPENDS
        message_runtime
)



set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake")

add_subdirectory(src/perception)
add_subdirectory(src/speech)
add_subdirectory(src/reasoning)
add_subdirectory(src/action)

