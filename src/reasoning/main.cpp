#include "ros/ros.h"

#include "NodeManager.h"

main(int argc, char **argv) {
    ros::init(argc, argv, "reasoning");

    NodeManager nm;

    ros::spin();
}