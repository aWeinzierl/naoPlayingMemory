#include "ros/ros.h"

#include "NodeManager.h"

main(int argc, char **argv) {
    ros::init(argc, argv, "reasoning");
    PrologClient pc;
    
    pc.test_prolog_query();
    ros::spin();
}
