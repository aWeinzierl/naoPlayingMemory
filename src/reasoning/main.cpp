#include "ros/ros.h"

#include "prolog/PrologClient.h"
#include "NodeManager.h"


main(int argc, char **argv) {
    ros::init(argc, argv, "reasoning");

    reasoning::PrologClient pc;

    NodeManager nm;
    
    pc.test_prolog_query();
    ros::spin();
}
