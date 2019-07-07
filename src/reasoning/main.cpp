#include "ros/ros.h"
#include "prolog/PrologClient.h"
#include "NodeManager.h"


main(int argc, char **argv) {
    ros::init(argc, argv, "reasoning");

    reasoning::PrologClient pc;
    pc.test_prolog_query();

    //NodeManager nm;


    ros::spin();
}
