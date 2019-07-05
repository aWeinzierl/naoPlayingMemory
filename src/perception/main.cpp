#include <ros/ros.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <std_srvs/Empty.h>
#include "vision/Vision.h"

main(int argc, char **argv) {
    ros::init(argc, argv, "vision");
    ros::NodeHandle n;

    perception::VisionClient VisionClient(n);
    ros::spin();
}