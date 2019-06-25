#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <zconf.h>
#include "SpeechClient.h"

main(int argc, char **argv) {
    ros::init(argc, argv, "action");

    std::string text;
    text = "Hello, I am going to kill you!";

    action::SpeechClient c;
    c.say_something(text);
}