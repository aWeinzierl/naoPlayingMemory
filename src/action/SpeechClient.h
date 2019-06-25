#pragma once

#include <std_msgs/String.h>
#include <ros/ros.h>


namespace action {
    class SpeechClient {
        ros::NodeHandle _n;
        ros::Publisher _chatter_pub;
        ros::Rate _r_sleep;

    public:
        SpeechClient():_r_sleep(20){
            _chatter_pub = _n.advertise<std_msgs::String>("speech",1);
        }

        void say_something(std::string text_to_say);
    };
}
