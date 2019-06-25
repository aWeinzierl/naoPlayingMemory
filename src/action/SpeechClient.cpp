#include <string>
#include <ros/ros.h>
#include "SpeechClient.h"

void action::SpeechClient::say_something(std::string text_to_say) {

    while(!_chatter_pub.getNumSubscribers()){
        _r_sleep.sleep();
    }

    std_msgs::String msg;
    msg.data = text_to_say;

    _chatter_pub.publish(msg);
}
