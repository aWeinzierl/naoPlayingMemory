#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <zconf.h>

namespace SPEECH {
    void say_something(std::string text_to_say){

        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("speech",1);
        ros::Rate r_sleep(20);
        while(!chatter_pub.getNumSubscribers()){
            r_sleep.sleep();
        }

        std_msgs::String msg;
        msg.data = text_to_say;

        chatter_pub.publish(msg);
    }
}

main(int argc, char **argv) {
    ros::init(argc, argv, "action");

    std::string text;
    text = "Hello";
    SPEECH::say_something(text);

}