#include <ros/ros.h>
#include <std_msgs/String.h>
#include <zconf.h>


main(int argc, char **argv) {
    ros::init(argc, argv, "action");

    ros::NodeHandle n;
    ros::Rate r_sleep(10);

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("speech",1);
    while(!chatter_pub.getNumSubscribers()){
        r_sleep.sleep();
    }
    std_msgs::String msg;

    msg.data="Hi my friend, Lets play a round of memory!";

    chatter_pub.publish(msg);

    msg.data="I will start, ok?";

    chatter_pub.publish(msg);

    "word_recognized"

    "rosrun nao_playing_memory action \n"
    "cogr@stud04:~/ros/workspace/memory$ rostopic echo /word_recognized \n"
    "^Ccogr@stud04:~/ros/workspace/memory$ rostopic info /word_recognized \n"
    "Type: naoqi_bridge_msgs/WordRecognized\n"
    "\n"
    "Publishers: \n"
    " * /nao_speech (http://stud04:40378/)\n"
    "\n"
    "Subscribers: None\n"
    ""

    "/word_recognized"
    "rosservice call /start_recognition "{}""




    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<std_msgs>("start_recognition");
    beginner_tutorials::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if (client.call(srv))






}