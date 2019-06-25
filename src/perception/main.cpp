#include <ros/ros.h>

main(int argc, char **argv) {
    ros::init(argc, argv, "perception");

    //Speech Recognition
    //ros::NodeHandle n;
    //ros.Publisher vocab_pub = n.advertise<naoqi_bridge_msgs::SpeechVocabularyActionGoal>("/speech_vocabulary_action/goal",1)




    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("speech",1);
    //while(!chatter_pub.getNumSubscribers()){
    //    r_sleep.sleep();
    //}
    //std_msgs::String msg;

    //msg.data="Hi my friend, Lets play a round of memory!";

    //chatter_pub.publish(msg);

    //msg.data="I will start, ok?";

    //chatter_pub.publish(msg);

    //"word_recognized"

    //"rosrun nao_playing_memory action \n"
    //"cogr@stud04:~/ros/workspace/memory$ rostopic echo /word_recognized \n"
    //"^Ccogr@stud04:~/ros/workspace/memory$ rostopic info /word_recognized \n"
    //"Type: naoqi_bridge_msgs/WordRecognized\n"
    //"\n"
    //"Publishers: \n"s
    //" * /nao_speech (http://stud04:40378/)\n"
    //"\n"
    //"Subscribers: None\n"
    //""

    //"/word_recognized"
    //"rosservice call /start_recognition "{}""
}