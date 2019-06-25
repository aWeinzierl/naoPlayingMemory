#include <ros/ros.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <std_srvs/Empty.h>

main(int argc, char **argv) {
    /*ros::init(argc, argv, "perception");

    //Speech Recognition
    ros::NodeHandle n;
    ros::Rate r_sleep(30);
    ros::Publisher vocab_pub = n.advertise<naoqi_bridge_msgs::SpeechVocabularyActionGoal>("/speech_vocabulary_action/goal",1);
    ros::ServiceClient start_reco_srv = n.serviceClient<std_srvs::Empty>("/start_recognition");
    ros::ServiceClient stop_reco_srv = n.serviceClient<std_srvs::Empty>("/stop_recognition");


    while(!chatter_pub.getNumSubscribers()){
            r_sleep.sleep();
                        };

    */
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