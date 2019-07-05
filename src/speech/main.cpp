#include <ros/ros.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <std_srvs/Empty.h>
#include "speech/SpeechRecognition.h"
#include "string.h"

main(int argc, char **argv) {
    ros::init(argc, argv, "speech");
    ros::NodeHandle n;

    std::vector<std::string> available_vocabulary;
    available_vocabulary.push_back("Yes");
    available_vocabulary.push_back("No");
    available_vocabulary.push_back("Hi Nao");

    std::string result;
    uint32_t duration=10;
    speech::SpeechRecognitionClient RecogClient(n);
    std::string text = "Hi, I am going to kill you!";
    RecogClient.talk(text);


    std::string request = "You want to play a round?";
    RecogClient.request_response_block(available_vocabulary,request, result);


}