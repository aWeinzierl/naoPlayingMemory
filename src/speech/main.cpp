#include <ros/ros.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <std_srvs/Empty.h>
#include "speech/SpeechRecognition.h"

main(int argc, char **argv) {
    ros::init(argc, argv, "speech");
    ros::NodeHandle n;

    std::vector<std::string> available_vocabulary;
    available_vocabulary.push_back("Yes");
    available_vocabulary.push_back("No");
    available_vocabulary.push_back("Hi Nao");

    std::string result;
    uint32_t duration=10;

    perception::SpeechRecognitionClient RecogClient(n);
    RecogClient.listen(available_vocabulary,result, duration);
    std::cout<<"Nao understood: "<<result<<std::endl;


}