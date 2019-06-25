#include <ros/ros.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_srvs/Empty.h>
#include "SpeechRecognition.h"
#include <string>
#include <iostream>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

perception::SpeechRecognitionClient::SpeechRecognitionClient(ros::NodeHandle &nodeHandle) {
    _vocab_pub = nodeHandle.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_action/goal", 1);
    _recog_start_srv = nodeHandle.serviceClient<std_srvs::Empty>("/start_recognition");
    _recog_stop_srv = nodeHandle.serviceClient<std_srvs::Empty>("/stop_recognition");
    _speech_recog_sub = nodeHandle.subscribe("/word_recognized", 1, &SpeechRecognitionClient::speech_recognition_callback, this);
}

void perception::SpeechRecognitionClient::listen(std::vector<std::string> &available_sentences, std::string &results,
                                                 int record_duration) {
    ros::Rate loop_rate(10);
    for(int i=0;i<available_sentences.size();i++){
        std::cout<<available_sentences.at(i)<<std::endl;
    }
    publish_vocab(available_sentences);
    _matches.clear();

    std_srvs::Empty empty;

    if(_recog_start_srv.call(empty)){
        std::cout<<"Start recording"<<std::endl;

        int wait_iters = 0;
        while(wait_iters < record_duration * 10) {
            ros::spinOnce();
            loop_rate.sleep();
            wait_iters ++;
        }

        if(_recog_stop_srv.call(empty)) {
            std::cout<<"Stop recording"<<std::endl;
        }

        if(_matches.empty()){
            std::cout<<"No word recognized. Maybe the recording duration was too short."<< std::endl;
        }

        else {
            for(int i=0; i<_matches.size();i++) {
                results.append(_matches.at(i));
                results.append(" ");
            }
            std::cout<<"nao understood: "<<results<<std::endl;
        }
    }
    else {
        std::cout<<"Could not start Recording"<<std::endl;
    }
}

void perception::SpeechRecognitionClient::speech_recognition_callback(
        const naoqi_bridge_msgs::WordRecognized::ConstPtr &msg) {
    std::cout<<"Here"<< std::endl;
    for (int i = 0; i < msg->words.size(); i++) {

        // Check confidence values
        if (msg->confidence_values[i] > 0.4f) {
            std::string log_msg = "MATCHED INPUT TO WORD " +
                                  msg->words[i] + " WITH " + patch::to_string(int(100*msg->confidence_values[i])) + "% CONFIDENCE";
            std::cout<<log_msg<<std::endl;
            _matches.push_back(msg->words[i]);
        }
        else{
            std::string log_msg = "MATCHED INPUT TO WORD " +
                                  msg->words[i] + " WITH " + patch::to_string(int(100*msg->confidence_values[i])) + "% CONFIDENCE";
            std::cout<<log_msg<<std::endl;
        }
    }
}

void perception::SpeechRecognitionClient::publish_vocab(std::vector<std::string> &vocab) {
    // Create emtpy message
    std_srvs::Empty empty;
    naoqi_bridge_msgs::SetSpeechVocabularyActionGoal msg_goal;
    int vocab_id = 0;
    msg_goal.goal_id.id = patch::to_string(vocab_id);
    msg_goal.goal.words = vocab;
    ros::Rate loop_rate(100);
    for(size_t i = 0; i < 10; i++) {
        _vocab_pub.publish(msg_goal);  // Publish the currently available command(s)
        ros::spinOnce();
        loop_rate.sleep();
    }

}
