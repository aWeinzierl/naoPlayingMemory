#include <ros/ros.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_srvs/Empty.h>
#include "SpeechRecognition.h"
#include <string>
#include <iostream>
#include <sstream>

namespace patch {
    template<typename T>
    std::string to_string(const T &n) {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
}

speech::SpeechRecognitionClient::SpeechRecognitionClient(ros::NodeHandle &nodeHandle) {
    _speech_pub = nodeHandle.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);
    _vocab_pub = nodeHandle.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>(
            "/speech_vocabulary_action/goal", 1);
    _recog_start_srv = nodeHandle.serviceClient<std_srvs::Empty>("/start_recognition");
    _recog_stop_srv = nodeHandle.serviceClient<std_srvs::Empty>("/stop_recognition");
    _speech_recog_sub = nodeHandle.subscribe("/word_recognized", 1,
                                             &SpeechRecognitionClient::speech_recognition_callback, this);
    _speech_status = nodeHandle.subscribe("/speech_action/status", 1, &SpeechRecognitionClient::speech_status, this);
}

void speech::SpeechRecognitionClient::listen(std::vector<std::string> &available_sentences, std::string &results,
                                             int record_duration) {
    ros::Rate loop_rate(10);

    publish_vocab(available_sentences);
    _matches.clear();

    std_srvs::Empty empty;

    if (_recog_start_srv.call(empty)) {
        std::cout << "Start recording" << std::endl;

        int wait_iters = 0;
        while (wait_iters < record_duration * 10) {
            ros::spinOnce();
            loop_rate.sleep();
            wait_iters++;
        }

        if (_recog_stop_srv.call(empty)) {
            std::cout << "Stop recording" << std::endl;
        }

        if (_matches.empty()) {
            std::cout << "No word recognized. Maybe the recording duration was too short." << std::endl;
        } else if (_matches.size() >= 2) {
            throw std::logic_error("multiple words recognized");
        } else {
            results = _matches[0];
            talk("i understood " + results);
        }
    } else {
        std::cout << "Could not start Recording" << std::endl;
    }
}

void speech::SpeechRecognitionClient::speech_recognition_callback(
        const naoqi_bridge_msgs::WordRecognized::ConstPtr &msg) {
    for (int i = 0; i < msg->words.size(); i++) {

        std::string log_msg = "MATCHED INPUT TO WORD " +
                              msg->words[i] + " WITH " + patch::to_string(int(100 * msg->confidence_values[i])) +
                              "% CONFIDENCE";
        std::cout << log_msg << std::endl;
        _matches.push_back(msg->words[i]);

    }
}

void speech::SpeechRecognitionClient::publish_vocab(std::vector<std::string> &vocab) {
    // Create emtpy message
    std_srvs::Empty empty;
    naoqi_bridge_msgs::SetSpeechVocabularyActionGoal msg_goal;
    int vocab_id = 0;
    msg_goal.goal_id.id = patch::to_string(vocab_id);
    msg_goal.goal.words = vocab;
    ros::Rate loop_rate(100);
    for (size_t i = 0; i < 10; i++) {
        _vocab_pub.publish(msg_goal);  // Publish the currently available command(s)
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void speech::SpeechRecognitionClient::talk(std::string sentence) {
    ros::Rate loop_rate(10);

    while (!_speech_pub.getNumSubscribers()) {
        loop_rate.sleep();
    }
    std::cout << "Talking: " << sentence << std::endl;
    std_srvs::Empty empty;  // Create Empty msg for stop record service

    naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_goal;
    msg_goal.goal_id.id = std::to_string(_vocab_id);
    msg_goal.goal.say = sentence;
    _vocab_id++; // Increase id counter to guarantee uniqueness
    _speech_pub.publish(msg_goal);

    // Wait 500 milli seconds to make sure the action status publishing
    // is received and we can check the speech status
    int i = 0;
    while (i < 5) {
        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }
    // Wait until speaking is done
    while (_currently_speaking) {
        // Wait
        ros::spinOnce();
        loop_rate.sleep();

    }
}

void speech::SpeechRecognitionClient::speech_status(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {
    // Check if there is a entry in the status list which contains a status value of on
    // indicating that Nao is currently busy speaking
    _currently_speaking = false;

    for (const auto &status: msg->status_list) {
        if (status.status == 1) {
            _currently_speaking = true;
        }
    }

}

void speech::SpeechRecognitionClient::request_response_block(std::vector<std::string> &vocabulary, std::string &request,
                                                             std::string &response) {
    // First Nao make the request
    // The call blocks until he is finished talking
    talk(request);
    // Wait for the answer. Fills the response parameter is the command was understood
    // Now we listen
    listen(vocabulary, response, 3);
}
