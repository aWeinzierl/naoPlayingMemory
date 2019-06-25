#pragma once

#include <ros/node_handle.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <actionlib_msgs/GoalStatusArray.h>

namespace perception{
    class State;

    class SpeechRecognitionClient {
        std::vector<std::string> _matches;

    public:
        explicit SpeechRecognitionClient(ros::NodeHandle& nodeHandle);

        void listen(std::vector<std::string>& available_sentences, std::string& results, int record_duration);
        void speech_recognition_callback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg);
        void publish_vocab(std::vector<std::string>& vocab);

    private:

        ros::Publisher _vocab_pub;
        ros::ServiceClient _recog_start_srv;
        ros::ServiceClient _recog_stop_srv;
        ros::Subscriber _speech_recog_sub;
    };
}
