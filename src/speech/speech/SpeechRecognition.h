#pragma once

#include <ros/node_handle.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <actionlib_msgs/GoalStatusArray.h>

namespace speech{

    class SpeechRecognitionClient {
        std::vector<std::string> _matches;

    public:
        explicit SpeechRecognitionClient(ros::NodeHandle& nodeHandle);
        void talk(std::string sentence);
        void listen(std::vector<std::string>& available_sentences, std::string& results, int record_duration);
        void speech_recognition_callback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg);
        void publish_vocab(std::vector<std::string>& vocab);
        void speech_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
        void request_response_block(std::vector<std::string> &vocabulary, std::string& request, std::string& response);

    private:
        ros::Publisher _speech_pub;
        ros::Publisher _vocab_pub;
        ros::ServiceClient _recog_start_srv;
        ros::ServiceClient _recog_stop_srv;
        ros::Subscriber _speech_recog_sub;
        ros::Subscriber _speech_status;
        uint32_t _vocab_id = 0;

        bool _currently_speaking = false;
    };
}
