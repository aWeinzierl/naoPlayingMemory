#pragma once

#include <ros/node_handle.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <actionlib_msgs/GoalStatusArray.h>

namespace speech {
    //client that contains the matches
    class SpeechRecognitionClient {
        std::vector <std::string> _matches;

    public:
        //need a ros nodehandle and creates publishers to different topics and clients to the required servers
        explicit SpeechRecognitionClient(ros::NodeHandle &nodeHandle);

        ///lets the nao say something, i.e. publishes the provided sentence to the speech action goal
        ///\param sentence to be said out loud by nao
        void talk(std::string sentence);

        ///requires the available vocabulary, publishes an empty request to the action server and listens to for the specified duration
        ///\param available_sentences that are supposed to be recognised
        ///\param results vector of strings  where the results get saved to
        ///\param record_duration how long the nao is supposed to listen
        void listen(std::vector <std::string> &available_sentences, std::string &results, int record_duration);

        ///callback to actionserver that gets the recognized words
        ///\param msg that contains the recognized vocabulary
        void speech_recognition_callback(const naoqi_bridge_msgs::WordRecognized::ConstPtr &msg);

        ///publish the vocabulary to be understood to the speech action server
        ///\param vocab vector of strings that contains the vocabulary to be understood
        void publish_vocab(std::vector <std::string> &vocab);

        ///check if the nao is speeking at the moment
        ///\param msg status msg that says is action is blocked
        void speech_status(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);

        ///lets the nao say out the question and listen to the answers and says out the answer
        ///\param vocabulary that is supposed to be understood
        ///\param request question, that is supposed to be spoken out
        ///\param response string that gets the understood response
        void request_response_block(std::vector <std::string> &vocabulary, std::string &request, std::string &response);

        ///let nao say something, similar to talk but publishes directly to speech node and not speech action goal
        ///\param text_to_say string that the nao is supposed to say
        void say_something(std::string text_to_say);

    private:
        //publishers and clients required
        ros::Publisher _chatter_pub;
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
