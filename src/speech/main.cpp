#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nao_playing_memory/AskQuestionAction.h>
#include <nao_playing_memory/SaySomethingAction.h>

#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <std_srvs/Empty.h>
#include "speech/SpeechRecognition.h"
#include "string.h"

///create a actionserver that provides the action saysomething2, that lets the nao say something with directly publishing to speech node///
class SaySomething2
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<nao_playing_memory::SaySomethingAction> say2_srv;

    std::string action_name;


    nao_playing_memory::SaySomethingFeedback say_feedback;
    nao_playing_memory::SaySomethingResult say_result;

public:
    SaySomething2(std::string name) :
        say2_srv (nh_, name, boost::bind(&SaySomething2::executeCB, this, _1), false),
            action_name(name){
        say2_srv.start();
    }


    ~SaySomething2(void){

    }

    void executeCB(const nao_playing_memory::SaySomethingGoalConstPtr &goal){
        ros::Rate r(10);
        bool success = true;

        std::string text = goal->text;

        speech::SpeechRecognitionClient RecogClient(nh_);

        RecogClient.say_something(text);



        if(say2_srv.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", action_name.c_str());
            say2_srv.setPreempted();
            success = false;
        }
        say2_srv.publishFeedback(say_feedback);


        if(success){
            ROS_INFO("%s: Succeeded", action_name.c_str());
            say2_srv.setSucceeded(say_result);
        }

    }
};

///creates actionserver that provides action saysomething that uses the speechactiongoal action to let nao say something///
class SaySomething
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<nao_playing_memory::SaySomethingAction> say_srv;

    std::string action_name;


    nao_playing_memory::SaySomethingFeedback say_feedback;
    nao_playing_memory::SaySomethingResult say_result;

public:
    SaySomething(std::string name) :
            say_srv (nh_, name, boost::bind(&SaySomething::executeCB, this, _1), false),
            action_name(name){
        say_srv.start();
    }


    ~SaySomething(void){

    }

    void executeCB(const nao_playing_memory::SaySomethingGoalConstPtr &goal){
        ros::Rate r(10);
        bool success = true;

        std::string text = goal->text;

        speech::SpeechRecognitionClient RecogClient(nh_);

        RecogClient.talk(text);



        if(say_srv.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", action_name.c_str());
            say_srv.setPreempted();
            success = false;
        }
        say_srv.publishFeedback(say_feedback);
        r.sleep();
        r.sleep();
        r.sleep();
        r.sleep();
        r.sleep();
        r.sleep();
        r.sleep();


        if(success){
            ROS_INFO("%s: Succeeded", action_name.c_str());
            say_srv.setSucceeded(say_result);
        }

    }
};
///create ask question action server, that provides the askquestion action ///
class AskQuestion
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<nao_playing_memory::AskQuestionAction> ask_srv;

    std::string action_name_1;


    nao_playing_memory::AskQuestionFeedback ask_feedback;
    nao_playing_memory::AskQuestionResult ask_result;

public:
    AskQuestion(std::string name) :
        ask_srv (nh_, name, boost::bind(&AskQuestion::executeCB, this, _1), false),
        action_name_1(name){
        ask_srv.start();
    }


    ~AskQuestion(void){

    }

    void executeCB(const nao_playing_memory::AskQuestionGoalConstPtr &goal){
        ros::Rate r(1);
        bool success = true;

        std::string question = goal->question;
        std::vector<std::string> possible_answers = goal->possible_answers;

        std::string answer;
        uint32_t duration=5;
        speech::SpeechRecognitionClient RecogClient(nh_);

        RecogClient.request_response_block(possible_answers,question,answer);
        ask_feedback.question_asked = true;
        ask_srv.publishFeedback(ask_feedback);


        if(ask_srv.isPreemptRequested() || !ros::ok()){
            ROS_INFO("%s: Preempted", action_name_1.c_str());
            ask_srv.setPreempted();
            success = false;
        }

        r.sleep();

        if(success){
            ask_result.answer = answer;
            ROS_INFO("%s: Succeeded", action_name_1.c_str());
            ask_srv.setSucceeded(ask_result);
        }

    }
};

main(int argc, char **argv) {
    ros::init(argc, argv, "speech");
    ros::NodeHandle n;

    /*
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
     */

    AskQuestion askQuestion("ask_question");
    SaySomething saySomethingAction("say_something");
    SaySomething2 saySomething2Action("say_something2");

    ros::spin();
    return 0;
}
