#pragma once

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <actionlib/client/simple_action_client.h>
#include <nao_playing_memory/AskQuestionAction.h>
#include <nao_playing_memory/SaySomethingAction.h>

#include "nao_playing_memory/Cards.h"
#include "nao_playing_memory/ConcealedCard.h"
#include "nao_playing_memory/Position.h"

#include "model/ConcealedCard.h"
#include "model/CardPosition.h"
#include "model/ExposedCard.h"

#include "ActionBlocker.h"
#include "StateProcessor.h"

class NodeManager {
private:
    ros::NodeHandle _n;
    reasoning::StateProcessor _sp;
    reasoning::PrologClient _pc;
    ros::Subscriber _sub;

    actionlib::SimpleActionClient<nao_playing_memory::AskQuestionAction> _question_node;
    actionlib::SimpleActionClient<nao_playing_memory::SaySomethingAction> _voice_node;

    boost::asio::io_service _io_service;
    boost::posix_time::seconds _interval;
    boost::asio::deadline_timer _timer;
    void execute_every_second();

    static reasoning::CardPosition cardPositionMap(const nao_playing_memory::Position &position);

    void vision_callback(const nao_playing_memory::Cards::ConstPtr &msg);

    void say_synchronous(std::string text);

    bool ask_to_play();

    void ask_to_turn_card(reasoning::ConcealedCard card);

public:

    NodeManager();

    void surrect();


    bool initialize_game_board();

    void ask_to_collect_cards();

    void ask_to_collect_cards(const std::vector<reasoning::ConcealedCard> &cards);
};
