#pragma once

#include <string>

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
    reasoning::PrologClient _pc;

    actionlib::SimpleActionClient<nao_playing_memory::AskQuestionAction> _question_node;
    actionlib::SimpleActionClient<nao_playing_memory::SaySomethingAction> _voice_node;

    static reasoning::CardPosition cardPositionMap(const nao_playing_memory::Position &position);

    void say_synchronous(std::string text);

    void say_it_is_your_turn();

    bool ask_to_play();

    void ask_to_turn_card(reasoning::ConcealedCard card);

    void say_someone_cheated();

    std::unordered_map<unsigned int, std::string> card_classes;

public:

    NodeManager();

    void surrect();

    bool initialize_game_board();

    //void ask_to_collect_cards();

    void ask_to_collect_cards(const std::vector<reasoning::ConcealedCard> &cards);
};
