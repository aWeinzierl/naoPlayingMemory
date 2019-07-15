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

    actionlib::SimpleActionClient <nao_playing_memory::AskQuestionAction> _question_node;
    actionlib::SimpleActionClient <nao_playing_memory::SaySomethingAction> _voice_node;
    std::unordered_map<unsigned int, std::string> _card_classes;

    /// converts the position message to reasoning::CardPosition
    /// \param position
    /// \return
    static reasoning::CardPosition cardPositionMap(const nao_playing_memory::Position &position);

    /// let nao speak a text und wait until he is done
    /// \param text text which nao is supposed to say
    void say_synchronous(std::string text);

    void say_it_is_your_turn();

    /// asks if some one wants to play memory
    /// \return someone wants play
    bool ask_to_play();

    /// asks the opponent to turn his card
    /// \param card  the card which the opponent should turn
    void ask_to_turn_card(reasoning::ConcealedCard card);

    void say_someone_cheated();

    /// checks if the game board is properly set up and saves the initial state in ontology
    /// \return if check was successful
    bool initialize_game_board();

    /// tells the opponent to collect the given cards and waits until he has done that
    /// \param cards
    void ask_to_collect_cards(const std::vector <reasoning::ConcealedCard> &cards);

    /// checks if a player wins
    /// \param nao_points points of nao
    /// \param opponent_points points of the opponents
    /// \return
    bool checkForWin(const unsigned int nao_points, const unsigned int opponent_points);

    /// coordinate everything which has to be done on a win
    /// \param game_is_running indicator if a game is active
    /// \param nao_is_bored indicator if nao is feeling bored
    /// \param nao_points naos game points
    /// \param opponent_points game points of the opponents
    void handleWin(bool &game_is_running, bool &nao_is_bored, const unsigned int nao_points,
                   const unsigned int opponent_points);

    /// wait for a player who wants to play
    void NodeManager::waitForAPlayer();

public:

    /// constructs a fully working NodeManager
    NodeManager();

    /// bring nao to live and manage everything synchronously
    void surrect();
};
