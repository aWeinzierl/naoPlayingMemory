#include "NodeManager.h"

#include <tuple>
#include <utility>

#include "model/ConcealedCard.h"
#include "CardStateRetriever.h"

void ask_to_turn_card(reasoning::CardPosition card_position);

NodeManager::NodeManager()
        : _question_node("ask_question", true),
          _voice_node("say_something", true){

    _question_node.waitForServer();
    _voice_node.waitForServer();
}

reasoning::CardPosition NodeManager::cardPositionMap(const nao_playing_memory::Position &position) {
    return {
            position.x,
            position.y
    };
}

void NodeManager::surrect() {
    ros::Rate rate(30);
    unsigned int nao_points=0;
    unsigned int opponent_points=0;
    //TODO Initialize max_num of points one can achieve depending on the number of init cards

    auto nao_is_bored = true;
    while (true) {
        while (nao_is_bored) {
            rate.sleep();
            ros::Duration(3).sleep();
            auto wants_play = ask_to_play();
            if (wants_play) {
                nao_is_bored = false;
                opponent_points = 0;
                nao_points = 0;
                std::cout<<"I will play a game"<<std::endl;
            }

        }

        if (!initialize_game_board()) {
            std::cout<<"The board looks like shit"<<std::endl;
            say_synchronous("Please put the cards correctly onto the board");
            nao_is_bored = true;
        }

        auto nao_is_in_charge = true;

        auto game_is_running = true;
        //TODO CREATE INITIAL INSTANCES FROM CARDS WITH void PrologClient::save(const ConcealedCard &concealed_card)
        while (game_is_running) {
            std::cout<<"Game is Running"<<std::endl;
            rate.sleep();
            if (nao_is_in_charge) {
                std::cout<<"It is my turn"<<std::endl;
                auto cards_to_turn = _pc.decide_action();

                if (cards_to_turn.size() == 2) {
                    for (const auto &card: cards_to_turn) {
                        ask_to_turn_card(card);
                    }
                    ask_to_collect_cards(cards_to_turn);
                    //TODO remove  instances from prolog
                    _pc.delete_cards(cards_to_turn[0].get_id(),cards_to_turn[1].get_id());
                    //Added point counter
                    nao_points += 1;
                } else {
                    std::cout<<"i choose  card: "<<std::to_string(cards_to_turn[0].get_id())<<std::endl;
                    auto random_card = cards_to_turn[0];
                    ask_to_turn_card(random_card);
                    auto revealed_card = ActionBlocker(30,5).wait_until_card_is_revealed(random_card.get_position());
                    _pc.save_action(revealed_card);
                    ros::Duration(2).sleep();
                    auto card = _pc.search_paired_card(random_card);
                    //std::cout<<"Card:"<<card.value().get_id()<<std::endl;
                    if (card.has_value()) {
                        std::cout<<"Card:"<<card.value().get_id()<<std::endl;
                        ask_to_turn_card(card.value());
                        ask_to_collect_cards({random_card, card.value()});
                        //TODO remove  instances from prolog
                        _pc.delete_cards(random_card.get_id(),card.value().get_id());
                        //Added point counter
                        nao_points += 1;
                    } else {
                        std::cout<<"I dont know any card with the same class"<<std::endl;
                        auto second_random_card = _pc.search_random_card();
                        if (!second_random_card.has_value()){
                            say_someone_cheated();
                            //TODO decide what to do
                            game_is_running=false;
                            break;
                        }
                        ask_to_turn_card(second_random_card.value());
                        ActionBlocker(30, 5).wait_until_card_is_revealed(second_random_card.value().get_position());
                        ros::Duration(0.5).sleep();
                        _pc.search_paired_card(random_card);
                        if (random_card.get_id() == second_random_card.value().get_id()) {
                            ask_to_collect_cards({random_card, second_random_card.value()});
                            std::cout<<"i am waiting for someone to collect the cards"<<std::endl;
                            //TODO remove  instances from prolog
                            _pc.delete_cards(random_card.get_id(),second_random_card.value().get_id());
                            //Added point counter
                            nao_points += 1;
                        } else {
                            nao_is_in_charge = false;
                        }
                    }
                }
            }
            //TODO check game state
            if(!nao_is_in_charge){
                say_it_is_your_turn();

                auto card1 = ActionBlocker(30, 3).wait_until_any_card_is_revealed();
                auto card2 = ActionBlocker(30, 3).wait_until_any_card_is_revealed();



                bool equal_cards = _pc.are_cards_equal(card1.get_id(),card2.get_id());
                if(equal_cards){
                    //TODO remove  instances from prolog
                    //TODO WAIT UNTIL CARDS ARE REMOVED
                    ActionBlocker(30, 5).wait_until_cards_removed({
                        card1.get_position(),
                        card2.get_position()
                    });
                    _pc.delete_cards(card1.get_id(),card2.get_id());
                    opponent_points += 1;
                }else{
                    nao_is_in_charge = true;
                    say_synchronous("ahah no success for you");
                }
            }

            if(opponent_points > 3 || nao_points > 3 || nao_points+opponent_points>=6){
                game_is_running = false;
                nao_is_bored = true;
                if (opponent_points > 3) {
                    say_synchronous("good job");
                } else if (nao_points > 3 ){
                    say_synchronous("looser! There is no shame in loosing against the master!");
                } else{
                    say_synchronous("I could not bring it over me to beat you.");
                }
            }
        }
    }
}

void NodeManager::say_someone_cheated() {
    say_synchronous("You cheated and I did not notice!");
}


void NodeManager::say_it_is_your_turn() {
    say_synchronous("It is your turn !");
}


void NodeManager::ask_to_turn_card(reasoning::ConcealedCard card) {
    say_synchronous("Turn card at position" +
                    std::to_string(card.get_position().get_x())
                    + " " +
                    std::to_string(card.get_position().get_y()) + "!");
}

bool NodeManager::ask_to_play() {
    std::vector<std::string> possible_answers = {
            "Yes", "No"
    };

    std::string question = "You want to play a game with me?";

    nao_playing_memory::AskQuestionGoal ask_goal;
    ask_goal.possible_answers = possible_answers;
    ask_goal.question = question;

    _question_node.sendGoal(ask_goal);
    bool finished_before_timeout = _question_node.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = _question_node.getState();
        ROS_INFO("Ask Action finished: %s", state.toString().c_str());
        auto result = _question_node.getResult();
        std::cout << "\"" <<result->answer << "\"?=" << "Yes" << (result->answer == "Yes") <<  std::endl;
        return result->answer == "Yes";

    } else {
        return false;
    }
}

bool NodeManager::initialize_game_board() {

    CardStateRetriever card_state_retriever(30);

    auto cards = card_state_retriever.retrieve_current_state();

    if (!std::get<1>(cards).empty() && !std::get<2>(cards).empty()) {
        return false;
    }
    _pc.reset();

    for (auto const &card : std::get<0>(cards)) {
        _pc.save(card);
    }
    return true;
}

void NodeManager::say_synchronous(std::string text) {
    nao_playing_memory::SaySomethingGoal say_goal;

    say_goal.text = std::move(text);
    _voice_node.sendGoal(say_goal);

    bool finished_before_timeout2 = _voice_node.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout2) {
        throw new std::runtime_error("took more than 30 seconds to speak the fucking sentence");
    }
}

void NodeManager::ask_to_collect_cards(const std::vector<reasoning::ConcealedCard> &cards) {
    for (const auto &card : cards) {
        say_synchronous("Please give me the card " +
        std::to_string(card.get_position().get_x())
        + " "+
        std::to_string(card.get_position().get_y()) + "!");

        ActionBlocker(30, 5).wait_until_card_is_removed(card.get_position());
    }
}
