#include "NodeManager.h"

#include <tuple>
#include <utility>

#include "model/ConcealedCard.h"
#include "CardStateRetriever.h"

void ask_to_turn_card(reasoning::CardPosition card_position);

NodeManager::NodeManager()
        : _question_node("ask_question", true),
          _voice_node("say_something2", true) {

    _question_node.waitForServer();
    _voice_node.waitForServer();

    card_classes = {
            {15, "Strawberry"},
            {16, "Strawberry"},
            {17, "Pizza"},
            {18, "Pizza"},
            {25, "Ninja"},
            {26, "Ninja"},
            {20, "Penguin"},
            {19, "Penguin"},
            {23, "Octopus"},
            {24, "Octopus"},
            {21, "Nao"},
            {22, "Nao"},
    };
}

reasoning::CardPosition NodeManager::cardPositionMap(const nao_playing_memory::Position &position) {
    return {
            position.x,
            position.y
    };
}

void NodeManager::surrect() {
    ros::Rate rate(30);
    unsigned int nao_points = 0;
    unsigned int opponent_points = 0;
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
                std::cout << "I will play a game" << std::endl;
            }

        }

        if (!initialize_game_board()) {
            std::cout << "The board looks bad, please reposition it" << std::endl;
            say_synchronous("Please put the cards correctly onto the board");
            nao_is_bored = true;
        }

        auto nao_is_in_charge = true;

        auto game_is_running = true;
        //TODO CREATE INITIAL INSTANCES FROM CARDS WITH void PrologClient::save(const ConcealedCard &concealed_card)
        while (game_is_running) {
            std::cout << "Game is Running" << std::endl;
            rate.sleep();
            if (nao_is_in_charge) {
                say_synchronous("It is my turn");
                ros::Duration(1).sleep();
                std::cout << "It is my turn" << std::endl;
                auto cards_to_turn = _pc.decide_action();
                if (cards_to_turn.size() == 2) {
                    std::cout<<"Telling to turn cards"<<std::endl;
                    for (const auto &card: cards_to_turn) {
                        ask_to_turn_card(card);
                        //say_synchronous(""+ + "")
                        ActionBlocker(30,5).wait_until_card_is_revealed(card.get_position());
                    }

                    ros::Duration(2).sleep();
                    ask_to_collect_cards(cards_to_turn);

                    _pc.delete_cards(cards_to_turn[0].get_id(), cards_to_turn[1].get_id());

                    nao_points += 1;
                } else {

                    std::cout << "i choose  card: " << std::to_string(cards_to_turn[0].get_id()) << std::endl;
                    auto random_card = cards_to_turn[0];
                    ask_to_turn_card(random_card);
                    auto revealed_card = ActionBlocker(30, 5).wait_until_card_is_revealed(random_card.get_position());
                    _pc.save_action(revealed_card);
                    ros::Duration(2).sleep();
                    auto card = _pc.search_paired_card(random_card);

                    if (card.has_value()) {

                        std::cout << "Card:" << card.value().get_id() << std::endl;
                        std::cout<<"Turn card"<<std::endl;
                        ask_to_turn_card(card.value());
                        ActionBlocker(30,5).wait_until_card_is_revealed(card.value().get_position());

                        ask_to_collect_cards({random_card, card.value()});

                        _pc.delete_cards(random_card.get_id(), card.value().get_id());
                        nao_points += 1;

                    } else {

                        std::cout << "I dont know any card with the same class" << std::endl;
                        auto second_random_card = _pc.search_random_card();
                        if (!second_random_card.has_value()) {
                            say_someone_cheated();
                            game_is_running = false;
                            break;
                        }
                        std::cout<<"turn card"<<std::endl;
                        ask_to_turn_card(second_random_card.value());
                        auto revealed_card2 = ActionBlocker(30, 15).wait_until_card_is_revealed(
                                second_random_card.value().get_position());
                        _pc.save_action(revealed_card2);
                        ros::Duration(0.5).sleep();
                        auto card3 = _pc.search_if_paired_card(second_random_card.value(),random_card);
                        if (card3.has_value()) {
                            ask_to_collect_cards({random_card, second_random_card.value()});
                            std::cout << "i am waiting for someone to collect the cards" << std::endl;
                            _pc.delete_cards(random_card.get_id(), second_random_card.value().get_id());

                            nao_points += 1;
                        } else {
                            say_synchronous(" No Luck for me , turn them back around ");
                            std::cout<<" No luck for me"<<std::endl;
                            ActionBlocker(30, 5).wait_until_cards_covered(
                                    {random_card.get_id(), second_random_card.value().get_id()});
                            nao_is_in_charge = false;
                        }
                    }
                }
            }
            if (opponent_points > 3 || nao_points > 3 || nao_points + opponent_points >= 6) {
                game_is_running = false;
                nao_is_bored = true;
                std::cout<<"game ended"<<std::endl;
                if (opponent_points > 3) {
                    say_synchronous(" God job, but next time I will win! ");
                } else if (nao_points > 3) {
                    say_synchronous(" Looser ! There is no shame in loosing against the master!");
                } else {
                    say_synchronous(" We are on the same Level ,  You are also a Master");
                }
                //_pc.reset();
                break;
            }
            if (!nao_is_in_charge) {
                say_it_is_your_turn();
                std::cout<<"Players turn"<<std::endl;

                auto cards = ActionBlocker(30, 15).wait_until_enough_cards_revealed(2);
                std::cout<<"Card1"<<cards[0].get_class()<<std::endl;
                std::cout<<"Card2"<<cards[1].get_class()<<std::endl;

                _pc.save_action(cards[0]);
                _pc.save_action(cards[1]);


                bool equal_cards = _pc.are_cards_equal(cards[0].get_id(), cards[1].get_id());
                if (equal_cards) {
                    std::cout<<"Oponent won pair"<<std::endl;
                    say_synchronous("Take your Cards");
                    ActionBlocker(30, 45).wait_until_cards_removed({
                                                                           cards[0].get_position(),
                                                                           cards[1].get_position()
                                                                   });
                    _pc.delete_cards(cards[0].get_id(), cards[1].get_id());
                    opponent_points += 1;
                } else {
                    std::cout<<"oponent had no luck"<<std::endl;
                    say_synchronous("ha ha ha no success for you, turn the cards around");
                    ActionBlocker(30, 5).wait_until_cards_covered({cards[0].get_id(), cards[1].get_id()});
                    nao_is_in_charge = true;
                }
            }

            if (opponent_points > 3 || nao_points > 3 || nao_points + opponent_points >= 6) {
                game_is_running = false;
                nao_is_bored = true;
                std::cout<<"game ended"<<std::endl;
                if (opponent_points > 3) {
                    say_synchronous("God job, but next time I will win!");
                } else if (nao_points > 3) {
                    say_synchronous(" Looser ! There is no shame in loosing against the master!");
                } else {
                    say_synchronous(" We are on the same Level ,  You are also a Master ");
                }
                //_pc.reset();
                break;
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
        std::cout << "\"" << result->answer << "\"?=" << "Yes" << (result->answer == "Yes") << std::endl;
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
    //_pc.reset();

    for (auto const &card : std::get<0>(cards)) {
        _pc.save(card);
    }
    return true;
}

void NodeManager::say_synchronous(std::string text) {
    std::cout<<"starting to speak"<<std::endl;
    nao_playing_memory::SaySomethingGoal say_goal;

    say_goal.text = std::move(text);
    _voice_node.sendGoal(say_goal);

    bool finished_before_timeout2 = _voice_node.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout2) {
        throw new std::runtime_error("took more than 30 seconds to speak the sentence");
    }
    std::cout<<"ending to speak"<<std::endl;
}

void NodeManager::ask_to_collect_cards(const std::vector<reasoning::ConcealedCard> &cards) {

    say_synchronous("Please give me the cards of class " + card_classes.find(cards[0].get_id())->second + "  in Positions" +
                    std::to_string(cards[0].get_position().get_x()) + " " +
                    std::to_string(cards[0].get_position().get_y()) + " and " +
                    std::to_string(cards[1].get_position().get_x()) + " " +
                    std::to_string(cards[1].get_position().get_y()) +
                    "!");

    ActionBlocker(30, 45).wait_until_cards_removed({cards[0].get_position(), cards[1].get_position()});

}
