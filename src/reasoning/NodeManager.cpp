#include "NodeManager.h"

#include <tuple>
#include <utility>

#include "model/ConcealedCard.h"
#include "CardStateRetriever.h"

void ask_to_turn_card(reasoning::CardPosition card_position);

void NodeManager::execute_every_second() {

    std::cout << "tick" << std::endl;

    auto actions = _sp.retrieve_actions();


    _timer.expires_at(_timer.expires_at() + _interval);
    _timer.async_wait(boost::bind(&NodeManager::execute_every_second, this));
}

NodeManager::NodeManager()
        : _interval(1),
          _timer(_io_service, _interval),
          _question_node("ask_question", true),
          _voice_node("say_something", true) {
    _sub = _n.subscribe("cards", 1000, &NodeManager::vision_callback, this);
    _timer.async_wait(boost::bind(&NodeManager::execute_every_second, this));


    _question_node.waitForServer();
    _voice_node.waitForServer();
}

reasoning::CardPosition NodeManager::cardPositionMap(const nao_playing_memory::Position &position) {
    return {
            position.x,
            position.y
    };
}

void NodeManager::vision_callback(const nao_playing_memory::Cards::ConstPtr &msg) {
    std::vector<reasoning::ConcealedCard> concealed_cards;
    concealed_cards.reserve(msg->concealed_card_list.size());

    for (const auto &card : msg->concealed_card_list) {
        concealed_cards.emplace_back(card.id, cardPositionMap(card.position));
    }

    std::vector<reasoning::ExposedCard> exposed_cards;
    exposed_cards.reserve(msg->exposed_card_list.size());

    for (const auto &card: msg->exposed_card_list) {
        exposed_cards.emplace_back(card.class_type, card.id, cardPositionMap(card.position));
    }

    std::vector<reasoning::CardPosition> invalid_positions;
    invalid_positions.reserve(msg->no_card_list.size());

    std::transform(msg->no_card_list.begin(), msg->no_card_list.end(), std::back_inserter(invalid_positions),
                   [](const nao_playing_memory::Position &key_value_pair) {
                       return cardPositionMap(key_value_pair);
                   });

    _sp.process_new_state(concealed_cards, exposed_cards, invalid_positions);
}

void NodeManager::surrect() {
    ros::Rate rate(30);

    auto nao_is_bored = true;
    while (true) {
        while (nao_is_bored) {
            auto wants_play = ask_to_play();
            if (wants_play) {
                nao_is_bored = false;
            }
            ros::Duration(5).sleep();
        }

        if (!initialize_game_board()) {
            say_synchronous("leg the fucking cards onto the board correctly");
            nao_is_bored = true;
        }

        auto nao_is_in_charge = true;

        auto game_is_running = true;
        while (game_is_running) {
            if (nao_is_in_charge) {
                auto cards_to_turn = _pc.decide_action();

                if (cards_to_turn.size() == 2) {
                    for (const auto &card: cards_to_turn) {
                        ask_to_turn_card(card);
                    }
                    ask_to_collect_cards(cards_to_turn);
                } else {
                    auto random_card = cards_to_turn[0];
                    ask_to_turn_card(random_card);
                    ActionBlocker(30).wait_until_card_is_revealed(random_card.get_id());
                    ros::Duration(0.1).sleep();
                    auto card = _pc.search_paired_card(random_card);
                    if (card.has_value()) {
                        ask_to_turn_card(card.value());
                        ask_to_collect_cards({random_card,card.value()});
                    } else {
                        auto second_random_card = _pc.search_random_card();
                        ask_to_turn_card(second_random_card);
                        ActionBlocker(30).wait_until_card_is_revealed(second_random_card.get_id());
                        ros::Duration(0.1).sleep();
                        _pc.search_paired_card(random_card);
                        if (random_card.get_id() == second_random_card.get_id()) {
                            ask_to_collect_cards({random_card, second_random_card});
                        } else {
                            nao_is_in_charge = false;
                        }
                    }
                }

                if(!nao_is_in_charge){

                }
            }
        }
    }
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

        return state.getText() == "Yes";

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

        ActionBlocker(30).wait_until_card_is_removed(card.get_position());
    }
}
