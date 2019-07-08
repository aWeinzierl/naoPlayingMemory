#include <utility>

#include "NodeManager.h"

#include <tuple>

#include "NaoState.h"
#include "model/ConcealedCard.h"
#include "CardStateRetriever.h"

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
          _voice_node("say_something", true){
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
    NaoState nao_state;

    while (true) {
        while (nao_state._is_bored) {
            auto wants_play = ask_to_play();
            if (wants_play) {
                nao_state._is_bored = false;
            }
            ros::Duration(5).sleep();
        }

        if (!initialize_game_board()){
            say_synchronous("leg the fucking cards onto the board correctly");
            nao_state._is_bored=true;
        }

        _pc.reset();


    }
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

    if (!std::get<1>(cards).empty() && !std::get<2>(cards).empty()){
        return false;
    }

    for ( auto const & card : std::get<0>(cards)){
        _pc.save(card);
    }
}

void NodeManager::say_synchronous(std::string text) {
    nao_playing_memory::SaySomethingGoal say_goal;

    say_goal.text = std::move(text);
    _voice_node.sendGoal(say_goal);

    bool finished_before_timeout2 = _voice_node.waitForResult(ros::Duration(30.0));

    if(!finished_before_timeout2){
        throw new std::runtime_error("took more than 30 seconds to speak the fucking sentence");
    }
}
