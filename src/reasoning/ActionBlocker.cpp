#include "ActionBlocker.h"

#include "FilterRecognizeTurn.h"
#include "CardStateRetriever.h"


void ActionBlocker::wait_until_card_is_revealed(unsigned int card_id) {
    auto sub = _n.subscribe("/cards", 1000, &ActionBlocker::vision_callback, this);
    std::cout<<"I am waiting until a card is revealed"<<std::endl;
    while (true) {
        std::cout<<"waiting"<<std::endl;
        _ros_rate.sleep();
        ros::spinOnce();
        auto actions = _sp.retrieve_actions();
        std::cout << "retrieved actions" << std::endl;
        if (std::find_if(
                actions.reveal_card.begin(), actions.reveal_card.end(),
                [card_id](const reasoning::RevealCardAction &action) {
                    std::cout << "compare" << std::endl;
                    return action.get_id()==card_id;
                }) != actions.reveal_card.end()) {
            std::cout<<"I am returning"<<std::endl;
            return;
        }
    }
}

reasoning::ExposedCard ActionBlocker::wait_until_any_card_is_revealed() {
    auto sub = _n.subscribe("/cards", 1000, &ActionBlocker::vision_callback, this);
    while (true) {
        _ros_rate.sleep();
        ros::spinOnce();
        auto actions = _sp.retrieve_actions();
        if (!actions.reveal_card.empty()) {
            if (actions.reveal_card.size()!=1){
                throw new std::runtime_error("found two revealed cards at one");
            }

            return actions.reveal_card[0];
        }
    }
}

AllCards map_card_state(const nao_playing_memory::Cards::ConstPtr &msg) {
    std::vector<reasoning::ConcealedCard> concealed_cards;
    concealed_cards.reserve(msg->concealed_card_list.size());

    for (const auto &card : msg->concealed_card_list) {
        concealed_cards.emplace_back(card.id, reasoning::CardPosition(card.position.x, card.position.y));
    }

    std::vector<reasoning::ExposedCard> exposed_cards;
    exposed_cards.reserve(msg->exposed_card_list.size());

    for (const auto &card: msg->exposed_card_list) {
        exposed_cards.emplace_back(card.class_type, card.id, reasoning::CardPosition(card.position.x, card.position.y));
    }

    std::vector<reasoning::CardPosition> invalid_positions;
    invalid_positions.reserve(msg->no_card_list.size());

    return std::make_tuple(concealed_cards, exposed_cards, invalid_positions);
}

void ActionBlocker::vision_callback(const nao_playing_memory::Cards::ConstPtr &msg) {
    auto states = map_card_state((msg));
    std::cout<<"I am in the actionblocker vision callback before processing new state"<<std::endl;
    _sp.process_new_state(std::get<0>(states), std::get<1>(states), std::get<2>(states));
}

ActionBlocker::ActionBlocker(unsigned int ros_rate, unsigned int persistence) :
        _ros_rate(ros_rate),
        _sp(persistence) {
}

void ActionBlocker::wait_until_card_is_removed(const reasoning::CardPosition &card_position) {
    auto sub = _n.subscribe("/cards", 1000, &ActionBlocker::vision_callback, this);
    while (true) {
        _ros_rate.sleep();
        ros::spinOnce();
        auto actions = _sp.retrieve_actions();
        if (std::find(
                actions.remove_card.begin(), actions.remove_card.end(), card_position)
            != actions.remove_card.end()) {
            return;
        }
    }
}

void ActionBlocker::wait_until_cards_removed(const std::vector<reasoning::CardPosition> &card_positions) {
    auto sub = _n.subscribe("/cards", 1000, &ActionBlocker::vision_callback, this);
    auto local_card_positions = card_positions;
    while (!local_card_positions.empty()) {
        _ros_rate.sleep();
        ros::spinOnce();
        auto actions = _sp.retrieve_actions();
        for(auto const& card : actions.remove_card){
            std::remove(local_card_positions.begin(),local_card_positions.end(), card); // NOLINT(bugprone-unused-return-value)
        }
    }
}
