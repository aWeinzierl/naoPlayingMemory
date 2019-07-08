#include "ActionBlocker.h"

void ActionBlocker::wait_until_card_is_revealed(unsigned int card_id) {
    _object_of_desire = reasoning::ConcealedCard(card_id, reasoning::CardPosition(0,0));
    auto sub = _n.subscribe("cards", 1000, &ActionBlocker::reveal_card_callback, this);
    while (!_has_found_action) {
        _ros_rate.sleep();
    }
}

void ActionBlocker::reveal_card_callback(const nao_playing_memory::Cards::ConstPtr &msg) {
    std::vector<reasoning::ExposedCard> exposed_cards;
    exposed_cards.reserve(msg->exposed_card_list.size());

    for (const auto &card: msg->exposed_card_list) {
        if (card.id == _object_of_desire.get_id()) {
            _has_found_action = true;
        }
    }
}


void ActionBlocker::remove_card_callback(const nao_playing_memory::Cards::ConstPtr &msg) {
    std::vector<reasoning::ExposedCard> exposed_cards;
    exposed_cards.reserve(msg->no_card_list.size());

    for (const auto &position: msg->no_card_list) {
        auto card_position = reasoning::CardPosition(position.x, position.y);
        if (card_position == _object_of_desire.get_position()) {
            _has_found_action = true;
        }
    }
}

ActionBlocker::ActionBlocker(unsigned int ros_rate) :
        _ros_rate(ros_rate),
        _has_found_action(0),
        _object_of_desire(0,{0,0}){
}

void ActionBlocker::wait_until_card_is_removed(reasoning::CardPosition card_position) {
    _object_of_desire = reasoning::ConcealedCard(0, card_position);
    auto sub = _n.subscribe("cards", 1000, &ActionBlocker::remove_card_callback, this);
    while (!_has_found_action) {
        _ros_rate.sleep();
    }
}