#include "CardStateRetriever.h"

AllCards CardStateRetriever::retrieve_current_state() {
    auto sub = _n.subscribe("cards", 1000, &CardStateRetriever::vision_callback, this);
    while(!_cards.has_value()){
        _ros_rate.sleep();
    }

    return _cards.value();
}

void CardStateRetriever::vision_callback(const nao_playing_memory::Cards::ConstPtr &msg) {

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

    _cards = std::make_tuple(
            concealed_cards, exposed_cards, invalid_positions);
}

CardStateRetriever::CardStateRetriever(unsigned int ros_rate) : _ros_rate(ros_rate) {
}
