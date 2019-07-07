#include "NodeManager.h"


void NodeManager::tick() {

    std::cout << "tick" << std::endl;

    auto actions = _sp.retrieve_actions();



    _timer.expires_at(_timer.expires_at() + _interval);
    _timer.async_wait(boost::bind(&NodeManager::tick, this));
}

NodeManager::NodeManager()
        : _interval(1), _timer(_io_service, _interval) {
    _sub = _n.subscribe("cards", 1000, &NodeManager::vision_callback, this);

    _timer.async_wait(boost::bind(&NodeManager::tick, this));
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
