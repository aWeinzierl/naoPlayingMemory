#include "ActionBlocker.h"

#include "FilterRecognizeTurn.h"
#include "CardStateRetriever.h"


reasoning::ExposedCard ActionBlocker::wait_until_card_is_revealed(const reasoning::CardPosition &card_pos) {
    while (true) {
        spin();
        auto actions = _sp.retrieve_actions();
        //std::cout << "retrieved actions" << std::endl;
        auto correct_action = std::find_if(
                actions.reveal_card.begin(), actions.reveal_card.end(),
                [card_pos](const reasoning::RevealCardAction &action) {
                    std::cout << "compare: " << action.get_position().get_x()<<action.get_position().get_y()<<" to: "<<card_pos.get_x()<<card_pos.get_y()<<std::endl;
                    return action.get_position()==card_pos;
                });

        if (correct_action != actions.reveal_card.end()) {
            _sub.shutdown();
            return *correct_action;
        }
    }
}

reasoning::ExposedCard ActionBlocker::wait_until_any_card_is_revealed() {
    while (true) {
        spin();
        auto actions = _sp.retrieve_actions();
        if (!actions.reveal_card.empty()) {
            if (actions.reveal_card.size() != 1) {
                throw new std::runtime_error("found two revealed cards at one");
            }

            return actions.reveal_card[0];
        }
    }
}

void ActionBlocker::spin() {
    _ros_rate.sleep();
    ros::spinOnce();
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
    _sp.process_new_state(std::get<0>(states), std::get<1>(states), std::get<2>(states));
}

ActionBlocker::ActionBlocker(unsigned int ros_rate, unsigned int persistence) :
        _ros_rate(ros_rate),
        _sp(persistence) {
    _sub = _n.subscribe("/cards", 1000, &ActionBlocker::vision_callback, this);
}

void ActionBlocker::wait_until_card_is_removed(const reasoning::CardPosition &card_position) {
    while (true) {
        spin();
        auto actions = _sp.retrieve_actions();
        if (std::find(
                actions.remove_card.begin(), actions.remove_card.end(), card_position)
            != actions.remove_card.end()) {
            return;
        }
    }
}

void ActionBlocker::wait_until_cards_removed(const std::vector<reasoning::CardPosition> &card_positions) {
    auto local_card_positions = card_positions;
    while (!local_card_positions.empty()) {
        spin();
        auto actions = _sp.retrieve_actions();
        for (auto const &card : actions.remove_card) {
            std::remove(local_card_positions.begin(), // NOLINT(bugprone-unused-return-value)
                        local_card_positions.end(),
                        card);
        }
    }
}