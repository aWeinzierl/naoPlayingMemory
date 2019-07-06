#include "StateProcessor.h"
#include "State.h"

namespace reasoning {

    void StateProcessor::process_new_state(const std::vector<ConcealedCard> &concealed_card,
                                           const std::vector<ExposedCard> &exposed_card,
                                           const std::vector<CardPosition> &unknown) {

        ActionDetections actions;
        //analise State from cards
        for (const auto &card: exposed_card) {
            auto triggered = _position_to_filter.find(card.get_position())->second.update(State::EXPOSED);
            if (triggered) {
                actions.reveal_card.emplace_back(card);
            }
        }
        for (const auto &card: concealed_card) {
            auto triggered = _position_to_filter.find(card.get_position())->second.update(State::CONCEALED);
            if (triggered) {
                actions.cover_card.emplace_back(card);
            }
        }
        for (const auto &position: unknown) {
            auto triggered = _position_to_filter.find(position)->second.update(State::UNKNOWN);
            if (triggered) {
                actions.remove_card.emplace_back(position);
            }
        }
    }

    StateProcessor::StateProcessor() {
        _position_to_filter = generate_filters();
    }
}