#pragma once

#include <string>
#include <queue>
#include <unordered_map>
#include <mutex>

#include <boost/circular_buffer.hpp>

#include "FilterRecognizeTurn.h"
#include "prolog/PrologClient.h"
#include "State.h"

namespace reasoning {

    struct ActionDetections {
        std::vector<RevealCardAction> reveal_card;
        std::vector<CoverCardAction> cover_card;
        std::vector<RemoveCardAction> remove_card;
    };

    struct Hash {
        std::size_t operator()(const CardPosition &cardPosition) const {
            return (cardPosition.get_x() << 1u) ^ cardPosition.get_y();
        }
    };

    class StateProcessor {

        ActionDetections _actions;
        std::mutex _actions_mutex;

        std::unordered_map<CardPosition, FilterRecognizeTurn, Hash> _position_to_filter;

        std::unordered_map<CardPosition, FilterRecognizeTurn, Hash> generate_filters() {
            for (unsigned int i = 0; i < 3; ++i) {
                for (unsigned int j = 0; j < 4; ++j) {
                    auto cardPosition = CardPosition(i, j);
                    auto filter = FilterRecognizeTurn(5, State::CONCEALED);
                    _position_to_filter.insert({cardPosition, filter});
                }
            }
        }

    public:
        StateProcessor();

        void process_new_state(const std::vector<ConcealedCard> &concealed_card,
                               const std::vector<ExposedCard> &exposed_card,
                               const std::vector<CardPosition> &unknown);

        ActionDetections retrieve_actions();

        void reset_found_actions();

    };


}