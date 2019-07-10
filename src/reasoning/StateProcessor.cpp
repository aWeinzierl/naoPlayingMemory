#include "StateProcessor.h"
#include "State.h"

namespace reasoning {

    bool StateProcessor::state_update_triggers_filter(const CardPosition& card_position, State state){


        auto filter = _position_to_filter.find(card_position);

        if (filter==_position_to_filter.end()){
            throw new std::logic_error("position does not exist");
        }

        return filter->second.update(state);
    }

    void StateProcessor::process_new_state(const std::vector<ConcealedCard> &concealed_card,
                                           const std::vector<ExposedCard> &exposed_card,
                                           const std::vector<CardPosition> &unknown) {


        std::lock_guard<std::mutex> lockGuard(_actions_mutex);



        for (const auto &card: exposed_card) {
            auto triggered = state_update_triggers_filter(card.get_position(),State::EXPOSED);
            if (triggered) {
                _actions.reveal_card.emplace_back(card);


            }
        }
        for (const auto &card: concealed_card) {
            //auto triggered =_position_to_filter.find(card.get_position())->second.update(State::CONCEALED);
            auto triggered = state_update_triggers_filter(card.get_position(),State::CONCEALED);
            if (triggered) {
                _actions.cover_card.emplace_back(card);
            }
        }
        for (const auto &position: unknown) {
            auto triggered = state_update_triggers_filter(position, State::UNKNOWN);
            if (triggered) {
                _actions.remove_card.emplace_back(position);
            }
        }
    }

    StateProcessor::StateProcessor(unsigned int persistence) {
        _position_to_filter = generate_filters(persistence);
        //std::cout<<"Filter map: "<<_position_to_filter<<std::endl;
    }

    ActionDetections StateProcessor::retrieve_actions() {

        std::lock_guard<std::mutex> lockGuard(_actions_mutex);
        auto actions = _actions;
        //std::cout << "retrieving" << std::endl;
        _actions = ActionDetections();

        return actions;
    }

    void StateProcessor::reset_found_actions() {
        _actions = ActionDetections();
    }

    std::unordered_map<CardPosition, FilterRecognizeTurn, Hash>StateProcessor::generate_filters(unsigned int persistence) {
        {
            for (unsigned int i = 1; i < 5; ++i) {
                for (unsigned int j = 1; j < 4; ++j) {
                    auto cardPosition = CardPosition(i, j);
                    auto filter = FilterRecognizeTurn(persistence, State::CONCEALED);
                    _position_to_filter.insert({cardPosition, filter});
                }
            }

            return _position_to_filter;
        }
    }


}