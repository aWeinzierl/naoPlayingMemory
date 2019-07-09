#include "StateProcessor.h"
#include "State.h"

namespace reasoning {

    void StateProcessor::process_new_state(const std::vector<ConcealedCard> &concealed_card,
                                           const std::vector<ExposedCard> &exposed_card,
                                           const std::vector<CardPosition> &unknown) {

        std::lock_guard<std::mutex> lockGuard(_actions_mutex);
        //analise State from cards
        std::cout << "code1" <<std::endl;
        std::cout << concealed_card.size() << std::endl;
        std::cout << "code2" <<std::endl;


        std::cout << "start updating"<<std::endl;
        for (const auto &card: exposed_card) {
            std::cout << "1" <<std::endl;
            auto triggered = _position_to_filter.find(card.get_position())->second.update(State::EXPOSED);
            std::cout << "2" <<std::endl;
            if (triggered) {
                std::cout << "3" <<std::endl;
                _actions.reveal_card.emplace_back(card);
                std::cout << "exposed"<<std::endl;
            }
        }
        for (const auto &card: concealed_card) {
            std::cout << "1" <<std::endl;
            auto triggered = _position_to_filter.find(card.get_position())->second.update(State::CONCEALED);
            std::cout << "2" <<std::endl;
            if (triggered) {
                std::cout << "3" <<std::endl;
                _actions.cover_card.emplace_back(card);
                std::cout << "concealed"<<std::endl;
            }
        }
        for (const auto &position: unknown) {
            auto triggered = _position_to_filter.find(position)->second.update(State::UNKNOWN);
            if (triggered) {
                _actions.remove_card.emplace_back(position);
                std::cout << "remove"<<std::endl;
            }
        }
        std::cout << "end updating"<<std::endl;
    }

    StateProcessor::StateProcessor(unsigned int persistence) {
        _position_to_filter = generate_filters(persistence);
    }

    ActionDetections StateProcessor::retrieve_actions() {


        //std::lock_guard<std::mutex> lockGuard(_actions_mutex);
        std::cout << "start retrieving"<<std::endl;
        auto actions = _actions;
        std::cout << "end retrieving" << std::endl;
        _actions = ActionDetections();

        return actions;
    }

    void StateProcessor::reset_found_actions() {
        _actions = ActionDetections();
    }
}