#include "StateProcessor.h"
#include "State.h"

namespace reasoning {

    bool StateProcessor::state_update_triggers_filter(const CardPosition& card_position, State state){
        auto filter = _position_to_filter.find(card.get_position());
        if (filter==_position_to_filter.end()){
            throw new std::logic_error("position does not exist");
        }

        return filter->second.update(state);
    }

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
            auto triggered = state_update_triggers_filter(card.get_position(),State::EXPOSED);
            std::cout << "2" <<std::endl;
            if (triggered) {
                std::cout << "3" <<std::endl;
                _actions.reveal_card.emplace_back(card);
                std::cout << "exposed"<<std::endl;
            }
        }
        for (const auto &card: concealed_card) {
            std::cout << "1" <<std::endl;
            std::cout<<"_card position: "<<card.get_position().get_x()<<" , "<< card.get_position().get_y()<<std::endl;
            auto triggered = state_update_triggers_filter(card.get_position(),State::CONCEALED);
            std::cout << "2" <<std::endl;
            if (triggered) {
                std::cout << "3" <<std::endl;
                _actions.cover_card.emplace_back(card);
                std::cout << "concealed"<<std::endl;
            }
        }
        for (const auto &position: unknown) {
            auto triggered = state_update_triggers_filter(position, State::UNKNOWN);
            if (triggered) {
                _actions.remove_card.emplace_back(position);
                std::cout << "remove"<<std::endl;
            }
        }
        std::cout << "end updating"<<std::endl;
    }

    StateProcessor::StateProcessor(unsigned int persistence) {
        _position_to_filter = generate_filters(persistence);
        //std::cout<<"Filter map: "<<_position_to_filter<<std::endl;
    }

    ActionDetections StateProcessor::retrieve_actions() {


        std::lock_guard<std::mutex> lockGuard(_actions_mutex);
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