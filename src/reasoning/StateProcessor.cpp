#include "StateProcessor.h"
#include "State.h"


namespace reasoning{
    void update_state()

    void StateProcessor::process_new_state(const std::vector<ConcealedCard>& ConcealedCard,const std::vector<ExposedCard>& ExposedCard,const std::vector<CardPosition>& NoCards)

        Actions actions;
        //analise State from cards
        for(const auto card&: ExposedCard){
            auto triggered =mapping.find(card.get_Position())->second.update(state::exposed)){
            if (triggered) {
                actions.turned_cards.emplace_back(card));
            }
        }
        for(const auto card&: ConcealedCard){
            auto triggered =mapping.find(card.get_Position())->second.update(state::concealed)){
            if (triggered) {
                actions.turned_cards.emplace_back(card));
            }
        }
        for(const auto card&: NoCard){
            auto triggered =mapping.find(card.get_Position())->second.update(state::not_av)){
            if (triggered) {
                actions.turned_cards.emplace_back(card));
            }
        }
    }
}