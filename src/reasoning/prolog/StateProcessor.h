#include <string>
#include <queue> 
#include "classes/CardPosition.h"
#include <unordered_map>
#include "ExposedCard.h"
#include "ConcealedCard.h"


namespace reasoning {
    enum class state{
        concealed,
        turned,
        not_av,
    };


    static const std::unordered_map<CardPosition,std::queue<state>> mapping;

    static std::unordered_map<CardPosition,std::queue<state>> generate_queues(){

    std::unordered_map<CardPosition,std::queue<state>> mapping;

    for(unsigned int i = 0; i< 3; ++i){
        for(unsigned int j = 0; j < 4; ++j){
            mapping[CardPosition(i,j)]=std::queue<state>();
        }
    };
    }



    class StateProcessor{

        public:
            void process_new_state(const std::vector<ConcealedCard>& ConcealedCard,const std::vector<ExposedCard>& ExposedCard,const std::vector<CardPosition> NoCards);



    }




}