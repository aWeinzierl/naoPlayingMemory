#include <string>
#include <queue> 
#include "prolog/classes/CardPosition.h"
#include <unordered_map>
#include "prolog/ExposedCard.h"
#include "prolog/ConcealedCard.h"
#include <boost/circular_buffer.hpp>
#include "FilterRecognizeTurn.h"
#include "prolog/PrologClient.h"
#include "State.h"



namespace reasoning {


    std::unordered_map<CardPosition,FilterRecognizeTurn> mapping;

    static std::unordered_map<CardPosition,FilterRecognizeTurn> generate_filterRecognizeTurn(){

    std::unordered_map<CardPosition,FilterRecognizeTurn> mapping;

    for(unsigned int i = 0; i< 3; ++i){
        for(unsigned int j = 0; j < 4; ++j){
            mapping[CardPosition(i,j)]=FilterRecognizeTurn(5);
        }
    };
    }



    class StateProcessor{

        public:
            void process_new_state(const std::vector<ConcealedCard>& ConcealedCard,const std::vector<ExposedCard>& ExposedCard,const std::vector<CardPosition> NoCards);



    }




}