#pragma once
#include <boost/circular_buffer.hpp>
#include "State.h"
#include <nonstd/optional.hpp>
#include "TurnCard.h"

namespace reasoning{


    class FilterRecognizeTurn{
   
        boost::circular_buffer<state> buffer;
        state _last_state;
    public:

        FilterRecognizeTurn(const unsigned int size) noexcept;
        //return optional from action
        bool update(const state state);
        
    };

                    

}