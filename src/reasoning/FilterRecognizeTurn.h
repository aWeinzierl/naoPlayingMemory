#pragma once

#include <boost/circular_buffer.hpp>
#include "State.h"
#include <nonstd/optional.hpp>

namespace reasoning {


    class FilterRecognizeTurn {

        boost::circular_buffer<State> buffer;
        State _last_state;
    public:

        explicit FilterRecognizeTurn(unsigned int size, State initial_state) noexcept;

        //return optional from action
        bool update(State state);

    };


}