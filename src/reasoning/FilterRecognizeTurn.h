#pragma once

#include <boost/circular_buffer.hpp>
#include <nonstd/optional.hpp>

#include "State.h"

namespace reasoning {
    class FilterRecognizeTurn {

        boost::circular_buffer<State> _buffer;
        State _last_state;

        /// check if all values in the ring are equal
        /// \return
        bool all_states_are_equal();

    public:
        /// constructs a working object
        /// \param size if that many times the filter has seen the value, it assumes that value
        /// \param initial_state assumed value at the beginning
        explicit FilterRecognizeTurn(unsigned int size, State initial_state) noexcept;

        /// pass the filter a new state
        /// \param state state which is being passed
        /// \return if the filter detected a changed state
        bool update(State state);
    };
}