#include <iostream>
#include "FilterRecognizeTurn.h"

namespace reasoning {
    FilterRecognizeTurn::FilterRecognizeTurn(const unsigned int size, State initial_state) noexcept
            : _buffer(size), _last_state(initial_state) {
        std::cout<<"Here i create the filter with buffer size: "<<size<<std::endl;
    }

    bool FilterRecognizeTurn::update(const State state) {
        //filter auf die que laufen lassen
        std::cout << "im way before" <<std::endl;
        /*std::cout << _buffer.size() <<std::endl;*/
        //return true;
        _buffer.push_back(state);
        std::cout << "im in the filter" <<std::endl;

        if (all_states_are_equal()
        && _last_state != *_buffer.begin()) {
            return true;
        }

        _last_state = *_buffer.begin();
        //return false;
    }

    bool FilterRecognizeTurn::all_states_are_equal() {
        std::cout<<"i am before this: loopong all states acnd checking ==="<<std::endl;
        State first_state = *_buffer.begin();
        for (const auto &state: _buffer) {
            std::cout<<"im loopong all states acnd checking ==="<<std::endl;
            if (state != first_state) {
                return false;
            }
        }
        return true;
    }
}