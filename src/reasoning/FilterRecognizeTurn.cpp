#include "FilterRecognizeTurn.h"

namespace reasoning {
    FilterRecognizeTurn::FilterRecognizeTurn(const unsigned int size, State initial_state) noexcept
            : _buffer(size), _last_state(initial_state) {
    }

    bool FilterRecognizeTurn::update(const State state) {
        //filter auf die que laufen lassen
        _buffer.push_back(state);

        if (all_states_are_equal()
            && _last_state != _buffer[0]) {
            return true;
        }

        _last_state = _buffer[0];
    }

    bool FilterRecognizeTurn::all_states_are_equal() {
        State first_state = _buffer[0];
        for (const auto &state: _buffer) {
            if (state != first_state) {
                return false;
            }
        }
        return true;
    }
}