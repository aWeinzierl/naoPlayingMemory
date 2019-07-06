#include "FilterRecognizeTurn.h"

namespace reasoning {
    FilterRecognizeTurn::FilterRecognizeTurn(const unsigned int size, State initial_state) noexcept
            : buffer(size), _last_state(initial_state) {
    }

    bool FilterRecognizeTurn::update(const State state) {
        //filter auf die que laufen lassen
        buffer.push_back(state);
        State comparator = buffer[0];
        for (int i = 1; i < 5; i++) {
            if (comparator != buffer[i]) {
                return false;
            }
        }
        return true;
    }
}