#include "CardPosition.h"

namespace reasoning {

    unsigned int CardPosition::get_x() const noexcept {
        return _x;
    }

    unsigned int CardPosition::get_y() const noexcept {
        return _y;
    }

    CardPosition::CardPosition(unsigned int x, unsigned int y) noexcept
            : _x(x), _y(y) {
    }
}