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

    bool operator==(const CardPosition &a, const CardPosition &b) {
        return a.get_x() == b.get_x() && a.get_y() == b.get_y();

    }
}