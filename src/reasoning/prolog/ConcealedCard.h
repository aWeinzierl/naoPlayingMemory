#pragma once

#include "classes/CardPosition.h"

namespace reasoning {
    struct ConcealedCard {
    private:
        CardPosition _position;
        unsigned int _id;

    public:
        ConcealedCard(unsigned int id, CardPosition position) noexcept;

        unsigned int get_id() const noexcept;

        const CardPosition &get_position() const noexcept;
    };
}