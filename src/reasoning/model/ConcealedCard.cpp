#include "ConcealedCard.h"

namespace reasoning {
    const CardPosition &ConcealedCard::get_position() const noexcept {
        return _position;
    }

    unsigned int ConcealedCard::get_id() const noexcept {
        return _id;
    }

    ConcealedCard::ConcealedCard(unsigned int id, CardPosition position) noexcept : _id(id), _position(position) {}
}