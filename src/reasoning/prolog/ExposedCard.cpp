#include <utility>

#include "ExposedCard.h"

namespace reasoning {

    ExposedCard::ExposedCard(std::string classType, unsigned int id, CardPosition position) noexcept
            : _class(std::move(classType)), _id(id), _position(position) {
    }

    const CardPosition &ExposedCard::get_position() const noexcept {
        return _position;
    }

    unsigned int ExposedCard::get_id() const noexcept {
        return _id;
    }

    const std::string &ExposedCard::get_class() const noexcept {
        return _class;
    }
}