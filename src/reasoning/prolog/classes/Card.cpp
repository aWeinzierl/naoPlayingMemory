#include "Card.h"

namespace reasoning {

    Card::Card(unsigned int id, CardPosition position, std::string classType)
            : _id(id), _position(position), _class(classType) {
    }

    unsigned int Card::get_id() const noexcept {
        return _id;
    }

    const CardPosition &Card::get_position() const noexcept{
        return _position;
    }

    const std::string &Card::get_class() const noexcept{
        return _class;
    }

}