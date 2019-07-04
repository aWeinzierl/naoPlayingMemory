#pragma once

#include <string>

#include "CardPosition.h"

namespace reasoning {

    struct Card {
    private:
        unsigned int _id;
        CardPosition _position;
        std::string _class;
        

    public:
        Card(unsigned int id, CardPosition position, std::string classType);

        unsigned  int get_id() const noexcept;

        const CardPosition& get_position() const noexcept;

        const std::string& get_class() const noexcept;
    };
}
