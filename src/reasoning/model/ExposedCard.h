#include <string>
#include "CardPosition.h"

namespace reasoning {

    struct ExposedCard {
    private:
        CardPosition _position;
        unsigned int _id;
        std::string _class;

    public:

        ExposedCard(std::string classType, unsigned int id, CardPosition position) noexcept;

        const std::string &get_class() const noexcept;

        unsigned int get_id() const noexcept;

        const CardPosition &get_position() const noexcept;
    };
}