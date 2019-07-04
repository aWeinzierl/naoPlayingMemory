#include <utility>

#include "ObjectProperty.h"

namespace reasoning{

    DataProperty::DataProperty(std::string name, uint value) noexcept
    : _name(std::move(name)), _value(std::move(value)){
    }

    const std::string &DataProperty::get_name() const noexcept {
        return _name;
    }

    const uint &DataProperty::get_value() const noexcept {
        return _value;
    }

}