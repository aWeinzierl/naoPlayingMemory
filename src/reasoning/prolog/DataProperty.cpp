#include <utility>

#include "DataProperty.h"

namespace reasoning{

    DataProperty::DataProperty(std::string name, Data value) noexcept
    : _name(name), _value(value){
    }

    const std::string &DataProperty::get_name() const noexcept {
        return _name;
    }

    const uint &DataProperty::get_value() const noexcept {
        return _value;
    }

    DataProperty<std::string>;
    DataProperty<uint>;
}

