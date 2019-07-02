#include <utility>

#include "ObjectProperty.h"

namespace reasoning{

    ObjectProperty::ObjectProperty(std::string name, Instance value) noexcept
    : _name(std::move(name)), _value(std::move(value)){
    }

    const std::string &ObjectProperty::get_name() const noexcept {
        return _name;
    }

    const Instance &ObjectProperty::get_value() const noexcept {
        return _value;
    }
}