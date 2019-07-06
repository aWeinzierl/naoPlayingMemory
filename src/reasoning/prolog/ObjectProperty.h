#pragma once

#include <string>

#include "Instance.h"

namespace reasoning {

    struct ObjectProperty {
    private:
        std::string _name;
        Instance _value;
        
    public:
        ObjectProperty(std::string name, Instance value) noexcept;

        const std::string &get_name() const noexcept;
        const Instance& get_value() const noexcept;
        
    };

}