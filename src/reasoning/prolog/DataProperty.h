#pragma once

#include <string>

#include "classes/Instance.h"

namespace reasoning {

    struct DataProperty {
    private:
        std::string _name;
        uint _value;
        
    public:
        DataProperty(std::string name,uint value) noexcept;
        
        const std::string &get_name() const noexcept;
        const uint &get_value() const noexcept;
        
    };

}