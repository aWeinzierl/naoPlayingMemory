#pragma once

#include <string>

#include "classes/Instance.h"

namespace reasoning {

    template<typename DataType>
    struct DataProperty {
    private:
        std::string _name;
        DataType _value;
        
    public:
        DataProperty(std::string name, DataType value) noexcept;
        
        const std::string &get_name() const noexcept;
        const DataType &get_value() const noexcept;
        
    };

}