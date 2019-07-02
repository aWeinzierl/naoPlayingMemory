#pragma once

#include <unordered_map>

namespace reasoning {

    enum ObjectProperty{
        hasPosition,
        hasTimeStamp
    };

    enum DataProperty{

    };


    std::unordered_map<ObjectProperty, std::string> object_property_to_string{
            {ObjectProperty ::hasTimeStamp, "hasTimeStamp"},
            {ObjectProperty ::hasPosition, "hasPosition"}
    };
}