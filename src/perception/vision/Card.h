#pragma once

#include <string>

struct Card {

    bool turned;
    bool visible;
    unsigned int aruco_id_top;
    unsigned int aruco_id_bottom;
    std::string object_type;
};



