#pragma once

#include <string>

struct Card {

    bool turned;
    unsigned int aruco_id_top;
    unsigned int aruco_id_bottom;
    std::string object_type;
};



