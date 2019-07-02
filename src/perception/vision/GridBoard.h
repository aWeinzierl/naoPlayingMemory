#pragma once

#include <vector>
#include "GridElement.h"

struct Position{
    float _x,_y;
};

struct BoardMarker{
    Position _position;
    unsigned int _aruco_id;
};

struct GridBoard {
    BoardMarker _left_marker;
    BoardMarker _right_marker;

    std::vector<std::vector<GridElement>> grid;
};