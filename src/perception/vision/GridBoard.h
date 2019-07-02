#pragma once

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "GridElement.h"

struct BoardMarker{
    cv::Mat _rvec;
    cv::Mat _tvec;
    unsigned int _aruco_id;
};

struct GridBoard {
    BoardMarker _left_marker;
    BoardMarker _right_marker;

    std::vector<std::vector<GridElement>> grid;
};