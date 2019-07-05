#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Card.h"
struct Position {
    float x;
    float y;
};

struct GridElement {
    Card card;
    cv::Mat _tvec;
    cv::Mat _rvec;

    Position _im_pos;
};