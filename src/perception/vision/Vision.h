#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <json_prolog/prolog.h>
#include <unordered_set>

#include "GridBoard.h"
#include "GridElement.h"
#include <cv.h>

namespace perception{
    class VisionClient {
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

        aruco::CameraParameters CameraParameters;
        aruco::MarkerDetector markerDetector;
        std::vector<aruco::Marker>markers;

        cv::Mat InImage;

        json_prolog::Prolog pl;


        int timepoint;
        double distance_thresh;
        bool game_initialized;

        GridBoard grid_board;
        std::unordered_set<unsigned int> top_ids;
        std::unordered_map<unsigned int, string> card_classes;



    public:
        explicit VisionClient(ros::NodeHandle& nodeHandle);
        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        void detect_cards();

    private:
        void initialize_game_grid();
        void check_cards();
        std::vector<GridElement> retrieve_edge_cards(const std::vector<GridElement> card_collection);
        Position get_relative_position(cv::Mat rvec, cv::Mat tvec);
        Position get_center(cv::Point r, cv::Point g, cv::Point b);
        GridElement find_closest_card(std::vector<GridElement> elements, Position center);
        GridElement find_closest_card_w_thresh(std::vector<GridElement> elements, Position center);

    };
}
