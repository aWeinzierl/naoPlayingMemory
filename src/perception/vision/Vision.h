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

#include "GridBoard.h"

namespace perception{
    class VisionClient {
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

        aruco::CameraParameters CameraParameters;
        aruco::MarkerDetector markerDetector;
        std::vector<aruco::Marker>markers;

        cv::Mat InImage;
        cv::Mat card_positions;
        cv::Mat game_grid;

        json_prolog::Prolog pl;

        map<string, int> classes;
        set<int> stored_markers;

        int timepoint;
        float position_thresh;

        bool game_initialized;

        GridBoard grid_board;


    public:
        explicit VisionClient(ros::NodeHandle& nodeHandle);
        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        void detect_cards();

    private:
        void initialize_game_grid();
        std::vector<GridElement> retrieve_edge_cards(const std::vector<GridElement> card_collection);
    };
}
