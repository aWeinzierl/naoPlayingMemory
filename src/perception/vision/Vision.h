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
        ros::Publisher cards_pub;

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
        std::unordered_map<unsigned int, unsigned int> top_back_map;



    public:
        ///
        /// \param nodeHandle
        explicit VisionClient(ros::NodeHandle& nodeHandle);

        /// callback used to get the image of the camera
        /// \param msg
        void image_callback(const sensor_msgs::ImageConstPtr& msg);

        ///function that calls the initialize game grid function and when its initialized it keeps on detecting the cards
        void detect_cards();

    private:

        ///checks if all cards are turned over and visible, then builds the card grid
        void initialize_game_grid();

        ///continuously checks the cards, builds an internal collection of concealed-, revealed- and not available cards
        void check_cards();

        ///publishes the cards messages to cards node
        void publish_cards();

        ///
        /// \param card_collection
        /// \return
        std::vector<GridElement> retrieve_edge_cards(const std::vector<GridElement> card_collection);

        /// calculate the center position of a
        /// \param r red corner position
        /// \param g green corner position
        /// \param b blue corner position
        /// \return
        Position get_center(cv::Point r, cv::Point g, cv::Point b);

        /// finds the closes grid element with respect to the position
        /// \param elements items to choose from
        /// \param center position which is used to determine the closes element
        /// \return the closes element
        GridElement find_closest_card(std::vector<GridElement> elements, Position center);

        /// Finds the closest grid element with respect to the position, using a threshold which has to be below the
        /// threshold which is given as a private member
        /// \param elements items to choose from
        /// \param center position which is used to determine the closes element
        /// \return the closes element, if one fits the threshold; else all values are zero/false
        GridElement find_closest_card_w_thresh(std::vector<GridElement> elements, Position center);

    };
}
