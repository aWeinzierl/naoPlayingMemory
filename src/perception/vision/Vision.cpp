//
// Created by cogr on 29.06.19.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <json_prolog/prolog.h>
#include <set>
#include <math.h>
#include <limits>

#include "Vision.h"
#include "Card.h"
#include "GridElement.h"
#include "GridBoard.h"

namespace perception{

    VisionClient::VisionClient(ros::NodeHandle &nodeHandle) : it_(nodeHandle) {
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
                                   &perception::VisionClient::image_callback, this);

        cv::Mat dist(1, 5, CV_32FC1);
        dist.at<float>(0, 0) = -0.066494;
        dist.at<float>(0, 1) = 0.095481;
        dist.at<float>(0, 2) = -0.000279;
        dist.at<float>(0, 3) = 0.002292;
        dist.at<float>(0, 4) = 0.000000;
        cv::Mat cameraP(3, 3, CV_32FC1);

        cameraP.at<float>(0, 0) = 551.543059;
        cameraP.at<float>(0, 1) = 0.000000;
        cameraP.at<float>(0, 2) = 327.382898;
        cameraP.at<float>(1, 0) = 0.000000;
        cameraP.at<float>(1, 1) = 553.736023;
        cameraP.at<float>(1, 2) = 225.026380;
        cameraP.at<float>(2, 0) = 0.000000;
        cameraP.at<float>(2, 1) = 0.000000;
        cameraP.at<float>(2, 2) = 1.000000;

        CameraParameters.setParams(cameraP, dist, cv::Size(640, 480));
        CameraParameters.resize(cv::Size(640, 480));

        position_thresh = 0.08;

        card_positions = cv::Mat::zeros(cv::Size(255, 5), CV_32FC1);

        timepoint = 0;

        game_initialized = false;
        grid_board._left_marker._aruco_id = 11;
        grid_board._right_marker._aruco_id = 12;

    }

    void VisionClient::image_callback(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        InImage = cv_ptr->image;

        timepoint++;
        detect_cards();
    }

    void VisionClient::detect_cards() {
        markerDetector.detect(InImage, markers, CameraParameters);

        if (game_initialized == false) {
            initialize_game_grid();
        }


        imshow("markers", this->InImage);
        cv::waitKey(10);
    }

    void VisionClient::initialize_game_grid() {
        std::vector<GridElement> element_collection;

        //create collection of cards and calculate grid positions

        for (auto &marker : markers) {
            marker.draw(InImage, cv::Scalar(0, 0, 255), 2);
            marker.calculateExtrinsics(0.06, CameraParameters, true);

            if (marker.id == grid_board._left_marker._aruco_id) {
                grid_board._left_marker._position._x = marker.Tvec.at<float>(0);
                grid_board._left_marker._position._y = marker.Tvec.at<float>(1);
            }else if (marker.id == grid_board._right_marker._aruco_id) {
                grid_board._right_marker._position._x = marker.Tvec.at<float>(0);
                grid_board._right_marker._position._y = marker.Tvec.at<float>(1);
            } else {
                Card tmp_card;
                tmp_card.aruco_id_top = marker.id;
                tmp_card.aruco_id_bottom = 0;
                tmp_card.turned = false;
                tmp_card.object_type = "";

                GridElement tmp_grid_element;
                tmp_grid_element.card = tmp_card;
                tmp_grid_element.pos_x = marker.Tvec.at<float>(0);
                tmp_grid_element.pos_x = marker.Tvec.at<float>(1);

                element_collection.push_back(tmp_grid_element);
            }
        }


        auto edges = retrieve_edge_cards(element_collection);

        double min_distance = 1000;
        GridElement closest_element;
        //find position of 1,1 (card closest to board aruco
        for (const auto &element : edges) {
            float diff_x = grid_board._left_marker._position._x - element.pos_x;
            float diff_y = grid_board._left_marker._position._y - element.pos_y;
            double distance = sqrt(pow(static_cast<double>(diff_x), 2.0) + pow(static_cast<double>(diff_y), 2.0));
            if (distance < min_distance) {
                closest_element = element;
            }
        }
    }

    std::vector<GridElement> VisionClient::retrieve_edge_cards(const std::vector<GridElement> elements) {
        GridElement min_x_element, min_y_element, max_x_element, max_y_element;
        std::vector<GridElement> min_max_elements;

        min_x_element.pos_x = std::numeric_limits<float >::max();
        min_y_element.pos_y = std::numeric_limits<float >::max();
        max_x_element.pos_x = std::numeric_limits<float >::min();
        max_y_element.pos_y = std::numeric_limits<float >::min();

        for(const auto& element : elements){
            if(element.pos_x < min_x_element.pos_x){
                min_x_element = element;
            }else if(element.pos_x > max_x_element.pos_x){
                max_x_element = element;
            }else if(element.pos_y < min_y_element.pos_y){
                min_y_element = element;
            }else if(element.pos_y > max_y_element.pos_y){
                max_y_element = element;
            }
        }
        min_max_elements.push_back(min_x_element);
        min_max_elements.push_back(min_y_element);
        min_max_elements.push_back(max_x_element);
        min_max_elements.push_back(max_y_element);

        return min_max_elements;
    }
}