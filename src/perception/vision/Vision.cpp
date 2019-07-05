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
#include <cv.h>
#include <unordered_set>

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

        timepoint = 0;

        game_initialized = false;
        grid_board._left_marker._aruco_id = 13;
        grid_board._right_marker._aruco_id = 14;

        distance_thresh = 20;

        for (int i=0;i<4;++i){
            grid_board.grid.emplace_back(std::vector<GridElement>(3));
        }

        top_ids = {15,16,17,18,19,20,21,22,23,24,25,26};
        card_classes = {
                {1, "Strawberry"},
                {2, "Strawberry"},
                {3, "Pizza"},
                {4, "Pizza"},
                {5, "Ninja"},
                {6, "Ninja"},
                {7, "Penguin"},
                {8, "Penguin"},
                {9, "Octopus"},
                {10, "Octopus"},
                {11, "Nao"},
                {12, "Nao"},

        };

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
        }else{
            check_cards();
        }
        int i,j;
        i=1;

        /*for(auto& cols : grid_board.grid){
            j=1;
            for(auto& grid_element : cols) {
                std::cout<<"Grid Pos: "<<i<<","<<j<<std::endl;
                std::cout<<"turned: "<<grid_element.card.turned<<std::endl;
                std::cout<<"id top: "<<grid_element.card.aruco_id_top<<std::endl;
                std::cout<<"class: "<<grid_element.card.object_type<<std::endl;
                std::cout<<"bottom: "<<grid_element.card.aruco_id_bottom<<std::endl;
                ++j;
            }
            ++i;
        }*/

        imshow("markers", this->InImage);
        cv::waitKey(10);
    }

    void VisionClient::check_cards() {
        std::vector<GridElement> element_collection;

        for (auto &marker : markers) {
            marker.draw(InImage, cv::Scalar(0, 0, 255), 2);
            marker.calculateExtrinsics(0.06, CameraParameters, true);
            Position center;
            center = get_center(marker[0], marker[1], marker[2]);

            if(marker.id != grid_board._left_marker._aruco_id && marker.id != grid_board._right_marker._aruco_id){
                Card tmp_card;
                if(top_ids.find(marker.id) != top_ids.end()){
                    tmp_card.aruco_id_top = marker.id;
                    tmp_card.turned = false;
                }else if(card_classes.find(marker.id) != card_classes.end()){
                    tmp_card.turned = true;
                    tmp_card.aruco_id_bottom = marker.id;
                    tmp_card.object_type = card_classes.find(marker.id)->second;
                }
                GridElement tmp_grid_element;
                tmp_grid_element.card = tmp_card;
                tmp_grid_element._im_pos = center;
                element_collection.push_back(tmp_grid_element);
            }
        }

        std::unordered_set<unsigned int> ids_found;
        for(auto& cols : grid_board.grid){
            for(auto& grid_element : cols) {
                GridElement tmp_elem;
                Position tmp_pos;
                tmp_pos = grid_element._im_pos;
                tmp_elem = find_closest_card_w_thresh(element_collection, tmp_pos);
                grid_element.card = tmp_elem.card;
                ids_found.emplace(tmp_elem.card.aruco_id_top);
                ids_found.emplace(tmp_elem.card.aruco_id_bottom);
            }
        }
    }

    void VisionClient::initialize_game_grid() {
        std::vector<GridElement> element_collection;

        //create collection of cards and calculate grid positions

        for (auto &marker : markers) {
            marker.draw(InImage, cv::Scalar(0, 0, 255), 2);
            marker.calculateExtrinsics(0.06, CameraParameters, true);
            Position center;
            center = get_center(marker[0], marker[1], marker[2]);

            if (marker.id == grid_board._left_marker._aruco_id) {
                grid_board._left_marker._tvec= marker.Tvec;
                grid_board._left_marker._rvec = marker.Rvec;

                grid_board._left_marker._im_pos = center;

            }else if (marker.id == grid_board._right_marker._aruco_id) {
                grid_board._right_marker._tvec = marker.Tvec;
                grid_board._right_marker._rvec = marker.Rvec;
                grid_board._right_marker._im_pos = center;

            } else {
                Card tmp_card;
                if(top_ids.find(marker.id) == top_ids.end()){
                    game_initialized = false;
                    std::cout<<"Please make sure that all cards are turned and restart."<<std::endl;
                    return;
                }
                tmp_card.aruco_id_top = marker.id;
                tmp_card.aruco_id_bottom = 0;
                tmp_card.turned = false;
                tmp_card.object_type = "";

                GridElement tmp_grid_element;
                tmp_grid_element.card = tmp_card;
                tmp_grid_element._tvec= marker.Tvec;
                tmp_grid_element._rvec = marker.Rvec;

                tmp_grid_element._im_pos = center;


                element_collection.push_back(tmp_grid_element);

            }
        }
        /*cv::Mat t_matrix;
        if(!element_collection.empty() && !grid_board._left_marker._rvec.empty()){
            for (auto& element : element_collection){
                element._im_pos = get_relative_position(element._rvec, element._tvec);
            }
        }*/

        std::vector<GridElement> edges = retrieve_edge_cards(element_collection);
        grid_board.grid[0][0] = edges[0];
        grid_board.grid[3][0] = edges[1];
        grid_board.grid[0][2] = edges[2];
        grid_board.grid[3][2] = edges[3];

        // find 2,1 and 3,1
        Position center_21, center_31;
        center_21.x = grid_board.grid[0][0]._im_pos.x + 0.33 * (grid_board.grid[3][0]._im_pos.x - grid_board.grid[0][0]._im_pos.x);
        center_21.y = grid_board.grid[0][0]._im_pos.y + 0.33 * (grid_board.grid[3][0]._im_pos.y - grid_board.grid[0][0]._im_pos.y);
        grid_board.grid[1][0] = find_closest_card(element_collection, center_21);

        center_31.x = grid_board.grid[0][0]._im_pos.x + 0.66 * (grid_board.grid[3][0]._im_pos.x - grid_board.grid[0][0]._im_pos.x);
        center_31.y = grid_board.grid[0][0]._im_pos.y + 0.66 * (grid_board.grid[3][0]._im_pos.y - grid_board.grid[0][0]._im_pos.y);
        grid_board.grid[2][0] = find_closest_card(element_collection, center_31);

        //find 1,2
        Position center_12;
        center_12.x = grid_board.grid[0][0]._im_pos.x + 0.5 * (grid_board.grid[0][2]._im_pos.x - grid_board.grid[0][0]._im_pos.x);
        center_12.y = grid_board.grid[0][0]._im_pos.y + 0.5 * (grid_board.grid[0][2]._im_pos.y - grid_board.grid[0][0]._im_pos.y);
        grid_board.grid[0][1] = find_closest_card(element_collection, center_12);

        //find 4,2
        Position center_42;
        center_42.x = grid_board.grid[3][0]._im_pos.x + 0.5 * (grid_board.grid[3][2]._im_pos.x - grid_board.grid[3][0]._im_pos.x);
        center_42.y = grid_board.grid[3][0]._im_pos.y + 0.5 * (grid_board.grid[3][2]._im_pos.y - grid_board.grid[3][0]._im_pos.y);
        grid_board.grid[3][1] = find_closest_card(element_collection, center_42);

        //find 2,2 and 3,2
        Position center_22, center_32;
        center_22.x = grid_board.grid[0][1]._im_pos.x + 0.33 * (grid_board.grid[3][1]._im_pos.x - grid_board.grid[0][1]._im_pos.x);
        center_22.y = grid_board.grid[0][1]._im_pos.y + 0.33 * (grid_board.grid[3][1]._im_pos.y - grid_board.grid[0][1]._im_pos.y);
        grid_board.grid[1][1] = find_closest_card(element_collection, center_22);

        center_32.x = grid_board.grid[0][1]._im_pos.x + 0.66 * (grid_board.grid[3][1]._im_pos.x - grid_board.grid[0][1]._im_pos.x);
        center_32.y = grid_board.grid[0][1]._im_pos.y + 0.66 * (grid_board.grid[3][1]._im_pos.y - grid_board.grid[0][1]._im_pos.y);
        grid_board.grid[2][1] = find_closest_card(element_collection, center_32);

        //find 2,3 and 3,3
        Position center_23, center_33;
        center_23.x = grid_board.grid[0][2]._im_pos.x + 0.33 * (grid_board.grid[3][2]._im_pos.x - grid_board.grid[0][2]._im_pos.x);
        center_23.y = grid_board.grid[0][2]._im_pos.y + 0.33 * (grid_board.grid[3][2]._im_pos.y - grid_board.grid[0][2]._im_pos.y);
        grid_board.grid[1][2] = find_closest_card(element_collection, center_23);

        center_33.x = grid_board.grid[0][2]._im_pos.x + 0.66 * (grid_board.grid[3][2]._im_pos.x - grid_board.grid[0][2]._im_pos.x);
        center_33.y = grid_board.grid[0][2]._im_pos.y + 0.66 * (grid_board.grid[3][2]._im_pos.y - grid_board.grid[0][2]._im_pos.y);
        grid_board.grid[2][2] = find_closest_card(element_collection, center_33);

        std::unordered_set<unsigned int> ids_found;
        for(const auto& cols : grid_board.grid){
            for(const auto& element : cols){
                if(ids_found.find(element.card.aruco_id_top) != ids_found.end()){
                    game_initialized = false;
                    return;
                }
                ids_found.emplace(element.card.aruco_id_top);
            }
        }

        if(ids_found.size() < 12 ){
            std::cout<<ids_found.size()<<std::endl;
            game_initialized = false;
        } else {
            game_initialized = true;
        }

    }

    std::vector<GridElement> VisionClient::retrieve_edge_cards(const std::vector<GridElement> elements) {
        GridElement c_1_1, c_4_1, c_1_3, c_4_3;
        std::vector<GridElement> edges;
        double min_distance = 1000;
        double max_distance = 0;

        for (const auto &element : elements) {
            float diff_x = grid_board._left_marker._im_pos.x - element._im_pos.x;
            float diff_y = grid_board._left_marker._im_pos.y - element._im_pos.y;
            double distance = sqrt(pow(static_cast<double>(diff_x), 2.0) + pow(static_cast<double>(diff_y), 2.0));
            if (distance < min_distance) {
                c_1_1 = element;
                min_distance = distance;
            }
        }
        min_distance = 1000;

        for (const auto &element : elements) {
            float diff_x = grid_board._left_marker._im_pos.x - element._im_pos.x;
            float diff_y = grid_board._left_marker._im_pos.y - element._im_pos.y;
            double distance = sqrt(pow(static_cast<double>(diff_x), 2.0) + pow(static_cast<double>(diff_y), 2.0));
            if (distance > max_distance) {
                c_4_3 = element;
                max_distance = distance;
            }
        }
        max_distance = 0;

        for (const auto &element : elements) {
            float diff_x = grid_board._right_marker._im_pos.x - element._im_pos.x;
            float diff_y = grid_board._right_marker._im_pos.y - element._im_pos.y;
            double distance = sqrt(pow(static_cast<double>(diff_x), 2.0) + pow(static_cast<double>(diff_y), 2.0));
            if (distance < min_distance) {
                c_4_1 = element;
                min_distance = distance;
            }
        }

        for (const auto &element : elements) {
            float diff_x = grid_board._right_marker._im_pos.x - element._im_pos.x;
            float diff_y = grid_board._right_marker._im_pos.y - element._im_pos.y;
            double distance = sqrt(pow(static_cast<double>(diff_x), 2.0) + pow(static_cast<double>(diff_y), 2.0));
            if (distance > max_distance) {
                c_1_3 = element;
                max_distance = distance;
            }
        }

        edges.push_back(c_1_1);
        edges.push_back(c_4_1);
        edges.push_back(c_1_3);
        edges.push_back(c_4_3);

        return edges;
    }

    Position VisionClient::get_relative_position(cv::Mat rvec, cv::Mat tvec) {
        //NOT WORKING PROPERLY
        Position rel_pos;
        cv::Mat R, R_t, invRvec, invTvec, composedRvec, composedTvec;
        cv::Rodrigues(grid_board._left_marker._rvec, R);
        cv::transpose(R, R_t);
        invTvec = -R_t * grid_board._left_marker._tvec;
        cv::Rodrigues(R_t, invRvec);
        cv::composeRT(rvec, tvec, invRvec, invTvec, composedRvec, composedTvec);
        /*std::cout<<"card RVec: "<<rvec<<std::endl;
        std::cout<<"card TVec: "<<tvec<<std::endl;
        std::cout<<"board RVec: "<<grid_board._left_marker._rvec<<std::endl;
        std::cout<<"board TVec: "<<grid_board._left_marker._tvec<<std::endl;
        std::cout<<"composed RVec: "<<composedRvec<<std::endl;
        std::cout<<"composed TVec: "<<composedTvec<<std::endl;
        std::cout<<"difference x: "<<std::endl;*/
        return rel_pos;
    }

    Position VisionClient::get_center(cv::Point r, cv::Point g, cv::Point b) {
        Position center;
        center.x = r.x + 0.5*(b.x-r.x);
        center.y = r.y + 0.5 *(b.y-r.y);
        return center;
    }

    GridElement VisionClient::find_closest_card(std::vector<GridElement> elements, Position center) {
        double min_distance = 1000;
        GridElement tmp_el;

        for(const auto& element : elements){
            float diff_x = center.x - element._im_pos.x;
            float diff_y = center.y - element._im_pos.y;
            double distance = sqrt(pow(static_cast<double>(diff_x), 2.0) + pow(static_cast<double>(diff_y), 2.0));
            if (distance < min_distance) {
                tmp_el = element;
                min_distance = distance;

            }
        }

        return tmp_el;
    }

    GridElement VisionClient::find_closest_card_w_thresh(std::vector<GridElement> elements, Position center) {
        double min_distance = distance_thresh;
        GridElement tmp_el;
        tmp_el.card.aruco_id_bottom = 0;
        tmp_el.card.aruco_id_bottom = 0;
        tmp_el.card.object_type = "Not available";

        for(const auto& element : elements){
            float diff_x = center.x - element._im_pos.x;
            float diff_y = center.y - element._im_pos.y;

            double distance = sqrt(pow(static_cast<double>(diff_x), 2.0) + pow(static_cast<double>(diff_y), 2.0));


            //Visualization
            cv::Point c, c2;
            c.x = center.x;
            c.y = center.y;
            c2.x = element._im_pos.x;
            c2.x = element._im_pos.y;
            cv::circle(InImage, c, min_distance,cv::Scalar(0, 0, 255));
            cv::circle(InImage, c2, 1,cv::Scalar(255, 255, 0));

            if (distance < min_distance) {
                tmp_el = element;
                min_distance = distance;

            }
        }
        return tmp_el;
    }
}