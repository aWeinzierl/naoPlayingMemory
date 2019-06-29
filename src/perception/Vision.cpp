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

#include "Vision.h"

perception::VisionClient::VisionClient(ros::NodeHandle &nodeHandle):it_(nodeHandle) {
    image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &perception::VisionClient::image_callback, this);

    cv::Mat dist(1,5,CV_32FC1);
    dist.at<float>(0,0)=-0.066494;
    dist.at<float>(0,1)=0.095481;
    dist.at<float>(0,2)=-0.000279;
    dist.at<float>(0,3)=0.002292;
    dist.at<float>(0,4)=0.000000;
    cv::Mat cameraP(3,3,CV_32FC1);

    cameraP.at<float>(0,0)=551.543059;
    cameraP.at<float>(0,1)=0.000000;
    cameraP.at<float>(0,2)=327.382898;
    cameraP.at<float>(1,0)=0.000000;
    cameraP.at<float>(1,1)=553.736023;
    cameraP.at<float>(1,2)=225.026380;
    cameraP.at<float>(2,0)=0.000000;
    cameraP.at<float>(2,1)=0.000000;
    cameraP.at<float>(2,2)=1.000000;

    CameraParameters.setParams(cameraP,dist,cv::Size(640,480));
    CameraParameters.resize(cv::Size(640,480));

    position_thresh = 0.08;

    card_positions = cv:: Mat::zeros(cv::Size(255,5), CV_32FC1);
    game_grid = cv:: Mat::zeros(cv::Size(3,4), CV_32FC1);

    timepoint = 0;

    game_initialized = false;
}

void perception::VisionClient::image_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    InImage = cv_ptr->image;

    timepoint++;
    detect_cards();
}

void perception::VisionClient::detect_cards() {
    markerDetector.detect(InImage, markers, CameraParameters);

    if(game_initialized==false){
        _initialize_game_grid();
    }


    imshow("markers",this->InImage);
    cv::waitKey(10);
}

void perception::VisionClient::_initialize_game_grid() {

    for(auto& marker : markers) {
        marker.draw(InImage, cv::Scalar(0,0,255),2);
        marker.calculateExtrinsics(0.06, CameraParameters, true);

        for(int i=0; i<3;++i){
            card_positions.at<float>(i,marker.id) = marker.Tvec.at<float>(i);
        }
    }

    std::cout<<"Card positions: "<<card_positions<<std::endl;
    int i;
    for(int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            int position = 0;
            position = i+j;
            game_grid.at<float>(i,j) = _get_card_position(position);
        }
    }


}

int perception::VisionClient::_get_card_position(int number) {
    switch(number){
        case 0:
            int min_x, min_y;

            min_x = _find_min_position(card_positions);
    }
}

int perception::VisionClient::_find_min_position(cv::Mat matrix) {
    int id = 0;
    int num_rows;
    float min_val = 10000.0;

    num_rows = matrix.rows;
    for(int col = 0; col<2; ++col){
        float curr_val = 0.0;

        for(int row=0; row<num_rows; ++row) {
            curr_val = matrix.at<float>(row, col);
            if (!curr_val == 0) {
                if (curr_val < min_val) {
                    min_val = curr_val;
                    id = row;
                }
            }
        }
    }
    return id;
}