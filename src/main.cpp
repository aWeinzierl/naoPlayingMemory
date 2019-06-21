#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <json_prolog/prolog.h>
#include <unordered_map>

#include "PrologClient.h"

using namespace std;
using namespace cv;
using namespace aruco;

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    CameraParameters TheCameraParameters;
    std::unordered_map<unsigned int, std::string> marker_to_class_associations_;
    std::unordered_map<unsigned int, unsigned int> marker_to_instance_id_associations_;
    std::unordered_map<unsigned int, cv::Mat> marker_to_last_significant_position_;
    PrologClient& _prologClient;
    uint _globalTime;

public:
    ImageConverter(PrologClient& prologClient)
            : it_(nh_), _prologClient(prologClient) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &ImageConverter::imageCb, this);

        Mat dist(1, 5, CV_32FC1);
        dist.at<float>(0, 0) = -0.066494;
        dist.at<float>(0, 1) = 0.095481;
        dist.at<float>(0, 2) = -0.000279;
        dist.at<float>(0, 3) = 0.002292;
        dist.at<float>(0, 4) = 0.000000;
        Mat cameraP(3, 3, CV_32FC1);

        cameraP.at<float>(0, 0) = 551.543059;
        cameraP.at<float>(0, 1) = 0.000000;
        cameraP.at<float>(0, 2) = 327.382898;
        cameraP.at<float>(1, 0) = 0.000000;
        cameraP.at<float>(1, 1) = 553.736023;
        cameraP.at<float>(1, 2) = 225.026380;
        cameraP.at<float>(2, 0) = 0.000000;
        cameraP.at<float>(2, 1) = 0.000000;
        cameraP.at<float>(2, 2) = 1.000000;


        TheCameraParameters.setParams(cameraP, dist, Size(640, 480));
        TheCameraParameters.resize(Size(640, 480));

        marker_to_class_associations_ = {
                {457, "Carrot"},
                {885, "Donut"},
                {785, "HotWing"},
                {943, "HotWing"},
                {251, "HotWing"},
                {279, "Donut"},
        };

        _globalTime=0;

    }

    ~ImageConverter() {
    }

    void clearLinuxConsole()
    {
        std::cout << "\033[2J\033[1;1H";
    }

    void drawMarkers(Mat &image, const std::vector<Marker> &markers) {
        for (auto const &marker : markers) {
            marker.draw(image, Scalar(0, 0, 255), 2);
        }
    }

    void recognizeObjects(const std::vector<Marker> &markers) {
        for (auto const &marker : markers) {
            auto markerId = marker.id;
            if (marker_to_class_associations_.find(markerId) == marker_to_class_associations_.end()) {
                // std::cout << "Found unkown marker with id " + std::to_string(markerId) + "\n";
                continue;
            }

            auto instanceIdSearchPtr = marker_to_instance_id_associations_.find(markerId);
            auto associatedClass = marker_to_class_associations_.find(markerId)->second;

            if (instanceIdSearchPtr == marker_to_instance_id_associations_.end()) {

                auto id = _prologClient.CreateTypeInstance(associatedClass);
                marker_to_instance_id_associations_[markerId] = id;


                std::cout << "Found new instance of class '" + associatedClass + "': '" + std::to_string(markerId) +
                             "'\n";
                std::cout << "    Registering new instance with id '" + std::to_string(id) + "'\n";
            } else {
                auto instanceId = instanceIdSearchPtr->second;
                //std::cout << "Found registered instance of class '" + associatedClass + "': '" +
                //             std::to_string(markerId) + "'\n";
                //std::cout << "    Instance is registered with id '" +
                //             std::to_string(marker_to_instance_id_associations_.find(markerId)->second) + "'\n";
            }
        }
    }

    void detectMotions(std::vector<Marker> &markers) {
        for (auto &marker: markers) {
            auto markerId = marker.id;
            if (marker_to_class_associations_.find(markerId) == marker_to_class_associations_.end()) {
                // std::cout << "Found unkown marker with id " + std::to_string(markerId) + "\n";
                continue;
            }

            marker.calculateExtrinsics(0.04,TheCameraParameters,true);
            auto markerPosition = marker.Tvec;

            auto lastPositionPtr = marker_to_last_significant_position_.find(markerId);

            if (lastPositionPtr == marker_to_last_significant_position_.end()) {
                marker_to_last_significant_position_[marker.id] = markerPosition.clone();
                std::cout << "    Tracking new instance" << "\n";
                continue;
            }

            auto threshold = 0.075f;
            Mat distanceMoved = (markerPosition - lastPositionPtr->second);
            if (distanceMoved.at<float>(0) >= threshold){
                lastPositionPtr->second.at<float>(0)=markerPosition.at<float>(0);
                std::cout << "    Marker with id '" << markerId << "' moved right" << std::endl;
                registerMotion(markerId, DIRECTION::RIGHT);
            }
            if (distanceMoved.at<float>(0) <= -threshold){
                lastPositionPtr->second.at<float>(0)=markerPosition.at<float>(0);
                std::cout << "    Marker with id '" << markerId << "' moved left" << std::endl;
                registerMotion(markerId, DIRECTION::LEFT);
            }
            if (distanceMoved.at<float>(1) >= threshold){
                lastPositionPtr->second.at<float>(1)=markerPosition.at<float>(1);
                std::cout << "    Marker with id '" << markerId << "' moved down" << std::endl;
                registerMotion(markerId, DIRECTION::DOWN);
            }
            if (distanceMoved.at<float>(1) <= -threshold){
                lastPositionPtr->second.at<float>(1)=markerPosition.at<float>(1);
                std::cout << "    Marker with id '" << markerId << "' moved up" << std::endl;
                registerMotion(markerId, DIRECTION::UP);
            }
            if (distanceMoved.at<float>(2) >= threshold){
                lastPositionPtr->second.at<float>(2)=markerPosition.at<float>(2);
                std::cout << "    Marker with id '" << markerId << "' moved away" << std::endl;
                registerMotion(markerId, DIRECTION::AWAY);
            }
            if (distanceMoved.at<float>(2) <= -threshold){
                lastPositionPtr->second.at<float>(2)=markerPosition.at<float>(2);
                std::cout << "    Marker with id '" << markerId << "' moved closer" << std::endl;
                registerMotion(markerId, DIRECTION::CLOSER);
            }
        }
    }

    void registerMotion(uint markerId, DIRECTION direction){
        _prologClient.Create_time_point_if_not_exists(_globalTime);

        auto classType = marker_to_class_associations_.find(markerId)->second;
        auto id = marker_to_instance_id_associations_.find(markerId)->second;
        PrologClient::Instance instance(classType, id);
        _prologClient.Register_motion_for_object(instance, _globalTime, direction);
        _prologClient.PrintMovementsOfInstance(instance, direction);

    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        _globalTime++;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat InImage = cv_ptr->image;

        MarkerDetector mDetector;
        vector<Marker> markers;
        mDetector.detect(InImage, markers, TheCameraParameters);
        //clearLinuxConsole();
        drawMarkers(InImage, markers);
        recognizeObjects(markers);

        detectMotions(markers);

        imshow("markers", InImage);
        waitKey(10);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tutorial_vision");
    PrologClient pc;
    ImageConverter ic(pc);

    ros::spin();
    return 0;
}
