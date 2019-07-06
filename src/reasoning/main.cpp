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

#include "prolog/PrologClient.h"

using namespace std;
using namespace cv;
using namespace aruco;
using namespace reasoning;






main(int argc, char **argv) {
    ros::init(argc, argv, "reasoning");
    PrologClient pc;
    
    pc.test_prolog_query();
    ros::spin();

}
