#include "aruco_detector.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_detector_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    Aruco::ArucoDetector detector(nh, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), cv::aruco::DetectorParameters::create(), "/camera/image_raw");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}