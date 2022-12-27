#include "aruco_detector/aruco_detected.h"
#include "aruco_detector/aruco_message.h"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace Aruco {
class ArucoDetector {
  public:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Mat image;
    std::string image_topic_name;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    ros::Subscriber cam_sub;           /*nh.subscribe(detector.image_topic_name, 1, imageCallback);*/
    ros::Publisher aruco_detected_pub; /*nh.advertise<aruco_detector::aruco_detected>("aruco_detector/aruco_detected", 1);*/
    aruco_detector::aruco_detected msg;

    cv::Rect get_expanded_rectangle(std::vector<cv::Point2f> corners, int extra_height, int original_cols, int original_rows);
    std::pair<std::vector<std::vector<cv::Point2f>>, int> secondary_detection(std::vector<std::vector<cv::Point2f>> rejected);
    void detect_aruco(bool publish);
    void add_arucos_to_dictionary();

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    ArucoDetector(ros::NodeHandle& nh_,
        cv::Ptr<cv::aruco::Dictionary> dictionary_,
        cv::Ptr<cv::aruco::DetectorParameters> parameters_,
        std::string image_topic_name_);
    ~ArucoDetector();
};
}  // namespace Aruco