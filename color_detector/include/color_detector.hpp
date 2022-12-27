#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace color {
class ColorDetector {
  public:
    ColorDetector(ros::NodeHandle nh, std::string image_topic, std::string image_pub_topic);
    ~ColorDetector(){};
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void red_only();
    int largest_contour_i();
    cv::Mat image;
    cv::Mat HSVimage;
    cv::Mat detect_screen_red;
    cv::Mat image_copy;
    cv::Mat thr;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Moments moments;

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
};
}  // namespace color