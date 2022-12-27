#include "color_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
namespace color {
ColorDetector::ColorDetector(ros::NodeHandle nh, std::string image_topic, std::string image_pub_topic) {
    nh_ = nh;
    image_sub_ = nh_.subscribe(image_topic, 1, &ColorDetector::imageCallback, this);
    image_pub_ = nh_.advertise<geometry_msgs::Point>(image_pub_topic, 1);
}

void ColorDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image = cv_ptr->image;
    cvtColor(image, HSVimage, cv::COLOR_BGR2HSV);
    red_only();
}
int ColorDetector::largest_contour_i() {
    int largest_area = 0;
    int largest_contour_index = 0;
    for (int i = 0; i < contours.size(); i++) {
        double a = contourArea(contours[i], false);
        if (a > largest_area) {
            largest_area = a;
            largest_contour_index = i;
        }
    }
    return largest_contour_index;
}
void ColorDetector::red_only() {
    cv::cvtColor(image, HSVimage, cv::COLOR_BGR2HSV);
    cv::inRange(HSVimage, cv::Scalar(0, 50, 50), cv::Scalar(15, 255, 255), detect_screen_red);
    cv::erode(detect_screen_red, detect_screen_red, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)));
    cv::findContours(detect_screen_red, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    image_copy = image.clone();
    cv::drawContours(image_copy, contours, largest_contour_i(), cv::Scalar(0, 255, 0), 2);
    cv::threshold(detect_screen_red, thr, 100, 255, cv::THRESH_BINARY);
    int largest_contour = largest_contour_i();
    if (largest_contour == 0) {
        moments = cv::moments(thr, true);
    } else {
        moments = cv::moments(contours[largest_contour]);
    }
    cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
    geometry_msgs::Point center_point;
    // area of the largest contour
    if (contours.size() == 0 || cv::contourArea(contours[largest_contour]) < 1000) {
        center_point.x = -1;
        center_point.y = -1;
        center_point.z = -1;
        image_pub_.publish(center_point);
        return;
    }

    center_point.x = center.x;
    center_point.y = center.y;
    center_point.z = 1;
    image_pub_.publish(center_point);
}
}  // namespace color