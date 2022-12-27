#include "aruco_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace Aruco {

ArucoDetector::ArucoDetector(ros::NodeHandle& nh_,
    cv::Ptr<cv::aruco::Dictionary> dictionary_,
    cv::Ptr<cv::aruco::DetectorParameters> parameters_,
    std::string image_topic_name_) {
    dictionary = dictionary_;
    parameters = parameters_;
    cam_sub = nh_.subscribe(image_topic_name_, 1, &Aruco::ArucoDetector::imageCallback, this);
    aruco_detected_pub = nh_.advertise<aruco_detector::aruco_detected>("aruco_detector/aruco_detected", 1);
    dictionary->maxCorrectionBits = 3;
    image_topic_name = image_topic_name_;
    // truncate dictionary to 10 ids
    dictionary->bytesList.resize(0);
    add_arucos_to_dictionary();
}

ArucoDetector::~ArucoDetector() {
}

cv::Rect ArucoDetector::get_expanded_rectangle(std::vector<cv::Point2f> corners, int extra_height, int original_cols, int original_rows) {
    auto rectangle = cv::boundingRect(corners);

    rectangle.x = std::max(0, rectangle.x - extra_height);
    rectangle.y -= std::max(0, rectangle.y - extra_height);
    rectangle.width = std::min(original_cols - rectangle.x, rectangle.width + 2 * extra_height);
    rectangle.height = std::min(original_rows - rectangle.y, rectangle.height + 2 * extra_height);

    return rectangle;
}

std::pair<std::vector<std::vector<cv::Point2f>>, int> ArucoDetector::secondary_detection(std::vector<std::vector<cv::Point2f>> rejected) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    // remove the rejected markers that are too small
    rejected.erase(std::remove_if(rejected.begin(), rejected.end(), [](std::vector<cv::Point2f> i) { return cv::contourArea(i) < 500; }), rejected.end());

    // order in descending order of area
    std::sort(rejected.begin(), rejected.end(), [](std::vector<cv::Point2f> i, std::vector<cv::Point2f> j) { return cv::contourArea(i) > cv::contourArea(j); });

    for (auto i : rejected) {
        for (int height = 10; height <= 120; height = height + 20) {
            auto rectangle = get_expanded_rectangle(i, height, image.cols, image.rows);

            cv::Mat subimage = image(rectangle);
            cv::cvtColor(subimage, subimage, cv::COLOR_BGR2GRAY);
            cv::waitKey(1);
            cv::threshold(subimage, subimage, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            cv::aruco::detectMarkers(subimage, dictionary, corners, ids, parameters);
            if (ids.size() > 0) {
                // correct the corners
                for (auto& j : corners) {
                    for (auto& k : j) {
                        k.x += rectangle.x;
                        k.y += rectangle.y;
                    }
                }
                return std::make_pair(corners, ids[0]);
            }
        }
    }
    return std::make_pair(corners, -1);
}
void ArucoDetector::detect_aruco(bool publish) {
    corners.clear();
    ids.clear();
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected);

    if (ids.size() > 0) {
    } else {
        auto corners_ids = secondary_detection(rejected);
        if (corners_ids.second != -1) {
            corners = corners_ids.first;
            ids.push_back(corners_ids.second);
            ROS_ERROR("Secondary detection");
        }
    }
    cv::Mat copy = image.clone();
    if (ids.size() > 0 && corners.size() != ids.size()) {
        // truncate corners to match ids
        corners.resize(ids.size());
    }

    cv::aruco::drawDetectedMarkers(image, corners, ids);
    cv::imshow("image", image);

    cv::waitKey(1);
    if (publish) {
        for (int i = 0; i < corners.size(); i++) {
            aruco_detector::aruco_message marker;
            marker.id = ids[i];
            for (int j = 0; j < corners[i].size(); j++) {
                geometry_msgs::Point point;
                point.x = corners[i][j].x;
                point.y = corners[i][j].y;
                marker.corners.push_back(point);
            }
            msg.detected_arucos.push_back(marker);
        }
        // publish message
        aruco_detected_pub.publish(msg);
        msg.detected_arucos.clear();
    }
}
void ArucoDetector::add_arucos_to_dictionary() {
    // 1 0 0 0
    // 1 1 1 1
    // 1 0 1 1
    // 0 1 0 1
    cv::Mat marker_bits = (cv::Mat_<unsigned char>(4, 4) << 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1);
    marker_bits = cv::aruco::Dictionary::getByteListFromBits(marker_bits);
    dictionary->bytesList.push_back(marker_bits);
    // 1 1 1 0
    // 1 0 1 1
    // 0 1 0 0
    // 1 0 0 0
    cv::Mat marker_bits2 = (cv::Mat_<unsigned char>(4, 4) << 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0);
    marker_bits2 = cv::aruco::Dictionary::getByteListFromBits(marker_bits2);
    dictionary->bytesList.push_back(marker_bits2);

    // 1 1 0 1
    // 1 0 0 0
    // 1 0 1 1
    // 0 1 1 1
    cv::Mat marker_bits3 = (cv::Mat_<unsigned char>(4, 4) << 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1);
    marker_bits3 = cv::aruco::Dictionary::getByteListFromBits(marker_bits3);
    dictionary->bytesList.push_back(marker_bits3);
}
void ArucoDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Detect the aruco markers
    detect_aruco(true);
}
}  // namespace Aruco
