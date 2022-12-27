#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>

#include <aruco_detector/aruco_detected.h>
#include <aruco_detector/aruco_message.h>

#include "pose_estimator/pose_estimator.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace Pose {
PoseEstimator::PoseEstimator(ros::NodeHandle nh_,
    std::string aruco_sub_topic,
    std::string color_sub_topic,
    std::string drone_sub_topic,
    std::string cam_sub_topic,
    std::string pub_topic,
    float rolling_average_count_) {
    nh = nh_;
    sub = nh_.subscribe(aruco_sub_topic, 1, &PoseEstimator::center_callBack, this);
    sub_color = nh_.subscribe(color_sub_topic, 1, &PoseEstimator::color_pose_callBack, this);
    sub_drone = nh_.subscribe(drone_sub_topic, 1, &PoseEstimator::camera_pose_callBack, this);
    sub_cam = nh_.subscribe(cam_sub_topic, 1, &PoseEstimator::camera_info_callBack, this);

    pose_pub = nh_.advertise<pose_estimator::pose_message>(pub_topic, 1);
    rolling_avg_count = rolling_average_count_;
    rolling_avg_wf = Eigen::Vector3f::Zero();
    drone_height = 0;
    color_detected = false;
    aruco_detected = false;

    cam_error_z = 16.4;
    cam_error_x = 5.1;
    cam_error_y = 4.2;

    return;
}
void PoseEstimator::calc_pose() {
    camtoDrone << 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f;
    // Coordinates of center of aruco wrt camera frame
    Eigen::Vector3f coordi_cam_frame = (K.inverse()) * pixel;
    // check all values of coordi_cam_frame
    world_frame = cam_height * coordi_cam_frame;
    world_frame[2] += cam_error_z;
    world_frame[1] += cam_error_y;
    world_frame[0] += cam_error_x;
    compute_rolling_avg();
    publish_pose(true);
    return;
}
void PoseEstimator::center_callBack(const aruco_detector::aruco_detected::ConstPtr& coords) {
    if (coords->detected_arucos.size() != 0) {
        center_x = 0.0, center_y = 0.0;
        for (int i = 0; i < 4; i++) {
            center_x += coords->detected_arucos[0].corners[i].x / 4.0;
            center_y += coords->detected_arucos[0].corners[i].y / 4.0;
            // sum_z = coords.detected_arucos[0].corners[i].x
        }
        pixel[0] = center_x;
        pixel[1] = center_y;
        pixel[2] = 1.0f;
        aruco_detected = true;
        pose.id = coords->detected_arucos[0].id;
        // ROS_INFO("x=%f,y=%f,z=%f", coords.detected_arucos[0].corners[0].x, pixel[1],pixel[2]);
        calc_pose();
    }
    // get average side length
    // else if()
    // {
    //     // THIS IS CALLED BUT LOOK IF COLOR IS DETECTED THEN YOU GOTTA PULISH BUT ELSE FALSE
    // }
    else {
        aruco_detected = false;
    }
    if (!aruco_detected && !color_detected) {
        publish_pose(false);
        pose.id = -1;
    }
    return;
}

void PoseEstimator::color_pose_callBack(const geometry_msgs::Point& color_center) {
    if (color_center.z == -1) {
        color_detected = false;
    } else {
        pixel[0] = color_center.x;
        pixel[1] = color_center.y;
        pixel[2] = 1.0f;
        color_detected = true;
        pose.id = 75;
        calc_pose();
    }
}

void PoseEstimator::camera_info_callBack(const sensor_msgs::CameraInfo::ConstPtr& camera_params) {
    if (flag)  // Intrinsic Params do not change
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) {
                K(i, j) = camera_params->K[count];
                // if ((i == 0 && j == 0) || (i == 1 && j == 1))
                //     K(i, j) = K(i, j) * 0.01;
                count++;
            }
        K = K * 0.5;
        K(2, 2) = 1.0;
        // get inverse
    }

    count = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            R(i, j) = camera_params->R[count];
            count++;
        }
    flag = false;
    return;
}
void PoseEstimator::drone_orientation_callBack(const sensor_msgs::Imu::ConstPtr& drone_orientation) {
    auto cam_orientation = drone_orientation->orientation;

    cam_rot_mat = Eigen::Quaternionf(cam_orientation.w, cam_orientation.x, cam_orientation.y, cam_orientation.z).toRotationMatrix();
    // std::cout << cam_rot_mat << std::endl;
    cam_rot_mat_inversed = cam_rot_mat.inverse();
    auto rpy = cam_rot_mat.eulerAngles(0, 1, 2);
    std::cout << "rpy" << rpy << std::endl;
    // std::cout << "cam_rot_mat" << std::endl << cam_rot_mat << std::endl;
    // std::cout << "image_rot_mat_wrt_cam" << std::endl << image_rot_mat_wrt_cam << std::endl;
}

void PoseEstimator::camera_pose_callBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    drone_height = (drone_height * rolling_avg_count + msg->pose.position.z) / (rolling_avg_count + 1);
    camera_pose[0] = 0.0;                         // drone_odom.pose.pose.position.x - cam_error_x;
    camera_pose[1] = 0.0;                         // drone_odom.pose.pose.position.y - cam_error_y;5.0
    camera_pose[2] = drone_height - cam_error_z;  // drone_odom.pose.pose.position.z - cam_error_z;16.4

    // scaling_factor provided camera is nadir!
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            if (i == j)
                cam_height(i, j) = camera_pose[2];
            else
                cam_height(i, j) = 0.0f;
        }
    return;
}
void PoseEstimator::get_image_rotation_matrix_wrt_cam() {
    // get rotation of world_frame vector wrt camera frame
    Eigen::Vector3f unit_vector = cam_frame.normalized();
    // std::cout << "unit_vector" << unit_vector << std::endl;
    // (0,0,1) is z axis of camera frame
    Eigen::Vector3f camera_z_axis(0.0f, 0.0f, 1.0f);
    float roll, pitch, yaw;
    // get roll pitch yaw from quaternion

    image_rot_mat_wrt_cam = Eigen::Quaternionf::FromTwoVectors(camera_z_axis, unit_vector).toRotationMatrix();
    image_rot_mat_wrt_cam_inversed = image_rot_mat_wrt_cam.inverse();
}
float PoseEstimator::z_rotation_correction() {
    // get phi theta psi from quaternion
    auto rpy = cam_rot_mat.eulerAngles(0, 1, 2);
    // sclaing factor is square root of 1 tan^2(theta) + tan^2(phi)
    float scaling_factor = sqrt(1 + tan(rpy[1]) * tan(rpy[1]) + tan(rpy[0]) * tan(rpy[0]));
    return scaling_factor;
}

void PoseEstimator::compute_pose_in_world_frame() {
    Eigen::Vector3f intermed;
    // ROS_INFO("x=%f,y=%f,z=%f", cam_frame[0], cam_frame[1], cam_frame[2]);
    intermed = cam_rot_mat_inversed * cam_frame;

    // intermed = cam_rot_mat_inversed * intermed;
    Eigen::Vector3f unit_world_frame = intermed.normalized();
    float length = fabs(drone_height / unit_world_frame[2]);
    // ROS_INFO("length=%f", length);
    world_frame = length * unit_world_frame;
}
void PoseEstimator::publish_pose(bool detected) {
    pose.center.header.stamp = ros::Time::now();

    if (!detected) {
        set_msg_to_not_detected();
        pose_pub.publish(pose);
        return;
    }

    pose.center.point.x = rolling_avg_wf[0];
    pose.center.point.y = rolling_avg_wf[1];
    pose.center.point.z = rolling_avg_wf[2];

    pose_pub.publish(pose);
    return;
}

void PoseEstimator::compute_rolling_avg() {
    rolling_avg_wf = (rolling_avg_wf * rolling_avg_count + world_frame) / (rolling_avg_count + 1);
}
// void PoseEstimator::set_msg_to_not_detected(geometry_msgs::PoseStamped& pose) {
void PoseEstimator::set_msg_to_not_detected() {
    pose.center.point.x = 0;
    pose.center.point.y = 0;
    pose.center.point.z = 0;
    pose.id = -1;
    return;
}
}  // namespace Pose