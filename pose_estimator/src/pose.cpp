#include <aruco_detector/aruco_detected.h>
#include <aruco_detector/aruco_message.h>
#include <geometry_msgs/PoseStamped.h>
#include <pose_estimator/pose_estimator.hpp>
#include <rosgraph_msgs/Log.h>

//-----------------NOTE------------------//
// 1. Only one aruco should be detected  //
//                                       //
//---------------------------------------//
//-----------------NEED------------------//
// 1. Update cam_error_params from ground//
// 2. Be sure on camtoDrone matrix       //
//---------------------------------------//

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_pose_estimator");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    Pose::PoseEstimator pose_estimator(nh,
                                       "/aruco_detector/aruco_detected",
                                       "/color_detector/color_detected",
                                       "/mavros/local_position/pose",
                                       "/camera/camera_info",
                                       "/pose_estimator/aruco_pose",
                                       17);
    while (ros::ok())
    {
        ros::spinOnce();
        geometry_msgs::PoseStamped pose;
        loop_rate.sleep();
    }
    return 0;
}