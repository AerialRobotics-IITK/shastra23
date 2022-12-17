#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#define BHV_INFO(X) ROS_INFO_STREAM("[BHV]: " << X)

namespace ariitk::state_machine
{

  static inline double pointDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
  {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
  }

  // base class for behaviors
  class Behavior
  {
  public:
    Behavior(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

    inline const geometry_msgs::Pose getPose()
    {
      return pose_;
    };

    void publishPoseSetpoint(const geometry_msgs::Pose &des_pose);
    void goToLocation(const geometry_msgs::Pose &des_pose);

  private:
    void odometryCallback(const nav_msgs::Odometry &odom);

    ros::Subscriber odom_sub_;

    ros::Publisher cmd_pose_pub_;

    geometry_msgs::Pose pose_;
  };

} // namespace ariitk::_state_machine