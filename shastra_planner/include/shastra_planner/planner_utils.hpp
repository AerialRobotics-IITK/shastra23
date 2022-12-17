#pragma once

#include <cxxabi.h>
#include <string>
#include <future>
#include <ros/ros.h>

using std::string;

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>

#include <shastra_msgs/UTMPose.h>
#include <shastra_msgs/TagPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/CommandTOL.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

#include <tf2/LinearMath/Quaternion.h>

#define echo(X) ROS_INFO_STREAM("[PLN]: " << X);
#define FSM_INFO(X) ROS_WARN_STREAM("[FSM]: " << X);

namespace ariitk::state_machine
{
    inline const bool verbose = true;

    inline double hover_height, land_height, step_height;
    inline double HOVER_TIME = 5.0, land_time;
    inline double DELAY_TIME = 0.5;

    inline int8_t ID_AR_LZ = 0, ID_AR_BOX = 5, ID_CLR_BOX = 75, ID_AR_DZ = 10;

    inline double TRANSITION_TIME = 5.0;

    inline ros::Rate LOOP_RATE(10);

    inline std_msgs::UInt16 lidar_dist_;
    inline geometry_msgs::PoseStamped mav_pose_, lz_pose_;
    inline shastra_msgs::TagPose tag_pose_;
    // inline shastra_msgs::UTMPose utm_pose_, lz_pose_;
    inline mavros_msgs::State mav_mode_;
    inline mavros_msgs::WaypointReached prev_wp;

    inline string mission_info = "mission_info", emag_control = "emag/control", odometry = "odometry", state = "state", utm_pose = "utm_pose", set_mode = "set_mode", mission_waypoint_pull = "mission/wpPull", pose_estimator = "pose_estimator/aruco_pose", lidar_distance = "lidar/distance/distance_raw", land = "mavros/cmd/land", mission_reached = "mission/reached";

    /*
            State Variables
    */
    inline bool CONTINUE_MISSION = true;
    inline bool BOX_ATTACHED = true;
    inline bool AT_LZ = false;

    /*
        callbacks
    */
    void mav_pose_cb_(const geometry_msgs::PoseStamped &msg);
    // void utm_pose_cb_(const shastra_msgs::UTMPose &msg);
    void lidar_dist_cb_(const std_msgs::UInt16 &msg);
    void state_cb_(const mavros_msgs::State &msg);
    void pose_estimator_cb_(const shastra_msgs::TagPose &msg);
    void wp_reached_cb_(const mavros_msgs::WaypointReached &msg);

    inline ros::NodeHandle NH; // global node handle

    /*
        publishers
    */
    inline ros::Publisher command_pub_ = NH.advertise<geometry_msgs::PoseStamped>(mission_info, 10);
    inline ros::Publisher emag_pub_ = NH.advertise<std_msgs::UInt8>(emag_control, 5);

    /*
        subscribers
    */
    inline ros::Subscriber mav_pose_sub_ = NH.subscribe(odometry, 10, mav_pose_cb_);
    inline ros::Subscriber loc_pose_sub_ = NH.subscribe(pose_estimator, 1, pose_estimator_cb_);
    inline ros::Subscriber lidar_dist_sub_ = NH.subscribe(lidar_distance, 5, lidar_dist_cb_);
    inline ros::Subscriber state_sub_ = NH.subscribe(state, 1, state_cb_);
    // inline ros::Subscriber utm_pose_sub_ = NH.subscribe(utm_pose, 1, utm_pose_cb_);
    inline ros::Subscriber mission_wp_sub = NH.subscribe(mission_reached, 10, wp_reached_cb_);

    /*
        service clients
    */
    inline ros::ServiceClient set_mode_client = NH.serviceClient<mavros_msgs::SetMode>(set_mode);
    inline ros::ServiceClient mission_client = NH.serviceClient<mavros_msgs::WaypointPull>(mission_waypoint_pull);
    inline ros::ServiceClient landing_client = NH.serviceClient<mavros_msgs::CommandTOL>(land);

    /*
        base class for states
    */
    template <class T, bool V = false>
    struct State : public boost::msm::front::state<>
    {
        State()
        {
            state_name = abi::__cxa_demangle(typeid(T).name(), 0, 0, nullptr);
            verbose = V;
        }

        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            if (verbose)
            {
                FSM_INFO("Entered " << state_name << " state");
            }
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            if (verbose)
            {
                FSM_INFO("Exited " << state_name << " state");
            }
        }

        std::string state_name;
        bool verbose;
    };
}