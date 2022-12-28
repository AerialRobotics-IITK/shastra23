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

#include <eigen3/Eigen/Core>

#include <shastra_msgs/UTMPose.h>
#include <shastra_msgs/TagPose.h>
#include <sensor_msgs/NavSatFix.h>
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

namespace state_machine
{
    static const bool verbose = true;
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

    double point_distance(geometry_msgs::PoseStamped p1, geometry_msgs::PointStamped p2);
}