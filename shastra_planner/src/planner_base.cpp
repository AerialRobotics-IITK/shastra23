#include <shastra_planner/planner.hpp>

namespace state_machine
{
    /*
        callbacks
    */
    void fsm::mav_pose_cb_(const geometry_msgs::PoseStamped &msg) { mav_pose_ = msg; }
    // void utm_pose_cb_(const shastra_msgs::UTMPose &msg) { utm_pose_ = msg; }
    void fsm::state_cb_(const mavros_msgs::State &msg) { mav_mode_ = msg; }
    void fsm::pose_estimator_cb_(const shastra_msgs::TagPose &msg)
    {
        tag_pose_ = msg;
        tag_pose_.pose.point.x /= 100;
        tag_pose_.pose.point.y /= 100;
        tag_pose_.pose.point.z /= 100; // converting to meters
    }
    void fsm::wp_reached_cb_(const mavros_msgs::WaypointReached &msg) { prev_wp = msg; }

    void statePublish(ros::NodeHandle nh, fsm_ *fsm)
    {
        ros::Publisher statePub = nh.advertise<std_msgs::String>("curr_state", 10);
        ros::Rate loopRate(10);

        std_msgs::String msg;
        while (ros::ok())
        {
            msg.data = fsm::state_names_[fsm->current_state()[0]];
            statePub.publish(msg);
            loopRate.sleep();
        }
    }

    void echo_state(fsm_ const &msg)
    {
        if (verbose)
            FSM_INFO("Current state -- " << fsm::state_names_[msg.current_state()[0]]);
    }

    /*
        Transition actions
    */
    /// @brief TakeOff Action: Wait for MAV position, set_mode to mission
    /// @param cmd
    void fsm::TakeOff(CmdTakeOff const &cmd)
    {
        if (verbose)
            echo("Starting Execution");

        ros::Rate loopRate(10);

        mav_pose_.pose.position.z = -DBL_MAX;
        if (verbose)
            echo("Waiting for MAV Position");
        while (mav_pose_.pose.position.z == -DBL_MAX)
        {
            ros::spinOnce();
            loopRate.sleep();
        }
        if (verbose)
            echo("Received MAV Position");
        lz_pose_ = mav_pose_; // takeoff position is landing zone pose as well

        mavros_msgs::SetMode mission_set_mode_;
        mission_set_mode_.request.custom_mode = "AUTO.MISSION";

        bool mode_set_ = false;

        while (!mode_set_)
        {
            ros::spinOnce();
            if (set_mode_client.call(mission_set_mode_) and mission_set_mode_.response.mode_sent)
            {
                if (verbose)
                    echo("MISSION_MODE enabled");
                mode_set_ = true;
            }
            loopRate.sleep();
        }

        if (verbose)
            echo("Changed to MISSION_MODE");

        return;
    }

    /// @brief GoToLandingZone Action: assumes mission_mode initially, receive odometry, continue with mission_mode, obtains left number of waypoints keep checking for any detection in pose_estimated topic.
    /// @param cmd
    void fsm::GoToGZ(CmdGridZone const &cmd)
    {
        mavros_msgs::SetMode mission_set_mode_;
        mission_set_mode_.request.custom_mode = "AUTO.MISSION";
        bool mode_set_ = false;

        ros::Rate loopRate(10);

        while (!mode_set_)
        {
            ros::spinOnce();
            if (set_mode_client.call(mission_set_mode_) and mission_set_mode_.response.mode_sent)
            {
                if (verbose)
                    echo("MISSION_MODE enabled");
                mode_set_ = true;
            }
            loopRate.sleep();
        }

        if (verbose)
            echo("Changed to MISSION_MODE");

        if (verbose)
            echo("Going to Grid in MISSION_MODE");
        if (verbose)
            echo("Waiting for odometry");
        mav_pose_.pose.position.z = -DBL_MAX;
        while (mav_pose_.pose.position.z == -DBL_MAX)
        {
            ros::spinOnce();
            loopRate.sleep();
        }
        if (verbose)
            echo("Received odometry"); // check if this is working, not used

        mavros_msgs::WaypointPull req;
        int num_wp = 0;

        while (num_wp == 0)
        { // assuming have more than 1 waypoint
            if (mission_client.call(req) && req.response.success)
            {
                num_wp = req.response.wp_received - 1;
            }
        }
        if (verbose)
        {
            echo("Received waypoints...");
            echo(num_wp);
        }

        // getting ready for offboard mode
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        mode_set_ = false;

        if (verbose)
            echo("Searching for box");

        geometry_msgs::PoseStamped mission_msg;
        q.setRPY(0, 0, 0);
        mission_msg.pose.orientation.w = q.getW();
        mission_msg.pose.orientation.x = q.getX();
        mission_msg.pose.orientation.y = q.getY();
        mission_msg.pose.orientation.z = q.getZ();

        for (int i = 0; i < 10;)
        {
            ros::spinOnce();
            loopRate.sleep();
            // check if actual detection by waiting
            if (tag_pose_.id.data == ID_AR_BOX || tag_pose_.id.data == ID_CLR_BOX)
                i += 1; // increase counter on detection
            else
                i -= 1; // decrement counter if no detection

            // exit mission, box not found throughout
            if (prev_wp.wp_seq == num_wp) // !PROBLEM
            {
                if (verbose)
                    echo("Reached last waypoint, going home");
                CONTINUE_MISSION = false;
                mission_msg.pose.position = lz_pose_.pose.position; // go to LZ
                mission_msg.pose.position.z = mav_pose_.pose.position.z;

                for (int i = 0; i < 10; i++) // 10 packets sent before offboard, will take 1 second to complete
                {
                    mission_msg.header.stamp = ros::Time::now();
                    command_pub_.publish(mission_msg);
                    loopRate.sleep();
                }

                while (!mode_set_)
                {
                    mission_msg.header.stamp = ros::Time::now();
                    command_pub_.publish(mission_msg);
                    if (set_mode_client.call(offb_set_mode) and offb_set_mode.response.mode_sent)
                    {
                        if (verbose)
                        {
                            echo("OFFBOARD_MODE enabled");
                            echo("Going to Landing Zone");
                            return;
                        }

                        mode_set_ = true;
                    }
                    loopRate.sleep();
                }
            }
        }

        // ar tag detected!
        echo("AR Tag detected");
        if (tag_pose_.id.data == ID_AR_BOX || tag_pose_.id.data == ID_CLR_BOX)
        {
            // send a few setpoints before starting
            mission_msg.pose.position = mav_pose_.pose.position; // mav_pose_ contains new position
            // hold position

            for (int i = 0; i < 10; i++) // 10 packets sent
            {
                mission_msg.header.stamp = ros::Time::now();
                command_pub_.publish(mission_msg);
                loopRate.sleep();
            }

            while (!mode_set_)
            {
                mission_msg.header.stamp = ros::Time::now();
                command_pub_.publish(mission_msg);
                if (set_mode_client.call(offb_set_mode) and offb_set_mode.response.mode_sent)
                {
                    if (verbose)
                    {
                        echo("OFFBOARD_MODE enabled");
                        echo("Descending to box started");
                    }
                    mode_set_ = true;
                }
                loopRate.sleep();
            }

            loopRate.sleep();
        }
    }

    void fsm::Descend(CmdDescend const &cmd)
    {
        if (verbose)
            echo("Descend mode in OFFBOARD_MODE");
        ros::Rate loopRate(10);

        geometry_msgs::PoseStamped mission_msg;
        q.setRPY(0, 0, 0);
        mission_msg.pose.orientation.w = q.getW();
        mission_msg.pose.orientation.x = q.getX();
        mission_msg.pose.orientation.y = q.getY();
        mission_msg.pose.orientation.z = q.getZ();
        mission_msg.header.stamp = ros::Time::now();
        // px coord system is NED
        // mavros coord system is different mavros_x -> px4_x, mavros_y-> -px4_y, mavros_z -> -px4_z
        while (mav_pose_.pose.position.z >= PICKUP_HOVER_HEIGHT)
        {
            ros::spinOnce();
            mission_msg.pose.position.x = mav_pose_.pose.position.x + ADVANCING_FACTOR_HORZ * tag_pose_.pose.point.x;
            mission_msg.pose.position.y = mav_pose_.pose.position.y - ADVANCING_FACTOR_HORZ * tag_pose_.pose.point.y;
            mission_msg.pose.position.z = mav_pose_.pose.position.z - ADVANCING_FACTOR_Z * tag_pose_.pose.point.z;
            command_pub_.publish(mission_msg);
            loopRate.sleep();
        }
        if (point_distance(mav_pose_, tag_pose_.pose) < DISTANCE_THRESHOLD)
            ALIGNED = true;

        if (verbose)
            echo("Detection based pose reached, trying to align in X now");
        mission_msg.pose.position.x = mav_pose_.pose.position.x + PICKUP_X_DEFLECTION;
        for (int i = 0; i < 10 && ros::ok(); i++)
        {
            command_pub_.publish(mission_msg);
            ros::spinOnce();
            loopRate.sleep();
        }
        if (verbose)
            echo("Drifting lower now, fingers crossed");
        mission_msg.pose.position.z = mav_pose_.pose.position.z - 0.05;
        for (int i = 0; i < 10 && ros::ok(); i++)
        {
            command_pub_.publish(mission_msg);
            ros::spinOnce();
            loopRate.sleep();
        }
    }

    void fsm::Ascend(CmdAscend const &cmd)
    {
        if (verbose)
            echo("Ascend mode in OFFBOARD_MODE");

        ros::Rate loopRate(10);

        geometry_msgs::PoseStamped mission_msg;
        mission_msg.pose.position.z = mav_pose_.pose.position.z + HOVER_HEIGHT;

        for (int i = 0; i < 10 && ros::ok(); i++)
        {
            command_pub_.publish(mission_msg);
            ros::spinOnce();
            loopRate.sleep();
        }
        return;
    }

    void fsm::GoToDZ(CmdDropZone const &cmd)
    {
        dz_pose_.pose.position = lz_pose_.pose.position;
        dz_pose_.pose.position.y = lz_pose_.pose.position.y + DZ_Y_OFFSET_FROM_LZ;
        dz_pose_.pose.position.z = HOVER_HEIGHT;
        if (verbose)
            echo("Going to DropZone");

        ros::Rate loopRate(10);

        for (int i = 0; i < 10; i++)
        {
            ros::spinOnce();
            command_pub_.publish(dz_pose_);
            loopRate.sleep();
        }

        // try to detect DropZone AR tag
        for (int i = 0; i < 10;)
        {
            ros::spinOnce();
            if (tag_pose_.id.data == ID_AR_DZ)
                i += 1;
            else
                i -= 1;
        }
        if (verbose)
            echo("DropZone detected");

        geometry_msgs::PoseStamped mission_msg;
        q.setRPY(0, 0, 0);
        mission_msg.pose.orientation.w = q.getW();
        mission_msg.pose.orientation.x = q.getX();
        mission_msg.pose.orientation.y = q.getY();
        mission_msg.pose.orientation.z = q.getZ();

        while (ros::ok())
        {
            ros::spinOnce();
            mission_msg.header.stamp = ros::Time::now();
            mission_msg.pose.position.x = mav_pose_.pose.position.x + ADVANCING_FACTOR_HORZ * tag_pose_.pose.point.x;
            mission_msg.pose.position.y = mav_pose_.pose.position.y - ADVANCING_FACTOR_HORZ * tag_pose_.pose.point.y;
            mission_msg.pose.position.z = HOVER_HEIGHT;
            command_pub_.publish(mission_msg);
            loopRate.sleep();
        }

        return;
    }

    void fsm::GoToLZ(CmdLandZone const &cmd)
    {
        if (verbose)
            echo("Going to LZ");

        ros::Rate loopRate(10);

        geometry_msgs::PointStamped mission_msg;

        mav_pose_.pose.position.z = -DBL_MAX;
        if (verbose)
            echo("Waiting for MAV position");
        while (mav_pose_.pose.position.z == -DBL_MAX)
        {
            ros::spinOnce();
            loopRate.sleep();
        }
        if (verbose)
            echo("Received MAV position");

        mission_msg.header.stamp = ros::Time::now();
        mission_msg.point.x = lz_pose_.pose.position.x;
        mission_msg.point.y = lz_pose_.pose.position.y;
        mission_msg.point.z = mav_pose_.pose.position.z;

        if (verbose)
            echo("Sending MAV to LZ: " << mission_msg.point.x << " " << mission_msg.point.y << " " << mission_msg.point.z);

        while (ros::ok())
        {
            ros::spinOnce();
            command_pub_.publish(mission_msg);
            if (point_distance(mav_pose_, mission_msg) < DISTANCE_THRESHOLD)
            {
                ALIGNED = true;
                return;
            }
            loopRate.sleep();
        }
        return;
    }

    /// @brief Hovering Action: Wait for HOVER_TIME seconds in the current position
    /// @param cmd
    void fsm::Hovering(CmdHover const &cmd)
    {
        if (verbose)
            echo("Hovering now");

        //? Problem rn is, I don't know how many messages need to be sent for
        //? the alloted sleep time! I also don't know where does mavros store them
        //? and in what form they are being used, why throw multiple at mavros?
        //! Try by running that mavros_node again, but end sending waypoints
        //* mavros needs at least 10 waypoints to work ok, that's what we concluded from trying

        /*
        mavros_msgs::SetMode hold_set_mode;
        bool mode_set_ = false;
        hold_set_mode.request.custom_mode = "AUTO.LOITER";
        if (verbose)
            echo("Changing mode to LOITER");
        while (!mode_set_)
        {
            ros::spinOnce();
            if (set_mode_client.call(hold_set_mode) && hold_set_mode.response.mode_sent)
            {
                if (verbose)
                    echo("HOLD enabled");
                mode_set_ = true;
            }
            loopRate.sleep();
        }
        if (verbose)
            echo("Changed mode to LOITER");
        */

        // publish current point

        ros::Rate hoverRate(1.0 / HOVER_TIME);
        hoverRate.sleep();
    }

    void fsm::Landing(CmdLand const &cmd)
    {
        if (verbose)
            echo("Landing now");

        ros::Rate loopRate(10);

        mavros_msgs::SetMode land_set_mode;
        bool mode_set_ = false;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        if (verbose)
            echo("Changing mode to LAND");
        while (!mode_set_)
        {
            ros::spinOnce();
            if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
            {
                if (verbose)
                    echo("LAND enabled");
                mode_set_ = true;
            }
            loopRate.sleep();
        }
        if (verbose)
            echo("Changed mode to LAND");
    }

    /*
        Guards
    */
    bool fsm::BoxAligned(CmdHover const &cmd)
    {
        if (ALIGNED)
        {
            ALIGNED = false;
            return true;
        }
        return false;
    }

    bool fsm::BoxVisible(CmdDescend const &cmd)
    {
        while (ros::ok())
        {
            ros::spinOnce();
            if (tag_pose_.id.data == ID_AR_BOX || tag_pose_.id.data == ID_CLR_BOX)
            {
                if (verbose)
                    echo("Box Visible");
                return true;
            }
            else
            {
                if (verbose)
                    echo("Box not Visible");
                return false;
            }
        }
        if (verbose)
            echo("ROS Not Ok");
        return false;
    }

    bool fsm::BoxNotVisible(CmdDropZone const &cmd)
    {
        while (ros::ok())
        {
            ros::spinOnce();
            if (tag_pose_.id.data == ID_AR_BOX || tag_pose_.id.data == ID_CLR_BOX)
            {
                if (verbose)
                    echo("Box Visible");
                return false;
            }
            else
            {
                if (verbose)
                    echo("Box not Visible");
                return true;
            }
        }
        if (verbose)
            echo("ROS Not Ok");
        return false;
    }

    bool fsm::StopMission(CmdLand const &cmd)
    {
        CONTINUE_MISSION = false;
        return false;
    }

    double point_distance(geometry_msgs::PoseStamped p1, geometry_msgs::PointStamped p2)
    {
        return sqrt(pow(p1.pose.position.x - p2.point.x, 2) + pow(p1.pose.position.y - p2.point.y, 2) + pow(p1.pose.position.z - p2.point.z, 2));
    }
}