#include <shastra_planner/planner.hpp>

namespace ariitk::state_machine
{
    /*
        callbacks
    */
    void mav_pose_cb_(const geometry_msgs::PoseStamped &msg) { mav_pose_ = msg; }
    // void utm_pose_cb_(const shastra_msgs::UTMPose &msg) { utm_pose_ = msg; }
    void state_cb_(const mavros_msgs::State &msg) { mav_mode_ = msg; }
    void pose_estimator_cb_(const shastra_msgs::TagPose &msg) { tag_pose_ = msg; }
    void wp_reached_cb_(const mavros_msgs::WaypointReached &msg) { prev_wp = msg; }

    /*
        Transition actions
    */
    /// @brief TakeOff Action: Wait for MAV position, set_mode to mission
    /// @param cmd
    void fsm::TakeOff(CmdTakeOff const &cmd)
    {
        if (verbose)
            echo("Starting Execution");

        mav_pose_.pose.position.z = -DBL_MAX;
        if (verbose)
            echo("Waiting for MAV Position");
        while (mav_pose_.pose.position.z == -DBL_MAX)
        {
            ros::spinOnce();
            LOOP_RATE.sleep();
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
            LOOP_RATE.sleep();
        }

        if (verbose)
            echo("Changed to MISSION_MODE");

        return;
    }

    /// @brief GoToLandingZone Action: assumes mission_mode initially, receive odometry, continue with mission_mode, obtains left number of waypoints keep checking for any detection in pose_estimated topic.
    /// @param cmd
    void fsm::GoToGZ(CmdGridZone const &cmd)
    {
        if (verbose)
            echo("Going to Grid in MISSION_MODE");
        if (verbose)
            echo("Waiting for odometry");
        mav_pose_.pose.position.z = -DBL_MAX;
        while (mav_pose_.pose.position.z == -DBL_MAX)
        {
            ros::spinOnce();
            LOOP_RATE.sleep();
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
        bool mode_set_ = false;

        if (verbose)
            echo("Searching for box");

        geometry_msgs::PoseStamped mission_msg;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        mission_msg.pose.orientation.w = q.getW();
        mission_msg.pose.orientation.x = q.getX();
        mission_msg.pose.orientation.y = q.getY();
        mission_msg.pose.orientation.z = q.getZ();

        for (int i = 0; i < 10;)
        {
            ros::spinOnce();
            LOOP_RATE.sleep();
            // check if actual detection by waiting
            if (tag_pose_.id.data == ID_AR_BOX || tag_pose_.id.data == ID_CLR_BOX)
                i += 0; // increase counter on detection
            else
                i -= 0; // decrement counter if no detection

            // exit mission, box not found throughout
            if (prev_wp.wp_seq == num_wp)
            {
                if (verbose)
                    echo("Reached last waypoint, going home");
                CONTINUE_MISSION = false;
                mission_msg.pose.position = lz_pose_.pose.position; // go to LZ
                mission_msg.pose.position.z = mav_pose_.pose.position.z;

                for (int i = 0; i < 10; i++) // 10 packets sent before offboard
                {
                    mission_msg.header.stamp = ros::Time::now();
                    command_pub_.publish(mission_msg);
                    LOOP_RATE.sleep();
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
                        }

                        mode_set_ = true;
                    }
                    LOOP_RATE.sleep();
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
                LOOP_RATE.sleep();
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
                LOOP_RATE.sleep();
            }

            LOOP_RATE.sleep();
        }
    }

    void fsm::Descend(CmdDescend const &cmd)
    {
        if (verbose)
            echo("Descend mode in OFFBOARD_MODE");
        while (ros::ok())
        {
            ros::spinOnce();
            LOOP_RATE.sleep();
        }
        // now assuming that tag_pose_ contains the correct location of box
    }

    void fsm::Ascend(CmdAscend const &cmd) {}

    void fsm::GoToDZ(CmdDropZone const &cmd) {}

    void fsm::GoToLZ(CmdLandZone const &cmd)
    {
        if (verbose)
            echo(" Going to LZ");

        geometry_msgs::PointStamped mission_msg;

        mav_pose_.pose.position.z = -DBL_MAX;
        if (verbose)
            echo("Waiting for MAV position");
        while (mav_pose_.pose.position.z == -DBL_MAX)
        {
            ros::spinOnce();
            LOOP_RATE.sleep();
        }
        if (verbose)
            echo("Received MAV position");

        mission_msg.header.stamp = ros::Time::now();
        mission_msg.point.x = lz_pose_.pose.position.x;
        mission_msg.point.y = lz_pose_.pose.position.y;
        mission_msg.point.z = mav_pose_.pose.position.z;

        if (verbose)
            echo("Sending MAV to LZ: " << mission_msg.point.x << " " << mission_msg.point.y << " " << mission_msg.point.z);

        command_pub_.publish(mission_msg);
        return;
    }

    /// @brief Hovering Action: Change to LOITER_MODE
    /// @param cmd
    void fsm::Hovering(CmdHover const &cmd)
    {
        if (verbose)
            echo("Hovering now");

        //? Problem rn is, I don't know how many messages need to be sent for
        //? the alloted sleep time! I also don't know where does mavros store them
        //? and in what form they are being used, why throw multiple at mavros?
        //! Try by running that mavros_node again, but end sending waypoints

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
            LOOP_RATE.sleep();
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
            LOOP_RATE.sleep();
        }
        if (verbose)
            echo("Changed mode to LAND");
    }

    /*
        Guards
    */
    bool fsm::BoxFound(CmdHover const &cmd) { return false; }
    bool fsm::BoxAligned(CmdHover const &cmd) { return false; }
    bool fsm::BoxVisible(CmdDescend const &cmd) { return false; }
    bool fsm::BoxNotVisible(CmdDropZone const &cmd) { return false; }
    bool fsm::StopMission(CmdLand const &cmd) { return false; }
}