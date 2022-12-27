#pragma once

#include <shastra_planner/planner_utils.hpp>

namespace state_machine
{
    namespace msm = boost::msm;
    namespace mpl = boost::mpl;

    /*
    State Machine Commands
    */
    // takeoff
    struct CmdTakeOff
    {
        CmdTakeOff() {}
    };
    // hover state in between different states
    struct CmdHover
    {
        CmdHover() {}
    };
    // go to grid starting position
    struct CmdGridZone
    {
        CmdGridZone() {}
    };
    // for approaching package
    struct CmdDescend
    {
        CmdDescend() {}
    };
    // for leaving with package
    struct CmdAscend
    {
        CmdAscend() {}
    };
    // go to drop zone
    struct CmdDropZone
    {
        CmdDropZone() {}
    };
    // go to landing zone at last
    struct CmdLandZone
    {
        CmdLandZone() {}
    };
    // land at last
    struct CmdLand
    {
        CmdLand() {}
    };

    struct fsm : public msm::front::state_machine_def<fsm>
    {
        fsm() {} // default constructability

        /*
        callbacks
        */
        void mav_pose_cb_(const geometry_msgs::PoseStamped &msg);
        void state_cb_(const mavros_msgs::State &msg);
        void pose_estimator_cb_(const shastra_msgs::TagPose &msg);
        void wp_reached_cb_(const mavros_msgs::WaypointReached &msg);

        fsm(ros::NodeHandle *nh)
        {
            nh_ = nh;

            CONTINUE_MISSION = true;
            ALIGNED = false;

            PICKUP_HOVER_HEIGHT = 0.4, PICKUP_X_DEFLECTION = 0.27, HOVER_HEIGHT = 1.5;
            HOVER_TIME = 10.0, DZ_Y_OFFSET_FROM_LZ = 4.0;
            DELAY_TIME = 0.5, DISTANCE_THRESHOLD = 0.1;
            ADVANCING_FACTOR_Z = 0.05, ADVANCING_FACTOR_HORZ = 0.5;

            ID_AR_LZ = 0, ID_AR_BOX = 2, ID_CLR_BOX = 75, ID_AR_DZ = 1;

            TRANSITION_TIME = 1.0;

            mission_info = "mission_info", emag_control = "emag_switch", odometry = "odometry", state = "state", utm_pose = "utm_pose", set_mode = "set_mode", mission_waypoint_pull = "mission/wpPull", pose_estimator = "pose_estimator/aruco_pose", land = "mavros/cmd/land", mission_reached = "mission/reached";

            emag_pub_ = nh_->advertise<std_msgs::UInt8>(emag_control, 5);
            command_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(mission_info, 10);

            mav_pose_sub_ = nh_->subscribe(odometry, 10, &state_machine::fsm::mav_pose_cb_, this);
            loc_pose_sub_ = nh_->subscribe(pose_estimator, 1, &state_machine::fsm::pose_estimator_cb_, this);
            state_sub_ = nh_->subscribe(state, 1, &state_machine::fsm::state_cb_, this);
            mission_wp_sub = nh_->subscribe(mission_reached, 10, &state_machine::fsm::wp_reached_cb_, this);

            set_mode_client = nh_->serviceClient<mavros_msgs::SetMode>(set_mode);
            mission_client = nh_->serviceClient<mavros_msgs::WaypointPull>(mission_waypoint_pull);
            landing_client = nh_->serviceClient<mavros_msgs::CommandTOL>(land);
        }

        /*
            State Variables
        */
        bool CONTINUE_MISSION;

        /*
            Parameters
        */
        double PICKUP_HOVER_HEIGHT, PICKUP_X_DEFLECTION, HOVER_HEIGHT;
        double HOVER_TIME, DZ_Y_OFFSET_FROM_LZ;
        double DELAY_TIME, DISTANCE_THRESHOLD;
        double ADVANCING_FACTOR_Z, ADVANCING_FACTOR_HORZ;

        int8_t ID_AR_LZ, ID_AR_BOX, ID_CLR_BOX, ID_AR_DZ;

        double TRANSITION_TIME;

        bool ALIGNED;

        geometry_msgs::PoseStamped mav_pose_, lz_pose_, dz_pose_;
        shastra_msgs::TagPose tag_pose_;
        mavros_msgs::State mav_mode_;
        mavros_msgs::WaypointReached prev_wp;
        tf2::Quaternion q;

        string mission_info, emag_control, odometry, state, utm_pose, set_mode, mission_waypoint_pull, pose_estimator, lidar_distance, land, mission_reached;

        ros::NodeHandle *nh_;
        /*
        publishers
        */
        ros::Publisher command_pub_;
        ros::Publisher emag_pub_;

        /*
            subscribers
        */
        ros::Subscriber mav_pose_sub_;
        ros::Subscriber loc_pose_sub_;
        // ros::Subscriber lidar_dist_sub_ = nh_.subscribe(lidar_distance, 5, lidar_dist_cb_);
        ros::Subscriber state_sub_;
        // ros::Subscriber utm_pose_sub_ = nh_.subscribe(utm_pose, 1, utm_pose_cb_);
        ros::Subscriber mission_wp_sub;

        /*
            service clients
        */
        ros::ServiceClient set_mode_client;
        ros::ServiceClient mission_client;
        ros::ServiceClient landing_client;
        // };

        typedef msm::active_state_switch_before_transition switch_policy;

        template <class Event, class FSM>
        void on_entry(Event const &, FSM &)
        {
            if (verbose)
                echo("Entered state machine");
        }

        template <class Event, class FSM>
        void on_exit(Event const &, FSM &)
        {
            if (verbose)
                echo("Exited state machine");
        }

        /*
            State definitions
        */
        static std::vector<std::string> state_names_;

        struct Rest : State<Rest, verbose>
        {
        };
        struct Hover : State<Hover, verbose>
        {
        };
        struct GridZone : State<GridZone, verbose>
        {
        };
        struct Descent : State<Descent, verbose>
        {
        };
        struct Ascent : State<Ascent, verbose>
        {
        };
        struct DropZone : State<DropZone, verbose>
        {
        };
        struct LandZone : State<LandZone, verbose>
        {
        };

        typedef Rest initial_state;

        /*
            Transition Actions
        */
        void TakeOff(CmdTakeOff const &cmd);
        void GoToGZ(CmdGridZone const &cmd);
        void Descend(CmdDescend const &cmd);
        void Ascend(CmdAscend const &cmd);
        void GoToDZ(CmdDropZone const &cmd);
        void GoToLZ(CmdLandZone const &cmd);
        void Hovering(CmdHover const &cmd);
        void Landing(CmdLand const &cmd);

        /*
            Transition Guards
        */
        bool BoxFound(CmdHover const &cmd);
        bool BoxAligned(CmdHover const &cmd);
        bool BoxVisible(CmdDescend const &cmd);
        bool BoxNotVisible(CmdDropZone const &cmd);
        bool StopMission(CmdLand const &cmd);

        // clang-format off
        struct transition_table : mpl::vector<
        
        //      Type        Start            Event            Next              Action				    Guard
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Rest          ,  CmdTakeOff   ,  Hover         ,  &fsm::TakeOff                               >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Hover         ,  CmdGridZone  ,  GridZone      ,  &fsm::GoToGZ                                >,
                  row<    Hover         ,  CmdDescend   ,  Descent       ,  &fsm::Descend     ,  &fsm::BoxVisible       >,
                a_row<    Hover         ,  CmdAscend    ,  Ascent        ,  &fsm::Ascend                                >,
                  row<    Hover         ,  CmdDropZone  ,  DropZone      ,  &fsm::GoToDZ      ,  &fsm::BoxNotVisible    >,
                a_row<    Hover         ,  CmdLandZone  ,  LandZone      ,  &fsm::GoToLZ                                >,
                  row<    Hover         ,  CmdLand      ,  Rest          ,  &fsm::Landing     ,  &fsm::StopMission      >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    GridZone      ,  CmdHover     ,  Hover         ,  &fsm::Hovering                              >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    Descent       ,  CmdHover     ,  Hover         ,  &fsm::Hovering    ,  &fsm::BoxAligned       >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Ascent        ,  CmdHover     ,  Hover         ,  &fsm::Hovering                              >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    DropZone      ,  CmdHover     ,  Hover         ,  &fsm::Hovering    ,  &fsm::BoxAligned       >
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++

        >{};
        // clang-format on 
    };

    // state machine back end declaration
    typedef msm::back::state_machine<fsm> fsm_;

    // helper function -- output current state
    void echo_state(fsm_ const &msg);

    // state publisher
    void statePublish(ros::NodeHandle nh, fsm_ *fsm);

} // namespace state_machine
