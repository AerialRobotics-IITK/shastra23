#pragma once

#include <shastra_planner/planner_utils.hpp>

namespace ariitk::state_machine
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
                  row<    GridZone      ,  CmdHover     ,  Hover         ,  &fsm::Hovering    ,  &fsm::BoxFound         >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                  row<    Descent       ,  CmdHover     ,  Hover         ,  &fsm::Hovering    ,  &fsm::BoxAligned       >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    Ascent        ,  CmdHover     ,  Hover         ,  &fsm::Hovering                              >,
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++
                a_row<    DropZone      ,  CmdHover     ,  Hover         ,  &fsm::Hovering                              >
        // +++ ------- + -------------- + ------------- + -------------- + ------------------ + ---------------------- +++

        >{};
        // clang-format on 
    };

    // state machine back end declaration
    typedef msm::back::state_machine<fsm> fsm_;

    // helper function -- output current state
    inline void echo_state(fsm_ const &msg)
    {
        if (verbose)
            FSM_INFO("Current state -- " << fsm::state_names_[msg.current_state()[0]]);
    }

    // state publisher
    inline void statePublish(ros::NodeHandle nh, fsm_ *fsm)
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

} // namespace ariitk::state_machine