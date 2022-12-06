#include <shastra_planner/planner.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace
{
    // events
    struct cmdTakeoff
    {
    };
    struct cmdLanding
    {
    };

    // front-end
    struct uav_ : public msm::front::state_machine_def<uav_>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &) { std::cout << "entering: Simulation" << std::endl; }
        template <class Event, class FSM>
        void on_exit(Event const &, FSM &) { std::cout << "leaving: Simulation" << std::endl; }

        struct Rest : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &)
            {
                std::cout << "finalizing end" << std::endl;
            }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &);
        };

        struct Hover : public msm::front::state<>
        {
            // events
            struct cmdReachInitArea
            {
            };
            struct cmdGridSearch
            {
            };
            struct cmdDetectPackage
            {
            };
            struct cmdAlignWithPackage
            {
            };
        };

        void takeoff(cmdTakeoff const &);
        void landing(cmdLanding const &);

        // Transition Table
        struct transition_table : mpl::vector2<
                                      //      Start        Event         Next      Action               Guard
                                      //    +---------+-------------+---------+---------------------+----------------------+
                                      a_row<Rest, cmdTakeoff, Hover, &uav_::takeoff>,
                                      //    +---------+-------------+---------+---------------------+----------------------+
                                      a_row<Hover, cmdLanding, Rest, &uav_::landing>>
        {
        };

        // Replaces the default no-transition response.
        template <class FSM, class Event>
        void no_transition(Event const &e, FSM &, int state)
        {
            std::cout << "no transition from state " << state
                      << " on event " << typeid(e).name() << std::endl;
        }
    };

}