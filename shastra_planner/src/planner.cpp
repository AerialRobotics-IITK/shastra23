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
    struct cmdLowerMav
    {
    };
    struct cmdLowerEmag
    {
    };
    struct cmdReachDrop
    {
    };
    struct cmdAlignAruco
    {
    };

    // front-end
    struct uav_ : public msm::front::state_machine_def<uav_>
    {
        template <class Event, class FSM>
        void on_entry(Event const &, FSM &) { std::cout << "entering: Simulation" << std::endl; }
        template <class Event, class FSM>
        void on_exit(Event const &, FSM &) { std::cout << "leaving: Simulation" << std::endl; }

        // The list of FSM states
        struct Rest : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "at Rest" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &);
        };

        struct Hover : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "entering: Hover" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &) { std::cout << "leaving: Hover" << std::endl; }
        };
        struct ReachingInitArea : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "entering: ReachingInitArea" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &) { std::cout << "leaving: ReachingInitArea" << std::endl; }
        };
        struct GridSearching : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "entering: GridSearching" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &) { std::cout << "leaving: GridSearching" << std::endl; }
        };
        struct AligningWithPackage : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "entering: AligningWithPackage" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &) { std::cout << "leaving: AligningWithPackage" << std::endl; }
        };
        struct LoweringMAV : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "entering: LoweringMAV" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &) { std::cout << "leaving: LoweringMAV" << std::endl; }
        };

        struct LoweringEmag : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "entering: LoweringMAV" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &) { std::cout << "leaving: LoweringMAV" << std::endl; }
        };

        struct ReachingDrop : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "entering: ReachingDrop" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &) { std::cout << "leaving: ReachingDrop" << std::endl; }
        };

        struct AligningWithAruco : public msm::front::state<>
        {
            template <class Event, class FSM>
            void on_entry(Event const &, FSM &) { std::cout << "entering: PickingPackage" << std::endl; }
            template <class Event, class FSM>
            void on_exit(Event const &, FSM &) { std::cout << "leaving: AligningWithPackage" << std::endl; }
        };

        // actions
        void Takeoff(cmdTakeoff const &) {}
        void ReachInit(cmdReachInitArea const &) {}
        void Search(cmdGridSearch const &) {}
        void Align(cmdAlignWithPackage const &) {}
        void LowerMav(cmdLowerMav const &) {}
        void LowerEmag(cmdLowerEmag const &) {} //LowerEmag and activate Emag

        void GoDrop(cmdReachDrop const &) {}
        void AlignAruco(cmdAlignAruco const &) {}


        // guards
        bool ExecMission(cmdReachInitArea const &) {}
        bool ReachedInit(cmdGridSearch const &) {}
        bool FoundPackage(cmdAlignWithPackage const &) {}
        bool Aligned(cmdLowerMav const &) {}

        bool PackageStuck(cmdReachDrop const &) {}
        bool MidPtDetected(cmdAlignAruco const &){}
        
        // Transition Table
        struct transition_table : mpl::vector8 <
            //      Start                       Event                    Next                   Action               Guard
            //    +---------               + -------------         + ---------             + -------------     +-------------
            a_row   <Rest,                  cmdTakeoff,             Hover,                  &uav_::Takeoff>,
            row     <Hover,                 cmdReachInitArea,       ReachingInitArea,       &uav_::ReachInit,   &uav_::ExecMission  >,
            row     <ReachingInitArea,      cmdGridSearch,          GridSearching,          &uav_::Search,      &uav_::ReachedInit  >,
            row     <GridSearching,         cmdAlignWithPackage,    AligningWithPackage,    &uav_::Align,       &uav_::FoundPackage >,
            row     <AligningWithPackage,   cmdLowerMav,            LoweringMAV,            &uav_::LowerMav,    &uav_::Aligned      >,
            a_row   <LoweringMAV,           cmdLowerEmag,           LoweringEmag,           &uav_::LowerEmag                        >,  
            row     <LoweringEmag,          cmdReachDrop,           ReachingDrop,           &uav_::GoDrop,      &uav_::PackageStuck >,
            row     <ReachingDrop,          cmdAlignAruco,          AligningWithAruco,      &uav_::AlignAruco,  &uav_::MidPtDetected> 
         >
         {
         };

        // a_row   <AligningWithPackage,   cmdLower,               PickingPackage,         &msm::front::ActionSequence_ <mpl::vector <uav_::LowerMav, uav_::LowerEmag>>
        // Replaces the default no-transition response.
        template <class FSM, class Event>
        void no_transition(Event const &e, FSM &, int state)
        {
            std::cout << "no transition from state " << state
                      << " on event " << typeid(e).name() << std::endl;
        }
    };

}

int main(){
    std::cout << "CHECK!";
}