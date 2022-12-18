#include <shastra_planner/planner.hpp>

using state_machine::echo_state;
std::vector<std::string> state_machine::fsm::state_names_ = {"Rest", "Hover", "GridZone", "Descent", "Ascent", "DropZone", "LandZone"};

int main(int argc, char **argv)
{
    namespace ar = state_machine;
    ros::init(argc, argv, "planner");
    ros::NodeHandle ph("~");
    ros::Rate loopRate(10);

    state_machine::fsm_ machine(&ph);

    ph.getParam("height/pickup", machine.PICKUP_HOVER_HEIGHT);
    ph.getParam("deflection/x", machine.PICKUP_X_DEFLECTION);
    ph.getParam("height/hover", machine.HOVER_HEIGHT);

    ph.getParam("delay/transition", machine.TRANSITION_TIME);
    ph.getParam("distance_threshold", machine.DISTANCE_THRESHOLD);

    ros::Rate transitRate(1.0 / machine.TRANSITION_TIME);

    machine.start();

    // auto state = std::async(std::launch::async, state_machine::statePublish, ph, &machine);

    machine.process_event(state_machine::CmdTakeOff());
    if (state_machine::verbose)
        echo_state(machine);

    int box_number = 0;

    while (machine.CONTINUE_MISSION and box_number < 2)
    {
        machine.process_event(state_machine::CmdGridZone());
        if (state_machine::verbose)
            echo_state(machine);

        // turn on EMag
        FSM_INFO("Turning ON Emag");
        std_msgs::UInt8 msg;
        msg.data = 1;
        for (int i = 0; i < 10; i++) // send msg 10 times
        {
            machine.emag_pub_.publish(msg);
            loopRate.sleep();
        }

        transitRate.sleep();
        // ? WHAT WOULD HAPPEN IN THIS TRANSIT SLEEP STATE? WHICH MODE?
        // * this planner would enter sleep, and mav would continue on its own
        machine.process_event(state_machine::CmdHover());
        if (state_machine::verbose)
            echo_state(machine);

        box_number += 1;
        transitRate.sleep();
        machine.process_event(state_machine::CmdDescend());
        if (state_machine::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(state_machine::CmdHover());
        if (state_machine::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(state_machine::CmdAscend());
        if (state_machine::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(state_machine::CmdHover());
        if (state_machine::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(state_machine::CmdDropZone());
        if (state_machine::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(state_machine::CmdHover());
        if (state_machine::verbose)
            echo_state(machine);

        // turn off EMag
        FSM_INFO("Turning OFF Emag");
        msg.data = 0;
        for (int i = 0; i < 10; i++) // send msg 10 times
        {
            machine.emag_pub_.publish(msg);
            loopRate.sleep();
        }

        box_number += 1;

        machine.process_event(state_machine::CmdHover());

        if (state_machine::verbose)
            echo_state(machine);
        if (box_number == 1)
        {
            //
        }
    }

    transitRate.sleep();
    machine.process_event(state_machine::CmdLandZone());
    if (state_machine::verbose)
        echo_state(machine);

    transitRate.sleep();
    machine.process_event(state_machine::CmdHover());
    if (state_machine::verbose)
        echo_state(machine);

    transitRate.sleep();
    machine.process_event(state_machine::CmdLand());
    if (state_machine::verbose)
        echo_state(machine);

    machine.stop();
    return 0;
}