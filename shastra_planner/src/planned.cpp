#include <shastra_planner/planner.hpp>

using ariitk::state_machine::echo_state;
std::vector<std::string> ariitk::state_machine::fsm::state_names_ = {"Rest", "Hover", "GridZone", "Descent", "Ascent", "DropZone", "LandZone"};

int main(int argc, char **argv)
{
    namespace ar = ariitk::state_machine;
    ros::init(argc, argv, "planner");
    ros::NodeHandle ph("~");

    ph.getParam("height/pickup", ar::PICKUP_HOVER_HEIGHT);
    ph.getParam("deflection/x", ar::PICKUP_X_DEFLECTION);
    ph.getParam("height/hover", ar::HOVER_HEIGHT);

    ph.getParam("delay/transition", ar::TRANSITION_TIME);
    ph.getParam("distance_threshold", ar::DISTANCE_THRESHOLD);

    ros::Rate transitRate(1.0 / ar::TRANSITION_TIME);

    ariitk::state_machine::fsm_ machine;
    machine.start();

    // auto state = std::async(std::launch::async, ariitk::state_machine::statePublish, ph, &machine);

    machine.process_event(ariitk::state_machine::CmdTakeOff());
    if (ar::verbose)
        echo_state(machine);

    int box_number = 0;

    while (ariitk::state_machine::CONTINUE_MISSION and box_number < 2)
    {
        machine.process_event(ariitk::state_machine::CmdGridZone());
        if (ar::verbose)
            echo_state(machine);

        // turn on EMag
        FSM_INFO("Turning ON Emag");
        std_msgs::UInt8 msg;
        msg.data = 1;
        for (int i = 0; i < 10; i++) // send msg 10 times
        {
            ariitk::state_machine::emag_pub_.publish(msg);
            ar::LOOP_RATE.sleep();
        }

        transitRate.sleep();
        // ? WHAT WOULD HAPPEN IN THIS TRANSIT SLEEP STATE? WHICH MODE?
        // * this planner would enter sleep, and mav would continue on its own
        machine.process_event(ariitk::state_machine::CmdHover());
        if (ar::verbose)
            echo_state(machine);

        box_number += 1;
        transitRate.sleep();
        machine.process_event(ariitk::state_machine::CmdDescend());
        if (ar::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(ariitk::state_machine::CmdHover());
        if (ar::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(ariitk::state_machine::CmdAscend());
        if (ar::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(ariitk::state_machine::CmdHover());
        if (ar::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(ariitk::state_machine::CmdDropZone());
        if (ar::verbose)
            echo_state(machine);

        transitRate.sleep();
        machine.process_event(ariitk::state_machine::CmdHover());
        if (ar::verbose)
            echo_state(machine);

        // turn off EMag
        FSM_INFO("Turning OFF Emag");
        msg.data = 0;
        for (int i = 0; i < 10; i++) // send msg 10 times
        {
            ariitk::state_machine::emag_pub_.publish(msg);
            ar::LOOP_RATE.sleep();
        }

        box_number += 1;

        machine.process_event(ariitk::state_machine::CmdHover());

        if (ar::verbose)
            echo_state(machine);
        if (box_number == 1)
        {
            //
        }
    }

    transitRate.sleep();
    machine.process_event(ariitk::state_machine::CmdLandZone());
    if (ar::verbose)
        echo_state(machine);

    transitRate.sleep();
    machine.process_event(ariitk::state_machine::CmdHover());
    if (ar::verbose)
        echo_state(machine);

    transitRate.sleep();
    machine.process_event(ariitk::state_machine::CmdLand());
    if (ar::verbose)
        echo_state(machine);

    machine.stop();
    return 0;
}