#pragma once
#include <uav_state_machine/generic/behavior.hpp>

namespace ariitk::state_machine
{

    class Ascend
    {
    public:
        struct Event
        {
        };

        void execute(const Event evt);
        void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private, const std::shared_ptr<Behavior> state_ptr);

    private:
        std::shared_ptr<Behavior> mav_state_;
    };

} // namespace ariitk::state_machine