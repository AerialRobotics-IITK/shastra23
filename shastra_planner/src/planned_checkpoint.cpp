#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

//** Node that continues publishing the last seen setpoint shown by the planner

geometry_msgs::PoseStamped pose;
void ckpt_cb(const geometry_msgs::PoseStamped &msg) { pose = msg; }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planned_checkpoint");
    ros::NodeHandle nh;
    ros::Subscriber ckpt_sub = nh.subscribe("mission_info", 10, ckpt_cb);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_local/position", 10, true);
    ros::Rate loopRate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        pose_pub.publish(pose);
        loopRate.sleep();
    }
    return 0;
}
