#include <ros/ros.h>
#include <heron_msgs/Drive.h>
#include <geometry_msgs/Wrench.h>
#include <heron_controller/heron_constants.h>

class ForceCompensator
{
    private:
        ros::NodeHandle node_;
        ros::Publisher cmd_pub_;
        ros::Publisher eff_pub_;
    public:
        ForceCompensator(ros::NodeHandle &n);
        ~ForceCompensator() {
        }

        double calculate_motor_setting (double thrust);
        double saturate_thrusters (double thrust);
        void pub_thrust_cmd (geometry_msgs::Wrench output);
        void pub_effective_wrench(double left_thrust,double right_thrust);

};
