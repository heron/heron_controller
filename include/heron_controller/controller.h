#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <control_toolbox/pid.h>
#include <string.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float32.h>
#include <heron_controller/heron_constants.h>
#include <heron_controller/force_compensator.h>
#include <heron_msgs/Helm.h>
#include <heron_msgs/Course.h>


class Controller {
    private:
        ros::NodeHandle node_;
        ForceCompensator *force_compensator_;
        geometry_msgs::Wrench force_output_;

        //IMU Feedback timeout
        double imu_data_time_, imu_data_timeout_;
        bool imu_timeout_;

        //Wrench output (raw forces)
        double wrench_cmd_time_,wrench_cmd_timeout_;

        //Yaw Rate Controller Details
        control_toolbox::Pid yr_pid_;
        ros::Publisher yr_dbg_pub_;
        double yr_kf_,yr_kp_, yr_ki_, yr_kd_,yr_imax_,yr_imin_;
        double yr_cmd_,yr_cmd_time_,last_yr_cmd_time_,yr_cmd_timeout_;
        double yr_meas_;

        //Yaw Control Details
        control_toolbox::Pid y_pid_;
        ros::Publisher y_dbg_pub_;
        double y_kf_,y_kp_, y_ki_, y_kd_,y_imax_,y_imin_;
        double y_cmd_,y_cmd_time_,last_y_cmd_time_,y_cmd_timeout_;
        double y_meas_;

        //Speed Control details
        double spd_cmd_;
        double max_fwd_vel_,max_fwd_force_,max_bck_vel_,max_bck_force_;

        int control_mode; //Helm, Heading or Raw Wrench

    public:
        Controller(ros::NodeHandle &n);
        ~Controller() {
            delete force_compensator_;
        }

        double yr_compensator();
        double y_compensator();
        void wrench_callback(const geometry_msgs::Wrench msg);
        void course_callback(const heron_msgs::Course msg);
        void helm_callback(const heron_msgs::Helm msg);
        void imu_callback(const sensor_msgs::Imu msg);
        void control_update(const ros::TimerEvent& event);
        void console_update(const ros::TimerEvent& event);
        double speed_control();
};
