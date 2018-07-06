#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <control_toolbox/pid.h>
#include <string.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float32.h>
#include <heron_controller/heron_constants.h>
#include <heron_controller/force_compensator.h>
#include <heron_msgs/Helm.h>
#include <heron_msgs/Course.h>
#include <heron_controller/ActivateControl.h>


class Controller {
    private:
        ros::NodeHandle node_;
        ForceCompensator *force_compensator_;
        geometry_msgs::Wrench force_output_;

        //GPS Velocity Feedback timeout
        double vel_data_time_, vel_data_timeout_, vel_cov_limit_;
        bool vel_timeout_;

        //IMU Feedback timeout
        double imu_data_time_, imu_data_timeout_, imu_cov_limit_;
        bool imu_timeout_;

        //Vars to hold time since last cmd
        double course_cmd_time_;
        double helm_cmd_time_;
        double wrench_cmd_time_;
        double twist_cmd_time_;

        control_toolbox::Pid fvel_pid_;
        ros::Publisher fvel_dbg_pub_;
        double fvel_kf_,fvel_kp_, fvel_ki_, fvel_kd_,fvel_imax_,fvel_imin_;
        double fvel_cmd_,fvel_meas_;

        //Yaw Rate Controller Details
        control_toolbox::Pid yr_pid_;
        ros::Publisher yr_dbg_pub_;
        double yr_kf_,yr_kp_, yr_ki_, yr_kd_,yr_imax_,yr_imin_;
        double yr_cmd_,yr_meas_;

        //Yaw Control Details
        control_toolbox::Pid y_pid_;
        ros::Publisher y_dbg_pub_;
        double y_kf_,y_kp_, y_ki_, y_kd_,y_imax_,y_imin_;
        double y_cmd_,y_meas_;

        //Speed Control details
        double max_fwd_vel_,max_fwd_force_,max_bck_vel_,max_bck_force_;

        int control_mode; //Helm, Course, Twist, or Raw Wrench

        ros::ServiceServer active_control_srv;
        bool is_active_control;

    public:
        Controller(ros::NodeHandle &n);
        ~Controller() {
            delete force_compensator_;
        }

        double fvel_compensator();
        double yr_compensator();
        double y_compensator();

        void fwd_vel_mapping();
        void update_fwd_vel_control();
        void update_yaw_rate_control();
        void update_yaw_control();

        void wrench_callback(const geometry_msgs::Wrench msg);
        void course_callback(const heron_msgs::Course msg);
        void helm_callback(const heron_msgs::Helm msg);
        void twist_callback(const geometry_msgs::Twist msg);

        void odom_callback(const nav_msgs::Odometry msg);

        void control_update(const ros::TimerEvent& event);
        void console_update(const ros::TimerEvent& event);

        bool activate_control_service(heron_controller::ActivateControl::Request& req, heron_controller::ActivateControl::Response& resp);
};
