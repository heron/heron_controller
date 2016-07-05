#include "heron_controller/controller.h"


Controller::Controller(ros::NodeHandle &n):node_(n) {
    force_compensator_ = new ForceCompensator(node_);

    //Assume no messages are being received. Don't send out anything new until commands are received
    control_mode = NO_CONTROL;

    node_.param<double>("imu_data_timeout", imu_data_timeout_,1/5.0); //if sensor feedback has not been recevied in this much amount of time, stop all autonomous behavior
    imu_data_time_ = 0;
    imu_timeout_ = true;

    //Setup Wrench Control
    node_.param<double>("wrench_cmd/timeout",wrench_cmd_timeout_,0.5);//If the commands dont show up in this much time don't send out drive commans
    wrench_cmd_time_ = 0;

    //Setup Yaw Rate Controller
    yr_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("yaw_rate_debug",1000);
    node_.param<double>("yaw_rate/kf", yr_kf_,10); //Feedforward Gain
    node_.param<double>("yaw_rate/kp", yr_kp_,2.0);  //Proportional Gain
    node_.param<double>("yaw_rate/kd", yr_kd_,1.0); //Derivative Gain
    node_.param<double>("yaw_rate/ki", yr_ki_,0.0); //Integral Gain
    node_.param<double>("yaw_rate/imax", yr_imax_,0.0); //Clamp Integral Outputs
    node_.param<double>("yaw_rate/imin", yr_imin_,0.0);
    node_.param<double>("yaw_rate/cmd_timeout", yr_cmd_timeout_,0.5);
    yr_meas_ = 0;

    //Setup Yaw Controller
    y_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("yaw_debug",1000);

    node_.param<double>("yaw/kp", y_kf_,5.0);
    node_.param<double>("yaw/kp", y_kp_,5.0);
    node_.param<double>("yaw/kd", y_kd_,1.0);
    node_.param<double>("yaw/ki", y_ki_,0.5);
    node_.param<double>("yaw/imax", y_imax_,0.0); //clamp integral output at max yaw yorque
    node_.param<double>("yaw/imin", y_imin_,0.0);
    node_.param<double>("yaw/cmd_timeout",y_cmd_timeout_,0.5);
    y_meas_ = 0;

    ROS_DEBUG("Yaw Rate Params (F,P,I,D,Max,Min):%f,%f,%f,%f,%f,%f",yr_kf_,yr_kp_, yr_ki_,yr_kd_,yr_imax_,yr_imin_);
    ROS_DEBUG("Yaw Params (F,P,I,D,Max,Min):%f,%f,%f,%f,%f,%f",y_kf_,y_kp_, y_ki_,y_kd_,y_imax_,y_imin_);

    yr_pid_.reset();
    yr_pid_.initPid(yr_kp_,yr_ki_,yr_kd_,yr_imax_,yr_imin_);
    yr_cmd_ = 0;
    yr_cmd_time_ = 0;
    last_yr_cmd_time_ = 0;

    //Setup Yaw Controller
    y_pid_.reset();
    y_pid_.initPid(y_kp_,y_ki_,y_kd_,y_imax_,y_imin_);
    y_cmd_ = 0;
    y_cmd_time_ = 0;
    last_y_cmd_time_ = 0;

    //Setup Speed Control (linear mapping)
    spd_cmd_ = 0;
    node_.param<double>("max/fwd_vel", max_fwd_vel_,MAX_FWD_VEL);
    node_.param<double>("max/fwd_force", max_fwd_force_,2*MAX_FWD_THRUST); //2 thrusters
    node_.param<double>("max/bck_vel",max_bck_vel_,MAX_BCK_VEL);
    node_.param<double>("max/bck_force",max_bck_force_,2*MAX_BCK_THRUST);
}

double Controller::yr_compensator() {
    //calculate pid torque z
    double yr_error = yr_cmd_ - yr_meas_;
    double yr_comp_output = yr_pid_.computeCommand(yr_error, ros::Duration(1/20.0));
    yr_comp_output = yr_comp_output + yr_kf_*yr_cmd_; //feedforward
    geometry_msgs::Vector3 dbg_info;
    dbg_info.x = yr_cmd_;
    dbg_info.y = yr_meas_;
    dbg_info.z = yr_comp_output;
    yr_dbg_pub_.publish(dbg_info);
    return yr_comp_output;
}

double Controller::y_compensator() {
    //calculate pid torque z

    if (y_meas_ < 0)
        y_meas_ = y_meas_ + 2*PI;

    double y_error = y_cmd_ - y_meas_;

    if (fabs(y_error) > PI){
        if (y_cmd_ > PI) //presumably y_meas_ < PI
            y_error =-( y_meas_ + (2*PI - y_cmd_));
        else //y_cmd_ < pi, y_meas > pi
            y_error =  y_cmd_ +  (2*PI - y_meas_);
    }

    double y_comp_output = y_pid_.computeCommand(y_error, ros::Duration(1/20.0));
    geometry_msgs::Vector3 dbg_info;
    dbg_info.x = y_cmd_;
    dbg_info.y = y_meas_;
    dbg_info.z = y_comp_output;
    y_dbg_pub_.publish(dbg_info);
    return y_comp_output;
}

//Callback to receive raw wrench commands (force along x axis and torque about z axis).
void Controller::wrench_callback(const geometry_msgs::Wrench msg) {
    force_output_.force.x = msg.force.x;
    force_output_.torque.z = msg.torque.z;
    wrench_cmd_time_ = ros::Time::now().toSec();
}


//Callback for yaw command which receives a yaw (rad) and speed (m/s) command
void Controller::course_callback(const heron_msgs::Course msg) {
    //Save Yaw Command to be processed when feedback is available
    y_cmd_ = msg.yaw;
    y_cmd_time_ = ros::Time::now().toSec();

    // Calculate speed command TODO: Can run in its own callback once speed feedback is available
    spd_cmd_ = msg.speed;
    force_output_.force.x = speed_control();
}

//Callback for helm commands which receives a thrust percentage (0..1) and a yaw rate (rad/s)
void Controller::helm_callback(const heron_msgs::Helm msg) {
    //Basic Helm Control

    //Calculate Thrust control
    double thrust = msg.thrust;
    if (thrust >= 0)
        force_output_.force.x = thrust * (max_fwd_force_/1);
    else
        force_output_.force.x = thrust * (max_bck_force_/1);

    //Save yaw rate command to be processed when feedback is available
    yr_cmd_ = msg.yaw_rate;
    yr_cmd_time_ = ros::Time::now().toSec();

}

//Callback for imu data (assuming ENU frame)
void Controller::imu_callback(const sensor_msgs::Imu msg) {

    imu_data_time_ = ros::Time::now().toSec();
    yr_meas_ = msg.angular_velocity.z;
    y_meas_ = tf::getYaw(msg.orientation);

    //run compensator at the same time as imu data received
    switch (control_mode)
    {
        case YAW_CONTROL:
            yr_cmd_ = y_compensator();
            force_output_.torque.z = yr_compensator();
            break;
        case YAW_RATE_CONTROL:
            force_output_.torque.z  = yr_compensator();
            break;
        case WRENCH_CONTROL:
        case NO_CONTROL:
        default:
            break;
     }
}

void Controller::console_update(const ros::TimerEvent& event) {

    std::string output="";
    switch (control_mode)
    {
        case YAW_CONTROL:
            output = "Boat controlling yaw position";
            break;
        case YAW_RATE_CONTROL:
            output = "Boat Controlling yaw rate";
            break;
        case WRENCH_CONTROL:
            output = "Boat in raw wrench/RC control";
            break;
        case NO_CONTROL:
            output = "No commands being processed";
            break;
        default:
            break;
     }

     if (imu_timeout_)
         output+=": IMU data not received or being received too slow";


     ROS_INFO("%s",output.c_str());
}

void Controller::control_update(const ros::TimerEvent& event) {

    if (ros::Time::now().toSec() - imu_data_time_ > imu_data_timeout_)
        imu_timeout_ = true;
    else
        imu_timeout_ = false;

    if (ros::Time::now().toSec() - y_cmd_time_ < y_cmd_timeout_ and !imu_timeout_) //prioritize yaw command, make sure imu data is fresh
        control_mode=YAW_CONTROL;
    else if(ros::Time::now().toSec() - yr_cmd_time_ < yr_cmd_timeout_ and !imu_timeout_)
        control_mode=YAW_RATE_CONTROL;
    else if(ros::Time::now().toSec() - wrench_cmd_time_ < wrench_cmd_timeout_)
        control_mode=WRENCH_CONTROL;
    else {
        control_mode=NO_CONTROL;
        force_output_.torque.z = 0;
        force_output_.force.z = 0;
        return;
    }

    force_compensator_->pub_thrust_cmd(force_output_);
}

double Controller::speed_control() {
    if (spd_cmd_ >= 0)
        return (spd_cmd_*((max_fwd_force_-10)/max_fwd_vel_));
    else
        return (spd_cmd_*((max_bck_force_-10)/max_bck_vel_));
}


int main(int argc, char **argv)
{
    ros::init(argc,argv, "controller");
    ros::NodeHandle nh;
    Controller kf_control(nh);
    //ros::Subscriber vel_sub = nh.subscribe("cmd_vel",1,&Controller::twist_callback, &kf_control);
    ros::Subscriber wrench_sub = nh.subscribe("cmd_wrench",1, &Controller::wrench_callback, &kf_control);
    ros::Subscriber helm_sub = nh.subscribe("cmd_helm",1, &Controller::helm_callback,&kf_control);
    ros::Subscriber course_sub = nh.subscribe("cmd_course",1, &Controller::course_callback,&kf_control);
    ros::Subscriber imu_sub = nh.subscribe("imu/data",1, &Controller::imu_callback, &kf_control);
    ros::Timer control_output = nh.createTimer(ros::Duration(1/50.0), &Controller::control_update,&kf_control);
    ros::Timer console_update = nh.createTimer(ros::Duration(1), &Controller::console_update, &kf_control);

    ros::spin();

    return 0;
}
